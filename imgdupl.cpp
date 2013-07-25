#include <sys/types.h>
#include <signal.h>
#include <syslog.h>
#include <stdarg.h>
#include <unistd.h>

#include <string>
#include <iostream>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstring>

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <boost/tuple/tuple.hpp>
#include <boost/scoped_array.hpp>
#include <boost/lexical_cast.hpp>

#include "mongoose.h"
#include "searcher.hpp"
#include "exc.hpp"
#include "serialize.hpp"
#include "data_types.hpp"
#include "json_writer.hpp"

using namespace imgdupl;

static volatile sig_atomic_t interrupted;

typedef enum  {
    UNKNOWN_COMMAND,
    TEST_COMMAND,
    CHECK_COMMAND
} CommandType;

const int x_size = 400;

void
init_signals(sigset_t *set)
{
    sigemptyset(set);

    sigaddset(set, SIGTERM);
    sigaddset(set, SIGINT);
    sigaddset(set, SIGHUP);

    pthread_sigmask(SIG_BLOCK, set, NULL);
}

void*
sigwaiter(void *arg)
{
    int       status, signum;
    sigset_t *set;

    set = static_cast<sigset_t*>(arg);

    while (true) {
        status = sigwait(set, &signum);
        if (status == 0) {
            if (signum == SIGINT || signum == SIGTERM || signum == SIGHUP) {
                interrupted = 1;
                break;
            }
        }
    }

    return NULL;
}

std::pair<bool, std::string>
process_data(struct mg_connection *conn, Searcher *searcher)
{
    const char *content_length_header = mg_get_header(conn, "Content-Length");
    const char *content_type_header = mg_get_header(conn, "Content-Type");

    if (content_length_header == NULL || content_type_header == NULL) {
        THROW_EXC("Invalid input");
    }

    size_t raw_content_length = boost::lexical_cast<size_t>(content_length_header);

    boost::scoped_array<char> raw_content_storage(new char[raw_content_length]);
    char *raw_content = raw_content_storage.get();

    int real_raw_content_length = mg_read(conn, raw_content, raw_content_length);
    if (real_raw_content_length <= 0) {
        THROW_EXC("Invalid input");
    }

    std::vector<char> content;

    content.reserve(real_raw_content_length);
    content.insert(content.end(), raw_content, raw_content + real_raw_content_length);

    cv::Mat image = cv::imdecode(content, CV_LOAD_IMAGE_GRAYSCALE);
    if (image.empty()) {
        THROW_EXC("Couldn't decode an image");
    }

    if (searcher->cfg.resize()) {
        cv::Mat resized_image;

        double ratio = static_cast<double>(x_size) / image.cols;
        cv::resize(image, resized_image, cv::Size(), ratio, ratio, cv::INTER_CUBIC);        

        return searcher->search(resized_image);
    } else {
        return searcher->search(image);
    }
}

void
send_response(struct mg_connection *conn, bool operation_status, bool check_status,
              const std::string &image_path, const char *custom_msg = NULL)
{
    char   content[4096];
    size_t content_length;

    yajl_gen json_generator = yajl_gen_alloc(NULL);

    if (operation_status) {
        // To form a valid JSON JsonMapWriter's destructor must be called,
        // so create separate scope for it
        {
            JsonMapWriter writer(json_generator);
            if (check_status == true) {
                writer.write("found", 1);
                writer.write("path", image_path);
            } else {
                writer.write("found", 0);
            }
        }

        const unsigned char *buf;
        size_t               size;

        yajl_gen_status status = yajl_gen_get_buf(json_generator, &buf, &size);
        THROW_EXC_IF_FAILED(status == yajl_gen_status_ok,
            "YAJL generator failed, error code: %u", status);

        content_length = std::min(size, sizeof(content));
        memcpy(content, buf, content_length);
        
        mg_printf(conn,
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: application/json\r\n"
            "Content-Length: %zu\r\n"
            "\r\n"
            "%s",
            content_length,
            content
        );
    } else {
        if (custom_msg) {
            syslog(LOG_ERR, "%s", custom_msg);
        }
        content_length = snprintf(content, sizeof(content), "Internal error. Check syslog logs for details.\n");
        mg_printf(conn,
            "HTTP/1.1 500 Internal Server Error\r\n"
            "Content-Type: text/plain\r\n"
            "Content-Length: %zu\r\n"
            "\r\n"
            "%s",
            content_length,
            content
        );
    }

    yajl_gen_free(json_generator);
}

void
handle_command(struct mg_connection *conn, const struct mg_request_info *request_info, CommandType type)
{
    try {
        Searcher *searcher = static_cast<Searcher*>(request_info->user_data);

        bool        check_status = false;
        std::string image_path;

        if (type == TEST_COMMAND) {
            THROW_EXC("Unimplemented method called");
        } else if (type == UNKNOWN_COMMAND) {
            THROW_EXC("Request of an unknown method");
        } else if (type == CHECK_COMMAND) {
            boost::tie(check_status, image_path) = process_data(conn, searcher);
        }
        send_response(conn, true, check_status, image_path);
    } catch (std::exception &e) {
        send_response(conn, false, false, "", e.what());
    }
}

void*
handler(enum mg_event event, struct mg_connection *conn)
{
    const struct mg_request_info *request_info = mg_get_request_info(conn);
    const void *processed = "yes";
    
    if (event == MG_NEW_REQUEST) {
        if (strcmp(request_info->uri, "/test") == 0) {
            handle_command(conn, request_info, TEST_COMMAND);
        } else if (strcmp(request_info->uri, "/check") == 0) {
            handle_command(conn, request_info, CHECK_COMMAND);
        } else {
            handle_command(conn, request_info, UNKNOWN_COMMAND);
        }
    } else if (event == MG_EVENT_LOG) {
        processed = NULL;
    } else {
        processed = NULL;
    }

    return const_cast<void*>(processed);
}

void
usage(const char *program)
{
    std::cout << program << " <ip:port> <images db> <access log file>" << std::endl;
    exit(EXIT_SUCCESS);
}

int
main(int argc, char **argv)
{
    cv::initModule_nonfree();

    Config config;

    try {
        bool initialized = config.init(argc, argv);
        if (!initialized) {
            exit(0);
        }
    } catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
        std::cerr << "Try run with --help to get the full list of supported options." << std::endl;
        exit(EXIT_FAILURE);
    }

    Searcher searcher(config);

    openlog("checker", LOG_PID, LOG_USER);

    const char *options[] = {
        "document_root", "/var/empty",
        "num_threads", "3",
        "listening_ports", "",
        "access_log_file", "",
        NULL
    };

    options[5] = config.bind_addr().c_str();
    options[7] = config.access_log_file().c_str();

    struct mg_context *ctx = mg_start(&handler, static_cast<void*>(&searcher), options);
    if (ctx == NULL) {
        std::cerr << "Cannot start server, fatal exit" << std::endl;
        exit(EXIT_FAILURE);
    }

    sigset_t sigset;
    init_signals(&sigset);
    mg_start_thread(sigwaiter, &sigset);

    while (!interrupted) {
        usleep(100);
    }

    mg_stop(ctx);
    closelog();

    return EXIT_SUCCESS;
}
