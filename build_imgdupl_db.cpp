#include <sys/types.h>
#include <signal.h>
#include <syslog.h>
#include <stdarg.h>
#include <unistd.h>

#include <string>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <cstdio>
#include <cstdlib>
#include <cerrno>
#include <cstring>

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include <boost/filesystem.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <sqlite3.h>

#include "exc.hpp"
#include "serialize.hpp"
#include "data_types.hpp"

using namespace imgdupl;

const int x_size = 400;

namespace bfs = boost::filesystem;

class Ctx {
public:

    explicit Ctx(std::string path):
        db_file(path),
        db_handler(NULL),
        data_insert_stmt(NULL)
    {
        detector = cv::Ptr<cv::FeatureDetector>(new cv::SurfFeatureDetector(400));
        extractor = cv::Ptr<cv::DescriptorExtractor>(new cv::SurfDescriptorExtractor());

        open_db();
        create_table_if_not_exists();

        create_insert_stmt();
        start_transaction();
    }

    ~Ctx()
    {
        sqlite3_finalize(data_insert_stmt);
        finish_transaction();
        sqlite3_close(db_handler);
        sqlite3_shutdown();
    }

    void save(const bfs::path &path, const KeyPoints &keypoints, const Descriptors &descriptors)
    {
        std::stringstream compressed;

        {
            namespace bio = boost::iostreams;

            bio::filtering_streambuf<bio::output> compressor;

            compressor.push(bio::gzip_compressor(bio::gzip::best_speed));
            compressor.push(compressed);

            boost::archive::binary_oarchive oa(compressor);

            oa << keypoints;
            oa << descriptors;
        }

        int rc;

        rc = sqlite3_bind_text(data_insert_stmt, 1, path.string().c_str(), path.string().size(), SQLITE_TRANSIENT);
        THROW_EXC_IF_FAILED(rc == SQLITE_OK, "sqlite3_bind_text() failed: \"%s\"", sqlite3_errmsg(db_handler));

        rc = sqlite3_bind_text(data_insert_stmt, 2, compressed.str().c_str(), compressed.str().size(),
            SQLITE_TRANSIENT);
        THROW_EXC_IF_FAILED(rc == SQLITE_OK, "sqlite3_bind_text() failed: \"%s\"", sqlite3_errmsg(db_handler));

        rc = sqlite3_step(data_insert_stmt);
        THROW_EXC_IF_FAILED(rc == SQLITE_DONE, "sqlite3_step() failed: \"%s\"", sqlite3_errmsg(db_handler));

        rc = sqlite3_reset(data_insert_stmt);
        THROW_EXC_IF_FAILED(rc == SQLITE_OK, "sqlite3_reset() failed: \"%s\"", sqlite3_errmsg(db_handler));
    }

    FeatureDetectorPtr     detector;
    DescriptorExtractorPtr extractor;

private:

    std::string   db_file;
    sqlite3      *db_handler;
    sqlite3_stmt *data_insert_stmt;

    Ctx() {}

    void open_db()
    {
        int rc = sqlite3_open_v2(db_file.c_str(), &db_handler, SQLITE_OPEN_READWRITE|SQLITE_OPEN_CREATE, NULL);
        THROW_EXC_IF_FAILED(rc == SQLITE_OK, "sqlite3_open_v2() failed");
    }

    bool table_exists()
    {
        std::string   st = "SELECT COUNT(*) FROM sqlite_master WHERE name ='images' and type='table'";
        sqlite3_stmt *stmt = NULL;

        int rc = sqlite3_prepare_v2(db_handler, st.c_str(), st.size(), &stmt, NULL);
        THROW_EXC_IF_FAILED(rc == SQLITE_OK, "sqlite3_prepare_v2() failed: \"%s\"", sqlite3_errmsg(db_handler));

        rc = sqlite3_step(stmt);
        THROW_EXC_IF_FAILED(rc == SQLITE_ROW, "sqlite3_step() failed (%i): \"%s\"",
            rc, sqlite3_errmsg(db_handler));

        bool result = false;

        int count = sqlite3_column_int(stmt, 0);
        if (count == 1) {
            result = true;
        }

        rc = sqlite3_finalize(stmt);
        THROW_EXC_IF_FAILED(rc == SQLITE_OK, "sqlite3_finalize() failed: \"%s\"", sqlite3_errmsg(db_handler));

        return result;
    }

    void create_table_if_not_exists()
    {
        if (table_exists()) {
            return;
        }

        std::string   st = "CREATE TABLE images (id INTEGER PRIMARY KEY AUTOINCREMENT, path TEXT, data BLOB)";
        sqlite3_stmt *stmt = NULL;

        int rc = sqlite3_prepare_v2(db_handler, st.c_str(), st.size(), &stmt, NULL);
        THROW_EXC_IF_FAILED(rc == SQLITE_OK, "sqlite3_prepare_v2() failed (%i): \"%s\"",
            rc, sqlite3_errmsg(db_handler));

        rc = sqlite3_step(stmt);
        THROW_EXC_IF_FAILED(rc == SQLITE_DONE, "sqlite3_step() failed (%i): \"%s\"",
            rc, sqlite3_errmsg(db_handler));

        rc = sqlite3_finalize(stmt);
        THROW_EXC_IF_FAILED(rc == SQLITE_OK, "sqlite3_finalize() failed (%i): \"%s\"",
            rc, sqlite3_errmsg(db_handler));
    }

    void create_insert_stmt()
    {
        std::string st = "INSERT INTO images (path, data) values (?, ?)";

        int rc = sqlite3_prepare_v2(db_handler, st.c_str(), st.size(), &data_insert_stmt, NULL);
        THROW_EXC_IF_FAILED(rc == SQLITE_OK, "sqlite3_prepare_v2() failed: \"%s\"", sqlite3_errmsg(db_handler));
    }

    void start_transaction()
    {
        char *errmsg;

        int rc = sqlite3_exec(db_handler, "BEGIN", NULL, NULL, &errmsg);
        THROW_EXC_IF_FAILED(rc == SQLITE_OK, "sqlite3_exec() failed: \"%s\"", errmsg);
    }

    void finish_transaction()
    {
        char *errmsg;

        int rc = sqlite3_exec(db_handler, "COMMIT", NULL, NULL, &errmsg);
        THROW_EXC_IF_FAILED(rc == SQLITE_OK, "sqlite3_exec() failed: \"%s\"", errmsg);   
    }
};

void
usage(const char *program)
{
    std::cout << "Usage: " << program << " <directory> <filename>" << std::endl;
    exit(EXIT_SUCCESS);
}

bool
process_file(Ctx &ctx, const bfs::path &filename)
{
    cv::Mat orig_image = cv::imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
    if (orig_image.empty()) {
        return false;
    }

    double ratio = static_cast<double>(x_size) / orig_image.cols;

    cv::Mat image;

    cv::resize(orig_image, image, cv::Size(), ratio, ratio, cv::INTER_CUBIC);

    KeyPoints   keypoints;
    Descriptors descriptors;

    ctx.detector->detect(image, keypoints);
    ctx.extractor->compute(image, keypoints, descriptors);

    ctx.save(filename, keypoints, descriptors);

    return true;
}

void
process_directory(Ctx &ctx, std::string directory)
{
    bfs::path root(directory);
    bfs::recursive_directory_iterator cur_it(root), end_it;

    for (; cur_it != end_it; ++cur_it) {
        if (bfs::is_regular_file(cur_it->path().string())) {
            bool status = process_file(ctx, cur_it->path());
            if (status == false) {
                std::cerr << "failed to process " << cur_it->path() << std::endl;
            }
        }
    }
}

int
main(int argc, char **argv)
{
    cv::initModule_nonfree();

    if (argc < 3) {
        usage(argv[0]);
    }

    std::string path = argv[1];
    std::string db_file = argv[2];

    try {
        Ctx ctx(db_file);

        if (bfs::exists(path)) {
            if (bfs::is_regular_file(path)) {
                bool status = process_file(ctx, path);
                if (!status) {
                    std::cerr << "failed to process " << path << std::endl;
                }
            } else if (bfs::is_directory(path)) {
                process_directory(ctx, path);
            }
        } else {
            THROW_EXC("\"%s\" does not exist", path.c_str());
        }
    } catch (std::exception &e) {
        std::cerr << "Oops: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
