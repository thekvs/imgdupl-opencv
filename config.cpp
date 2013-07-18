#include "config.hpp"

namespace imgdupl {

bool
Config::init(int argc, char **argv)
{
    namespace bpo = boost::program_options;

    bpo::options_description generic("Basic options");

    generic.add_options()
        ("help,h", "produce help message")
        ("config", bpo::value<std::string>(), "configuration file")
        ;

    bpo::options_description advanced("Advanced options");

    advanced.add_options()
        ("bind-addr", bpo::value<std::string>(), "ip:port to bind")
        ("db-file", bpo::value<std::string>(), "database file")
        ("access-log-file", bpo::value<std::string>(), "access log file")
        ("detector", bpo::value<std::string>()->default_value(std::string("SURF")), "image features detector type")
        ("extractor", bpo::value<std::string>()->default_value(std::string("SURF")), "image features extractor type")
        ("matcher", bpo::value<std::string>()->default_value(std::string("FlannBased")), "image features matcher type")
        ("resize", bpo::value<bool>()->zero_tokens()->default_value(false), "resize image before processing")
        ;

    bpo::options_description cmdline_options;
    cmdline_options.add(generic).add(advanced);

    bpo::options_description config_file_options;
    config_file_options.add(advanced);

    try {
        bpo::store(bpo::parse_command_line(argc, argv, cmdline_options), vm);
        bpo::notify(vm);
    } catch (std::exception &e) {
        THROW_EXC("Parser failed: %s", e.what());
    }

    if (vm.count("help")) {
        std::cout << cmdline_options << std::endl;
        return false;
    }

    if (!vm.count("config")) {
        std::cerr << "Config file is not specified. Run " << argv[0] <<
            " --help for options." << std::endl;
        return false;
    }

    std::string   filename = vm["config"].as<std::string>();
    std::ifstream file(filename.c_str(), std::ios::in|std::ios::binary);

    if (file.fail()) {
        THROW_EXC("Can't open file: %s", filename.c_str());
    }

    try {
        bpo::store(bpo::parse_config_file(file, config_file_options), vm);
        bpo::notify(vm);
    } catch (std::exception &e) {
        THROW_EXC("Parser failed: %s", e.what());
    }

    check_if_param_specified("bind-addr");
    check_if_param_specified("db-file");
    check_if_param_specified("access-log-file");

    return true;
}

} // namespace