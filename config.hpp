#ifndef __CONFIG_HPP_INCLUDED__
#define __CONFIG_HPP_INCLUDED__

#include <iostream>
#include <fstream>
#include <string>

#include <boost/program_options.hpp>

#include "exc.hpp"

namespace imgdupl {

class Config {
public:

    Config() {};

    bool init(int argc, char **argv);

    std::string config() const {
        return vm["config"].as<std::string>();
    }

    std::string bind_addr() const {
        return vm["bind-addr"].as<std::string>();
    }

    std::string db_file() const {
        return vm["db-file"].as<std::string>();
    }

    std::string access_log_file() const {
        return vm["access-log-file"].as<std::string>();
    }

    std::string detector() const {
        return vm["detector"].as<std::string>();
    }

    std::string extractor() const {
        return vm["extractor"].as<std::string>();
    }

    std::string matcher() const {
        return vm["matcher"].as<std::string>();
    }

    bool resize() const {
        return vm["resize"].as<bool>();
    }

private:

    boost::program_options::variables_map vm;

    void check_if_param_specified(const std::string &param) {
        THROW_EXC_IF_FAILED(vm.count(param) == true,
            "mandatory configuration parameter '%s' is not specified", param.c_str());
    }
};

} // namespace

#endif
