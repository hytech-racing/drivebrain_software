#include "arg_parse.hpp"

#include <iostream>

std::tuple<std::string, std::string, bool> parse_arguments(int argc, char* argv[]) {
    
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    std::string param_path = "config/drivebrain_config.json";
    std::string dbc_path;

    bool use_secondary_can = true;

    desc.add_options()
        ("help,h", "produce help message")
        ("param-path,p", po::value<std::string>(&param_path), "Path to the parameter JSON file")
        ("dbc-path,d", po::value<std::string>(&dbc_path), "Path to the DBC file (optional)")
        ("2-can,c", po::value<bool>(&use_secondary_can), "use secondary CAN");
        
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        std::exit(0);
    }

    return {param_path, dbc_path, use_secondary_can};
}