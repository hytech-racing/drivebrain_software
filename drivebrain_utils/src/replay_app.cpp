#include "MCAPReplay.hpp"
#include <iostream>
#include <boost/program_options.hpp>

int main(int argc, char* argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");


    spdlog::set_level(spdlog::level::debug);
    std::string mcap_filepath, dbc_filepath, config_filepath;
    desc.add_options()
        ("help,h", "produce help message")
        ("mcap,m", po::value<std::string>(&mcap_filepath), "Path to mcap file")
        ("config,c", po::value<std::string>(&config_filepath), "Path to config file")
        ("dbc,d", po::value<std::string>(&dbc_filepath), "Path to dbc file");
    
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc<<std::endl;
        std::exit(0);
    }

    util::MCAPReplay replay_util(config_filepath, dbc_filepath);
    replay_util.start(mcap_filepath);
    return 0;
}