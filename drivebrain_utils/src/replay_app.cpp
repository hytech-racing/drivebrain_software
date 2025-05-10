#include "MCAPReplay.hpp"
#include <iostream>
#include <boost/program_options.hpp>

int main(int argc, char* argv[])
{
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");


    std::string mcap_filepath;
    desc.add_options()
        ("help,h", "produce help message")
        ("mcap,m", po::value<std::string>(&mcap_filepath), "Path to mcap file");
    
    util::MCAPReplay replay_util;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc<<std::endl;
        std::exit(0);
    }

    replay_util.start(mcap_filepath);
    return 0;
}