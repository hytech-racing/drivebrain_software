#include <JsonFileHandler.hpp>
#include <CANComms.hpp>
#include <SimpleController.hpp>
#include <StateEstimator.hpp>
#include <MCUETHComms.hpp>
#include <VNComms.hpp>
#include <MsgLogger.hpp>
#include <MCAPProtobufLogger.hpp>
#include <mcap/writer.hpp>
#include <DrivebrainBase.hpp>
#include <foxglove_server.hpp>
#include "DriveBrainApp.hpp" 
#include <array>

#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::seconds
#include <condition_variable>

#include <cassert>

#include <boost/program_options.hpp>
#include <boost/asio.hpp>

#include <memory>
#include <optional>

#include <mcap/mcap.hpp>
#include <thread> // For std::this_thread::sleep_for
#include <chrono> // For std::chrono::seconds

#include <csignal>
#include <cstdlib>

#include <versions.h>

#include "hytech_msgs.pb.h"
#include <iostream>
#include <sstream>

#include <spdlog/spdlog.h>

// TODO first application will have

// - [x] message queue that can send messages between the CAN driver and the controller
// - [x] CAN driver that can receive the pedals messages
// - [ ] fix the CAN messages that cant currently be encoded into the protobuf messages
// - [x] simple controller

std::atomic<bool> stop_signal{false};
// Signal handler function
void signal_handler(int signal)
{
    spdlog::info("Interrupt signal ({}) received. Cleaning up...", signal);
    stop_signal.store(true); // Set running to false to exit the main loop or gracefully terminate
}


std::pair<std::string, std::string> parse_arguments(int argc, char* argv[]) {
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    std::string param_path = "config/drivebrain_config.json";
    std::string dbc_path;

    desc.add_options()
        ("help,h", "produce help message")
        ("param-path,p", po::value<std::string>(&param_path), "Path to the parameter JSON file")
        ("dbc-path,d", po::value<std::string>(&dbc_path), "Path to the DBC file (optional)");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        std::exit(0);
    }

    return {param_path, dbc_path};
}

int main(int argc, char *argv[])
{
    std::signal(SIGINT, signal_handler);
    
    try {

        auto [param_path, dbc_path] = parse_arguments(argc, argv);

        DriveBrainSettings settings{
            .run_db_service = true,
            .run_io_context = true,
            .run_process_loop = true
        };
        
        DriveBrainApp app(param_path, dbc_path, settings);

        app.run();
    } catch (const std::exception& e) {
        spdlog::error("Error in main: {}", e.what());
        return 1;
    }
    
    return 0;
}
