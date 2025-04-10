
#include <JsonFileHandler.hpp>
#include <CANComms.hpp>
#include <SimpleSpeedController.hpp>
#include <ControllerManager.hpp>
#include <StateEstimator.hpp>
#include <MCUETHComms.hpp>
#include <VNComms.hpp>
#include <MsgLogger.hpp>
#include <DrivebrainMCAPLogger.hpp>
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

#include "arg_parse.hpp"

// TODO first application will have

// - [x] message queue that can send messages between the CAN driver and the controller
// - [x] CAN driver that can receive the pedals messages
// - [ ] fix the CAN messages that cant currently be encoded into the protobuf messages
// - [x] simple controller

int main(int argc, char *argv[])
{
    try {
        
        auto [param_path, dbc_path, second_can] = parse_arguments(argc, argv);

        DriveBrainSettings settings{
            .run_db_service = true,
            .run_io_context = true,
            .run_process_loop = true,
            .use_vectornav = false,
            .use_secondary_can = second_can
        };
        
        std::cout <<"creating app" <<std::endl;
        DriveBrainApp app(param_path, dbc_path, settings);
        std::cout <<"app created" <<std::endl;
        app.run();
    } catch (const std::exception& e) {
        spdlog::error("Error in main: {}", e.what());
        return 1;
    }
    
    return 0;
}
