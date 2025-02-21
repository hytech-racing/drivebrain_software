#include "../include/AeroComms.hpp"
#include "hytech_msgs.pb.h"
#include <boost/asio.hpp>
#include <iostream>
#include <memory>

class MockLogger {
public:
    void log_string(const std::string& message, int level) {
        std::cout << "[LOG]: " << message << " [Level: " << level << "]" << std::endl;
    }
};

class MockStateEstimator {
public:
    void handle_recv_process(std::shared_ptr<google::protobuf::Message> msg) {
        std::cout << "[STATE ESTIMATOR]: Received Protobuf Message" << std::endl;
    }
};

int main() {
    boost::asio::io_context io;
    core::JsonFileHandler json_handler("/path/to/config.json");
    core::Logger logger(core::LogLevel::INFO);
    auto message_logger = std::make_shared<loggertype>(
        "aero_test_log",
        true,
        [](std::shared_ptr<google::protobuf::Message> msg) {
            std::cout << "Logging test message." << std::endl;
        },
        []() {
            std::cout << "Flushing test logs." << std::endl;
        },
        [](const std::string &error) {
            std::cerr << "Logger test error: " << error << std::endl;
        },
        [](std::shared_ptr<google::protobuf::Message> status) {
            std::cout << "Test status update received." << std::endl;
        }
    );
    bool construction_failed = false;
    estimation::Tire_Model_Codegen_MatlabModel matlab_estimator(logger, json_handler, construction_failed);

    if (construction_failed) {
        std::cerr << "Matlab Model Test Construction Failed." << std::endl;
    }
    core::StateEstimator state_estimator(logger, message_logger, matlab_estimator);
    comms::AeroDriver driver(json_handler, logger, message_logger, state_estimator, io);
    if (!driver.init()) {
        return 1;
    }
    driver.start_receive();
    io.run();
    return 0;
}
