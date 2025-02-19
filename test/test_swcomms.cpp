#include <boost/asio.hpp>
#include <iostream>
#include <memory>
#include <thread>
#include "SWComms.hpp"
#include "hytech_msgs.pb.h"

class MockLogger : public core::Logger {
public:
    MockLogger() : core::Logger(core::LogLevel::INFO) {}  

    void log_string(const std::string &message, core::LogLevel lvl) {
        std::cout << "[MOCK LOGGER] " << message << " [Level: " << static_cast<int>(lvl) << "]" << std::endl;
    }
};

class MockStateEstimator : public core::StateEstimator {
public:
    void handle_recv_process(std::shared_ptr<google::protobuf::Message> msg) {
        auto scale_data = std::dynamic_pointer_cast<hytech_msgs::WeighScaleData>(msg);
        if (scale_data) {
            std::cout << "[STATE ESTIMATOR]: Received WeighScaleData Message\n";
            std::cout << "  LF: " << scale_data->weight_lf() << "\n";
            std::cout << "  LR: " << scale_data->weight_lr() << "\n";
            std::cout << "  RF: " << scale_data->weight_rf() << "\n";
            std::cout << "  RR: " << scale_data->weight_rr() << "\n";
        } else {
            std::cerr << "[ERROR]: Invalid Protobuf message received in MockStateEstimator!\n";
        }
    }
};

int main() {
    boost::asio::io_context io;

    core::JsonFileHandler json_handler("config.json"); 
    MockLogger logger;
    std::shared_ptr<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>> message_logger =
        std::make_shared<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>>(
            "log_extension",
            false,
            [](std::shared_ptr<google::protobuf::Message> msg) { std::cout << "Logging message\n"; },
            []() { std::cout << "Flush triggered\n"; },
            [](const std::string &msg) { std::cout << "Error: " << msg << "\n"; },
            [](std::shared_ptr<google::protobuf::Message> msg) { std::cout << "Message stored\n"; }
        );

    MockStateEstimator state_estimator;

    comms::SWDriver sw_driver(json_handler, logger, message_logger, state_estimator, io);

    if (!sw_driver.init()) {
        std::cerr << "[TEST ERROR]: SWDriver failed to initialize!\n";
        return 1;
    }

    std::thread io_thread([&]() { io.run(); });

    std::cout << "[TEST]: SWDriver initialized and running...\n";

    std::this_thread::sleep_for(std::chrono::seconds(5));

    io.stop();
    io_thread.join();

    std::cout << "[TEST]: SWDriver test completed successfully.\n";
    return 0;
}
