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
    comms::AeroDriver driver;

    if (!driver.init()) {
        return 1;
    }

    driver.start_receive();
    io.run();

    return 0;
}