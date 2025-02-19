#pragma once

#include <Logger.hpp>
#include <MsgLogger.hpp>
#include <StateEstimator.hpp>
#include "hytech_msgs.pb.h"
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <memory>
#include <deque>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <optional>
#include <cstring>
#include <vector>
#include <map>
#include <unordered_map>
#include <google/protobuf/message.h>

using SerialPort = boost::asio::serial_port;
using loggertype = core::MsgLogger<std::shared_ptr<google::protobuf::Message>>;

namespace comms {

class SWDriver : public core::common::Configurable
{
public:
    SWDriver(core::JsonFileHandler &json_file_handler, core::Logger &logger, std::shared_ptr<loggertype> message_logger, core::StateEstimator &state_estimator, boost::asio::io_context &io_context);
    bool init();

    struct config {
        int baud_rate;
    };

    void log_proto_message(std::shared_ptr<google::protobuf::Message> msg);

private:
    core::Logger &_logger;
    core::StateEstimator &_state_estimator;
    boost::array<std::uint8_t, 512> _output_buff;
    boost::array<std::uint8_t, 512> _input_buff;
    SerialPort _serial;
    std::shared_ptr<loggertype> _message_logger;
    config _config;

    void _start_receive();
};

}
