#pragma once

// drivebrain_core
#include <Configurable.hpp>
#include <JsonFileHandler.hpp>
#include <MsgLogger.hpp>

#include "hytech_msgs.pb.h"

// protobuf
#include <google/protobuf/any.pb.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/message.h>

// boost
#include <boost/array.hpp>
#include <boost/asio.hpp>


// https://wiki.hytechracing.org/books/software/page/speedtech-lap-timing-protocol-description

namespace comms {
class SpeedTechComms : public core::common::Configurable {
    private:
        struct config {
            int baud_rate;
            std::string port_name;
        } _config;

    public:
        SpeedTechComms(
            core::JsonFileHandler &json_file_handler,
            boost::asio::io_context &io_context);
        
        bool init();
        
        void update_msg_logger(std::shared_ptr<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>> message_logger) {
            _message_logger = message_logger;
        }

        void process_buffer(const boost::array<std::uint8_t, 512> &buff, std::size_t length);

    private:
        void _start_recv();
    
    private:
        boost::array<std::uint8_t, 512> _input_buff;
        boost::asio::serial_port _serial;
        std::shared_ptr<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>> _message_logger = nullptr;
};
} // namespace comms