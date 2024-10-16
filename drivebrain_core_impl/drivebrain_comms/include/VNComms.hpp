#pragma once

#include <Configurable.hpp>
#include <Logger.hpp>
#include <MsgLogger.hpp>
#include <StateEstimator.hpp>

// protobuf
#include <google/protobuf/any.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/dynamic_message.h>
#include "hytech_msgs.pb.h"

// boost
#include <boost/asio.hpp>
#include <boost/array.hpp>

// c++ stl includes
#include <memory>
#include <deque>
#include <variant>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <functional>
#include <optional>

#include <unistd.h>
#include <cstring>

#include "libvncxx/vntime.h"
#include "libvncxx/packetfinder.h"
#include "libvncxx/packet.h"

using namespace vn::xplat;
using namespace vn::protocol::uart;
using SerialPort = boost::asio::serial_port;
using loggertype = core::MsgLogger<std::shared_ptr<google::protobuf::Message>>;

namespace comms {
    class VNDriver : public core::common::Configurable
    {
        public:
            VNDriver(core::JsonFileHandler &json_file_handler, core::Logger &logger, std::shared_ptr<loggertype> message_logger, ::core::StateEstimator &state_estimator, boost::asio::io_context &io_context);
            bool init();
            struct config {
                int baud_rate;
                int freq_divisor;
            };

        private: 
            // Private variables
            core::Logger& _logger;
            core::StateEstimator &_state_estimator;


            vn::protocol::uart::PacketFinder _processor;
            boost::array<std::uint8_t, 512> _output_buff;
            boost::array<std::uint8_t, 512> _input_buff;
            SerialPort _serial;
            std::shared_ptr<loggertype> _message_logger; 
            config _config;    

        public: 
            // Public methods
            void log_proto_message(std::shared_ptr<google::protobuf::Message> msg);  
        
        private:
            // Private methods
            static void _handle_recieve(void *userData, vn::protocol::uart::Packet &packet, size_t runningIndexOfPacketStart, TimeStamp ts);
            void _configure_binary_outputs();
            void _start_recieve();

    };
}