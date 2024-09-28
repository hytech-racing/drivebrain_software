#pragma once

#include <Configurable.hpp>
#include <Logger.hpp>

// protobuf
#include <google/protobuf/any.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/dynamic_message.h>

// boost
#include <boost/asio.hpp>

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

#include "hytech_msgs.pb.h"

using SerialPort = boost::asio::serial_port;

using namespace vn::xplat;
using namespace vn::protocol::uart;

namespace comms {
    class VNDriver : public core::common::Configurable
    {
        public:
            VNDriver(core::JsonFileHandler &json_file_handler, core::Logger &logger, std::shared_ptr<loggertype> message_logger, core::StateEstimator &state_estimator)

        private: 
            // Private variables
            core::Logger& _logger;
            core::StateEstimator &_state_estimator;


            vn::protocol::uart::PacketFinder _processor;
            boost::asio::io_service _io;
            boost::array<std::uint8_t, 512> _output_buff;
            boost::array<std::uint8_t, 512> _input_buff;
            SerialPort _serial;
            std::shared_ptr<loggertype> _message_logger
            
            // Private methods
            void _handle_recieve(void *userData, vn::protocol::uart::Packet &packet, size_t runningIndexOfPacketStart, TimeStamp ts);
            void _configure_binary_outputs();
            void _start_recieve();
            void _set_baud_rate(int rate, int port);

    }
}