#ifndef AEROCOMMS_HPP
#define AEROCOMMS_HPP

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

    class AeroDriver {
    public:
        AeroDriver(core::JsonFileHandler &json_file_handler, core::Logger &logger, std::shared_ptr<loggertype> message_logger, core::StateEstimator &state_estimator, boost::asio::io_context& io);
        void start_receive();
        bool init();

        
    
    private:
        void _start_receive(boost::asio::serial_port& serial_port);
        void standby_mode();
        void configure_serial_port(boost::asio::serial_port& serial);
        void send_command(boost::asio::serial_port& serial, const std::string& command);
        std::vector<float> extract_sensor_readings(const boost::array<char, 512>& buffer);
        void log_proto_message(const std::vector<float>& readings);
    
        core::Logger &_logger;
        core::StateEstimator &_state_estimator;
        std::shared_ptr<loggertype> _message_logger;
        boost::asio::serial_port _serial1;
        boost::asio::serial_port _serial2;
        boost::array<char, 512> _input_buff;
    };
}

#endif // AEROCOMMS_HPP