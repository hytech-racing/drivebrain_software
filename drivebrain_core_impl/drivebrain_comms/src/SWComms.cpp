#include "../include/SWComms.hpp"

#include <iostream>
#include <cstring>
#include <cerrno>
#include <cctype>

#include "hytech_msgs.pb.h"
#include "base_msgs.pb.h"

namespace comms {

    bool SWDriver::init()
    {
        _logger.log_string("Opening SW driver.", core::LogLevel::INFO);

        auto device_name = get_parameter_value<std::string>("device_name");
        _config.baud_rate = get_parameter_value<int>("baud_rate").value();
        auto port = get_parameter_value<int>("port");

        boost::system::error_code ec;
        _serial.open(device_name.value(), ec);

        if (ec)
        {
            std::cerr << "Error opening serial port: " << ec.message() << std::endl;
            _logger.log_string("Failed to open SW driver device.", core::LogLevel::INFO);
            return false;
        }

        _serial.set_option(SerialPort::baud_rate(_config.baud_rate));
        _serial.set_option(SerialPort::character_size(8));
        _serial.set_option(SerialPort::parity(SerialPort::parity::none));
        _serial.set_option(SerialPort::stop_bits(SerialPort::stop_bits::one));
        _serial.set_option(SerialPort::flow_control(SerialPort::flow_control::none));


        

        return true;
    }

    SWDriver::SWDriver(core::JsonFileHandler &json_file_handler, core::Logger &logger, std::shared_ptr<loggertype> message_logger, core::StateEstimator &state_estimator, boost::asio::io_context& io)
        : core::common::Configurable(logger, json_file_handler, "SWDriver"),
          _logger(logger),
          _state_estimator(state_estimator),
          _message_logger(message_logger),
          _serial(io)
    {
        init();

        _logger.log_string("Starting SW driver receive.", core::LogLevel::INFO);

        _start_receive();
    }

    void SWDriver::log_proto_message(std::shared_ptr<google::protobuf::Message> msg)
    {
        _state_estimator.handle_recv_process(static_cast<std::shared_ptr<google::protobuf::Message>>(msg));
        _message_logger->log_msg(static_cast<std::shared_ptr<google::protobuf::Message>>(msg));
    }


void SWDriver::_start_receive()
{
    _serial.async_read_some(
        boost::asio::buffer(_input_buff),
        [&](const boost::system::error_code &ec, std::size_t bytesCount)
        {
            if (ec)
            {
                if (ec != boost::asio::error::operation_aborted)
                {
                    std::cerr << "ERROR: " << ec.message() << std::endl;
                }
                return;
            }

            std::string input_data;
            for (std::size_t i = 0; i < bytesCount; ++i)
            {
                if (std::isprint(_input_buff[i]) || std::isspace(_input_buff[i]))
                {
                    input_data += static_cast<char>(_input_buff[i]);
                }
            }

            std::istringstream input_stream(input_data);
            std::string line;

            float weight_lf = 0.0f;
            float weight_lr = 0.0f;
            float weight_rf = 0.0f;
            float weight_rr = 0.0f;
            float weight_total = 0.0f;

            while (std::getline(input_stream, line))
            {
                try
                {
                    if (line.find("1:") == 0)
                    {
                        weight_lf = std::stof(line.substr(2));
                    }
                    else if (line.find("2:") == 0)
                    {
                        weight_lr = std::stof(line.substr(2));
                    }
                    else if (line.find("3:") == 0)
                    {
                        weight_rf = std::stof(line.substr(2));
                    }
                    else if (line.find("4:") == 0)
                    {
                        weight_rr = std::stof(line.substr(2));
                    }
                    else if (line.find("TOTAL") != std::string::npos)
                    {
                        size_t pos = line.find(" ");
                        if (pos != std::string::npos)
                        {
                            weight_total = std::stof(line.substr(0, pos));
                        }
                    }
                }
                catch (const std::invalid_argument &e)
                {
                    std::cerr << "Invalid data format: unable to parse float from line '" << line << "'" << std::endl;
                    return;
                }
            }

            std::shared_ptr<hytech_msgs::SWData> msg_out = std::make_shared<hytech_msgs::SWData>();
            msg_out->set_weight_lf(weight_lf);
            msg_out->set_weight_lr(weight_lr);
            msg_out->set_weight_rf(weight_rf);
            msg_out->set_weight_rr(weight_rr);
            msg_out->set_weight_total(weight_total);

            log_proto_message(static_cast<std::shared_ptr<google::protobuf::Message>>(msg_out));

            _start_receive();
        });
}


