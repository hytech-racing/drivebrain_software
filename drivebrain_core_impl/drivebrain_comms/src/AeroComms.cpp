#include "../include/AeroComms.hpp"

#include <iostream>
#include <cstring>
#include <cerrno>
#include <cctype>
#include <regex>
#include <chrono>
#include <vector>

#include "hytech_msgs.pb.h"
#include "base_msgs.pb.h"

namespace comms {

    bool AeroDriver::init()
    {
        _logger.log_string("Opening Aero driver.", core::LogLevel::INFO);

        auto device_name1 = get_parameter_value<std::string>("device_name1");
        auto device_name2 = get_parameter_value<std::string>("device_name2");
        _config.baud_rate = get_parameter_value<int>("baud_rate").value();

        boost::system::error_code ec;
        _serial1.open(device_name1.value(), ec);
        _serial2.open(device_name2.value(), ec);

        if (ec)
        {
            std::cerr << "Error opening serial ports: " << ec.message() << std::endl;
            _logger.log_string("Failed to open Aero driver devices.", core::LogLevel::INFO);
            return false;
        }

        _serial1.set_option(SerialPort::baud_rate(_config.baud_rate));
        _serial1.set_option(SerialPort::character_size(8));
        _serial1.set_option(SerialPort::parity(SerialPort::parity::none));
        _serial1.set_option(SerialPort::stop_bits(SerialPort::stop_bits::one));
        _serial1.set_option(SerialPort::flow_control(SerialPort::flow_control::none));

        _serial2.set_option(SerialPort::baud_rate(_config.baud_rate));
        _serial2.set_option(SerialPort::character_size(8));
        _serial2.set_option(SerialPort::parity(SerialPort::parity::none));
        _serial2.set_option(SerialPort::stop_bits(SerialPort::stop_bits::one));
        _serial2.set_option(SerialPort::flow_control(SerialPort::flow_control::none));

        return true;
    }

    AeroDriver::AeroDriver(core::JsonFileHandler &json_file_handler, core::Logger &logger, std::shared_ptr<loggertype> message_logger, core::StateEstimator &state_estimator, boost::asio::io_context& io)
        : core::common::Configurable(logger, json_file_handler, "AeroDriver"),
          _logger(logger),
          _state_estimator(state_estimator),
          _message_logger(message_logger),
          _serial1(io),
          _serial2(io)
    {
        init();
        _logger.log_string("Starting Aero driver receive.", core::LogLevel::INFO);
        _start_receive(_serial1);
        _start_receive(_serial2);
    }

    void AeroDriver::log_proto_message(std::shared_ptr<google::protobuf::Message> msg)
    {
        _state_estimator.handle_recv_process(msg);
        _message_logger->log_msg(msg);
    }

    void AeroDriver::_start_receive(boost::asio::serial_port& serial_port)
    {
        serial_port.async_read_some(
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

                static auto last_time = std::chrono::steady_clock::now();
                auto current_time = std::chrono::steady_clock::now();
                double time_diff = std::chrono::duration<double>(current_time - last_time).count();
                last_time = current_time;

                if (time_diff > 0)
                {
                    std::cout << "Message Rate: " << 1.0 / time_diff << " Hz\n";
                }

                std::string input_data;
                for (std::size_t i = 0; i < bytesCount; ++i)
                {
                    if (std::isprint(_input_buff[i]))
                    {
                        input_data += static_cast<char>(_input_buff[i]);
                    }
                }

                std::istringstream input_stream(input_data);
                std::string line;
                std::vector<float> sensor_readings;

                std::regex number_regex(R"([-+]?\d*\.?\d+)");
                while (std::getline(input_stream, line))
                {
                    std::sregex_iterator it(line.begin(), line.end(), number_regex);
                    std::sregex_iterator end;

                    while (it != end)
                    {
                        sensor_readings.push_back(std::stof(it->str()));
                        ++it;
                    }
                }

                auto msg_out = std::make_shared<hytech_msgs::AeroData>();
                for (float value : sensor_readings)
                {
                    msg_out->add_readings_pa(value);
                }

                log_proto_message(msg_out);

                _start_receive(serial_port);
            });
    }
}
