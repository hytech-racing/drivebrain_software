#include "../include/AeroComms.hpp"
#include <boost/asio.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <iomanip>
#include <vector>
#include <array>
#include <cstring>
#include <cstdint>
#include <mutex>
#include <chrono>
#include "hytech_msgs.pb.h"

namespace comms {

    bool AeroDriver::init() {
        _logger.log_string("Opening Aero driver.", core::LogLevel::INFO);

        auto device_name1 = get_parameter_value<std::string>("device_name1");
        auto device_name2 = get_parameter_value<std::string>("device_name2");
        _config.baud_rate = get_parameter_value<int>("baud_rate").value();

        boost::system::error_code ec;
        _serial1.open(device_name1.value(), ec);
        _serial2.open(device_name2.value(), ec);

        if (ec) {
            std::cerr << "Error opening serial ports: " << ec.message() << std::endl;
            _logger.log_string("Failed to open Aero driver devices.", core::LogLevel::INFO);
            return false;
        }

        configure_serial_port(_serial1, _config.baud_rate);
        configure_serial_port(_serial2, _config.baud_rate);

        send_command(_serial1, "@D");
        send_command(_serial2, "@D");

        _logger.log_string("Aero driver initialized and in standby mode.", core::LogLevel::INFO);

        return true;
    }

    void AeroDriver::_start_receive(boost::asio::serial_port& serial_port) {
        serial_port.async_read_some(
            boost::asio::buffer(_input_buff),
            [&](const boost::system::error_code &ec, std::size_t bytesCount) {
                if (ec) {
                    if (ec != boost::asio::error::operation_aborted) {
                        std::cerr << "ERROR: " << ec.message() << std::endl;
                    }
                    standby_mode();
                    return;
                }

                if (bytesCount == 0) {
                    standby_mode();
                    return;
                }

                _logger.log_string("Data received, processing...", core::LogLevel::INFO);
                std::vector<float> sensor_readings = extract_sensor_readings(_input_buff);
                log_proto_message(sensor_readings);
                
                _start_receive(serial_port);
            });
    }

    void AeroDriver::standby_mode() {
        _logger.log_string("No input detected. Entering standby mode.", core::LogLevel::INFO);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        _logger.log_string("Monitoring for new input...", core::LogLevel::INFO);
        _start_receive(_serial1);
        _start_receive(_serial2);
    }

    std::vector<float> AeroDriver::extract_sensor_readings(const std::array<uint8_t, 47>& buffer) {
        std::vector<float> readings(8);
        if (buffer[0] != '#') {
            std::cerr << "Invalid frame received\n";
            return readings;
        }

        for (int i = 0; i < 8; ++i) {
            std::memcpy(&readings[i], &buffer[1 + i * 4], sizeof(float));
        }
        return readings;
    }

    void AeroDriver::log_proto_message(const std::vector<float>& readings) {
        auto msg_out = std::make_shared<hytech_msgs::AeroData>();
        for (float value : readings) {
            msg_out->add_readings_pa(value);
        }

        _state_estimator.handle_recv_process(msg_out);
    }
}
