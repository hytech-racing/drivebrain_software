#include "SWComms.hpp"

#include <iostream>
#include <cstring>
#include <cerrno>
#include <cctype>
#include <regex>
#include <chrono>

#include "hytech_msgs.pb.h"
#include "base_msgs.pb.h"

namespace comms {

    SWDriver::SWDriver(core::JsonFileHandler &json_file_handler,
                       core::Logger &logger,
                       std::shared_ptr<loggertype> message_logger,
                       core::StateEstimator &state_estimator,
                       boost::asio::io_context& io)
        : core::common::Configurable(logger, json_file_handler, "SWDriver"),
          _logger(logger),
          _state_estimator(state_estimator),
          _message_logger(message_logger),
          _serial(io),
          _retry_timer(io),
          _active_connection(false)
    {
        standby_mode();
    }

    void SWDriver::standby_mode() {
        _logger.log_string("Entering SW standby mode. Waiting for device...", core::LogLevel::INFO);
        attempt_connection();
    }

    void SWDriver::attempt_connection() {
        boost::system::error_code ec;

        if (_serial.is_open()) {
            _serial.close(ec);
        }

        auto device_name = get_parameter_value<std::string>("device_name");
        _serial.open(device_name.value(), ec);

        if (!ec) {
            _logger.log_string("SW device connected. Configuring serial port...", core::LogLevel::INFO);

            _config.baud_rate = get_parameter_value<int>("baud_rate").value();

            _serial.set_option(SerialPort::baud_rate(_config.baud_rate));
            _serial.set_option(SerialPort::character_size(8));
            _serial.set_option(SerialPort::parity(SerialPort::parity::none));
            _serial.set_option(SerialPort::stop_bits(SerialPort::stop_bits::one));
            _serial.set_option(SerialPort::flow_control(SerialPort::flow_control::none));

            _active_connection = false;
            _start_receive();
        } else {
            std::ostringstream oss;
            oss << "Waiting for SW device... (" << ec.message() << ")";
            _logger.log_string(oss.str(), core::LogLevel::INFO);

            _retry_timer.expires_after(std::chrono::seconds(2));
            _retry_timer.async_wait([this](const boost::system::error_code& timer_ec) {
                if (!timer_ec) {
                    attempt_connection();
                }
            });
        }
    }

    void SWDriver::_start_receive() {
        _serial.async_read_some(
            boost::asio::buffer(_input_buff),
            [&](const boost::system::error_code &ec, std::size_t bytesCount) {
                if (ec || bytesCount == 0) {
                    if (ec != boost::asio::error::operation_aborted) {
                        std::cerr << "SWDriver ERROR: " << ec.message() << std::endl;
                    }
                    _active_connection = false;
                    standby_mode();
                    return;
                }

                if (!_active_connection) {
                    _logger.log_string("SW data stream started.", core::LogLevel::INFO);
                    _active_connection = true;
                }

                std::string input_data;
                for (std::size_t i = 0; i < bytesCount; ++i) {
                    if (std::isprint(_input_buff[i])) {
                        input_data += static_cast<char>(_input_buff[i]);
                    }
                }

                std::istringstream input_stream(input_data);
                std::string line;
                float weight_lf = 0.0f, weight_lr = 0.0f, weight_rf = 0.0f, weight_rr = 0.0f;

                std::regex number_regex(R"([-+]?\d*\.?\d+)");
                while (std::getline(input_stream, line)) {
                    std::vector<float> weights;
                    std::sregex_iterator it(line.begin(), line.end(), number_regex);
                    std::sregex_iterator end;

                    while (it != end) {
                        weights.push_back(std::stof(it->str()));
                        ++it;
                    }

                    if (weights.size() >= 8) {
                        weight_lf = weights[1];
                        weight_lr = weights[3];
                        weight_rf = weights[5];
                        weight_rr = weights[7];
                    }
                }

                auto msg_out = std::make_shared<hytech_msgs::WeighScaleData>();
                msg_out->set_weight_lf(weight_lf);
                msg_out->set_weight_lr(weight_lr);
                msg_out->set_weight_rf(weight_rf);
                msg_out->set_weight_rr(weight_rr);

                log_proto_message(msg_out);

                _start_receive();  
            });
    }

    void SWDriver::log_proto_message(std::shared_ptr<google::protobuf::Message> msg) {
        _state_estimator.handle_recv_process(msg);
        _message_logger->log_msg(msg);
    }

    bool SWDriver::init() {
        return true;
    }

}
