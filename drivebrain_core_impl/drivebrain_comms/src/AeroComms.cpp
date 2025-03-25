#include "AeroComms.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <cstring>

namespace comms {

    AeroDriver::AeroDriver(core::JsonFileHandler &json_file_handler,
                           core::Logger &logger,
                           std::shared_ptr<loggertype> message_logger,
                           core::StateEstimator &state_estimator,
                           boost::asio::io_context& io)
        : _logger(logger), _state_estimator(state_estimator),
          _message_logger(message_logger), _serial1(io), _active_connection(false) {}

    bool AeroDriver::init() {
        _logger.log_string("Opening Aero driver.", core::LogLevel::INFO);

        boost::system::error_code ec;
        _serial1.open("/dev/ttyACM0", ec);
        if (ec) {
            std::cerr << "Error opening serial port: " << ec.message() << std::endl;
            return false;
        }

        configure_serial_port(_serial1);
        send_command(_serial1, "@D");

        _logger.log_string("Aero driver initialized and in standby mode.", core::LogLevel::INFO);
        return true;
    }

    void AeroDriver::start_receive() {
        _start_receive(_serial1);
    }

    void AeroDriver::_start_receive(boost::asio::serial_port& serial_port) {
        serial_port.async_read_some(
            boost::asio::buffer(_input_buff),
            [this, &serial_port](const boost::system::error_code &ec, std::size_t bytesCount) {
                if (ec) {
                    if (ec != boost::asio::error::operation_aborted) {
                        std::cerr << "ERROR: " << ec.message() << std::endl;
                    }
                    _active_connection = false;
                    standby_mode();
                    return;
                }

                if (bytesCount == 0) {
                    _active_connection = false;
                    standby_mode();
                    return;
                }

                if (!_active_connection) {
                    _logger.log_string("Device connected. Receiving data stream...", core::LogLevel::INFO);
                    _active_connection = true;
                }

                std::vector<float> sensor_readings = extract_sensor_readings(_input_buff);

                std::ostringstream oss;
                oss << "Received sensor readings: ";
                for (float val : sensor_readings) {
                    oss << std::fixed << std::setprecision(2) << val << " ";
                }
                _logger.log_string(oss.str(), core::LogLevel::DEBUG);

                log_proto_message(sensor_readings);
                _start_receive(serial_port);
            });
    }

    void AeroDriver::standby_mode() {
        _logger.log_string("No input detected. Entering standby mode...", core::LogLevel::INFO);

        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(2));

            boost::system::error_code ec;
            if (_serial1.is_open()) {
                _serial1.close(ec);
            }

            _serial1.open("/dev/ttyACM0", ec);
            if (!ec) {
                _logger.log_string("Device reconnected. Restarting data stream.", core::LogLevel::INFO);
                configure_serial_port(_serial1);
                send_command(_serial1, "@D");
                _active_connection = false;
                start_receive();
                return;
            }
        }
    }

    std::vector<float> AeroDriver::extract_sensor_readings(const boost::array<char, 512>& buffer) {
        std::vector<float> readings;
        if (buffer[0] != '#') {
            std::cerr << "Invalid frame received\n";
            return readings;
        }

        for (size_t i = 0; i < 8; ++i) {
            float value;
            std::memcpy(&value, &buffer[1 + i * 4], sizeof(float));
            readings.push_back(value);
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

    void AeroDriver::configure_serial_port(boost::asio::serial_port& serial) {
        serial.set_option(boost::asio::serial_port::baud_rate(500000));
        serial.set_option(boost::asio::serial_port::character_size(8));
        serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    }

    void AeroDriver::send_command(boost::asio::serial_port& serial, const std::string& command) {
        boost::system::error_code ec;
        boost::asio::write(serial, boost::asio::buffer(command), ec);

        if (ec) {
            std::cerr << "Error sending command '" << command << "': " << ec.message() << std::endl;
        }
    }
}

int main() {
    boost::asio::io_context io;
    core::JsonFileHandler json_handler("/path/to/config.json");
    core::Logger logger(core::LogLevel::INFO);
    auto message_logger = std::make_shared<loggertype>(
        "aero_log",
        true,
        [](std::shared_ptr<google::protobuf::Message> msg) {
            std::cout << "Logging message." << std::endl;
        },
        []() {
            std::cout << "Flushing logs." << std::endl;
        },
        [](const std::string &error) {
            std::cerr << "Logger error: " << error << std::endl;
        },
        [](std::shared_ptr<google::protobuf::Message> status) {
            std::cout << "Status update received." << std::endl;
        }
    );

    bool construction_failed = false;
    estimation::Tire_Model_Codegen_MatlabModel matlab_estimator(logger, json_handler, construction_failed);
    if (construction_failed) {
        std::cerr << "Matlab Model Construction Failed." << std::endl;
        return 1;
    }

    core::StateEstimator state_estimator(logger, message_logger, matlab_estimator);
    comms::AeroDriver driver(json_handler, logger, message_logger, state_estimator, io);

    if (!driver.init()) {
        return 1;
    }

    driver.start_receive();
    io.run();
    return 0;
}
