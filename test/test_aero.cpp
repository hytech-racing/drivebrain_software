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

class MockLogger {
public:
    void log_string(const std::string& message, int level) {
        std::cout << "[LOG]: " << message << " [Level: " << level << "]" << std::endl;
    }
};

class MockStateEstimator {
public:
    void handle_recv_process(std::shared_ptr<google::protobuf::Message> msg) {
        std::cout << "[STATE ESTIMATOR]: Received Protobuf Message" << std::endl;
    }
};

class TestAeroDriver {
public:
    TestAeroDriver(boost::asio::io_context& io)
        : _serial1(io), _serial2(io), _logger(), _state_estimator() {}

    bool init(const std::string& device_name1, const std::string& device_name2, int baud_rate) {
        boost::system::error_code ec;

        _serial1.open(device_name1, ec);
        if (ec) {
            std::cerr << "Error opening " << device_name1 << ": " << ec.message() << std::endl;
            return false;
        }

        _serial2.open(device_name2, ec);
        if (ec) {
            std::cerr << "Error opening " << device_name2 << ": " << ec.message() << std::endl;
            return false;
        }

        configure_serial_port(_serial1, baud_rate);
        configure_serial_port(_serial2, baud_rate);

        std::cout << "Serial ports initialized successfully.\n";

        send_command(_serial1, "@D");
        send_command(_serial2, "@D");

        return true;
    }

    void start_receive() {
        start_receive_port1();
        start_receive_port2();
    }

private:
    boost::asio::serial_port _serial1;
    boost::asio::serial_port _serial2;
    std::array<uint8_t, 47> _input_buff1;
    std::array<uint8_t, 47> _input_buff2;
    std::vector<float> pressures_port1;
    std::vector<float> pressures_port2;
    std::mutex output_mutex;
    MockLogger _logger;
    MockStateEstimator _state_estimator;

    void configure_serial_port(boost::asio::serial_port& serial, int baud_rate) {
        serial.set_option(boost::asio::serial_port::baud_rate(baud_rate));
        serial.set_option(boost::asio::serial_port::character_size(8));
        serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    }

    void send_command(boost::asio::serial_port& serial, const std::string& command) {
        boost::system::error_code ec;
        boost::asio::write(serial, boost::asio::buffer(command), ec);

        if (ec) {
            std::cerr << "Error sending command '" << command << "': " << ec.message() << std::endl;
        } else {
            std::cout << "Successfully sent command: " << command << std::endl;
        }
    }

    void start_receive_port1() {
        _serial1.async_read_some(
            boost::asio::buffer(_input_buff1),
            [this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
                if (!ec && bytes_transferred >= 47) {
                    pressures_port1 = extract_pressures(_input_buff1);
                    print_combined_output();
                } else {
                    std::cerr << "Error reading from Port 1: " << ec.message() << " (Bytes received: " << bytes_transferred << ")\n";
                }
                this->start_receive_port1();
            });
    }

    void start_receive_port2() {
        _serial2.async_read_some(
            boost::asio::buffer(_input_buff2),
            [this](const boost::system::error_code& ec, std::size_t bytes_transferred) {
                if (!ec && bytes_transferred >= 47) {
                    pressures_port2 = extract_pressures(_input_buff2);
                    print_combined_output();
                } else {
                    std::cerr << "Error reading from Port 2: " << ec.message() << " (Bytes received: " << bytes_transferred << ")\n";
                }
                this->start_receive_port2();
            });
    }

    std::vector<float> extract_pressures(const std::array<uint8_t, 47>& buffer) {
        std::vector<float> pressures(8);
        if (buffer[0] != '#') {
            std::cerr << "Invalid frame received\n";
            return pressures;
        }

        for (int i = 0; i < 8; ++i) {
            std::memcpy(&pressures[i], &buffer[1 + i * 4], sizeof(float));
        }
        return pressures;
    }

    void print_combined_output() {
        std::lock_guard<std::mutex> lock(output_mutex);
        if (pressures_port1.empty() || pressures_port2.empty()) return;

        std::cout << "Pressure Readings | Port 1: ";
        for (const auto& p : pressures_port1) {
            std::cout << std::fixed << std::setprecision(2) << p << "  ";
        }

        std::cout << "| Port 2: ";
        for (const auto& p : pressures_port2) {
            std::cout << std::fixed << std::setprecision(2) << p << "  ";
        }

        std::cout << std::endl;
    }
};

int main() {
    boost::asio::io_context io;
    TestAeroDriver driver(io);

    if (!driver.init("/dev/ttyACM0", "/dev/ttyACM1", 500000)) {
        return 1;
    }

    driver.start_receive();
    io.run();

    return 0;
}
