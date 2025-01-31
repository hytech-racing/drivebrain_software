#include "../include/AeroComms.hpp"

#include <boost/asio.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <iomanip>
#include <regex>
#include <chrono>

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
        _serial2.open(device_name2, ec);

        if (ec) {
            std::cerr << "Error opening serial ports: " << ec.message() << std::endl;
            return false;
        }

        _serial1.set_option(boost::asio::serial_port::baud_rate(baud_rate));
        _serial1.set_option(boost::asio::serial_port::character_size(8));
        _serial1.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        _serial1.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        _serial1.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));

        _serial2.set_option(boost::asio::serial_port::baud_rate(baud_rate));
        _serial2.set_option(boost::asio::serial_port::character_size(8));
        _serial2.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        _serial2.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        _serial2.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));

        std::cout << "Serial ports initialized successfully.\n";
        return true;
    }

    void start_receive() {
        _serial1.async_read_some(
            boost::asio::buffer(_input_buff),
            [&](const boost::system::error_code& ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cerr << "Error reading data: " << ec.message() << std::endl;
                    return;
                }
                process_data(bytes_transferred, "Port 1");
                start_receive();
            });
        
        _serial2.async_read_some(
            boost::asio::buffer(_input_buff),
            [&](const boost::system::error_code& ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cerr << "Error reading data: " << ec.message() << std::endl;
                    return;
                }
                process_data(bytes_transferred, "Port 2");
                start_receive();
            });
    }

private:
    void process_data(std::size_t bytes_transferred, const std::string& port_name) {
        std::string input_data;
        for (std::size_t i = 0; i < bytes_transferred; ++i) {
            if (std::isprint(_input_buff[i])) {
                input_data += static_cast<char>(_input_buff[i]);
            }
        }

        std::regex number_regex(R"([-+]?\d*\.?\d+)");
        std::vector<float> readings;
        std::sregex_iterator it(input_data.begin(), input_data.end(), number_regex);
        std::sregex_iterator end;
        while (it != end) {
            readings.push_back(std::stof(it->str()));
            ++it;
        }

        auto msg_out = std::make_shared<hytech_msgs::AeroData>();
        for (float value : readings) {
            msg_out->add_readings_pa(value);
        }

        _state_estimator.handle_recv_process(msg_out);
        
        std::cout << "Protobuf message sent from " << port_name << " containing values: ";
        for (const auto& val : readings) {
            std::cout << val << " ";
        }
        std::cout << "\n";
    }

    boost::asio::serial_port _serial1;
    boost::asio::serial_port _serial2;
    boost::array<char, 512> _input_buff;
    MockLogger _logger;
    MockStateEstimator _state_estimator;
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
