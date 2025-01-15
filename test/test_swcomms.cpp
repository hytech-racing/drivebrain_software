#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <unordered_map>
#include <iomanip>
#include "hytech_msgs.pb.h"

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

class TestSWDriver {
public:
    TestSWDriver(boost::asio::io_context& io)
        : _serial(io), _logger(), _state_estimator() {}

    bool init(const std::string& device_name, int baud_rate) {
        boost::system::error_code ec;
        _serial.open(device_name, ec);

        if (ec) {
            std::cerr << "Error opening serial port: " << ec.message() << std::endl;
            return false;
        }

        _serial.set_option(boost::asio::serial_port::baud_rate(baud_rate));
        _serial.set_option(boost::asio::serial_port::character_size(8));
        _serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        _serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        _serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));

        std::cout << "Serial port initialized successfully.\n";
        return true;
    }

    void start_receive() {
        _serial.async_read_some(
            boost::asio::buffer(_input_buff),
            [&](const boost::system::error_code& ec, std::size_t bytes_transferred) {
                if (ec) {
                    std::cerr << "Error reading data: " << ec.message() << std::endl;
                    return;
                }

                process_data(bytes_transferred);
                start_receive();
            });
    }

private:
    void process_data(std::size_t bytes_transferred) {
        std::string input_data;
        for (std::size_t i = 0; i < bytes_transferred; ++i) {
            if (std::isprint(_input_buff[i]) || std::isspace(_input_buff[i])) {
                input_data += static_cast<char>(_input_buff[i]);
            }
        }

        std::istringstream input_stream(input_data);
        std::string line;
        std::array<float, 13> parsed_data = {0};

        std::unordered_map<char, int> identifier_to_index = {
            {'0', 0}, {'1', 1}, {'2', 2}, {'3', 3},
            {'4', 4}, {'5', 5}, {'6', 6}, {'7', 7},
            {'8', 8}, {'9', 9}, {':', 10}, {';', 11}, {'<', 12}
        };

        int line_count = 0;
        while (std::getline(input_stream, line) && line_count < 13) {
            try {
                float value = std::stof(line.substr(2));
                parsed_data[identifier_to_index[line[0]]] = value;
            } catch (const std::invalid_argument&) {
                std::cerr << "Invalid data format: unable to parse line: " << line << std::endl;
                return;
            }
            line_count++;
        }

        std::shared_ptr<hytech_msgs::SWData> msg_out = std::make_shared<hytech_msgs::SWData>();
        msg_out->set_weight_lf(parsed_data[0]);
        msg_out->set_weight_rf(parsed_data[1]);
        msg_out->set_weight_lr(parsed_data[2]);
        msg_out->set_weight_rr(parsed_data[3]);
        msg_out->set_weight_left(parsed_data[4]);
        msg_out->set_weight_right(parsed_data[5]);
        msg_out->set_weight_front(parsed_data[6]);
        msg_out->set_weight_rear(parsed_data[7]);
        msg_out->set_weight_front_bite(parsed_data[8]);
        msg_out->set_weight_rear_bite(parsed_data[9]);
        msg_out->set_weight_cross(parsed_data[10]);
        msg_out->set_weight_total(parsed_data[11]);
        msg_out->set_weight_total_selected(parsed_data[12]);

        _state_estimator.handle_recv_process(msg_out);
        std::cout << "Protobuf message logged successfully.\n";
    }

    boost::asio::serial_port _serial;
    boost::array<char, 512> _input_buff;
    MockLogger _logger;
    MockStateEstimator _state_estimator;
};

int main() {
    boost::asio::io_context io;
    TestSWDriver driver(io);

    if (!driver.init("/dev/ttyUSB0", 115200)) {
        return 1;
    }

    driver.start_receive();

    io.run();

    return 0;
}
