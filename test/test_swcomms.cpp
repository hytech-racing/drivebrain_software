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

        float weight_lf = 0.0f;
        float weight_lr = 0.0f;
        float weight_rf = 0.0f;
        float weight_rr = 0.0f;
        float weight_total = 0.0f;

        while (std::getline(input_stream, line)) {
            try {
                if (line.find("1:") == 0) {
                    weight_lf = std::stof(line.substr(2));
                } else if (line.find("2:") == 0) {
                    weight_lr = std::stof(line.substr(2));
                } else if (line.find("3:") == 0) {
                    weight_rf = std::stof(line.substr(2));
                } else if (line.find("4:") == 0) {
                    weight_rr = std::stof(line.substr(2));
                } else if (line.find("TOTAL") != std::string::npos) {
                    size_t pos = line.find(" ");
                    if (pos != std::string::npos) {
                        weight_total = std::stof(line.substr(0, pos));
                    }
                }
            } catch (const std::invalid_argument&) {
                std::cerr << "Invalid data format: unable to parse line: " << line << std::endl;
                return;
            }
        }

        std::shared_ptr<hytech_msgs::SWData> msg_out = std::make_shared<hytech_msgs::SWData>();
        msg_out->set_weight_lf(weight_lf);
        msg_out->set_weight_lr(weight_lr);
        msg_out->set_weight_rf(weight_rf);
        msg_out->set_weight_rr(weight_rr);
        msg_out->set_weight_total(weight_total);

        _state_estimator.handle_recv_process(msg_out);
        std::cout << "Protobuf message logged successfully:\n"
                  << "  LF: " << weight_lf << "\n"
                  << "  LR: " << weight_lr << "\n"
                  << "  RF: " << weight_rf << "\n"
                  << "  RR: " << weight_rr << "\n"
                  << "  TOTAL: " << weight_total << "\n";
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
