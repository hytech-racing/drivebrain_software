#include "SurreyAeroComms.hpp"

#include <Configurable.hpp>
#include <spdlog/spdlog.h>


#include "JSONUtils.hpp"

#include "hytech_msgs.pb.h"

namespace comms {

SurreyAeroComms::SurreyAeroComms(core::JsonFileHandler &json_file_handler,
                                 boost::asio::io_context& io)
    : Configurable(json_file_handler, "SurreyAeroComms"),
        _serial(io) {}

void SurreyAeroComms::_configure_serial_port(boost::asio::serial_port& serial) {
    serial.set_option(boost::asio::serial_port::baud_rate(500000));
    serial.set_option(boost::asio::serial_port::character_size(8));
    serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
}

bool SurreyAeroComms::init() {

    LOAD_PARAM_OR_FAIL(port_name, std::string, _config);
    boost::system::error_code ec;
    (void)_serial.open(_config.port_name, ec);
    if (ec) {
        spdlog::error("Error opening serial port: {}", ec.message());
        return false;
    }

    _configure_serial_port(_serial);
    if(!_send_command("@D"))
    {
        return false;
    }

    spdlog::info("Aero driver initialized, starting receive");
    set_configured();

    _start_receive();
    return true;
}


void SurreyAeroComms::_start_receive() {
    _serial.async_read_some(
    boost::asio::buffer(_input_buff),
    [&](const boost::system::error_code &ec, std::size_t bytesCount) {
        std::vector<float> sensor_readings = _extract_sensor_readings(_input_buff);
        
        _log_proto_message(sensor_readings);
        _start_receive();
    });
}

std::vector<float> SurreyAeroComms::_extract_sensor_readings(const boost::array<std::uint8_t, 512>& buffer) {
    std::vector<float> readings;
    if (buffer[0] != '#') {
        spdlog::error("invalid aero sensor frame received");
        return readings;
    }


    auto now_time = std::chrono::steady_clock::now();

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - _timestamp_of_last_debug_p);

    for (size_t i = 0; i < 8; ++i) {
        float value;
        std::memcpy(&value, &buffer[1 + i * 4], sizeof(float));
        
        readings.push_back(value);
    }
    
    if(elapsed.count() > 100)
    {
        spdlog::debug("readings:");
        for(auto read : readings)
        {
            spdlog::debug("{}", read);
        }
        _timestamp_of_last_debug_p = now_time;
    }
    
    
    return readings;
}

void SurreyAeroComms::_log_proto_message(const std::vector<float>& readings) {
    auto msg_out = std::make_shared<hytech_msgs::AeroData>();
    for (float value : readings) {
        msg_out->add_readings_pa(value);
    }
    log(msg_out);
}

    

bool SurreyAeroComms::_send_command(const std::string& command) {
    boost::system::error_code ec;
    boost::asio::write(_serial, boost::asio::buffer(command), ec);

    if (ec) {
        spdlog::error("Error sending aero surrey sensor command '{}': with error {}", command, ec.message());
        return false;
    }
    return true;
}
}
