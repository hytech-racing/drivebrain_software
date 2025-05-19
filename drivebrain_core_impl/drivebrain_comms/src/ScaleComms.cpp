#include "ScaleComms.hpp"


#include "JSONUtils.hpp"
#include <hytech_msgs.pb.h>

#include <regex>

namespace comms {
ScaleComms::ScaleComms(core::JsonFileHandler &json_file_handler, boost::asio::io_context &io)
    : Configurable(json_file_handler, "ScaleComms"), _serial(io) {}

bool ScaleComms::init()
{
    LOAD_PARAM_OR_FAIL(baud_rate, int, _config);
    LOAD_PARAM_OR_FAIL(port_name, std::string, _config);

    boost::system::error_code ec;
    (void)_serial.open(_config.port_name, ec);
    if (ec) {
        spdlog::error("Error opening serial port for ScaleComms: {}", ec.message());
        return false;
    }

    _configure_serial_port(_serial);

    set_configured();
    _start_receive();
    return true;
}

void ScaleComms::_configure_serial_port(boost::asio::serial_port& serial) {
    serial.set_option(boost::asio::serial_port::baud_rate(static_cast<unsigned int>(_config.baud_rate)));
    serial.set_option(boost::asio::serial_port::character_size(8));
    serial.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    serial.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    serial.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
}

std::optional<ScaleComms::ScaleData> ScaleComms::_parse_buffer(const boost::array<std::uint8_t, 512>& buffer, std::size_t bytes_count) {
    
    std::string input_data;
    for (std::size_t i = 0; i < bytes_count; ++i) {
        
        if (std::isprint(_input_buff[i])) {
            input_data += static_cast<char>(_input_buff[i]);
        }
    }
    std::regex pattern(R"(1:\s*([+-]?[0-9]*[.]?[0-9]+)\s*2:\s*([+-]?[0-9]*[.]?[0-9]+)\s*3:\s*([+-]?[0-9]*[.]?[0-9]+)\s*4:\s*([+-]?[0-9]*[.]?[0-9]+))");

    std::smatch match;

    if (std::regex_search(input_data, match, pattern)) {
        ScaleData data = {};
        data.corner_weights_lbs.FL = std::stod(match[1]);
        data.corner_weights_lbs.FR = std::stod(match[2]);
        data.corner_weights_lbs.RL = std::stod(match[3]);
        data.corner_weights_lbs.RR = std::stod(match[4]);
        spdlog::info("scale data fl {} fr {} rl {} rr {}", data.corner_weights_lbs.FL, data.corner_weights_lbs.FR, data.corner_weights_lbs.RL, data.corner_weights_lbs.RR);
        return data;
    } else {
        spdlog::info("erm, no match for:{}", input_data);
    }
    return std::nullopt;
}

void ScaleComms::_log_proto_message(const ScaleData & data)
{
    auto msg_out = std::make_shared<hytech_msgs::WeighScaleData>();
    msg_out->set_weight_lf(data.corner_weights_lbs.FL);
    msg_out->set_weight_lr(data.corner_weights_lbs.FR);
    msg_out->set_weight_rf(data.corner_weights_lbs.RL);
    msg_out->set_weight_rr(data.corner_weights_lbs.RR);
    this->log(msg_out);
}

void ScaleComms::_start_receive() {
    _serial.async_read_some(
    boost::asio::buffer(_input_buff),
    [&](const boost::system::error_code &ec, std::size_t bytes_count) {
        auto scale_data = _parse_buffer(_input_buff, bytes_count);
        if(scale_data)
        {
            _log_proto_message(*scale_data);
        }
        _start_receive();
    });
}
} // namespace comms
