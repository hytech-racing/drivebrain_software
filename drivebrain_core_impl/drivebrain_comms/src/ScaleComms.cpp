#include "ScaleComms.hpp"

#include "JSONUtils.hpp"

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

ScaleComms::ScaleData ScaleComms::_parse_buffer(const boost::array<std::uint8_t, 512>& buffer, std::size_t bytes_count) {
    ScaleData data{};
    return data;
}

void ScaleComms::_log_proto_message(const ScaleData & data)
{
    
}

void ScaleComms::_start_receive() {
    _serial.async_read_some(
    boost::asio::buffer(_input_buff),
    [&](const boost::system::error_code &ec, std::size_t bytes_count) {
        // auto scale_data = _parse_buffer(_input_buff, bytes_count);
        std::string input_data;
        for (std::size_t i = 0; i < bytes_count; ++i) {
            if (std::isprint(_input_buff[i])) {
                input_data += static_cast<char>(_input_buff[i]);
            }
        }
        spdlog::debug("{}", input_data);
        // _log_proto_message(scale_data);
        _start_receive();
    });
}
} // namespace comms
