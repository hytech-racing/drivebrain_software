#include <Configurable.hpp>

#include <JSONUtils.hpp>
#include <SpeedTechComms.hpp>
#include <cstddef>
#include <spdlog/spdlog.h>

namespace comms 
{
SpeedTechComms::SpeedTechComms(core::JsonFileHandler &json_file_handler,
                               boost::asio::io_context &io_context)
                                : core::common::Configurable(json_file_handler, "SpeedTechComms"),
                                _serial(io_context)
{
}

void SpeedTechComms::process_buffer(const boost::array<std::uint8_t, 512> &buff, std::size_t length)
{
    if(length == 17)
    {
        // modulo 256 checksum
        std::uint8_t checksum =0;
        for(size_t i = 2; i < 16; i++)
        {
            checksum += buff[i];
        }
        if (checksum != buff[16])
        {
            spdlog::warn("speedtech data seemingly corrupted, invalid checksum not logging");
            return;
        }

        auto lap_count = static_cast<std::size_t>(buff[10]);

        uint32_t lap_time_sec_int = ((buff[11] << 16) | (buff[12] << 8) | buff[13]);
        uint16_t lap_time_msec_int = ((buff[14] << 8) | buff[15]);
        float lap_time_seconds = static_cast<float>(lap_time_sec_int);
        float lap_time_millis = static_cast<float>(lap_time_msec_int);
        auto lap_time = lap_time_seconds + (lap_time_millis/1000.0f);

        std::shared_ptr<hytech_msgs::SpeedTechLapTime> speed_tech_lap_time_msg = std::make_shared<hytech_msgs::SpeedTechLapTime>();
        speed_tech_lap_time_msg->set_laptime(lap_time);
        speed_tech_lap_time_msg->set_lapcount(lap_count);

        spdlog::info("lapcount {} laptime {}", lap_count, lap_time);
        this->log(speed_tech_lap_time_msg);

    } else {
        spdlog::warn("speedtech message byte length incorrect");
    }
}

bool SpeedTechComms::init()
{
    LOAD_PARAM_OR_FAIL(baud_rate, int, _config);
    LOAD_PARAM_OR_FAIL(port_name, std::string, _config);

    boost::system::error_code ec;
    auto ec_ret = _serial.open(_config.port_name, ec);

    if (ec)
    {
        spdlog::warn("Error: {}", ec.message());
        spdlog::info("failed to open speedtech serial port");
        return false;
    }
    using SerialPort = boost::asio::serial_port;
    _serial.set_option(SerialPort::baud_rate(_config.baud_rate));
    _serial.set_option(SerialPort::character_size(8));
    _serial.set_option(SerialPort::parity(SerialPort::parity::none));
    _serial.set_option(SerialPort::stop_bits(SerialPort::stop_bits::one));
    _serial.set_option(SerialPort::flow_control(SerialPort::flow_control::none));
    
    set_configured();

    _start_recv();
    return true;
}

void SpeedTechComms::_start_recv()
{
    _serial.async_read_some(
        boost::asio::buffer(_input_buff),
        [&](const boost::system::error_code &ec, std::size_t bytesCount) -> void
        {
            if (ec)
            {
                if (ec != boost::asio::error::operation_aborted)
                {
                    spdlog::error("speedtech comms ERROR: {}", ec.message());
                }
                return;
            }
            // _logger.log_string("logging", core::LogLevel::INFO);
            process_buffer(_input_buff, bytesCount);
            // Initiate another asynchronous read
            _start_recv();
        }
    );
}
}