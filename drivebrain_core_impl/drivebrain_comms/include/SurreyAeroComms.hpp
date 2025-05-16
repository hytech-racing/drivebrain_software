#ifndef __SURREYAEROCOMMS_H__
#define __SURREYAEROCOMMS_H__


#include <Configurable.hpp>
#include <Loggable.hpp>
#include <MsgLogger.hpp>

#include <StateEstimator.hpp>

#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <cstring>
#include <ratio>
#include <vector>

#include <google/protobuf/message.h>



namespace comms {

    class SurreyAeroComms : public core::common::Loggable<std::shared_ptr<google::protobuf::Message>>, public core::common::Configurable {
        struct config {
            std::string port_name;
        } _config;
    public:
        SurreyAeroComms(core::JsonFileHandler &json_file_handler, boost::asio::io_context& io);
        bool init() override final;
    
    private:
        void _start_receive();
        void _configure_serial_port(boost::asio::serial_port& serial);
        bool _send_command(const std::string& command);
        std::optional<std::vector<float>> _extract_sensor_readings(const boost::array<std::uint8_t, 512>& buffer);
        void _log_proto_message(const std::vector<float>& readings);
    private:
        boost::asio::serial_port _serial;
        std::chrono::steady_clock::time_point _timestamp_of_last_debug_p;

        boost::array<std::uint8_t, 512> _input_buff{};
    };
}

#
#endif // __SURREYAEROCOMMS_H__