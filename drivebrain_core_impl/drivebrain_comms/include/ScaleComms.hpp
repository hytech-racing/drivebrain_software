#pragma once

#include <Configurable.hpp>
#include <JsonFileHandler.hpp>
#include <Loggable.hpp>
#include <MsgLogger.hpp>
#include <Utils.hpp>

#include <boost/asio.hpp>
#include <boost/array.hpp>

#include <google/protobuf/message.h>

namespace comms {
class ScaleComms : public core::common::Loggable<std::shared_ptr<google::protobuf::Message>>,
                   public core::common::Configurable {
    struct config {
        int baud_rate;
        std::string port_name;
    } _config;
    
    struct ScaleData {
        veh_vec<float> corner_weights_lbs;
    };
  public:
    
    ScaleComms(core::JsonFileHandler &json_file_handler, boost::asio::io_context &io);
    bool init() override final;

  private:
    void _configure_serial_port(boost::asio::serial_port &serial);
    private:
        void _start_receive();
        std::optional<ScaleData> _parse_buffer(const boost::array<std::uint8_t, 512>& buffer, std::size_t bytes_count);
        void _log_proto_message(const ScaleData & data);
    private:
        boost::asio::serial_port _serial;
        boost::array<std::uint8_t, 512> _input_buff{};
};
} // namespace comms
