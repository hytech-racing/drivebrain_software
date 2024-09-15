#ifndef __ETHCOMMS_H__
#define __ETHCOMMS_H__

#include <Logger.hpp>
#include <StateEstimator.hpp>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>

#include <google/protobuf/message.h>

#include <memory>

// - [ ] boost asio socket for udp port comms
// - [ ] handle receiving UDP messages on a specific port
// - [ ] handle parsing of UDP message as protobuf message on the port
namespace comms
{
    class MCUETHComms
    {
        public:
            MCUETHComms() = delete;
            MCUETHComms(core::Logger& logger, core::StateEstimator &state_estimator, boost::asio::io_context& io_context, uint16_t recv_port, uint16_t send_port);
            void send_message(std::shared_ptr<google::protobuf::Message> msg_out);
        private:
            void _handle_receive(const boost::system::error_code &error, std::size_t size);
            void _start_receive();
        private:
            core::Logger &_logger;
            core::StateEstimator &_state_estimator;
            std::array<uint8_t, 2048> _recv_buffer;
            uint16_t _send_port;
            boost::asio::ip::udp::socket _socket;
            boost::asio::ip::udp::endpoint _remote_endpoint;

    };

}


#endif // __ETHCOMMS_H__