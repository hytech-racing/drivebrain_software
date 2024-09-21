#ifndef __ETHCOMMS_H__
#define __ETHCOMMS_H__

#include <Logger.hpp>
#include <StateEstimator.hpp>
#include <MsgLogger.hpp>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>

#include <google/protobuf/message.h>
#include "hytech_msgs.pb.h"
#include <memory>

// - [x] boost asio socket for udp port comms
// - [x] handle receiving UDP messages on a specific port
// - [x] handle parsing of UDP message as protobuf message on the port
// TODO:
// figure out if we want to keep the queue work flow for sending or if we want to
// instead use just a direct pointer / ref to a generic driver interface that we
// can give to the estimation / control thread to handle the sending of the control msgs

namespace comms
{
    class MCUETHComms
    {
    public:
        using deqtype = core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>>;
        using loggertype = core::MsgLogger<std::shared_ptr<google::protobuf::Message>>;
        MCUETHComms() = delete;
        MCUETHComms(core::Logger &logger,
                    deqtype &in_deq,
                    std::shared_ptr<loggertype> message_logger,
                    core::StateEstimator &state_estimator,
                    boost::asio::io_context &io_context,
                    const std::string &send_ip,
                    uint16_t recv_port,
                    uint16_t send_port);

    private:
        void _handle_send_msg_from_queue();
        void _send_message(std::shared_ptr<google::protobuf::Message> msg_out);
        void _handle_receive(const boost::system::error_code &error, std::size_t size);
        void _start_receive();
        void _handle_send(std::array<uint8_t, 2048> /*message*/,
                          const boost::system::error_code & /*error*/,
                          std::size_t /*bytes_transferred*/);

    private:
        core::Logger &_logger;
        std::shared_ptr<loggertype> _message_logger;
        core::StateEstimator &_state_estimator;
        std::array<uint8_t, 2048> _recv_buffer;
        std::array<uint8_t, 2048> _send_buffer;

        uint16_t _send_port;
        std::string _send_ip;
        boost::asio::ip::udp::socket _socket;
        boost::asio::ip::udp::endpoint _remote_endpoint;
        std::shared_ptr<hytech_msgs::MCUOutputData> _mcu_msg;
        deqtype &_input_deque_ref; // "input" = the messages that get input to the ethernet comms driver to send out
        std::thread _output_thread;
    };

}

#endif // __ETHCOMMS_H__