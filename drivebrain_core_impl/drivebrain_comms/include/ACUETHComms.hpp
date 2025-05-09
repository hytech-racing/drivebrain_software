#ifndef __ACUCOMMS_H__
#define __ACUCOMMS_H__

#include <Logger.hpp>
#include <StateEstimator.hpp>
#include <MsgLogger.hpp>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>

#include <cstdint>
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
    struct ETHCommPorts
    {
        uint16_t acu_port;
        uint16_t vcr_port;
        uint16_t vcf_port;
    };
    class ACUETHComms
    {
    public:
        
        using loggertype = core::MsgLogger<std::shared_ptr<google::protobuf::Message>>;
        ACUETHComms() = delete;
        ~ACUETHComms();
        ACUETHComms(core::Logger &logger,
                    std::shared_ptr<loggertype> message_logger,
                    boost::asio::io_context &io_context,
                    ETHCommPorts ports);
        void update_msg_logger(std::shared_ptr<loggertype> message_logger) {
            _message_logger = message_logger;
        }
        
    private:
        void _handle_receive(const boost::system::error_code &error, std::size_t size);
        void _start_receive();
        void _handle_send(std::array<uint8_t, 2048> /*message*/,
                          const boost::system::error_code & /*error*/,
                          std::size_t /*bytes_transferred*/);
    private:
        core::Logger &_logger;
        std::shared_ptr<loggertype> _message_logger;
        std::array<uint8_t, 2048> _recv_buffer;
        boost::asio::ip::udp::socket _socket;
        boost::asio::ip::udp::endpoint _remote_endpoint;
        std::shared_ptr<hytech_msgs::ACUAllData> _acu_msg;
        bool _running = false;
        std::thread _output_thread;
    };

}

#endif // __ACUCOMMS_H__