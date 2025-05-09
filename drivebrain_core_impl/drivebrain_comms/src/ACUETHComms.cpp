#include <ACUETHComms.hpp>
#include "hytech_msgs.pb.h"
#include <spdlog/spdlog.h>


using boost::asio::ip::udp;
namespace comms
{
    
    ACUETHComms::ACUETHComms(core::Logger &logger,
                             std::shared_ptr<loggertype> message_logger,
                             boost::asio::io_context &io_context,
                             ETHCommPorts ports) : _logger(logger),
                                                   _message_logger(message_logger),
                                                   _socket(io_context, udp::endpoint(udp::v4(), ports.acu_port))
    {
        _acu_msg = std::make_shared<hytech_msgs::ACUAllData>();
        _start_receive();
    }

    ACUETHComms::~ACUETHComms()
    {
        _running = false;
        spdlog::warn("Destructed ACU ETH COMMS");
    }
    
    void ACUETHComms::_handle_receive(const boost::system::error_code &error, std::size_t size)
    {

        if (!error)
        {
            _acu_msg->ParseFromArray(_recv_buffer.data(), size);
            auto out_msg = static_cast<std::shared_ptr<google::protobuf::Message>>(_acu_msg);
            if (_message_logger)
            {
                _message_logger->log_msg(out_msg);
            } else {
                spdlog::warn("Message logger not real");
            }
            
            _start_receive();
        }
    }

    void ACUETHComms::_start_receive()
    {
        using namespace boost::placeholders;
        _socket.async_receive_from(
            boost::asio::buffer(_recv_buffer), _remote_endpoint,
            boost::bind(&ACUETHComms::_handle_receive, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }
}