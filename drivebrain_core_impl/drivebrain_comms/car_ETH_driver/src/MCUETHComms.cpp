#include <MCUETHComms.hpp>
#include <drivebrain_vehicle_manager.pb.h>

using boost::asio::ip::udp;

namespace comms
{

    MCUETHComms::MCUETHComms(core::Logger &logger,
                             core::StateEstimator &state_estimator,
                             boost::asio::io_context &io_context,
                             uint16_t recv_port,
                             uint16_t send_port) : _logger(logger),
                                                   _state_estimator(state_estimator),
                                                   _socket(io_context, udp::endpoint(udp::v4(), recv_port)),
                                                   _send_port(send_port)
    {
        _start_receive();
    }
    void MCUETHComms::send_message(std::shared_ptr<google::protobuf::Message> msg_out)
    {
    }

    void MCUETHComms::_handle_receive(const boost::system::error_code &error, std::size_t size)
    {
        if (!error)
        {
            _logger.log_string("recvd msg", core::LogLevel::INFO);
            std::shared_ptr<google::protobuf::Message> mcu_msg;

            _state_estimator.handle_recv_process();
            
            _start_receive();
            
        }
    }
    void MCUETHComms::_start_receive()
    {
        using namespace boost::placeholders;
        _socket.async_receive_from(
            boost::asio::buffer(_recv_buffer), _remote_endpoint,
            boost::bind(&MCUETHComms::_handle_receive, this,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred));
    }

}