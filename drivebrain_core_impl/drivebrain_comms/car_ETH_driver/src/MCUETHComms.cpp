#include <MCUETHComms.hpp>
#include <drivebrain_core_msgs/v1/drivebrain_vehicle_manager.pb.h>

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
                                                   _send_socket(io_context, udp::endpoint(boost::asio::ip::make_address("127.0.0.1"), send_port)),
                                                   _send_port(send_port)
    {
        _mcu_msg = std::make_shared<drivebrain_core_msgs::MCUData>();
        _start_receive();
    }
    void MCUETHComms::send_message(std::shared_ptr<google::protobuf::Message> msg_out)
    {
        // TODO fill the buffer with proper stuff from the message to send
        msg_out->SerializeToArray(_send_buffer.data(), msg_out->ByteSizeLong());
        _socket.async_send_to(boost::asio::buffer(_send_buffer, msg_out->ByteSizeLong()),
                              udp::endpoint(boost::asio::ip::make_address("127.0.0.1"), _send_port),
                              boost::bind(&MCUETHComms::_handle_send,
                                          this,
                                          _send_buffer,
                                          boost::asio::placeholders::error,
                                          boost::asio::placeholders::bytes_transferred));
    }

    void MCUETHComms::_handle_receive(const boost::system::error_code &error, std::size_t size)
    {
        if (!error)
        {
            _logger.log_string("recvd msg", core::LogLevel::INFO);
            _mcu_msg->ParseFromArray(_recv_buffer.data(), size);
            _logger.log_string("parsed msg", core::LogLevel::INFO);
            _state_estimator.handle_recv_process(static_cast<std::shared_ptr<google::protobuf::Message>>(_mcu_msg));
            _start_receive();
        }
    }
    void MCUETHComms::_handle_send(std::array<uint8_t, 2048> /*message*/,
                                   const boost::system::error_code & /*error*/,
                                   std::size_t /*bytes_transferred*/)
    {
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