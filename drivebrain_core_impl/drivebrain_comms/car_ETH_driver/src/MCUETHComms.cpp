#include <MCUETHComms.hpp>

using boost::asio::ip::udp;

namespace comms
{

    MCUETHComms::MCUETHComms(boost::asio::io_context &io_context, uint16_t recv_port, uint16_t send_port) : _socket(io_context, udp::endpoint(udp::v4(), recv_port)),
                                                                                                            _send_port(send_port)
    {
        _start_receive();
    }
    void MCUETHComms::send_message(std::shared_ptr<google::protobuf::Message> msg_out)
    {}
    
    void MCUETHComms::_handle_receive(const boost::system::error_code &error, std::size_t size)
    {
        if (!error)
        {
            std::cout << "recvd msg" << std::endl;

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