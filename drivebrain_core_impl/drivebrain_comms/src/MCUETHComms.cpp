#include <MCUETHComms.hpp>
#include "hytech_msgs.pb.h"

using boost::asio::ip::udp;
namespace comms
{
    MCUETHComms::MCUETHComms(core::Logger &logger,
                             deqtype &in_deq,
                             std::shared_ptr<loggertype> message_logger,
                             core::StateEstimator &state_estimator,
                             boost::asio::io_context &io_context,
                             const std::string &send_ip,
                             uint16_t recv_port,
                             uint16_t send_port) : _logger(logger),
                                                   _input_deque_ref(in_deq),
                                                   _message_logger(message_logger),
                                                   _socket(io_context, udp::endpoint(udp::v4(), recv_port)),
                                                   _state_estimator(state_estimator),
                                                   _send_port(send_port),
                                                   _send_ip(send_ip)
    {
        _mcu_msg = std::make_shared<hytech_msgs::MCUOutputData>();
        _logger.log_string("starting out thread", core::LogLevel::INFO);
        _running = true;
        _output_thread = std::thread(&MCUETHComms::_handle_send_msg_from_queue, this);
        _logger.log_string("starting eth comms recv", core::LogLevel::INFO);
        _start_receive();
        _logger.log_string("started eth comms", core::LogLevel::INFO);
    }
    MCUETHComms::~MCUETHComms()
    {
        _running = false;
        _input_deque_ref.cv.notify_all();
        _output_thread.join();
        std::cout <<"destructed MCU ETH COMMS" <<std::endl;
    }

    void MCUETHComms::_handle_send_msg_from_queue()
    {
        // we will assume that this queue only has messages that we want to send
        while (_running)
        {
            {
                std::unique_lock lk(_input_deque_ref.mtx);
                // TODO unfuck this, queue management shouldnt live within the queue itself
                _input_deque_ref.cv.wait(lk, [this]()
                                         { return !_input_deque_ref.deque.empty() || !_running; });

                if (_input_deque_ref.deque.empty())
                {
                    return;
                }
                for (const auto &msg : _input_deque_ref.deque)
                {
                    _send_message(msg);
                    _message_logger->log_msg(msg);
                }
                _input_deque_ref.deque.clear();
            }
        }
    }
    void MCUETHComms::_send_message(std::shared_ptr<google::protobuf::Message> msg_out)
    {
        msg_out->SerializeToArray(_send_buffer.data(), msg_out->ByteSizeLong());
        _socket.async_send_to(boost::asio::buffer(_send_buffer, msg_out->ByteSizeLong()),
                              udp::endpoint(boost::asio::ip::make_address(_send_ip.c_str()), _send_port),
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
            _mcu_msg->ParseFromArray(_recv_buffer.data(), size);
            auto out_msg = static_cast<std::shared_ptr<google::protobuf::Message>>(_mcu_msg);
            _state_estimator.handle_recv_process(out_msg);
            _message_logger->log_msg(out_msg);
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