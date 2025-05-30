#include "ETHSendComms.hpp"
namespace comms
{
using boost::asio::ip::udp;
ETHSendComms::ETHSendComms(boost::asio::io_context &io_context, uint16_t send_port, std::string send_ip, bool bind) 
: _endp(boost::asio::ip::make_address(send_ip.c_str()), send_port),
_socket(io_context)
{
    _socket.open(boost::asio::ip::udp::v4());

    if (bind) {
        _socket.bind(_endp);
    }
    _running = true;
    _output_thread = std::thread(&comms::ETHSendComms::_handle_send_msg_from_queue, this);
}

ETHSendComms::~ETHSendComms()
{
    spdlog::info("destructing ETHSendComms");
    _running = false;
    _queue.cv.notify_all();
    _output_thread.join();
    spdlog::info("destructed ETHSendComms");
}

void ETHSendComms::enqueue_msg_to_send(std::shared_ptr<google::protobuf::Message> send_msg)
{
    {
        spdlog::debug("enqueing msg");
        std::unique_lock lk(_queue.mtx);
        _queue.deque.push_back(send_msg);
        _queue.cv.notify_all();
    }
}

// ran within the thread
void ETHSendComms::_handle_send_msg_from_queue()
{
    // we will assume that this queue only has messages that we want to send
    while (_running)
    {
        {
            std::unique_lock lk(_queue.mtx);
            // TODO unfuck this, queue management shouldnt live within the queue itself
            _queue.cv.wait(lk, [this]()
                                        { return !_queue.deque.empty() || !_running; });

            if (_queue.deque.empty())
            {
                return;
            }
            for (const auto &msg : _queue.deque)
            {
                _send_message(msg);
                this->log(msg);
            }
            _queue.deque.clear();
        }
    }
}

// don do nothin
void ETHSendComms::_send_completition_handler(std::array<uint8_t, 4096> /*message*/,
                                        const boost::system::error_code & /*error*/,
                                        std::size_t /*bytes_transferred*/)
{

}

void ETHSendComms::_send_message(std::shared_ptr<google::protobuf::Message> msg_out)
{
    spdlog::info("sending msg over ethernet");
    msg_out->SerializeToArray(_send_buffer.data(), msg_out->ByteSizeLong());
    _socket.async_send_to(boost::asio::buffer(_send_buffer, msg_out->ByteSizeLong()),
                            _endp,
                            boost::bind(&ETHSendComms::_send_completition_handler,
                                        this,
                                        _send_buffer,
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));
}

}