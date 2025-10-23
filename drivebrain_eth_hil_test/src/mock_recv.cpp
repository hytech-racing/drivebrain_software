#include <array>
#include <boost/asio.hpp>
#include <iostream>
#include <chrono>

using boost::asio::ip::udp;

std::array<uint8_t, 4096> _recv_buffer;
boost::asio::io_context _io_context;
boost::asio::ip::udp::socket _socket(_io_context, udp::endpoint(udp::v4(), 7766));
boost::asio::ip::udp::endpoint _remote_endpoint;

void start_recv() {
    _socket.async_receive_from(
        boost::asio::buffer(_recv_buffer), _remote_endpoint,
        [](const boost::system::error_code &ec, std::size_t n) {
            if(!ec && n>0) {
                auto now = std::chrono::system_clock::now();
                auto t = std::chrono::system_clock::to_time_t(now);
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
                std::cout << "[" << ms.count() << "] " << "recvd " << n << " bytes\n";
                start_recv();
            } else if (ec) { std::cerr << ec.message() << "\n"; }
        });

}

  
int main() {
    start_recv();
    return 0;
}
