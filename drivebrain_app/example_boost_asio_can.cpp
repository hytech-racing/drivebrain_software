#include <boost/asio.hpp>
#include <iostream>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <cstring>
#include <unistd.h>

class CanSocket {
public:
    CanSocket(boost::asio::io_context& io_context, const std::string& interface_name)
        : socket_(io_context) {
        open_socket(interface_name);
        do_read();  // Start receiving messages
    }

    // Public function to send CAN messages
    void send_message(const struct can_frame& frame) {
        boost::asio::async_write(socket_, boost::asio::buffer(&frame, sizeof(frame)),
            [this](boost::system::error_code ec, std::size_t /*bytes_transferred*/) {
                if (!ec) {
                    std::cout << "Message sent successfully!" << std::endl;
                } else {
                    std::cerr << "Error sending CAN message: " << ec.message() << std::endl;
                }
            });
    }

private:
    void open_socket(const std::string& interface_name) {
        int raw_socket = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (raw_socket < 0) {
            std::cerr << "Error creating CAN socket: " << strerror(errno) << std::endl;
            return;
        }

        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, interface_name.c_str());
        ioctl(raw_socket, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (::bind(raw_socket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            std::cerr << "Error binding CAN socket: " << strerror(errno) << std::endl;
            ::close(raw_socket);
            return;
        }

        socket_.assign(raw_socket);  // Assign the native socket to Boost.Asio descriptor
    }

    // Function to start reading CAN messages asynchronously
    void do_read() {
        boost::asio::async_read(socket_, boost::asio::buffer(&frame_, sizeof(frame_)),
            [this](boost::system::error_code ec, std::size_t bytes_transferred) {
                if (!ec && bytes_transferred == sizeof(frame_)) {
                    handle_receive(frame_);
                    do_read();  // Continue reading for the next frame
                } else if (ec) {
                    std::cerr << "Error receiving CAN message: " << ec.message() << std::endl;
                }
            });
    }

    // Handle received CAN messages
    void handle_receive(const struct can_frame& frame) {
        std::cout << "Received CAN ID: " << std::hex << frame.can_id << std::endl;
        std::cout << "Data: ";
        for (int i = 0; i < frame.can_dlc; i++) {
            std::cout << std::hex << static_cast<int>(frame.data[i]) << " ";
        }
        std::cout << std::endl;
    }

    boost::asio::posix::stream_descriptor socket_;
    struct can_frame frame_;
};

int main() {
    boost::asio::io_context io_context;
    CanSocket can_socket(io_context, "vcan0");

    // Example of sending a CAN message
    struct can_frame frame;
    frame.can_id = 0x123;
    frame.can_dlc = 8;
    std::memcpy(frame.data, "\x01\x02\x03\x04\x05\x06\x07\x08", 8);

    // Send the CAN message via the public function
    can_socket.send_message(frame);

    io_context.run();  // Start the Boost.Asio event loop
    return 0;
}
