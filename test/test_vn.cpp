#include <csignal>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include "libvncxx/vntime.h"
#include "libvncxx/packetfinder.h"
#include <cstdint>
#include <optional>
#include <iterator>
#include <iostream>
using SerialPort = boost::asio::serial_port;

using namespace vn::xplat;
void validPacketFoundHandler(void *userData, vn::protocol::uart::Packet &packet, size_t runningIndexOfPacketStart, TimeStamp ts)
{
    if (packet.type() == vn::protocol::uart::Packet::TYPE_BINARY)
    {
        vn::math::vec3f vel;
        // See if this is a binary packet type we are expecting.
        if (!packet.isCompatible(vn::protocol::uart::COMMONGROUP_NONE, vn::protocol::uart::TIMEGROUP_NONE,
                                 vn::protocol::uart::IMUGROUP_NONE,
                                 vn::protocol::uart::GPSGROUP_NONE,
                                 vn::protocol::uart::ATTITUDEGROUP_NONE,
                                 vn::protocol::uart::INSGROUP_VELBODY, vn::protocol::uart::GPSGROUP_NONE))
        {
            // Not the type of binary packet we are expecting.
            return;
        }
        // Ok, we have our expected binary output packet. Since there are many
        // ways to configure the binary data output, the burden is on the user
        // to correctly parse the binary packet. However, we can make use of
        // the parsing convenience methods provided by the VnUartPacket structure.
        // When using these convenience methods, you have to extract them in
        // the order they are organized in the binary packet per the User Manual.
        vel = packet.extractVec3f();
    }
}


void readData(SerialPort& m_serial, boost::array<std::uint8_t, 512>& m_inputBuf, vn::protocol::uart::PacketFinder& processor)
{
    m_serial.async_read_some(
        boost::asio::buffer(m_inputBuf),
        [&](const boost::system::error_code& ec, std::size_t bytesCount)
        {
            if (ec) {
                if (ec != boost::asio::error::operation_aborted) {
                    std::cerr << "ERROR: " << ec.message() << std::endl;
                }
                return;
            }

            processor.processReceivedData((char *)(m_inputBuf.data()), bytesCount);
            // Initiate another asynchronous read
            readData(m_serial, m_inputBuf, processor);
        }
    );
}

int main()
{
    vn::protocol::uart::PacketFinder processor;
    processor.registerPossiblePacketFoundHandler(nullptr, validPacketFoundHandler);
    boost::asio::io_service io;
    SerialPort m_serial(io);
    boost::system::error_code ec;
    std::string dev = "/dev/ttyS0";
    m_serial.open(dev, ec);
    if (ec) {
        std::cerr << "ERROR: Failed to open " << dev << std::endl;
        return 1;
    }
    m_serial.set_option(SerialPort::baud_rate(115200));
    m_serial.set_option(SerialPort::character_size(8));
    m_serial.set_option(SerialPort::parity(SerialPort::parity::none));
    m_serial.set_option(SerialPort::stop_bits(SerialPort::stop_bits::one));
    m_serial.set_option(SerialPort::flow_control(SerialPort::flow_control::none));
    
    boost::array<std::uint8_t, 512> m_inputBuf;

    // readData(m_serial, m_inputBuf, processor);
    io.run();

    return 0;
}