#include <csignal>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include "libvncxx/vntime.h"
#include "libvncxx/packetfinder.h"
#include "libvncxx/packet.h"
#include <cstdint>
#include <optional>
#include <iterator>
#include <iostream>

using SerialPort = boost::asio::serial_port;

using namespace vn::xplat;
using namespace vn::protocol::uart;

void validPacketFoundHandler(void *userData, vn::protocol::uart::Packet &packet, size_t runningIndexOfPacketStart, TimeStamp ts)
{
    if (packet.type() == vn::protocol::uart::Packet::TYPE_BINARY)
    {
        vn::math::vec3f vel;
        // See if this is a binary packet type we are expecting.
        if (!packet.isCompatible((CommonGroup::COMMONGROUP_YAWPITCHROLL | CommonGroup::COMMONGROUP_ANGULARRATE), // Note use of binary OR to configure flags.
                                 TimeGroup::TIMEGROUP_NONE,
                                 ImuGroup::IMUGROUP_UNCOMPACCEL,
                                 GpsGroup::GPSGROUP_NONE,
                                 AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY,
                                 (InsGroup::INSGROUP_INSSTATUS | InsGroup::INSGROUP_POSLLA | InsGroup::INSGROUP_VELBODY),
                                 GpsGroup::GPSGROUP_NONE))
        {
            // Not the type of binary packet we are expecting.
            std::cout << "ERROR: packet is not what we want" << std::endl;
            return;
        }

        // Ok, we have our expected binary output packet. Since there are many
        // ways to configure the binary data output, the burden is on the user
        // to correctly parse the binary packet. However, we can make use of
        // the parsing convenience methods provided by the VnUartPacket structure.
        // When using these convenience methods, you have to extract them in
        // the order they are organized in the binary packet per the User Manual.
        auto ypr_data = packet.extractVec3f();
        auto angular_rate_data = packet.extractVec3f();
        auto uncomp_accel = packet.extractVec3f();
        auto linear_accel_body = packet.extractVec3f();
        uint16_t ins_status = packet.extractUint16();
        auto pos_lla = packet.extractVec3d();
        auto vel_body = packet.extractVec3f();

        std::cout << "got data" << std::endl;
        std::cout << "ypr " << ypr_data.x << " " << ypr_data.y << " " << ypr_data.z << std::endl;
        std::cout << "uncomp_accel " << uncomp_accel.x << " " << uncomp_accel.y << " " << uncomp_accel.z << std::endl;
        std::cout << "linear_accel_body " << linear_accel_body.x << " " << linear_accel_body.y << " " << linear_accel_body.z << std::endl;
        std::cout << "ins_status " << static_cast<uint16_t>(ins_status &= 0x3) << std::endl;
        std::cout << "pos_lla " << pos_lla.x <<" " << pos_lla.y <<" " << pos_lla.z << std::endl;
        std::cout << "vel_body " << vel_body.x <<" "<< vel_body.y <<" "<<vel_body.z << std::endl;
    }
    else
    {
        std::cout << "packet not correct" << std::endl;
    }
}

void readData(SerialPort &m_serial, boost::array<std::uint8_t, 512> &m_inputBuf, vn::protocol::uart::PacketFinder &processor)
{
    m_serial.async_read_some(
        boost::asio::buffer(m_inputBuf),
        [&](const boost::system::error_code &ec, std::size_t bytesCount)
        {
            if (ec)
            {
                if (ec != boost::asio::error::operation_aborted)
                {
                    std::cerr << "ERROR: " << ec.message() << std::endl;
                }
                return;
            }

            processor.processReceivedData((char *)(m_inputBuf.data()), bytesCount);
            // Initiate another asynchronous read
            readData(m_serial, m_inputBuf, processor);
        });
}

void write_data(SerialPort &m_serial, boost::array<std::uint8_t, 512> &m_outputBuf)
{
    auto numOfBytes = Packet::genWriteSerialBaudRate(
        ErrorDetectionMode::ERRORDETECTIONMODE_NONE,
        (char *)m_outputBuf.data(),
        m_outputBuf.size(),
        921600,
        1);

    boost::asio::async_write(m_serial,
                             boost::asio::buffer(m_outputBuf.data(), numOfBytes),
                             [](const boost::system::error_code &ec, std::size_t bytes_transferred)
                             {
                                 if (!ec)
                                 {
                                     std::cout << "Successfully sent " << bytes_transferred << " bytes.\n";
                                 }
                                 else
                                 {
                                     std::cerr << "Error sending data: " << ec.message() << "\n";
                                 }
                             });

    numOfBytes = Packet::genWriteBinaryOutput1(
        ErrorDetectionMode::ERRORDETECTIONMODE_NONE,
        (char *)m_outputBuf.data(),
        m_outputBuf.size(),
        AsyncMode::ASYNCMODE_PORT1,
        2,
        (CommonGroup::COMMONGROUP_YAWPITCHROLL | CommonGroup::COMMONGROUP_ANGULARRATE), // Note use of binary OR to configure flags.
        TimeGroup::TIMEGROUP_NONE,
        ImuGroup::IMUGROUP_UNCOMPACCEL,
        GpsGroup::GPSGROUP_NONE,
        AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY,
        (InsGroup::INSGROUP_INSSTATUS | InsGroup::INSGROUP_POSLLA | InsGroup::INSGROUP_VELBODY),
        GpsGroup::GPSGROUP_NONE);

    boost::asio::async_write(m_serial,
                             boost::asio::buffer(m_outputBuf.data(), numOfBytes),
                             [](const boost::system::error_code &ec, std::size_t bytes_transferred)
                             {
                                 if (!ec)
                                 {
                                     std::cout << "Successfully sent " << bytes_transferred << " bytes.\n";
                                 }
                                 else
                                 {
                                     std::cerr << "Error sending data: " << ec.message() << "\n";
                                 }
                             });
}

int main()
{
    vn::protocol::uart::PacketFinder processor;
    processor.registerPossiblePacketFoundHandler(nullptr, validPacketFoundHandler);
    boost::asio::io_service io;
    SerialPort m_serial(io);
    boost::system::error_code ec;
    std::string dev = "/dev/ttyUSB0";
    m_serial.open(dev, ec);

    if (ec)
    {
        std::cerr << "ERROR: Failed to open " << dev << std::endl;
        return 1;
    }
    m_serial.set_option(SerialPort::baud_rate(115200));
    m_serial.set_option(SerialPort::character_size(8));
    m_serial.set_option(SerialPort::parity(SerialPort::parity::none));
    m_serial.set_option(SerialPort::stop_bits(SerialPort::stop_bits::one));
    m_serial.set_option(SerialPort::flow_control(SerialPort::flow_control::none));

    boost::array<std::uint8_t, 512> m_inputBuf;
    boost::array<std::uint8_t, 512> m_oututBuf;

    write_data(m_serial, m_oututBuf);
    m_serial.set_option(SerialPort::baud_rate(921600));
    readData(m_serial, m_inputBuf, processor);
    io.run();
    return 0;
}