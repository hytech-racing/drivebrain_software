#include <VNComms.hpp>

// standard includes
#include <iostream>
#include <cstring>
#include <cerrno>
#include <cctype>

#include "hytech_msgs.pb.h"

#include "libvncxx/vntime.h"
#include "libvncxx/packetfinder.h"
#include "libvncxx/packet.h"

using namespace vn::xplat;
using namespace vn::protocol::uart;


namespace comms
{

    VNDriver::VNDriver(core::JsonFileHandler &json_file_handler, core::Logger &logger, core::StateEstimator &state_estimator)
    {
        Configurable(logger, json_file_handler, "VNDriver");
        _logger(logger);
        _mcu_message = std::make_shared<hytech_msgs::VNOutputData>();
        _state_estimator(state_estimator);

        // Try to establish a connection to the driver
        _logger.log_string("Opening device.", core::LogLevel::INFO);

        boost::system::error_code ec;

        auto device_name = get_parameter_value<std::string>("device_name");
        auto baud_rate = get_parameter_value<int>("baud_rate");
        auto port = get_parameter_value<int>("port");

        _processor.registerPossiblePacketFoundHandler(nullptr, _handle_recieve);
        SerialPort _serial(_io);
        boost::system::error_code ec;
        std::string device_name = "/dev/ttyUSB0";

        _serial.open(device_name, ec);

        if (ec)
        {
            _logger.log_string("Failed to open device.", core::LogLevel::Error);
            return 1;
        }

        _logger.log_string("Setting baud rate.", core::LogLevel::INFO);

        // Set the baud rate of the device along with other configs
        _serial.set_option(SerialPort::baud_rate(115200));
        _serial.set_option(SerialPort::character_size(8));
        _serial.set_option(SerialPort::parity(SerialPort::parity::none));
        _serial.set_option(SerialPort::stop_bits(SerialPort::stop_bits::one));
        _serial.set_option(SerialPort::flow_control(SerialPort::flow_control::none));

        _set_baud_rate(baud_rate, port);

        // Configures the binary outputs for the device
        _logger.log_string("Configuring binary outputs.", core::LogLevel::INFO);

        _configure_binary_outputs();

        // Starts read
        _logger.log_string("Starting vn driver recieve.", core::LogLevel::INFO);

        _start_recieve();
    }

    void VNDriver::_configure_binary_outputs()
    {

        num_of_bytes = Packet::genWriteBinaryOutput1(
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
                                 boost::asio::buffer(_output_buff.data(), num_of_bytes),
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
        return 1;
    }

    void VNDriver::_set_baud_rate(int rate, int port)
    {
        auto num_of_bytes = vn::protocol::uart::Packet.genWriteSerialBaudRate(
            ErrorDetectionMode::ERRORDETECTIONMODE_NONE,
            (char *)_output_buff.data(),
            _output_buff.size(),
            rate,
            port);

        boost::asio::async_write(_serial, boost::asio::buffer(_output_buff.data(), num_of_bytes),
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

        _serial.set_option(SerialPort::baud_rate(rate));

        return 0;
    }

    void VNDriver::_handle_recieve(void *userData, vn::protocol::uart::Packet &packet, size_t runningIndexOfPacketStart, TimeStamp ts)
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
            
            // Extract data in correct order
            auto ypr_data = packet.extractVec3f();
            auto angular_rate_data = packet.extractVec3f();
            auto uncomp_accel = packet.extractVec3f();
            auto linear_accel_body = packet.extractVec3f();
            uint16_t ins_status = packet.extractUint16();
            auto pos_lla = packet.extractVec3d();
            auto vel_body = packet.extractVec3f();

            // Create the protobuf message to send
            _mcu_message->set_yaw(ypr_data.x);
            _mcu_message->set_roll(ypr_data.z);
            _mcu_message->set_pitch(ypr_data.y);
            _mcu_message->set_uncomp_accel_x(uncomp_accel.x);
            _mcu_message->set_uncomp_accel_y(uncomp_accel.y);
            _mcu_message->set_uncomp_accel_z(uncomp_accel.z);
            _mcu_message->set_accel_body_x(linear_accel_body.x);
            _mcu_message->set_accel_body_y(linear_accel_body.y);
            _mcu_message->set_accel_body_z(linear_accel_body.z);
            _mcu_message->set_ins_status(static_cast<uint16_t>(ins_status &= 0x3));
            _mcu_message->set_pos_lla_x(pos_lla.x);
            _mcu_message->set_pos_lla_y(pos_lla.y);
            _mcu_message->set_pos_lla_z(pos_lla.z);
            _mcu_message->set_vel_body_x(vel_body.x);
            _mcu_message->set_vel_body_y(vel_body.y);
            _mcu_message->set_vel_body_z(vel_body.z);    

            _state_estimator.handle_recv_process(static_cast<std::shared_ptr<google::protobuf::Message>>(_mcu_message));        
        }
        else
        {
            std::cout << "packet not correct" << std::endl;
        }
    }

    void VNDriver::_start_recieve()
    {
        _serial.async_read_some(
            boost::asio::buffer(_input_buff),
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

                _processor.processReceivedData((char *)(_input_buff.data()), bytesCount);
                // Initiate another asynchronous read
                _start_recieve(_serial, _input_buff, _processor);
            });
    }
}
