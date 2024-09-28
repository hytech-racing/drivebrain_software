#include <VNComms.hpp>

// standard includes
#include <iostream>
#include <cstring>
#include <cerrno>
#include <cctype>

#include "hytech_msgs.pb.h"
#include "base_msgs.pb.h"

#include "libvncxx/vntime.h"
#include "libvncxx/packetfinder.h"
#include "libvncxx/packet.h"

using namespace vn::xplat;
using namespace vn::protocol::uart;
using loggertype = core::MsgLogger<std::shared_ptr<google::protobuf::Message>>;


namespace comms
{

    VNDriver::VNDriver(core::JsonFileHandler &json_file_handler, core::Logger &logger, std::shared_ptr<loggertype> message_logger, core::StateEstimator &state_estimator)
    {
        Configurable(logger, json_file_handler, "VNDriver");
        _logger(logger);
        _message_logger(message_logger);
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

        auto num_of_bytes = Packet::genWriteBinaryOutput1(
            ErrorDetectionMode::ERRORDETECTIONMODE_NONE,
            (char *)_output_buff.data(),
            _output_buff.size(),
            AsyncMode::ASYNCMODE_PORT1,
            2,
            (CommonGroup::COMMONGROUP_YAWPITCHROLL | CommonGroup::COMMONGROUP_ANGULARRATE), // Note use of binary OR to configure flags.
            TimeGroup::TIMEGROUP_NONE,
            ImuGroup::IMUGROUP_UNCOMPACCEL,
            GpsGroup::GPSGROUP_NONE,
            AttitudeGroup::ATTITUDEGROUP_LINEARACCELBODY,
            (InsGroup::INSGROUP_INSSTATUS | InsGroup::INSGROUP_POSLLA | InsGroup::INSGROUP_VELBODY),
            GpsGroup::GPSGROUP_NONE);

        boost::asio::async_write(_serial,
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
    }

    void VNDriver::_set_baud_rate(int rate, int port)
    {
        auto num_of_bytes = vn::protocol::uart::Packet::genWriteSerialBaudRate(
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
            std::shared_ptr<hytech_msgs::VNData> msg_out = std::make_shared<hytech_msgs::VNData>();

            hytech_msgs::xyz_vector* linear_vel_msg = msg_out->mutable_vn_vel_m_s();
            linear_vel_msg->set_x(vel_body.x);
            linear_vel_msg->set_y(vel_body.y);
            linear_vel_msg->set_z(vel_body.z);

            hytech_msgs::xyz_vector* linear_accel_msg = msg_out->mutable_vn_linear_accel_m_ss();
            linear_accel_msg->set_x(linear_accel_body.x);
            linear_accel_msg->set_y(linear_accel_body.y);
            linear_accel_msg->set_z(linear_accel_body.z);

            hytech_msgs::xyz_vector* linear_accel_uncomp_msg = msg_out->mutable_vn_linear_accel_uncomp_m_ss();
            linear_accel_uncomp_msg->set_x(uncomp_accel.x);
            linear_accel_uncomp_msg->set_y(uncomp_accel.y);
            linear_accel_uncomp_msg->set_z(uncomp_accel.z);

            hytech_msgs::xyz_vector* angular_rate_data_msg = msg_out->mutable_vn_angular_rate_rad_s();
            angular_rate_data_msg->set_x(angular_rate_data.x);
            angular_rate_data_msg->set_y(angular_rate_data.y);
            angular_rate_data_msg->set_z(angular_rate_data.z);

            hytech_msgs::ypr_vector* ypr_data_msg = msg_out->mutable_vn_ypr_rad();
            ypr_data_msg->set_yaw(ypr_data.x);
            ypr_data_msg->set_pitch(ypr_data.y);
            ypr_data_msg->set_roll(ypr_data.z);

            hytech_msgs::GPS_data* vn_gps_msg = msg_out->mutable_vn_gps();
            vn_gps_msg->set_lat(pos_lla.x);
            vn_gps_msg->set_lon(pos_lla.y);

            hytech_msgs::vn_status* vn_ins_msg = msg_out->mutable_status();
            vn_ins_msg->set_ins_status(hytech_msgs::INSStatus::TRACKING_2);

            _state_estimator.handle_recv_process(static_cast<std::shared_ptr<google::protobuf::Message>>(msg_out));      
            _message_logger.log_message(static_cast<std::shared_ptr<google::protobuf::Message>>(msg_out));
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
