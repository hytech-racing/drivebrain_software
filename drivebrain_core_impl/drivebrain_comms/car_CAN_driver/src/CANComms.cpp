// this
#include <CANComms.hpp>

// standard includes
#include <iostream>
#include <cstring>
#include <cerrno>
#include <cctype>
// networking includes
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <variant>

// https://docs.kernel.org/networking/can.html

std::string comms::CANDriver::_to_lowercase(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c)
                   { return std::tolower(c); } // correct
    );
    return s;
}

bool comms::CANDriver::init()
{
    auto canbus_device = get_parameter_value<std::string>("canbus_device");

    auto dbc_file_path = _dbc_path ? _dbc_path : get_parameter_value<std::string>("path_to_dbc");

    if (!(canbus_device && dbc_file_path))
    {
        _logger.log_string("couldnt get params", core::LogLevel::ERROR);
        return false;
    }
    std::shared_ptr<dbcppp::INetwork> net;
    {
        std::ifstream idbc((*dbc_file_path).c_str());
        net = dbcppp::INetwork::LoadDBCFromIs(idbc);
    }

    for (const auto &msg : net->Messages())
    {
        _messages.insert(std::make_pair(msg.Id(), msg.Clone()));
        _messages_names_and_ids.insert(std::make_pair(_to_lowercase(msg.Name()), msg.Id()));
    }

    if (!_open_socket(*canbus_device))
    {
        _logger.log_string("couldnt open socket", core::LogLevel::ERROR);
        return false;
    }

    _logger.log_string("inited, started read", core::LogLevel::INFO);

    _do_read();
    return true;
}

bool comms::CANDriver::_open_socket(const std::string &interface_name)
{
    int raw_socket = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (raw_socket < 0)
    {
        auto err_str = std::string("Error creating CAN socket: ") + std::string(strerror(errno));
        _logger.log_string(err_str.c_str(), core::LogLevel::ERROR);

        return false;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, interface_name.c_str());
    ioctl(raw_socket, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (::bind(raw_socket, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
    {
        auto err_str = std::string("Error binding CAN socket: ") + std::string(strerror(errno));
        _logger.log_string(err_str.c_str(), core::LogLevel::ERROR);

        ::close(raw_socket);
        return false;
    }

    _socket.assign(raw_socket); // Assign the native socket to Boost.Asio descriptor
    return true;
}

void comms::CANDriver::_do_read()
{
    boost::asio::async_read(_socket, boost::asio::buffer(&_frame, sizeof(_frame)),
                            [this](boost::system::error_code ec, std::size_t bytes_transferred)
                            {
                                if (!ec && bytes_transferred == sizeof(_frame))
                                {
                                    // std::cout << "recvd" <<std::endl;
                                    _handle_recv_CAN_frame(_frame);
                                    _do_read(); // Continue reading for the next frame
                                }
                                else if (ec)
                                {
                                    std::cerr << "Error receiving CAN message: " << ec.message() << std::endl;
                                }
                            });
}

void comms::CANDriver::_send_message(const struct can_frame &frame)
{
    boost::asio::async_write(_socket, boost::asio::buffer(&frame, sizeof(frame)),
                             [this](boost::system::error_code ec, std::size_t /*bytes_transferred*/)
                             {
                                 if (ec)
                                 {
                                     std::cerr << "Error sending CAN message: " << ec.message() << std::endl;
                                 }
                             });
}

void comms::CANDriver::_handle_recv_CAN_frame(const struct can_frame &frame)
{
    std::shared_ptr<google::protobuf::Message> msg = pb_msg_recv(frame);
    {
        std::unique_lock lk(_output_deque_ref.mtx);
        _output_deque_ref.deque.push_back(msg);
        _output_deque_ref.cv.notify_all();
    }
}

// gets a protobuf message from just the name of it
std::shared_ptr<google::protobuf::Message> comms::CANDriver::_get_pb_msg_by_name(const std::string &name)
{
    // Create a dynamic message factory

    std::shared_ptr<google::protobuf::Message> prototype_message;
    const google::protobuf::Descriptor *desc = google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName(name);
    prototype_message.reset(google::protobuf::MessageFactory::generated_factory()->GetPrototype(desc)->New());
    if (!prototype_message)
    {
        std::cerr << "Failed to create prototype message" << std::endl;
        return nullptr;
    }
    return prototype_message;
}

// Function to set fields dynamically using reflection
void comms::CANDriver::set_field_values_of_pb_msg(const std::unordered_map<std::string, comms::CANDriver::FieldVariant> &field_values, std::shared_ptr<google::protobuf::Message> message)
{
    const google::protobuf::Descriptor *descriptor = message->GetDescriptor();
    const google::protobuf::Reflection *reflection = message->GetReflection();

    for (int i = 0; i < descriptor->field_count(); ++i)
    {
        const google::protobuf::FieldDescriptor *field = descriptor->field(i);

        // Get the field name and check if it exists in the provided field_values map
        std::string field_name = field->name();
        auto it = field_values.find(field_name);

        if (it != field_values.end())
        {
            // Set field values based on field type
            if (field->is_repeated())
            {
                std::cerr << "Unsupported field type for field: " << field_name << std::endl;
                continue;
            }
            switch (field->type())
            {
            case google::protobuf::FieldDescriptor::TYPE_ENUM:
                reflection->SetEnumValue(message.get(), field, (int)std::get<int>(it->second));
                break;
            case google::protobuf::FieldDescriptor::TYPE_FLOAT:
                // std::cout << "uh yo " << it->second.index() <<std::endl;
                reflection->SetFloat(message.get(), field, (float)std::get<double>(it->second));
                // std::cout << "Set float field: " << field_name << " = " << std::get<double>(it->second) << std::endl;
                break;
            case google::protobuf::FieldDescriptor::TYPE_BOOL:
                reflection->SetBool(message.get(), field, (bool)std::get<double>(it->second));
                //                std::cout << "Set bool field: " << field_name << " = " << std::get<double>(it->second) << std::endl;
                break;
            case google::protobuf::FieldDescriptor::TYPE_DOUBLE:
                reflection->SetDouble(message.get(), field, std::get<double>(it->second));
                // std::cout << "Set double field: " << field_name << " = " << std::get<double>(it->second) << std::endl;
                break;
            case google::protobuf::FieldDescriptor::TYPE_INT32:
                reflection->SetInt32(message.get(), field, (int32_t)std::get<double>(it->second));
                break;
            default:
                break;
                // std::cout << "warning, no valid type detected" << std::endl;
            }
        }
    }
}

std::shared_ptr<google::protobuf::Message> comms::CANDriver::pb_msg_recv(const can_frame &frame)
{
    auto iter = _messages.find(frame.can_id);
    if (iter != _messages.end())
    {
        std::unique_ptr<dbcppp::IMessage> msg = iter->second->Clone();
        // std::cout << "Received Message: " << msg->Name() << "\n";
        std::shared_ptr<google::protobuf::Message> msg_to_populate = _get_pb_msg_by_name(_to_lowercase(msg->Name()));

        std::unordered_map<std::string, comms::CANDriver::FieldVariant> msg_field_map;
        for (const dbcppp::ISignal &sig : msg->Signals())
        {
            // sig.Decode(frame.data);
            // std::cout << "sig name " << sig.Name() << std::endl;
            const dbcppp::ISignal *mux_sig = msg->MuxSignal();

            if (sig.MultiplexerIndicator() != dbcppp::ISignal::EMultiplexer::MuxValue ||
                (mux_sig && mux_sig->Decode(frame.data) == sig.MultiplexerSwitchValue()))
            {
                auto raw_value = sig.Decode(frame.data);
                
                // Check if the signal has enum descriptions (ValueEncodingDescriptions)
                if (sig.ValueEncodingDescriptions_Size() > 0)
                {
                    // Enum signal: Cast the decoded raw value to an integer and store it
                    msg_field_map[sig.Name()] = static_cast<int>(raw_value);
                }
                else{
                    // TODO get correct type from raw signal and store it in the map. right now they are all doubles
                    msg_field_map[sig.Name()] = sig.RawToPhys(raw_value);
                    // std::cout << "\t" << sig.Name() << "=" << sig.RawToPhys(sig.Decode(frame.data)) << sig.Unit() << "\n";
                }

            }
        }
        set_field_values_of_pb_msg(msg_field_map, msg_to_populate);
        return msg_to_populate;
    }

    return nullptr;
}

comms::CANDriver::FieldVariant comms::CANDriver::get_field_value(std::shared_ptr<google::protobuf::Message> message, const std::string &field_name)
{
    if (!message)
    {
        std::cerr << "Message is null" << std::endl;
        return std::monostate{};
    }

    const google::protobuf::Descriptor *descriptor = message->GetDescriptor();
    const google::protobuf::Reflection *reflection = message->GetReflection();
    const google::protobuf::FieldDescriptor *field = descriptor->FindFieldByName(field_name);

    if (field == nullptr)
    {
        std::cerr << "Field not found: " << field_name << std::endl;
        return std::monostate{};
    }

    // Retrieve the value based on the field type
    switch (field->cpp_type())
    {
    case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
        return reflection->GetInt32(*message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
        return reflection->GetInt64(*message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
        return reflection->GetUInt32(*message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
        return reflection->GetUInt64(*message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
        return reflection->GetDouble(*message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
        return reflection->GetFloat(*message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
        return reflection->GetBool(*message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
        return reflection->GetString(*message, field);
    case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
        return reflection->GetEnum(*message, field)->name();
    case google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE:
        return "Nested messages not supported";
    default:
        std::cerr << "Unsupported field type" << std::endl;
        return std::monostate{};
    }
}

std::optional<can_frame> comms::CANDriver::_get_CAN_msg(std::shared_ptr<google::protobuf::Message> pb_msg)
{
    // - [x] get an associated CAN message based on the name of the input message

    can_frame frame{};
    std::string type_url = pb_msg->GetTypeName();
    std::string messageTypeName = type_url.substr(type_url.find_last_of('.') + 1);
    // std::cout << "got message type name of " << messageTypeName << std::endl;

    if (_messages_names_and_ids.find(messageTypeName) != _messages_names_and_ids.end())
    {
        uint64_t id = _messages_names_and_ids[messageTypeName];
        std::unique_ptr<dbcppp::IMessage> msg = _messages[id]->Clone();
        frame.can_id = id;
        frame.len = msg->MessageSize();
        for (const auto &sig : msg->Signals())
        {
            comms::CANDriver::FieldVariant field_value = get_field_value(pb_msg, sig.Name());

            std::visit([&sig, &frame](const FieldVariant &arg)
                       {
            if (std::holds_alternative<std::monostate>(arg)) {
                std::cout << "No value found or unsupported field" << std::endl;
            } else if (std::holds_alternative<float>(arg)){
                auto val = std::get<float>(arg);
                sig.Encode(sig.PhysToRaw(val), frame.data);
            } else if(std::holds_alternative<int32_t>(arg)){
                auto val = std::get<int32_t>(arg);
                sig.Encode(sig.PhysToRaw(val), frame.data);
            } else if(std::holds_alternative<int64_t>(arg)){
                auto val = std::get<int64_t>(arg);
                sig.Encode(sig.PhysToRaw(val), frame.data);
            } else if(std::holds_alternative<uint64_t>(arg)){
                auto val = std::get<uint64_t>(arg);
                sig.Encode(sig.PhysToRaw(val), frame.data);
            } else if(std::holds_alternative<bool>(arg)){
                auto val = std::get<bool>(arg);
                sig.Encode(val, frame.data);
            } else {
                std::cout <<"uh not supported yet" <<std::endl;
            } },
                       field_value);
        }
    }
    else
    {
        std::cout << "WARNING: not creating a frame to send due to not finding frame name" << std::endl;
        return std::nullopt;
    }

    return frame;
}

void comms::CANDriver::_handle_send_msg_from_queue()
{
    // we will assume that this queue only has messages that we want to send
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> q;
    while (true)
    {
        {
            std::unique_lock lk(_input_deque_ref.mtx);
            // TODO unfuck this, queue management shouldnt live within the queue itself
            _input_deque_ref.cv.wait(lk, [this]()
                                     { return !_input_deque_ref.deque.empty(); });

            if (_input_deque_ref.deque.empty())
            {
                return;
            }

            q.deque = _input_deque_ref.deque;
            _input_deque_ref.deque.clear();
        }

        for (const auto &msg : q.deque)
        {
            std::optional<can_frame> can_msg = _get_CAN_msg(msg);
            if(can_msg)
            {
                // std::cout << "sending" <<std::endl;
                _send_message(*can_msg);
            }
        }
        q.deque.clear();
    }
}