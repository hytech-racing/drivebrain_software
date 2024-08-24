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
    auto dbc_file_path = get_parameter_value<std::string>("path_to_dbc");

    if (!(canbus_device && dbc_file_path))
    {
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
        std::cout << msg.Name() << std::endl;
    }

    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;

    _CAN_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (_CAN_socket < 0)
    {
        std::cerr << "Error while opening socket: " << strerror(errno) << std::endl;
        return false;
    }

    if (canbus_device)
    {
        std::strcpy(ifr.ifr_name, (*canbus_device).c_str());
    }
    else
    {
        return false;
    }
    ioctl(_CAN_socket, SIOCGIFINDEX, &ifr); // get the interface index for can0

    // Bind the socket to the can0 interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(_CAN_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        std::cerr << "Error in socket bind: " << strerror(errno) << std::endl;
        return false;
    }

    std::cout << "Listening on " << *canbus_device << "..." << std::endl;
    return true;
}

bool comms::CANDriver::_open_socket(const std::string& interface_name)
{
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

    _socket.assign(raw_socket);  // Assign the native socket to Boost.Asio descriptor
}

void comms::CANDriver::_do_read()
{
    boost::asio::async_read(_socket, boost::asio::buffer(&_frame, sizeof(_frame)),
            [this](boost::system::error_code ec, std::size_t bytes_transferred) {
                if (!ec && bytes_transferred == sizeof(_frame)) {
                    _handle_recv_CAN_frame(_frame);
                    _do_read();  // Continue reading for the next frame
                } else if (ec) {
                    std::cerr << "Error receiving CAN message: " << ec.message() << std::endl;
                }
            });
}

void comms::CANDriver::_handle_recv_CAN_frame(const struct can_frame& frame)
{
    auto msg = pb_msg_recv(frame);
    {
        std::unique_lock lk(_output_deque_ref.mtx);
        _output_deque_ref.deque.push_back(msg);
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
            case google::protobuf::FieldDescriptor::TYPE_FLOAT:
                reflection->SetFloat(message.get(), field, std::get<double>(it->second));
                std::cout << "Set float field: " << field_name << " = " << std::get<float>(it->second) << std::endl;
                break;
            case google::protobuf::FieldDescriptor::TYPE_BOOL:
                reflection->SetBool(message.get(), field, std::get<double>(it->second));
                std::cout << "Set bool field: " << field_name << " = " << std::get<bool>(it->second) << std::endl;
                break;
            case google::protobuf::FieldDescriptor::TYPE_DOUBLE:
                reflection->SetDouble(message.get(), field, std::get<double>(it->second));
                std::cout << "Set double field: " << field_name << " = " << std::get<double>(it->second) << std::endl;
                break;
            case google::protobuf::FieldDescriptor::TYPE_INT32:
                reflection->SetInt32(message.get(), field, std::get<double>(it->second));
                break;
            default:
                std::cout << "warning, no valid type detected" << std::endl;
            }
        }
    }
}

std::shared_ptr<google::protobuf::Message> comms::CANDriver::pb_msg_recv(const can_frame &frame)
{
    auto iter = _messages.find(frame.can_id);
    if (iter != _messages.end())
    {
        auto msg = iter->second->Clone();
        std::cout << "Received Message: " << msg->Name() << "\n";
        auto msg_to_populate = _get_pb_msg_by_name(_to_lowercase(msg->Name()));

        std::unordered_map<std::string, comms::CANDriver::FieldVariant> msg_field_map;
        for (const dbcppp::ISignal &sig : msg->Signals())
        {
            // sig.Decode(frame.data);
            std::cout << "sig name " << sig.Name() << std::endl;
            const dbcppp::ISignal *mux_sig = msg->MuxSignal();
            
            if (sig.MultiplexerIndicator() != dbcppp::ISignal::EMultiplexer::MuxValue ||
                (mux_sig && mux_sig->Decode(frame.data) == sig.MultiplexerSwitchValue()))
            {
                // TODO get correct type from raw signal and store it in the map. right now they are all doubles
                msg_field_map[sig.Name()] = sig.RawToPhys(sig.Decode(frame.data));
                std::cout << "\t" << sig.Name() << "=" << sig.RawToPhys(sig.Decode(frame.data)) << sig.Unit() << "\n";
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

can_frame comms::CANDriver::_get_CAN_msg(std::shared_ptr<google::protobuf::Message> pb_msg)
{
    // - [x] get an associated CAN message based on the name of the input message

    can_frame frame{};
    std::string type_url = pb_msg->GetTypeName();
    std::string messageTypeName = type_url.substr(type_url.find_last_of('.') + 1);
    std::cout << "got message type name of " << messageTypeName << std::endl;
    auto id = _messages_names_and_ids[messageTypeName];
    std::cout << id << std::endl;
    auto msg = _messages[id]->Clone();
    frame.can_id = id;
    frame.len = msg->MessageSize();
    for (const auto &sig : msg->Signals())
    {
        std::cout << sig.Name() << std::endl;
        auto field_value = get_field_value(pb_msg, sig.Name());

        std::visit([&sig, &frame](const FieldVariant &arg)
                   {
        if (std::holds_alternative<std::monostate>(arg)) {
            std::cout << "No value found or unsupported field" << std::endl;
        } else if (std::holds_alternative<float>(arg)){
            std::cout << "Field value: " << std::get<float>(arg) << std::endl;
            auto val = std::get<float>(arg);
            sig.Encode(sig.PhysToRaw(val), frame.data);
        } else if(std::holds_alternative<int32_t>(arg)){
            std::cout << "Field value: " << std::get<int32_t>(arg) << std::endl;
            auto val = std::get<int32_t>(arg);
            sig.Encode(sig.PhysToRaw(val), frame.data);
        } else if(std::holds_alternative<int64_t>(arg)){
            std::cout << "Field value: " << std::get<int64_t>(arg) << std::endl;
            auto val = std::get<int64_t>(arg);
            sig.Encode(sig.PhysToRaw(val), frame.data);
        } else if(std::holds_alternative<uint64_t>(arg)){
            std::cout << "Field value: " << std::get<uint64_t>(arg) << std::endl;
            auto val = std::get<uint64_t>(arg);
            sig.Encode(sig.PhysToRaw(val), frame.data);
        } else if(std::holds_alternative<bool>(arg)){
            std::cout << "Field value: " << std::get<bool>(arg) << std::endl;
            auto val = std::get<bool>(arg);
            sig.Encode(val, frame.data);
        } else {
            std::cout <<"uh not supported yet" <<std::endl;
        } },
                   field_value);
    }
    return frame;
}

// void comms::CANDriver::handle_send_msg_from_queue(core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> &input_deque)
// {
//     // we will assume that this queue only has messages that we want to send
//     core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> q;
//     {
//         std::unique_lock lk(input_deque.mtx);

//         if (input_deque.deque.empty())
//         {
//             return;
//         }

//         q.deque = input_deque.deque;
//         input_deque.deque.clear();
//     }

//     for (const auto &msg : q.deque)
//     {
//         // TODO send the messages
//         auto can_msg = _get_CAN_msg(msg);
//     }
// }
