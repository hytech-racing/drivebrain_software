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

bool comms::CANDriver::init()
{
    auto canbus_device = get_parameter_value<std::string>("canbus_device");
    auto dbc_file_path = get_parameter_value<std::string>("path_to_dbc");

    auto to_lower = [](std::string s)
    {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c)
                       { return std::tolower(c); } // correct
        );
        return s;
    };

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
        _messages_names_and_ids.insert(std::make_pair(to_lower(msg.Name()), msg.Id()));
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

void comms::CANDriver::handle_output_msg_from_queue(core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> &output_deque)
{
}


// Function to set fields dynamically using reflection
void comms::CANDriver::set_field_values_of_pb_msg(std::shared_ptr<google::protobuf::Message> message, const std::unordered_map<std::string, std::variant<float, bool, double, int, uint32_t, std::string>> &field_values)
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
                reflection->SetFloat(message.get(), field, std::get<float>(it->second));
                std::cout << "Set float field: " << field_name << " = " << std::get<float>(it->second) << std::endl;
                break;
            case google::protobuf::FieldDescriptor::TYPE_BOOL:
                reflection->SetBool(message.get(), field, std::get<bool>(it->second));
                std::cout << "Set bool field: " << field_name << " = " << std::get<bool>(it->second) << std::endl;
                break;
            case google::protobuf::FieldDescriptor::TYPE_DOUBLE:
                reflection->SetDouble(message.get(), field, std::get<double>(it->second));
                std::cout << "Set double field: " << field_name << " = " << std::get<double>(it->second) << std::endl;
                break;
            default:
                std::cout << "warning, no valid type detected" << std::endl;
            }
        }
    }
}

std::shared_ptr<google::protobuf::Message> comms::CANDriver::_get_pb_msg_by_name(const std::string &msg_name)
{

    const google::protobuf::Descriptor *descriptor = google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName(msg_name);

    // Create a dynamic message factory
    google::protobuf::DynamicMessageFactory dynamic_factory;
    const google::protobuf::Message *prototype_message = dynamic_factory.GetPrototype(descriptor);
    if (!prototype_message)
    {
        std::cerr << "Failed to create prototype message for type: " << msg_name << std::endl;
        return {};
    }

    std::shared_ptr<google::protobuf::Message> p(prototype_message->New());
    delete prototype_message;
    delete descriptor;

    return p;
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

    can_frame frame {};
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
        }},
                   field_value);
    }
    return frame;
}

void comms::CANDriver::handle_send_msg_from_queue(core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> &input_deque)
{
    // we will assume that this queue only has messages that we want to send
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> q;
    {
        std::unique_lock lk(input_deque.mtx);

        if (input_deque.deque.empty())
        {
            return;
        }

        q.deque = input_deque.deque;
        input_deque.deque.clear();
    }

    for (const auto &msg : q.deque)
    {
        // TODO send the messages
        auto can_msg = _get_CAN_msg(msg);
    }
}
