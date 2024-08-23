// this
#include <CANComms.hpp>

// standard includes
#include <iostream>
#include <cstring>
#include <cerrno>

// networking includes
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

// https://docs.kernel.org/networking/can.html

bool comms::CANDriver::init()
{
    auto canbus_device = get_parameter_value<std::string>("canbus_device");
    auto dbc_file_path = get_parameter_value<std::string>("path_to_dbc");
    if (!(canbus_device && dbc_file_path))
    {
        return false;
    }
    std::unique_ptr<dbcppp::INetwork> net;
    {
        std::ifstream idbc((*dbc_file_path).c_str());
        net = dbcppp::INetwork::LoadDBCFromIs(idbc);
    }

    for (const auto &msg : net->Messages())
    {
        _messages.insert(std::make_pair(msg.Id(), msg.Clone()));
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

void comms::CANDriver::handle_output_msg_from_queue(core::common::ThreadSafeDeque<google::protobuf::Any> &output_deque)
{
}

// Function to set fields dynamically using reflection
void comms::CANDriver::set_field_values(google::protobuf::Message* message, const std::unordered_map<std::string, float>& field_values) {
    const google::protobuf::Descriptor* descriptor = message->GetDescriptor();
    const google::protobuf::Reflection* reflection = message->GetReflection();

    for (int i = 0; i < descriptor->field_count(); ++i) {
        const google::protobuf::FieldDescriptor* field = descriptor->field(i);

        // Get the field name and check if it exists in the provided field_values map
        std::string field_name = field->name();
        auto it = field_values.find(field_name);

        if (it != field_values.end()) {
            // Set field values based on field type
            if (field->type() == google::protobuf::FieldDescriptor::TYPE_FLOAT && !field->is_repeated()) {
                reflection->SetFloat(message, field, it->second);
                std::cout << "Set float field: " << field_name << " = " << it->second << std::endl;
            } 
            // You can add more types (int, string, etc.) here as needed
            else {
                std::cerr << "Unsupported field type for field: " << field_name << std::endl;
            }
        }
    }
}

google::protobuf::Any comms::CANDriver::_get_pb_msg_by_name(const std::string &msg_name)
{
    google::protobuf::Any any_message;
    const google::protobuf::Descriptor *descriptor = google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName(msg_name);

    // Create a dynamic message factory
    google::protobuf::DynamicMessageFactory dynamic_factory;
    const google::protobuf::Message *prototype_message = dynamic_factory.GetPrototype(descriptor);
    if (!prototype_message)
    {
        std::cerr << "Failed to create prototype message for type: " << msg_name << std::endl;
        return {};
    }
    // Create a new message instance from the prototype
    google::protobuf::Message *message = prototype_message->New();

    // Optionally set fields on the message here if needed

    // Pack the message into the Any object
    if (!any_message.PackFrom(*message))
    {
        std::cerr << "Failed to pack message into Any" << std::endl;
    }

    // Clean up
    delete message;
    return any_message;
}

can_frame comms::CANDriver::_get_CAN_msg(const google::protobuf::Any &msg)
{
    // - [ ] get an associated CAN message based on the name of the input message

    // Get the type name
    std::string type_url = msg.type_url();
    std::string messageTypeName = type_url.substr(type_url.find_last_of('/') + 1);

    // Get a Descriptor for the message type
    const google::protobuf::Descriptor *descriptor = google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName(messageTypeName);

    if (descriptor)
    {
        // Use a DynamicMessageFactory to create a new message instance of the appropriate type
        google::protobuf::DynamicMessageFactory factory;
        std::unique_ptr<google::protobuf::Message> message(factory.GetPrototype(descriptor)->New());

        // Unpack the message
        if (msg.UnpackTo(message.get()))
        {
            std::cout << "Successfully unpacked message of type: " << messageTypeName << std::endl;
            // Do something with the message...
        }
    }
    else
    {
        std::cerr << "Unknown message type: " << messageTypeName << std::endl;
    }

    // - [ ] encode it into the can_frame struct
    return {};
}

void comms::CANDriver::handle_send_msg_from_queue(core::common::ThreadSafeDeque<google::protobuf::Any> &input_deque)
{
    // we will assume that this queue only has messages that we want to send
    core::common::ThreadSafeDeque<google::protobuf::Any> q;
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
