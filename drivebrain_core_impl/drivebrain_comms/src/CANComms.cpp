// this
#include <CANComms.hpp>

// standard includes
#include <cctype>
#include <cerrno>
#include <cstring>
#include <iostream>
// networking includes
#include <fcntl.h>
#include <filesystem>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <variant>
// logging includes
#include "spdlog/spdlog.h"

// https://docs.kernel.org/networking/can.html

std::string comms::CANDriver::_to_lowercase(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return std::tolower(c); } // correct
    );
    return s;
}
comms::CANDriver::~CANDriver() {
    _running = false;
    _input_deque_ref.cv.notify_all();
    _output_thread.join();
}
bool comms::CANDriver::init() {
    auto canbus_device = get_parameter_value<std::string>("canbus_device");

    auto dbc_file_path = _dbc_path ? _dbc_path : get_parameter_value<std::string>("path_to_dbc");


    if (!(canbus_device && dbc_file_path)) {
        _logger.log_string("couldnt get params", core::LogLevel::ERROR);
        return false;
    } else if (!std::filesystem::exists(*dbc_file_path)) {
        std::string msg("params file does not exist! ");
        msg += " ";
        msg += (*dbc_file_path);
        _logger.log_string(msg, core::LogLevel::ERROR);
        return false;
    }
    std::shared_ptr<dbcppp::INetwork> net;
    {
        std::ifstream idbc((*dbc_file_path).c_str());
        net = dbcppp::INetwork::LoadDBCFromIs(idbc);
    }

    for (const auto &msg : net->Messages()) {
        _messages.insert(std::make_pair(msg.Id(), msg.Clone()));
        _messages_names_and_ids.insert(std::make_pair(_to_lowercase(msg.Name()), msg.Id()));
    }

    if (!_open_socket(*canbus_device)) {
        _logger.log_string("couldnt open socket", core::LogLevel::ERROR);
        return false;
    }

    _logger.log_string("inited, started read", core::LogLevel::INFO);

    _do_read();
    _configured = true;
    return true;
}

bool comms::CANDriver::_open_socket(const std::string &interface_name) {
    int raw_socket = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (raw_socket < 0) {
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

    if (::bind(raw_socket, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
        auto err_str = std::string("Error binding CAN socket: ") + std::string(strerror(errno));
        _logger.log_string(err_str.c_str(), core::LogLevel::ERROR);

        ::close(raw_socket);
        return false;
    }

    _socket.assign(raw_socket); // Assign the native socket to Boost.Asio descriptor
    return true;
}

void comms::CANDriver::_do_read() {
    boost::asio::async_read(_socket, boost::asio::buffer(&_frame, sizeof(_frame)),
                            [this](boost::system::error_code ec, std::size_t bytes_transferred) {
                                if (!ec && bytes_transferred == sizeof(_frame)) {
                                    _handle_recv_CAN_frame(_frame);
                                    _do_read(); // Continue reading for the next frame
                                } else if (ec) {
                                    spdlog::error("Error receiving CAN message: {}", ec.message());
                                }
                            });
}

void comms::CANDriver::_send_message(const struct can_frame &frame) {
    boost::asio::async_write(
        _socket, boost::asio::buffer(&frame, sizeof(frame)),
        [this](boost::system::error_code ec, std::size_t /*bytes_transferred*/) {
            if (ec) {
                spdlog::error("Error sending CAN message: {}", ec.message());
            }
        });
}

void comms::CANDriver::_handle_recv_CAN_frame(const struct can_frame &frame) {
    auto msg = pb_msg_recv(frame);
    if (msg) {
        _state_estimator.handle_recv_process(msg);
        _message_logger->log_msg(msg);
    }
}

// gets a protobuf message from just the name of it
std::shared_ptr<google::protobuf::Message>
comms::CANDriver::_get_pb_msg_by_name(const std::string &name) {
    // Create a dynamic message factory
    std::shared_ptr<google::protobuf::Message> prototype_message;

    const google::protobuf::Descriptor *desc = google::protobuf::DescriptorPool::generated_pool()->FindMessageTypeByName("hytech." + name);
    if (!desc) {
        spdlog::error("Prototype message does not exist in descriptor pool");
        return nullptr;
    }
    prototype_message.reset(google::protobuf::MessageFactory::generated_factory()->GetPrototype(desc)->New());
    if (!prototype_message)
    {
        spdlog::error("Failed to create prototype message");
        return nullptr;
    }
    return prototype_message;
}

// Function to set fields dynamically using reflection
void comms::CANDriver::set_field_values_of_pb_msg(
    const std::unordered_map<std::string, comms::CANDriver::FieldVariant> &field_values,
    std::shared_ptr<google::protobuf::Message> message) {
    const google::protobuf::Descriptor *descriptor = message->GetDescriptor();
    const google::protobuf::Reflection *reflection = message->GetReflection();

    for (int i = 0; i < descriptor->field_count(); ++i) {
        const google::protobuf::FieldDescriptor *field = descriptor->field(i);

        // Get the field name and check if it exists in the provided field_values
        // map
        std::string field_name = field->name();
        auto it = field_values.find(field_name);

        if (it != field_values.end()) {
            // Set field values based on field type
            if (field->is_repeated()) {
                spdlog::warn("Unsupported field type for field: {}", field_name);
                continue;
            }
            
            switch (field->type()) 
            {
            case google::protobuf::FieldDescriptor::TYPE_ENUM:
                
                reflection->SetEnumValue(message.get(), field, (int)std::get<int32_t>(it->second));
                
                break;
            case google::protobuf::FieldDescriptor::TYPE_FLOAT:
                
                if (std::holds_alternative<double>(it->second))
                {
                    reflection->SetFloat(message.get(), field, (float)std::get<double>(it->second));    
                    
                } else if (std::holds_alternative<int32_t>(it->second))
                {
                    reflection->SetFloat(message.get(), field, (float)std::get<int32_t>(it->second));    
                    
                } else {
                    spdlog::warn("unable to parse float for field name {}. unknown field variant type", field_name);
                }
                break;
            case google::protobuf::FieldDescriptor::TYPE_BOOL:
                
                if (std::holds_alternative<double>(it->second))
                {
                    reflection->SetBool(message.get(), field, (bool)std::get<double>(it->second));    
                    
                } else if (std::holds_alternative<int32_t>(it->second))
                {
                    reflection->SetBool(message.get(), field, (bool)std::get<int32_t>(it->second));    
                    
                } else {
                    spdlog::warn("unable to parse bool for field name {}. unknown field variant type", field_name);
                }
                
                
                break;
            case google::protobuf::FieldDescriptor::TYPE_DOUBLE:
                
                reflection->SetDouble(message.get(), field, std::get<double>(it->second));
                
                break;
            case google::protobuf::FieldDescriptor::TYPE_INT32:
                
                reflection->SetInt32(message.get(), field, (int32_t)std::get<double>(it->second));
                
                break;
            default:
                break;
            }
        }
    }
}

std::shared_ptr<google::protobuf::Message> comms::CANDriver::pb_msg_recv(const can_frame &frame) {
    auto iter = _messages.find(frame.can_id);
    if (iter != _messages.end())
    {
        auto msg = iter->second->Clone();
        auto msg_to_populate = _get_pb_msg_by_name(_to_lowercase(msg->Name()));
        if (!msg_to_populate) {
            return nullptr;
        }

        std::unordered_map<std::string, comms::CANDriver::FieldVariant> msg_field_map;
        for (const dbcppp::ISignal &sig : msg->Signals()) {
            const dbcppp::ISignal *mux_sig = msg->MuxSignal();

            if (sig.MultiplexerIndicator() != dbcppp::ISignal::EMultiplexer::MuxValue ||
                (mux_sig && mux_sig->Decode(frame.data) == sig.MultiplexerSwitchValue()))
            {
                auto raw_value = sig.Decode(frame.data);

                // Check if the signal has enum descriptions (ValueEncodingDescriptions)
                if (sig.ValueEncodingDescriptions_Size() > 1)
                {
                    // Enum signal: Cast the decoded raw value to an integer and store it
                    msg_field_map[sig.Name()] = static_cast<int>(raw_value);
                }
                else {
                    // TODO get correct type from raw signal and store it in the map. right now they are all doubles
                    msg_field_map[sig.Name()] = sig.RawToPhys(raw_value);
                }

            }
        } 
        set_field_values_of_pb_msg(msg_field_map, msg_to_populate);
        return msg_to_populate;
    }

    return nullptr;
}

comms::CANDriver::FieldVariant
comms::CANDriver::get_field_value(std::shared_ptr<google::protobuf::Message> message,
                                  const std::string &field_name) {
    if (!message) {
        spdlog::error("Message is null");
        return std::monostate{};
    }

    const google::protobuf::Descriptor *descriptor = message->GetDescriptor();
    const google::protobuf::Reflection *reflection = message->GetReflection();
    const google::protobuf::FieldDescriptor *field = descriptor->FindFieldByName(field_name);

    if (field == nullptr) {
        spdlog::warn("Field not found: {}", field_name);
        return std::monostate{};
    }

    // Retrieve the value based on the field type
    switch (field->cpp_type()) {
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
        spdlog::warn("Nested messages not supported");
        return std::monostate{};
    default:
        spdlog::warn("Unsupported field type");
        return std::monostate{};
    }
}

std::optional<can_frame>
comms::CANDriver::_get_CAN_msg(std::shared_ptr<google::protobuf::Message> pb_msg) {
    // - [x] get an associated CAN message based on the name of the input message
    can_frame frame{};
    std::string type_url = pb_msg->GetTypeName();
    std::string messageTypeName = type_url.substr(type_url.find_last_of('.') + 1);

    if (_messages_names_and_ids.find(messageTypeName) != _messages_names_and_ids.end())
    {
        auto id = _messages_names_and_ids[messageTypeName];
        auto msg = _messages[id]->Clone();
        frame.can_id = id;
        frame.len = msg->MessageSize();
        for (const auto &sig : msg->Signals())
        {
            auto field_value = get_field_value(pb_msg, sig.Name());

            std::visit([&sig, &frame](const FieldVariant &arg)
                       {
            if (std::holds_alternative<std::monostate>(arg)) {
                spdlog::info("No value found or unsupported field");
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
            
            } else if (std::holds_alternative<std::string>(arg)){
                auto enum_name = std::get<std::string>(arg);
                bool found = false;

                // iterating to find correct value
                for (const auto &enc : sig.ValueEncodingDescriptions())
                {
                    if (enc.Description() == enum_name)
                    {
                        sig.Encode(enc.Value(), frame.data);
                        found = true;
                        break;
                    }
                }

                if (!found)
                {
                    spdlog::info("enum not found");
                }
            } else {
                spdlog::info("uh not supported yet");
            } },
                       field_value);
        }
    } else {
        spdlog::warn("WARNING: not creating a frame to send due to not finding frame name");
        return std::nullopt;
    }

    return frame;
}

void comms::CANDriver::_handle_send_msg_from_queue() {
    // we will assume that this queue only has messages that we want to send
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> q;
    while (_running) {
        {
            std::unique_lock lk(_input_deque_ref.mtx);
            // TODO unfuck this, queue management shouldnt live within the queue
            // itself
            _input_deque_ref.cv.wait(
                lk, [this]() { return !_input_deque_ref.deque.empty() || !_running; });

            if (_input_deque_ref.deque.empty()) {
                return;
            }

            q.deque = _input_deque_ref.deque;
            _input_deque_ref.deque.clear();
        }

        for (const auto &msg : q.deque)
        {
            auto can_msg = _get_CAN_msg(msg);
            if (can_msg)
            {
                _send_message(*can_msg);
                _message_logger->log_msg(msg);
            }
        }
        q.deque.clear();
    }
}