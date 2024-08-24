#pragma once
#include <Configurable.hpp>
#include <DriverBus.hpp>
#include <deque>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <google/protobuf/any.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/dynamic_message.h>

#include <hytech.pb.h>

#include <Signal.h>
#include <Network.h>
#include <memory>
#include <variant>
// TODO
// - [ ] be able to hook into the driver bus tx and tx queues
// - [ ] implement functions to be able to create a CAN message from the protobuf message and output it to the CAN bus
// - [ ] needs to be able to take in an abstract pb message
// - [ ] get all fields and the message name
// - [ ] for each field get their values
// - [ ] align the message name and message fields to a CAN message
// - [ ] populate the fields of the CAN message with the data gotten from the pb message
// - [ ] implement functions to be able to deserialize a CAN message from the bus and create a protobuf message from the result

// https://docs.kernel.org/networking/can.html

namespace comms
{
    class CANDriver : public core::common::Configurable
    {
    public:
        using FieldVariant = std::variant<int32_t, int64_t, uint32_t, uint64_t, float, double, bool, std::string, std::monostate>;

        CANDriver(core::JsonFileHandler &json_file_handler) : Configurable(json_file_handler, "CANDriver") {}
        bool init();
        void handle_send_msg_from_queue(core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> &input_deque);
        void handle_output_msg_from_queue(core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> &output_deque);
        void set_field_values_of_pb_msg(std::shared_ptr<google::protobuf::Message> message, const std::unordered_map<std::string, std::variant<float, bool, double, int, uint32_t, std::string>> &field_values);

        FieldVariant get_field_value(std::shared_ptr<google::protobuf::Message> message, const std::string& field_name);

        std::shared_ptr<google::protobuf::Message> _get_pb_msg_by_name(const std::string &msg_name);

        // private:
        can_frame _get_CAN_msg(std::shared_ptr<google::protobuf::Message> msg);

    private:
        std::unordered_map<uint64_t, std::unique_ptr<dbcppp::IMessage>> _messages;
        std::unordered_map<std::string, uint64_t> _messages_names_and_ids;
        int _CAN_socket; // can socket bound to
    };
}
