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
// TODO - [ ] be able to hook into the driver bus tx and tx queues

// https://docs.kernel.org/networking/can.html

namespace comms
{
    class CANDriver : public core::common::Configurable
    {
    public:
        CANDriver(core::JsonFileHandler &json_file_handler) : Configurable(json_file_handler, "CANDriver") {}

        bool init();

        void handle_send_msg_from_queue(core::common::ThreadSafeDeque<google::protobuf::Any> &input_deque);

        void handle_output_msg_from_queue(core::common::ThreadSafeDeque<google::protobuf::Any> &output_deque);


        void set_field_values(google::protobuf::Message* message, const std::unordered_map<std::string, float>& field_values);
        google::protobuf::Any _get_pb_msg_by_name(const std::string &msg_name);
    
    private:
        can_frame _get_CAN_msg(const google::protobuf::Any &msg);
        
    private:
        
        std::unordered_map<uint64_t, std::unique_ptr<dbcppp::IMessage>> _messages;
        int _CAN_socket; // can socket bound to
    };
}
