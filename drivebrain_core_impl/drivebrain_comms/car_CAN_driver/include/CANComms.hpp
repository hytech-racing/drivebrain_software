#pragma once
#include <Configurable.hpp>
#include <DriverBus.hpp>
#include <deque>

#include <linux/can.h>
#include <linux/can/raw.h>

// TODO - [ ] be able to hook into the driver bus tx and tx queues

// https://docs.kernel.org/networking/can.html

namespace comms
{
    class CANDriver : public core::common::Configurable
    {
    public:
        // using common::Configurable::Configurable;
        CANDriver(core::JsonFileHandler &json_file_handler) : Configurable(json_file_handler, "CANDriver") {}

        bool init();

        template <typename MsgType>
        void handle_send_msg_from_queue(core::common::ThreadSafeDeque<MsgType> &input_deque);
        
        template <typename MsgType>
        void handle_output_msg_from_queue(core::common::ThreadSafeDeque<MsgType> &output_deque);

    private:
        template <typename MsgType>
        can_frame _get_CAN_msg(const MsgType &msg);
    private:
        int _CAN_socket; // can socket bound to
    };
}
