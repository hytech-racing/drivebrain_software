#include <CANComms.hpp>

#include <iostream>
#include <cstring>
#include <cerrno>

#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

// https://docs.kernel.org/networking/can.html

bool comms::CANDriver::init()
{
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    
    _CAN_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (_CAN_socket < 0) {
        std::cerr << "Error while opening socket: " << strerror(errno) << std::endl;
        return false;
    }
    
    auto canbus_device = get_parameter_value<std::string>("canbus_device");
    
    if(canbus_device)
    {
        std::strcpy(ifr.ifr_name, (*canbus_device).c_str());
    } else {
        return false;
    }
    ioctl(_CAN_socket, SIOCGIFINDEX, &ifr); // get the interface index for can0

    // Bind the socket to the can0 interface
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(_CAN_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Error in socket bind: " << strerror(errno) << std::endl;
        return false;
    }

    std::cout << "Listening on can0..." << std::endl;
    return true;
}

template<typename MsgType>
void comms::CANDriver::handle_output_msg_from_queue(core::common::ThreadSafeDeque<MsgType>& output_deque)
{

}

template <typename MsgType>
can_frame comms::CANDriver::_get_CAN_msg(const MsgType& msg)
{
    
}

template<typename MsgType>
void comms::CANDriver::handle_send_msg_from_queue(core::common::ThreadSafeDeque<MsgType>& input_deque)
{
    // we will assume that this queue only has messages that we want to send    
    core::common::ThreadSafeDeque<MsgType> q;
    {
        std::unique_lock lk(input_deque.mtx);

        if(input_deque.deque.empty() )
        {
            return;
        }

        q.deque = input_deque.deque;
        input_deque.deque.clear();
    }

    for (const auto & msg : q.deque)
    {
        // TODO send the messages
        auto can_msg = _get_CAN_msg(msg);
        
    }

}




