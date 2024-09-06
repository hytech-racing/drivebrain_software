#ifndef __STATEESTIMATOR_H__
#define __STATEESTIMATOR_H__

// TODO:
// - [ ] implement the CAN driver connection that can create the flat state of the car

// implement a thing that can maintain a "flexible" state of the car such that
// it can build up a state of the car from the messages coming in. It starts out with an empty
// state schema from the protobuf messages. all messages are assumed flat.

// - if a new message containing state information comes it (that has not arrived before), the state is "appended" to with the new value
// - if a new message containing state information comes in that has been seen before, the previous state is over-written.
//   each state update will have with it a timestamp

#include <DriverBus.hpp>
#include <VehicleDataTypes.hpp>

namespace core
{

    class StateEstimator
    {
    public:
        StateEstimator(core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> &msg_input_queue) : _msg_in_queue(msg_input_queue)
        {
            _vehicle_state = {};
        }
        ~StateEstimator()
        {
            
        }
        
        void start_recv_thread();
        core::VehicleState get_latest_state();

    private:
        std::thread _recv_thread;
        core::VehicleState _vehicle_state;
        using namespace core::common;
        ThreadSafeDeque<MsgType> _msg_in_queue;
    }
}

#endif // __STATEESTIMATOR_H__