#ifndef __STATEESTIMATOR_H__
#define __STATEESTIMATOR_H__

// TODO:
// - [ ] implement the CAN driver connection that can help create the internal state of the car from the data coming in from the CAN bus

// implement a thing that can maintain a "flexible" state of the car such that
// it can build up a state of the car from the messages coming in. It starts out with an empty
// state schema from the protobuf messages. all messages are assumed flat.

// - if a new message containing state information comes it (that has not arrived before), the state is "appended" to with the new value
// - if a new message containing state information comes in that has been seen before, the previous state is over-written.
//   each state update will have with it a timestamp

#include <mutex>
#include <thread>
#include <utility>
#include <chrono>
#include <memory>

#include <hytech.pb.h>

// protobuf
#include <google/protobuf/any.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/dynamic_message.h>

#include <DriverBus.hpp>
#include <VehicleDataTypes.hpp>
#include <Logger.hpp>

// while we can just have one queue input, if we allowed for multiple queue inputs that each have their own threads
// that can update pieces of the state that would be optimal.

// TODO:
// - [ ] write tests for the timestamp checking / verification of the state data
// - [ ] implement the ability to kick off threads for a vector of input queues
namespace core
{
    class StateEstimator
    {
    public:
        using tsq = core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>>;
        
        
        StateEstimator(core::Logger &shared_logger) : _logger(shared_logger)
        {
            _vehicle_state = {};
            // initialize the 3 state variables to have a zero timestamp
            std::chrono::microseconds zero_start_time{0};
            _timestamp_array = {zero_start_time, zero_start_time, zero_start_time};
        }
        ~StateEstimator()  =default;
        void handle_recv_process(std::shared_ptr<google::protobuf::Message> message);
        std::pair<core::VehicleState, bool> get_latest_state_and_validity();

    private:
        template <size_t arr_len>
        bool _validate_stamps(const std::array<std::chrono::microseconds, arr_len> &timestamp_arr);

    private:
        core::Logger &_logger;

        bool _run_recv_threads = false;
        std::mutex _state_mutex;
        core::VehicleState _vehicle_state;
        std::array<std::chrono::microseconds, 3> _timestamp_array;

    };
}

#endif // __STATEESTIMATOR_H__