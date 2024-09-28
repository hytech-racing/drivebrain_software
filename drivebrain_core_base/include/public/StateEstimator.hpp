#ifndef __STATEESTIMATOR_H__
#define __STATEESTIMATOR_H__

// TODO:
// - [x] implement the CAN driver connection that can help create the internal state of the car from the data coming in from the CAN bus
// - [x] implement the CAN driver connection that can help create the internal state of the car from the data coming in from the CAN bus

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
#include <Configurable.hpp>

// TODO:
// - [ ] write tests for the timestamp checking / verification of the state data
// - [ ] implement the ability to kick off threads for a vector of input queues
namespace core
{
    class StateEstimator : public common::Configurable
    class StateEstimator : public common::Configurable
    {
    public:
        struct config {
            int threshold_microseconds;
        };

        StateEstimator(core::JsonFileHandler &json_file_handler, core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> &msg_input_queue) : 
        Configurable(json_file_handler, "StateEstimator"),
        _msg_in_queue(msg_input_queue)
        {
            _vehicle_state = {};
            _start_recv_thread();
            std::chrono::microseconds zero_start_time {0}; 
            _timestamp_array = { zero_start_time, zero_start_time, zero_start_time};
            (void)init();
        }
        ~StateEstimator()
        {
           _run_recv_thread =false;
           _recv_thread.join(); 
        }

        bool init();
        
        std::pair<core::VehicleState, bool> get_latest_state_and_validity();

    private:
        template <size_t arr_len>
        bool _validate_stamps(const std::array<std::chrono::microseconds, arr_len> &timestamp_arr);

    private:
        core::Logger &_logger;
        config _config;
        bool _run_recv_threads = false;
        std::mutex _state_mutex;
        core::VehicleState _vehicle_state;
        std::array<std::chrono::microseconds, 3> _timestamp_array;
        common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>>& _msg_in_queue;
        config _config; 
    };
}

#endif // __STATEESTIMATOR_H__