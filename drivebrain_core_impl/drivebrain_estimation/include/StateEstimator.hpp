#ifndef __STATEESTIMATOR_H__
#define __STATEESTIMATOR_H__

// TODO:
// - [x] implement the CAN driver connection that can help create the internal state of the car from the data coming in from the CAN bus

// implement a thing that can maintain a "flexible" state of the car such that
// it can build up a state of the car from the messages coming in. It starts out with an empty
// state schema from the protobuf messages. all messages are assumed flat.

// - if a new message containing state information comes it (that has not arrived before), the state is "appended" to with the new value
// - if a new message containing state information comes in that has been seen before, the previous state is over-written.
//   each state update will have with it a timestamp
#include <Loggable.hpp>
#include <Configurable.hpp>
#include <DriverBus.hpp>
#include <VehicleDataTypes.hpp>

#include <mutex>
#include <thread>
#include <utility>
#include <chrono>
#include <memory>

#include "hytech_msgs.pb.h"
#include "base_msgs.pb.h"

// protobuf
#include <google/protobuf/any.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/dynamic_message.h>



// while we can just have one queue input, if we allowed for multiple queue inputs that each have their own threads
// that can update pieces of the state that would be optimal.

// TODO:
// - [ ] write tests for the timestamp checking / verification of the state data

// user story:
// i want the ability to add in new estimation components by composition or construction
    // how will we know what the estimator is changing / adding as far as state variables? -> this will get annoying 
// i dont want to have to change code in here every time we add a new state variable / data derived 
// from the raw sensor data input

// new ideas: 
// for a more generic state estimator we can template the class based on the raw input data struct 
// and the vehicle state struct. 

// for now i will just move the state estimator into the estimation impl and call it a day 
namespace core
{
    class StateEstimator : public core::common::Loggable<std::shared_ptr<google::protobuf::Message>>,
                           public core::common::Configurable
    {
        struct config
        {
            float fl_sus_pot_min;
            float fl_sus_pot_min_mm;
            float fl_sus_pot_max;
            float fl_sus_pot_max_mm;
            float fr_sus_pot_min;
            float fr_sus_pot_min_mm;
            float fr_sus_pot_max;
            float fr_sus_pot_max_mm;
            float rl_sus_pot_min;
            float rl_sus_pot_min_mm;
            float rl_sus_pot_max;
            float rl_sus_pot_max_mm;
            float rr_sus_pot_min;
            float rr_sus_pot_min_mm;
            float rr_sus_pot_max;
            float rr_sus_pot_max_mm;
            float fl_load_cell_offset;
            float fl_load_cell_scale;
            float fr_load_cell_offset;
            float fr_load_cell_scale;
            float rl_load_cell_offset;
            float rl_load_cell_scale;
            float rr_load_cell_offset;
            float rr_load_cell_scale;
        } _config;

    using loggertype = core::MsgLogger<std::shared_ptr<google::protobuf::Message>>;

    public:
        using tsq = core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>>;
        StateEstimator(core::JsonFileHandler &json_file_handler) : 
        
        Configurable(json_file_handler, "StateEstimator")
        {
            _vehicle_state = {}; // initialize to all zeros
            _raw_input_data = {};
            _vehicle_state.state_is_valid = true;
            _vehicle_state.prev_MCU_recv_millis = -1; // init the last mcu recv millis to < 0
            // initialize the 3 state variables to have a zero timestamp
            std::chrono::microseconds zero_start_time{0};
            _timestamp_array = {zero_start_time, zero_start_time, zero_start_time, zero_start_time};
        }
        ~StateEstimator();

        void handle_recv_process(std::shared_ptr<google::protobuf::Message> message);
        std::pair<core::VehicleState, bool> get_latest_state_and_validity();
        void set_previous_control_output(ControllerOutput prev_control_output);
        core::VehicleState append_state_variables_from_raw_inputs(core::VehicleState vs, core::RawInputData raw_inputs);
        bool init() override final;


    private:
        void _recv_low_level_state(std::shared_ptr<google::protobuf::Message> message);
        void _recv_inverter_states(std::shared_ptr<google::protobuf::Message> msg);

        template <size_t ind, typename inverter_dynamics_msg>
        void _handle_set_inverter_dynamics(std::shared_ptr<google::protobuf::Message> msg);

        template <size_t ind, typename inverter_temps_msg>
        void _handle_set_inverter_temps(std::shared_ptr<google::protobuf::Message> msg);

        std::shared_ptr<hytech_msgs::VehicleData> _set_ins_state_data(core::VehicleState current_state, std::shared_ptr<hytech_msgs::VehicleData> msg_out);

        template <size_t arr_len>
        bool _validate_stamps(const std::array<std::chrono::microseconds, arr_len> &timestamp_arr);
        std::shared_ptr<hytech_msgs::VehicleData> _set_computed_states(core::VehicleState current_state, std::shared_ptr<hytech_msgs::VehicleData> msg_out);

        void _set_float_veh_vec_message_member(veh_vec<float> from, hytech_msgs::veh_vec_float * to_set);
    private:

        bool _run_recv_threads = false;
        std::mutex _state_mutex;
        core::VehicleState _vehicle_state;
        core::RawInputData _raw_input_data;
        std::array<std::chrono::microseconds, 4> _timestamp_array;
        
        std::shared_ptr<loggertype> _message_logger;

        std::chrono::seconds _last_debug_veh_state_print;
        std::chrono::microseconds _debug_maxtime_diff{-1};
        std::chrono::microseconds _debug_maxjitter_diff{-1};
    };
}

#endif // __STATEESTIMATOR_H__