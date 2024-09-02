#pragma once

#include <array>
#include <memory>
#include <thread>
#include <chrono>
// for setting the priority of the thread to be as high as possible
#include <pthread.h>
#include <sched.h>


#include <Controller.hpp>
#include <Configurable.hpp>
#include <Utils.hpp>
#include <DriverBus.hpp>

// requirements:
// - shall handle ability to be constructed to contain the controllers,
//   handle providing them the inputs from state estimation and / or drivers

// - controller manager shall not affect the output of any controller.

// - each controller shall be able to handle re-configuration at runtime,
//   even while being the "active" controller, however the controller manager
//   shall handle when its configuration gets updated

// - vehicle manager shall handle stepping of each controller at their
//   desired rates and be able to switch between

// - vehicle manager shall be able to change the mode of the drivetrain dependent
//   on what mode the desired controller wants the drivetrain to be in (speed / torque)

// TODOs:
// - [ ] implement handling for switching between controllers
    // - [ ] add in queue handling for the vehicle manager
// - [ ] implement ability to switch between controllers

// What I want to be able to do with the vehicle manager is be general about what controllers we are swapping between and evaluating during runtime.
// each controller (for now) will be stack allocated, however they wont always be running at the same time. We also want the ability to pause 
// evaluation of all controllers, but this isnt something that we have to have at the start. 

// We also want to be able to expose an interface to the user to use GRPC to interact with this 
// vehicle manager to allow for control pieces of our car, like say run specific routines like 
// start, stop lap time recording or set start location of lap timing. This, however, probably 
// wouldnt be a part of the vehicle manager itself.

// For the sake of the controllers, I want to have a direct connection between the state estimator and the Controller Manager
// The controller manager will sample the state estimator (there will be a single state that the state estimator updates internally).
    // on a side note, we need to make sure that we copy over the vehicle state here after locking onto the mutex and then unlock
        // on a side note, how are we going to prevent mutex deadlocking on the shared state?  
namespace control
{

    struct DriverInput
    {
        float requested_accel;
        float requested_brake;
    };
    // this state is not passed into the controllers, this is just used to evaluate whether or not we can switch between controllers.
    struct VehicleSafetyState
    {
        bool is_ready_to_driver;
        DriverInput input;
    };

    struct SpeedControlOut
    {
        veh_vec<float> desired_rpms;
        veh_vec<float> positive_torque_lim_nm;
        veh_vec<float> negative_torque_lim_nm;
    };

    struct TorqueControlOut
    {
        veh_vec<float> desired_torques_nm;
    };
    
    template <size_t NumControllers, typename ControllerOutput, typename VehicleState>
    class ControllerManager : public core::common::Configurable
    {
        using output_deq_type = std::shared_ptr<core::common::ThreadSafeDeque<ControllerOutput>>;
        // using vehicle_manager_cmd_queue = std::shared_ptr<core::common::ThreadSafeDeque<>>;
    public:
        
        ControllerManager(core::JsonFileHandler &json_file_handler,

            std::array<control::Controller *>, NumControllers > controllers, 
            input_deq_type controller_input, 
            output_deq_type controller_output,
            ) :
                Configurable(json_file_handler, "ControllerManager"),
                _controllers(controllers),
                _input_queue(controller_input),
                _output_queue(controller_output)
                 {}        
        ~ControllerManager() = default;
        

        void process_commands_thread() {
            while(!_stop_threads)
            {
                
            }
        }

        void run() {
            while(!_stop_threads)
            {
                auto start_time = std::chrono::high_resolution_clock::now();

            }
        }

        
    private:
        bool _can_switch_controller(const VehicleState& current_state)
    private:
        input_deq_type _input_queue;
        output_deq_type _output_queue;
        std::array<control::Controller *>, NumControllers > _controllers;
        bool _stop_threads = false;
    };
}