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
#include <VehicleManager.hpp>
#include <VehicleDataTypes.hpp>

// will be based on: https://github.com/hytech-racing/MCU/blob/adc599c9a2a3d1afe4ee22fcad0fd1116c474500/lib/systems/include/TorqueControllerMux.h

// TODO look into sharing tc mux between MCU and drivebrain software


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
            // we will do this by having the state estimator and the controller ticking in the same loop so no need for worrying about threading


namespace control
{



    template <size_t NumControllers, typename VehicleState>
    class ControllerManager : public core::common::Configurable
    {
    public:
        
        enum class ControllerManagerStatus
        {
            NO_ERROR =0,
            ERROR_CONTROLLER_INDEX_OUT_OF_RANGE=1,
            ERROR_SPEED_DIFF_TOO_HIGH=2,
            ERROR_TORQUE_DIFF_TOO_HIGH=3,
            ERROR_DRIVER_ON_PEDAL=3 
        };

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
        
        bool attempt_controller_change(size_t new_controller_index)
        {
            static const size_t num_controllers = NumControllers; 
            if(new_controller_index > (num_controllers-1))
            {
                return false;
            } else {
                return true;
            }
        }

        float get_active_controller_timestep()
        {
            return _controllers[_current_controller_index]->get_dt_sec();
        }



        ControllerOutput step_active_controller(const core::ControllerInput& input) {
            return _controllers[_current_controller_index]->step_controller(input);
        }

        
    private:
        
        ControllerManagerStatus _can_switch_controller(const VehicleState& current_state, const ControllerOutput& previous_output, const ControllerOutput& next_controller_output);

    private:
        std::array<control::Controller *>, NumControllers > _controllers;
        size_t _current_controller_index = 0;
    };
}
#include "TorqueControllerMux.tpp"
