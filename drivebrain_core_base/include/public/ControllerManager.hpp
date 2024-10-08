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
// - [ ] implement handling for switching between controllers -> yoink logic from new tcmux
// - [ ] implement function to pass in vehicle state to the active controller

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

// TODO: remove vehicle state input param from step or find another way to get the vehicle state into switch controller
// probably do one of the other -> pass it in to the method or pass it in to the constructor, both are implemented right now

namespace control
{

    // TODO turn into concept
    template <typename ControllerType, size_t NumControllers>
    class ControllerManager : public core::common::Configurable
    {
    public:
        /// @brief contructs instance of the controller manager
        /// @param json_file_handler current file handler to handle controller configurations
        /// @param controllers list of controllers that the manager will mux between and manager
        /// @param state_estimator instance to allow for direct communication between controllers and state estimator
        ControllerManager(core::JsonFileHandler &json_file_handler, std::array<ControllerType *, NumControllers> controllers, core::StateEstimator state_estimator) : Configurable(json_file_handler, "ControllerManager"),
                                                                                                                                                                        _controllers(controllers),
                                                                                                                                                                        _state_estimator(state_estimator)

        {
        }
        ~ControllerManager() = default;

        /// @brief configurable required init function
        /// @return true or false depending on success of init
        bool init();

        /// @brief attempts to switch the active controller
        /// @param new_controller_index desired controller index
        /// @return true if it successfully switches and false if it does not
        /// @note if it returns false the _current_state member variable will have an altered status variable 
        bool swap_active_controller(size_t new_controller_index);
        
        /// @brief 
        /// @return 
        float get_active_controller_timestep()
        {
            return _controllers[_current_controller_index]->get_dt_sec();
        }

        /// @brief allows access to controller manager state for efficient communication
        /// @return ControllerManagerState types: ControllerManagerStatus, ControllerOutput
        core::control::ControllerManagerState get_current_state()
        {
            return _current_state;
        }

        /// @brief evaluates the currently active controller 
        /// @param input current vehicle state maintained by the state estimator
        /// @return respective controller output to command the drivetrain
        core::ControllerOutput step_active_controller(const core::VehicleState& input)
        {   
            if(stepping){
                _current_state.current_controller_output = _controllers[_current_controller_index]->step_controller(input)
            }
            return _current_state.current_controller_output;
        }

        /// @brief stops the active controller from being evaluated at its time step
        /// @return true if it successfully stops the stepping and false if it was already stopped
        bool pause_stepping()
        {
            if(stepping)
            {
                stepping = false;
                return true;
            }
            else
            {
                //if you pause while its not stepping
                return false;
            }
        }

        /// @brief starts the active controller's evaluation at its time step
        /// @return true if it successfully starts the stepping and false if it was already stepping
        bool unpause_stepping()
        {
            if(!stepping)
            {
                stepping = true;
                return true;
            }
            else
            {
                //if you unpause while its stepping 
                return false;
            }
        }

    private:
        core::control::ControllerManagerStatus _can_switch_controller(const core::VehicleState &current_state, const core::ControllerOutput &previous_output, const core::ControllerOutput &next_controller_output);
        std::array<ControllerType *, NumControllers> _controllers;
        size_t _current_controller_index = 0;
        core::StateEstimator _state_estimator;
        core::control::ControllerManagerState _current_state;
        //flag for pause/unpause -> could be temporary or permanent way of pausing
        bool stepping = true;

        float _max_switch_rpm, _max_torque_switch, _max_accel_switch_req;
    };
}
#include "ControllerManager.tpp"
