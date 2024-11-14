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

// - vehicle manager shall handle evaluating of each controller at their
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

namespace control
{
    template <typename ControllerType, size_t NumControllers>
    class ControllerManager : public core::common::Configurable
    {
    public:
        /// @brief contructs instance of the controller manager
        /// @param json_file_handler current file handler to handle controller configurations
        /// @param controllers list of controllers that the manager will mux between and manager
        /// @param state_estimator instance to allow for direct communication between controllers and state estimator
        ControllerManager(core::Logger &logger, core::JsonFileHandler &json_file_handler, std::array<ControllerType *, NumControllers> controllers) : Configurable(logger, json_file_handler, "ControllerManager"),
                                                                                                                                _controllers(controllers), _logger_inst(logger)
        {
        }
        ~ControllerManager() = default;

        /// @brief configurable required init function
        /// @return true or false depending on success of init
        bool init();

        /// @brief attempts to switch the active controller
        /// @param new_controller_index desired controller index
        /// @return true if it successfully switches and false if it does not
        /// @note if it returns false the _current_ctr_manager_state member variable will have an altered status variable 
        bool swap_active_controller(size_t new_controller_index, const core::VehicleState& input);
        
        /// @brief fetches the active controllers desired seconds between controller evaluations
        /// @return the period in seconds for the active controller
        float get_active_controller_timestep()
        {
            return _controllers[_current_controller_index]->get_dt_sec();
        }

        /// @brief allows access to controller manager state for efficient communication
        /// @return ControllerManagerState types: ControllerManagerStatus, ControllerOutput
        core::control::ControllerManagerState get_current_ctr_manager_state()
        {
            return _current_ctr_manager_state;
        }

        /// @brief evaluates the currently active controller 
        /// @param input current vehicle state maintained by the state estimator
        /// @return respective controller output to command the drivetrain
        core::ControllerOutput step_active_controller(const core::VehicleState& input)
        {   
            return _controllers[_current_controller_index]->step_controller(input);
        }

    private:
        core::control::ControllerManagerStatus _can_switch_controller(const core::VehicleState &current_state, const core::ControllerOutput &previous_output, const core::ControllerOutput &next_controller_output);
        size_t _current_controller_index = 0;
        core::control::ControllerManagerState _current_ctr_manager_state;
        std::array<ControllerType *, NumControllers> _controllers;
        core::Logger _logger_inst;
        float _max_switch_rpm, _max_torque_switch, _max_accel_switch_req, _max_requested_rpm;
    };
}
#include "ControllerManager.tpp"