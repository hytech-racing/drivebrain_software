#pragma once

#include "TorqueController.hpp"

namespace control {

class SimpleTorqueController : public TorqueController {
public:
    SimpleTorqueController(core::Logger& logger, core::JsonFileHandler& config)
        : TorqueController(logger, config) {}

    float get_dt_sec() override {
        return 0.01f; // 100Hz control loop
    }

    core::ControllerOutput step_controller(const core::VehicleState& input) override {
        core::ControllerOutput output;
        
        // Simple torque control based on accelerator pedal
        core::TorqueControlOut torque_out;
        float max_torque = 21.42f; // Maximum torque in Nm
        
        // Apply torque based on accelerator pedal position
        torque_out.desired_torques_nm.FL = input.input.requested_accel * max_torque;
        torque_out.desired_torques_nm.FR = input.input.requested_accel * max_torque;
        torque_out.desired_torques_nm.RL = input.input.requested_accel * max_torque;
        torque_out.desired_torques_nm.RR = input.input.requested_accel * max_torque;
        
        output.out = torque_out;
        return output;
    }

    bool init() override {
        return true;
    }
};

} // namespace control 