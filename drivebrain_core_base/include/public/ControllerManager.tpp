
#include <ControllerManager.hpp>

template <veh_vec VehicleVector>
bool _verify_abs_less_than(VehicleVector input_to_check, float value)
{
    return ();
}

template <size_t NumControllers>
control::ControllerManager::ControllerManagerStatus control::ControllerManager<NumControllers>::_can_switch_controller(const VehicleState &current_state,
                                                                                                                       const ControllerOutput &previous_output,
                                                                                                                       const ControllerOutput &next_controller_output)
{
    using status_type = control::ControllerManager::ControllerManagerStatus;

    bool speedPreventsModeChange = false;
    // Check if torque delta permits mode change
    bool torqueDeltaPreventsModeChange = false;
    
    float max_value = _
    bool speedPreventsModeChange = (abs(input_to_check.FL) > value) && (abs(input_to_check.FR) > value) && (abs(input_to_check.RL) > value) && (abs(input_to_check.RR) > value)
    
    // only if the torque delta is positive do we not want to switch to the new one
    torqueDeltaPreventsModeChange = (desired_controller_out.torqueSetpoints[i] - previous_controller_command.torqueSetpoints[i]) > max_torque_pos_change_delta_;
    if (speedPreventsModeChange)
    {
        return status_type::ERROR_SPEED_DIFF_TOO_HIGH;
    }
    if (torqueDeltaPreventsModeChange)
    {
        return status_type::ERROR_TORQUE_DIFF_TOO_HIGH;
    }

    return status_type::NO_ERROR;
}