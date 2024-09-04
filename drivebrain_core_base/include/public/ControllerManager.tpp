
#include <ControllerManager.hpp>

bool controll::ControllerManager::init()
{
    auto max_switch_speed = get_parameter_value<float>("max_controller_switch_speed_ms");
    auto max_torque_diff = get_parameter_value<float>("max_torque_switch_diff_nm");
    auto max_accel_switch_request = get_parameter_value<float>("max_accel_switch_float");

    if (!(max_switch_speed && max_torque_diff && max_accel_switch_request))
    {
        std::cout << "ERROR: couldnt get params" << std::endl;
        return false;
    }

    if ((*max_accel_switch_request) > 1.0)
    {
        std::cout << "ERROR: max accel switch float is greater than max value possible (1.0)" << std::endl;
    }

    _max_switch_rpm = ((*max_switch_speed) * constants::METERS_PER_SECOND_TO_RPM);
    _max_torque_diff = *max_torque_diff;
    _max_accel_switch_req = *max_accel_switch_request;

    return true;
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

    bool speedPreventsModeChange = (abs(current_state.current_rpms.FL) > _max_switch_rpm) &&
                                   (abs(current_state.current_rpms.FR) > _max_switch_rpm) &&
                                   (abs(current_state.current_rpms.RL) > _max_switch_rpm) &&
                                   (abs(current_state.current_rpms.RR) > _max_switch_rpm);


    // TODO add checking for both controller types and if we are switching between controller output types what that looks like
    
    // if(const TorqueControlOut* torque_val = std::get_if<TorqueControlOut>(previous_output));
    torqueDeltaPreventsModeChange = ( abs((abs(previous_torque_control_output.desired_torques_nm.FL) - abs(next_torque_controller_output.desired_torques_nm.FL)) > _max_torque_diff) ) &&
                                    ( abs((abs(previous_torque_control_output.desired_torques_nm.FR) - abs(next_torque_controller_output.desired_torques_nm.FR)) > _max_torque_diff) ) &&
                                    ( abs((abs(previous_torque_control_output.desired_torques_nm.RL) - abs(next_torque_controller_output.desired_torques_nm.RL)) > _max_torque_diff) ) &&
                                    ( abs((abs(previous_torque_control_output.desired_torques_nm.RR) - abs(next_torque_controller_output.desired_torques_nm.RR)) > _max_torque_diff) );

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