
#include <ControllerManager.hpp>

template <typename ControllerType, size_t NumControllers>
bool control::ControllerManager<ControllerType, NumControllers>::init()
{
    std::optional max_switch_speed = get_parameter_value<float>("max_controller_switch_speed_ms");
    std::optional max_torque_switch = get_parameter_value<float>("max_torque_switch_nm");
    std::optional max_accel_switch_request = get_parameter_value<float>("max_accel_switch_float");

    if (!(max_switch_speed && max_torque_switch && max_accel_switch_request))
    {
        std::cout << "ERROR: couldnt get params" << std::endl;
        return false;
    }

    if ((*max_accel_switch_request) > 1.0)
    {
        std::cout << "ERROR: max accel switch float is greater than max value possible (1.0)" << std::endl;
        return false;
    }

    _max_switch_rpm = ((*max_switch_speed) * constants::METERS_PER_SECOND_TO_RPM);
    _max_torque_switch = *max_torque_switch;
    _max_accel_switch_req = *max_accel_switch_request;

    return true;
}

template <typename ControllerType, size_t NumControllers>
core::control::ControllerManagerStatus control::ControllerManager<ControllerType, NumControllers>::_can_switch_controller(const core::VehicleState &current_state,
                                                                                                                       const core::ControllerOutput &previous_output,
                                                                                                                       const core::ControllerOutput &next_controller_output)
{
    using status_type = core::control::ControllerManagerStatus;

    bool speedPreventsChange = true;
    // Check if torque delta permits mode change
    bool torqueDeltaPreventsModeChange = true;

    // shared function to check if values are above a maximum value
    auto check_veh_vec = [](veh_vec<float> vehicle_vector, float max_val, bool check_with_abs) -> bool
    {
        if (check_with_abs)
        {
            return (abs(vehicle_vector.FL) > max_val) &&
                   (abs(vehicle_vector.FR) > max_val) &&
                   (abs(vehicle_vector.RL) > max_val) &&
                   (abs(vehicle_vector.RR) > max_val);
        }
        else
        {
            return (vehicle_vector.FL > max_val) &&
                   (vehicle_vector.FR > max_val) &&
                   (vehicle_vector.RL > max_val) &&
                   (vehicle_vector.RR > max_val);
        }
    };

    // check to see if current drivetrain rpms are too high to switch controller
    if (check_veh_vec(current_state.current_rpms, _max_switch_rpm, true))
    {
        return status_type::ERROR_SPEED_DIFF_TOO_HIGH;
    }


    // function to check whether or not the controller output is with range. 
    // can determine what type the controller output is and checks to see whether or not it has issues.

    // if the controller output is a speed controller type: checks both desired rpms level and max torque limit level to verify range.
    // if the controller output is a torque controller type: only checks the torque setpoint
    auto verify_controller_output = [this, &check_veh_vec](const core::ControllerOutput &controller_output) -> bool
    {
        if (const core::SpeedControlOut *pval = std::get_if<core::SpeedControlOut>(&controller_output.out))
        {
            if (check_veh_vec(pval->desired_rpms, _max_switch_rpm, true))
            {
                return true;
            }
            else if (check_veh_vec(pval->torque_lim_nm, _max_torque_switch, false))
            {
                return true;
            } else {
                return false;
            }
        }
        else if (const core::TorqueControlOut *pval = std::get_if<core::TorqueControlOut>(&controller_output.out))
        {
            if (check_veh_vec(pval->desired_torques_nm, _max_torque_switch, false))
            {
                return true;
            }
        }
        else
        {
            return true;
        }
    };

    bool currentControllerOutputPreventingSwitch = verify_controller_output(previous_output);
    bool proposedControllerOutputPreventingSwitch = verify_controller_output(next_controller_output);

    // TODO add checking for both controller types and if we are switching between controller output types what that looks like

    // instead of checking the difference between controller outputs, we will just make sure that the outputs themselves are under specific val

    // cases:

    // 1: "trajectory/speed" control (speed control with torque limit)
    // check: make sure that either the setpoint rpm is low enough OR (AND?) the torque setpoint is low enough
    // 2: "torque" control (just torque setpoints)
    // check: make sure the torque setpoint is low enough

    // only if the torque delta is positive do we not want to switch to the new one
    // torqueDeltaPreventsModeChange = (desired_controller_out.torqueSetpoints[i] - previous_controller_command.torqueSetpoints[i]) > max_torque_pos_change_delta_;
    // if (speedPreventsChange)
    // {
    //     return status_type::ERROR_SPEED_DIFF_TOO_HIGH;
    // }
    // if (torqueDeltaPreventsModeChange)
    // {
    //     return status_type::ERROR_TORQUE_DIFF_TOO_HIGH;
    // }

    return status_type::NO_ERROR;
}