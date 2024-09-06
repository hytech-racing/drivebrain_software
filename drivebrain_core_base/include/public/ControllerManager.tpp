
#include <ControllerManager.hpp>

bool controll::ControllerManager::init()
{
    auto max_switch_speed = get_parameter_value<float>("max_controller_switch_speed_ms");
    auto max_torque_switch = get_parameter_value<float>("max_torque_switch_nm");
    auto max_accel_switch_request = get_parameter_value<float>("max_accel_switch_float");

    if (!(max_switch_speed && max_torque_switch && max_accel_switch_request))
    {
        std::cout << "ERROR: couldnt get params" << std::endl;
        return false;
    }

    if ((*max_accel_switch_request) > 1.0)
    {
        std::cout << "ERROR: max accel switch float is greater than max value possible (1.0)" << std::endl;
    }

    _max_switch_rpm = ((*max_switch_speed) * constants::METERS_PER_SECOND_TO_RPM);
    _max_torque_switch = *max_torque_switch;
    _max_accel_switch_req = *max_accel_switch_request;

    return true;
}

template <size_t NumControllers>
control::ControllerManager::ControllerManagerStatus control::ControllerManager<NumControllers>::_can_switch_controller(const VehicleState &current_state,
                                                                                                                       const ControllerOutput &previous_output,
                                                                                                                       const ControllerOutput &next_controller_output)
{
    using status_type = control::ControllerManager::ControllerManagerStatus;

    bool speedPreventsChange = true;
    // Check if torque delta permits mode change
    bool torqueDeltaPreventsModeChange = true;

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

    if (check_veh_vec_abs(current_state.current_rpms, _max_switch_rpm, true))
    {
        return status_type::ERROR_SPEED_DIFF_TOO_HIGH;
    }

    auto verify_controller_output = [this](const ControllerOutput &controller_output) -> bool
    {
        if (const SpeedControlOut *pval = std::get_if<SpeedControlOut>(&previous_output.out))
        {
            if (check_veh_vec(pval->desired_rpms, _max_switch_rpm, true))
            {
                return true;
            }

            else if (check_veh_vec(pval->positive_torque_lim_nm, _max_torque_switch, false))
            {
                return true;
            } else {
                return false;
            }
        }
        else if (const TorqueControlOut *pval = std::get_if<TorqueControlOut>(&previous_output.out))
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
    bool proposedControllerOutputPreventingSwitch = verify_controller_output(previous_output);

    // TODO add checking for both controller types and if we are switching between controller output types what that looks like

    // instead of checking the difference between controller outputs, we will just make sure that the outputs themselves are under specific val

    // cases:

    // 1: "trajectory/speed" control (speed control with torque limit)
    // check: make sure that either the setpoint rpm is low enough OR (AND?) the torque setpoint is low enough
    // 2: "torque" control (just torque setpoints)
    // check: make sure the torque setpoint is low enough

    // only if the torque delta is positive do we not want to switch to the new one
    torqueDeltaPreventsModeChange = (desired_controller_out.torqueSetpoints[i] - previous_controller_command.torqueSetpoints[i]) > max_torque_pos_change_delta_;
    if (speedPreventsChange)
    {
        return status_type::ERROR_SPEED_DIFF_TOO_HIGH;
    }
    if (torqueDeltaPreventsModeChange)
    {
        return status_type::ERROR_TORQUE_DIFF_TOO_HIGH;
    }

    return status_type::NO_ERROR;
}