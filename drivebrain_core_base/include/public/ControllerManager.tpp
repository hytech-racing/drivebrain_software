#include <ControllerManager.hpp>

template <typename ControllerType, size_t NumControllers>
bool control::ControllerManager<ControllerType, NumControllers>::init()
{
    std::optional max_switch_speed = get_parameter_value<float>("max_controller_switch_speed_ms");
    std::optional max_torque_switch = get_parameter_value<float>("max_torque_switch_nm");
    std::optional max_accel_switch_request = get_parameter_value<float>("max_accel_switch_float");
    std::optional max_requested_rpm = get_parameter_value<float>("max_switch_requested_rpm");
    if (!(max_switch_speed && max_torque_switch && max_accel_switch_request && max_requested_rpm))
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
    _max_requested_rpm = *max_requested_rpm;
    _current_ctr_manager_state = {
        .current_status = core::control::ControllerManagerStatus::NO_ERROR,
        .current_controller_output = std::monostate{}
    };
    return true;
}

template <typename ControllerType, size_t NumControllers>
core::control::ControllerManagerStatus control::ControllerManager<ControllerType, NumControllers>::_can_switch_controller(const core::VehicleState &current_state,
                                                                                                                       const core::ControllerOutput &previous_output,
                                                                                                                       const core::ControllerOutput &next_controller_output)
{
    using status_type = core::control::ControllerManagerStatus;
    // shared function to check if values are above a maximum value
    auto check_veh_vec = [](veh_vec<float> vehicle_vector, float max_val, bool check_with_abs) -> bool
    {
        if (check_with_abs)
        {
            return (abs(vehicle_vector.FL) > max_val) ||
                   (abs(vehicle_vector.FR) > max_val) ||
                   (abs(vehicle_vector.RL) > max_val) ||
                   (abs(vehicle_vector.RR) > max_val);
        }
        else
        {
            return (vehicle_vector.FL > max_val) ||
                   (vehicle_vector.FR > max_val) ||
                   (vehicle_vector.RL > max_val) ||
                   (vehicle_vector.RR > max_val);
        }
    };
    // check to see if current drivetrain rpms are too high to switch controller
    if (check_veh_vec(current_state.current_rpms, _max_switch_rpm, true))
    {
        _current_ctr_manager_state.current_status = status_type::ERROR_SPEED_TOO_HIGH;
        return _current_ctr_manager_state.current_status;
    }
    if(current_state.input.requested_accel > _max_accel_switch_req)
    {
        _current_ctr_manager_state.current_status = status_type::ERROR_DRIVER_ON_PEDAL;
        return _current_ctr_manager_state.current_status;
    }
    // function to check whether or not the controller output is with range. 
    // can determine what type the controller output is and checks to see whether or not it has issues.
    // if the controller output is a speed controller type: checks both desired rpms level and max torque limit level to verify range.
    // if the controller output is a torque controller type: only checks the torque setpoint
    auto verify_controller_output = [this, &check_veh_vec](const core::ControllerOutput &controller_output) -> status_type
    {   
        // 1: "trajectory/speed" control (speed control with torque limit)
        // check: make sure that either the setpoint rpm is low enough OR (AND?) the torque setpoint is low enough
        if (const core::SpeedControlOut *pval = std::get_if<core::SpeedControlOut>(&controller_output.out))
        {
            if (check_veh_vec(pval->desired_rpms, _max_requested_rpm, true))
            {
                return status_type::ERROR_OUTPUT_EXCEEDS_PHYS_LIMITS;
            }
            else if (check_veh_vec(pval->torque_lim_nm, _max_torque_switch, false))
            {
                return status_type::ERROR_TORQUE_TOO_HIGH;
            } else {
                return status_type::NO_ERROR;
            }
        }
        // 2: "torque" control (just torque setpoints)
        // check: make sure the torque setpoint is low enough
        else if (const core::TorqueControlOut *pval = std::get_if<core::TorqueControlOut>(&controller_output.out))
        {
            if (check_veh_vec(pval->desired_torques_nm, _max_torque_switch, false))
            {
                return status_type::ERROR_TORQUE_TOO_HIGH;
            }
            else
            {
                return status_type::NO_ERROR;
            }
        }
        else
        {
            return status_type::ERROR_CONTROLLER_NO_TORQUE_OR_SPEED_OUTPUT;
        }
    };
    status_type prev_status = verify_controller_output(previous_output);
    status_type switch_status = verify_controller_output(next_controller_output);
    if(prev_status == status_type::NO_ERROR && switch_status == status_type::NO_ERROR)
    {
        _current_ctr_manager_state.current_status = status_type::NO_ERROR;
    }
    else if(prev_status != status_type::NO_ERROR && switch_status == status_type::NO_ERROR)
    {
        _current_ctr_manager_state.current_status = prev_status;
    }
    else
    {
        _current_ctr_manager_state.current_status = switch_status;
    }
    return _current_ctr_manager_state.current_status;
}

template <typename ControllerType, size_t NumControllers>
bool control::ControllerManager<ControllerType, NumControllers>::swap_active_controller(size_t new_controller_index, const core::VehicleState& input)
{   
    using status_type = core::control::ControllerManagerStatus;
    static const size_t num_controllers = _controllers.size();
    if (new_controller_index > (num_controllers - 1) || new_controller_index < 0)
    {
        _current_ctr_manager_state.current_status = status_type::ERROR_CONTROLLER_INDEX_OUT_OF_RANGE;
        _logger_inst.log_string("switch mode failed with error code: " + std::to_string(static_cast<int>(_current_ctr_manager_state.current_status)), static_cast<core::LogLevel>(1));
        return false;
    }
    else
    {
        if(_current_controller_index == new_controller_index)
        {
            _current_ctr_manager_state.current_status = status_type::ERROR_REQUESTING_SAME_CTR_TYPE;
            _logger_inst.log_string("switch mode failed with error code: " + std::to_string(static_cast<int>(_current_ctr_manager_state.current_status)), static_cast<core::LogLevel>(1));
            return false;
        }
    }

    if(_can_switch_controller(input, {_controllers[_current_controller_index]->step_controller(input)}, {_controllers[new_controller_index]->step_controller(input)}) == status_type::NO_ERROR)
    {
        _current_controller_index = new_controller_index;
        _logger_inst.log_string("switched mode: " + std::to_string(new_controller_index), static_cast<core::LogLevel>(0));
        return true;
    }
    else
    {
        _logger_inst.log_string("switch mode failed with error code: " + std::to_string(static_cast<int>(_current_ctr_manager_state.current_status)), static_cast<core::LogLevel>(1));
        return false;
    }
    //TODO: swap drivetrain controller when new controller mode wants to
}