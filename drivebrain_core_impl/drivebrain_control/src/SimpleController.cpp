#include <SimpleController.hpp>
#include <variant>

template <typename DesiredType>
std::optional<DesiredType> type_helper(core::common::Configurable::ParamTypes param_val)
{
    if(std::is_arithmetic_v<DesiredType>)
    {
        
    } else if(std::is_same_v<DesiredType, std::string>)
    {   
        
    } else {
        std::cout << "WARNING: returning nullopt since the type is not a valid param type that we can cast to" << std::endl;
    }

    return std::nullopt;
}

void control::SimpleController::_handle_param_updates(const std::unordered_map<std::string, core::common::Configurable::ParamTypes> &new_param_map)
{
    // TODO make this easier to work with, rn variants can shift between any of the param types at runtime in the cache
    if(auto pval = std::get_if<float>(&new_param_map.at("max_regen_torque")))
    {
        _config.max_reg_torque = *pval;
    } else if(auto pval = std::get_if<int>(&new_param_map.at("max_regen_torque")))
    {
        _config.max_reg_torque = *pval;
    }
     
}

bool control::SimpleController::init()
{
    auto max_torque = get_live_parameter<float>("max_torque");
    auto max_regen_torque = get_live_parameter<float>("max_regen_torque");
    auto rear_torque_scale = get_live_parameter<float>("rear_torque_scale");
    auto regen_torque_scale = get_live_parameter<float>("regen_torque_scale");

    if (!(max_torque && max_regen_torque && rear_torque_scale && regen_torque_scale))
    {
        return false;
    }

    _config = {*max_torque, *max_regen_torque, *rear_torque_scale, *regen_torque_scale};

    param_update_handler_sig.connect(boost::bind(&control::SimpleController::_handle_param_updates, this,  std::placeholders::_1));
    
    return true;
}

drivetrain_command control::SimpleController::step_controller(const mcu_pedal_readings &in)
{
    // Both pedals are not pressed and no implausibility has been detected
    // accelRequest goes between 1.0 and -1.0
    float accelRequest = (in.accel_percent_float() / 100.0) - (in.brake_percent_float() / 100.0);

    float torqueRequest;

    drivetrain_command cmd_out;
    if (accelRequest >= 0.0)
    {
        // Positive torque request
        torqueRequest = ((float)accelRequest) * _config.max_torque;

        cmd_out.set_drivetrain_traj_torque_lim_fl((torqueRequest * (2.0 - _config.rear_torque_scale)));
        cmd_out.set_drivetrain_traj_torque_lim_fr((torqueRequest * (2.0 - _config.rear_torque_scale)));
        cmd_out.set_drivetrain_traj_torque_lim_rl((torqueRequest * _config.rear_torque_scale));
        cmd_out.set_drivetrain_traj_torque_lim_rr((torqueRequest * _config.rear_torque_scale));
    }
    else
    {
        // Negative torque request
        torqueRequest = _config.max_reg_torque * accelRequest * -1.0;
        cmd_out.set_drivetrain_traj_torque_lim_fl((torqueRequest * (2.0 - _config.regen_torque_scale)));
        cmd_out.set_drivetrain_traj_torque_lim_fr((torqueRequest * (2.0 - _config.regen_torque_scale)));
        cmd_out.set_drivetrain_traj_torque_lim_rl((torqueRequest * _config.regen_torque_scale));
        cmd_out.set_drivetrain_traj_torque_lim_rr((torqueRequest * _config.regen_torque_scale));
    }

    return cmd_out;
}