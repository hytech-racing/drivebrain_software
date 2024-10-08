#include <SimpleController.hpp>
#include <variant>
#include <VehicleDataTypes.hpp>

void control::SimpleController::_handle_param_updates(const std::unordered_map<std::string, core::common::Configurable::ParamTypes> &new_param_map)
{
    // TODO make this easier to work with, rn variants can shift between any of the param types at runtime in the cache
    if(auto pval = std::get_if<float>(&new_param_map.at("max_torque")))
    {
        _config.max_torque = *pval;
        std::cout << "setting new max torque " << _config.max_torque << std::endl;
    }
    
    if(auto pval = std::get_if<float>(&new_param_map.at("max_regen_torque")))
    {
        _config.max_reg_torque = *pval;
        std::cout << "setting new max regen torque " << _config.max_reg_torque << std::endl;
    }
    if (auto pval = std::get_if<float>(&new_param_map.at("rear_torque_scale")))
    {
        _config.rear_torque_scale = *pval;
        std::cout << "setting new rear_torque_scale " << _config.rear_torque_scale << std::endl;
    }
    if (auto pval = std::get_if<float>(&new_param_map.at("regen_torque_scale")))
    {
        _config.regen_torque_scale = *pval;
        std::cout << "setting new regen_torque_scale " << _config.regen_torque_scale << std::endl;
    }
    if(auto pval = std::get_if<float>(&new_param_map.at("positive_speed_set")))
    {
        _config.positive_speed_set = *pval;
        std::cout << "setting new positive_speed_set " << _config.positive_speed_set << std::endl;
    }
}

bool control::SimpleController::init()
{
    std::optional max_torque = get_live_parameter<torque_nm>("max_torque");
    std::optional max_regen_torque = get_live_parameter<torque_nm>("max_regen_torque");
    std::optional rear_torque_scale = get_live_parameter<float>("rear_torque_scale");
    std::optional regen_torque_scale = get_live_parameter<float>("regen_torque_scale");
    std::optional positive_speed_set = get_live_parameter<speed_m_s>("positive_speed_set");

    if (!(max_torque && max_regen_torque && rear_torque_scale && regen_torque_scale && positive_speed_set))
    {
        return false;
    }

    _config = {*max_torque, *max_regen_torque, *rear_torque_scale, *regen_torque_scale, *positive_speed_set};

    param_update_handler_sig.connect(boost::bind(&control::SimpleController::_handle_param_updates, this, std::placeholders::_1));

    return true;
}

core::SpeedControlOut control::SimpleController::step_controller(const core::VehicleState &in)
{
    // Both pedals are not pressed and no implausibility has been detected
    // accelRequest goes between 1.0 and -1.0
    float accelRequest = (in.input.requested_accel) - (in.input.requested_brake);

    torque_nm torqueRequest;

    hytech_msgs::MCUCommandData cmd_out;
    
    if (accelRequest >= 0.0)
    {
        // Positive torque request
        torqueRequest = ((float)accelRequest) * _config.max_torque;
        
        auto max_rpm = _config.positive_speed_set * constants::METERS_PER_SECOND_TO_RPM;
        cmd_out.mutable_desired_rpms()->set_fl(max_rpm);
        cmd_out.mutable_desired_rpms()->set_fr(max_rpm);
        cmd_out.mutable_desired_rpms()->set_rl(max_rpm);
        cmd_out.mutable_desired_rpms()->set_rr(max_rpm);

        cmd_out.mutable_torque_limit_nm()->set_fl((torqueRequest * (2.0 - _config.rear_torque_scale)));
        cmd_out.mutable_torque_limit_nm()->set_fr((torqueRequest * (2.0 - _config.rear_torque_scale)));
        cmd_out.mutable_torque_limit_nm()->set_rl((torqueRequest * _config.rear_torque_scale));
        cmd_out.mutable_torque_limit_nm()->set_rr((torqueRequest * _config.rear_torque_scale));
    }
    else
    {
        // Negative torque request
        torqueRequest = _config.max_reg_torque * accelRequest * -1.0;
        cmd_out.mutable_desired_rpms()->set_fl(0);
        cmd_out.mutable_desired_rpms()->set_fr(0);
        cmd_out.mutable_desired_rpms()->set_rl(0);
        cmd_out.mutable_desired_rpms()->set_rr(0);

        cmd_out.mutable_torque_limit_nm()->set_fl((torqueRequest * (2.0 - _config.regen_torque_scale)));
        cmd_out.mutable_torque_limit_nm()->set_fr((torqueRequest * (2.0 - _config.regen_torque_scale)));
        cmd_out.mutable_torque_limit_nm()->set_rl((torqueRequest * _config.regen_torque_scale));
        cmd_out.mutable_torque_limit_nm()->set_rr((torqueRequest * _config.regen_torque_scale));
    }

    return speed_cmd_out;
}