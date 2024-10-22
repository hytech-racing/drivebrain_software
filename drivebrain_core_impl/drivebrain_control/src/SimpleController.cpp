#include <SimpleController.hpp>
#include <variant>
#include <VehicleDataTypes.hpp>

void control::SimpleController::_handle_param_updates(const std::unordered_map<std::string, core::common::Configurable::ParamTypes> &new_param_map)
{
    // TODO make this easier to work with, rn variants can shift between any of the param types at runtime in the cache
    if (auto pval = std::get_if<float>(&new_param_map.at("max_torque")))
    {   
        
        std::unique_lock lk(_config_mutex);
        _config.max_torque = *pval;
        std::cout << "setting new max torque " << _config.max_torque << std::endl;
    }

    if (auto pval = std::get_if<float>(&new_param_map.at("max_regen_torque")))
    {
        std::unique_lock lk(_config_mutex);
        _config.max_reg_torque = *pval;
        std::cout << "setting new max regen torque " << _config.max_reg_torque << std::endl;
    }
    if (auto pval = std::get_if<float>(&new_param_map.at("rear_torque_scale")))
    {
        std::unique_lock lk(_config_mutex);
        _config.rear_torque_scale = *pval;
        std::cout << "setting new rear_torque_scale " << _config.rear_torque_scale << std::endl;
    }
    if (auto pval = std::get_if<float>(&new_param_map.at("regen_torque_scale")))
    {
        std::unique_lock lk(_config_mutex);
        _config.regen_torque_scale = *pval;
        std::cout << "setting new regen_torque_scale " << _config.regen_torque_scale << std::endl;
    }
    if (auto pval = std::get_if<float>(&new_param_map.at("positive_speed_set")))
    {
        std::unique_lock lk(_config_mutex);
        _config.positive_speed_set = *pval;
        std::cout << "setting new positive_speed_set " << _config.positive_speed_set << std::endl;
    }
}

bool control::SimpleController::init()
{
    auto max_torque = get_live_parameter<float>("max_torque");
    auto max_regen_torque = get_live_parameter<float>("max_regen_torque");
    auto rear_torque_scale = get_live_parameter<float>("rear_torque_scale");
    auto regen_torque_scale = get_live_parameter<float>("regen_torque_scale");
    auto positive_speed_set = get_live_parameter<float>("positive_speed_set");
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
    config cur_config;
    {
        std::unique_lock lk(_config_mutex);
        cur_config = _config;
    }
    // Both pedals are not pressed and no implausibility has been detected
    // accelRequest goes between 1.0 and -1.0
    float accelRequest = (in.input.requested_accel) - (in.input.requested_brake);

    float torqueRequest;

    // hytech_msgs::MCUCommandData cmd_out;
    core::SpeedControlOut cmd_out;
    cmd_out.mcu_recv_millis = in.prev_MCU_recv_millis; // heartbeat

    if (accelRequest >= 0.0)
    {
        // Positive torque request
        torqueRequest = ((float)accelRequest) * cur_config.max_torque;

        auto max_rpm = cur_config.positive_speed_set * constants::METERS_PER_SECOND_TO_RPM;
        cmd_out.desired_rpms.FL = max_rpm;
        cmd_out.desired_rpms.FR = max_rpm;
        cmd_out.desired_rpms.RL = max_rpm;
        cmd_out.desired_rpms.RR = max_rpm;

        cmd_out.torque_lim_nm.FL = (torqueRequest * (2.0 - cur_config.rear_torque_scale));
        cmd_out.torque_lim_nm.FR = (torqueRequest * (2.0 - cur_config.rear_torque_scale));
        cmd_out.torque_lim_nm.RL = (torqueRequest * cur_config.rear_torque_scale);
        cmd_out.torque_lim_nm.RR = (torqueRequest * cur_config.rear_torque_scale);
    }
    else
    {
        // Negative torque request
        torqueRequest = cur_config.max_reg_torque * accelRequest * -1.0;
        cmd_out.desired_rpms.FL = 0;
        cmd_out.desired_rpms.FR = 0;
        cmd_out.desired_rpms.RL = 0;
        cmd_out.desired_rpms.RR = 0;

        cmd_out.torque_lim_nm.FL = (torqueRequest * (2.0 - cur_config.rear_torque_scale));
        cmd_out.torque_lim_nm.FR = (torqueRequest * (2.0 - cur_config.rear_torque_scale));
        cmd_out.torque_lim_nm.RL = (torqueRequest * cur_config.rear_torque_scale);
        cmd_out.torque_lim_nm.RR = (torqueRequest * cur_config.rear_torque_scale);
    }

    return cmd_out;
}