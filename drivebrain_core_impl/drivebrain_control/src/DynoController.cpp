#include <DynoController.hpp>
#include <variant>
#include <VehicleDataTypes.hpp>
#include <spdlog/spdlog.h>

void control::DynoController::_handle_param_updates(const std::unordered_map<std::string, core::common::Configurable::ParamTypes> &new_param_map)
{
        // TODO make this easier to work with, rn variants can shift between any of the param types at runtime in the cache
    if (auto pval = std::get_if<float>(&new_param_map.at("max_torque")))
    {   
        
        std::unique_lock lk(_config_mutex);
        _config.max_torque = *pval;
        spdlog::info("Setting new max torque: {}", _config.max_torque);
    }

    if (auto pval = std::get_if<float>(&new_param_map.at("positive_speed_set")))
    {
        std::unique_lock lk(_config_mutex);
        _config.positive_speed_set = *pval;
        spdlog::info("Setting new positive speed set: {}", _config.positive_speed_set);
    }

}

bool control::DynoController::init()
{
    std::optional max_torque = get_live_parameter<torque_nm>("max_torque");
    std::optional positive_speed_set = get_live_parameter<float>("positive_speed_set");

    if (!(max_torque  && positive_speed_set))
    {
        return false;
    }

    _config = {*max_torque, *positive_speed_set};

    param_update_handler_sig.connect(boost::bind(&control::DynoController::_handle_param_updates, this, std::placeholders::_1));
    set_configured();
    return true;
}

core::ControllerOutput control::DynoController::step_controller(const core::VehicleState &in)
{
    config cur_config;
    {
        std::unique_lock lk(_config_mutex);
        cur_config = _config;
    }

    core::SpeedControlOut type_set = {};
    core::ControllerOutput cmd_out = {};
    cmd_out.out = type_set;
    auto& speed_out = std::get<core::SpeedControlOut>(cmd_out.out);

    auto rpm_setpoint = std::min((float)20000.0,cur_config.positive_speed_set);
    speed_out.desired_rpms.FL = rpm_setpoint;
    speed_out.desired_rpms.FR = rpm_setpoint;
    speed_out.desired_rpms.RL = rpm_setpoint;
    speed_out.desired_rpms.RR = rpm_setpoint;

    auto torque_limit = std::min((float)21.0, cur_config.max_torque);
    speed_out.torque_lim_nm.FL = torque_limit;
    speed_out.torque_lim_nm.FR = torque_limit;
    speed_out.torque_lim_nm.RL = torque_limit;
    speed_out.torque_lim_nm.RR = torque_limit;    

    return cmd_out;
}

