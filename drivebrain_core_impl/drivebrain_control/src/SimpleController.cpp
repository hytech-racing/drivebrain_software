#include <SimpleController.hpp>

bool control::SimpleController::init()
{
    auto max_torque = get_parameter_value<float>("max_torque");
    auto max_regen_torque = get_parameter_value<float>("max_regen_torque");
    auto rear_torque_scale = get_parameter_value<float>("rear_torque_scale");
    auto regen_torque_scale = get_parameter_value<float>("regen_torque_scale");
    
    if (!(max_torque && max_regen_torque && rear_torque_scale && regen_torque_scale))
    {
        return false;
    }

    _config = {*max_torque, *max_regen_torque, *rear_torque_scale, *regen_torque_scale};
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
        cmd_out.set_drivetrain_traj_torque_lim_fl((torqueRequest* (2.0 - _config.regen_torque_scale)));
        cmd_out.set_drivetrain_traj_torque_lim_fr((torqueRequest* (2.0 - _config.regen_torque_scale)));
        cmd_out.set_drivetrain_traj_torque_lim_rl((torqueRequest* _config.regen_torque_scale));
        cmd_out.set_drivetrain_traj_torque_lim_rr((torqueRequest* _config.regen_torque_scale));
    }
    
    return cmd_out;
}