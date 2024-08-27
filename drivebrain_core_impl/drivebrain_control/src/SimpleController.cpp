#include <SimpleController.hpp>

bool control::SimpleController::init()
{
    auto max_torque = get_parameter_value<float>("max_torque");
    auto max_regen_torque = get_parameter_value<float>("max_regen_torque");
    // auto torque_balance = get_parameter_value<float>("torque_balance");
    if(! (max_torque && max_regen_torque))
    {
        return false;
    }

    _config= {*max_torque, *max_regen_torque};
    return true;
}

drivetrain_command control::SimpleController::step_controller(const mcu_pedal_readings& in)
{
    // Both pedals are not pressed and no implausibility has been detected
    // accelRequest goes between 1.0 and -1.0
    float accelRequest = in.accel_percent_float() - in.brake_percent_float();
    // std::cout <<accelRequest <<std::endl;
    float torqueRequest;

    
    drivetrain_command cmd_out;
    if (accelRequest >= 0.0)
    {
        // Positive torque request
        torqueRequest = accelRequest * _config.max_torque;


        cmd_out.set_drivetrain_traj_torque_lim_fl(torqueRequest);
        cmd_out.set_drivetrain_traj_torque_lim_fr(torqueRequest);
        cmd_out.set_drivetrain_traj_torque_lim_rl(torqueRequest);
        cmd_out.set_drivetrain_traj_torque_lim_rr(torqueRequest);

    }
    else
    {
        // Negative torque request
        torqueRequest = _config.max_reg_torque * accelRequest * -1.0;
        cmd_out.set_drivetrain_traj_torque_lim_fl(torqueRequest);
        cmd_out.set_drivetrain_traj_torque_lim_fr(torqueRequest);
        cmd_out.set_drivetrain_traj_torque_lim_rl(torqueRequest);
        cmd_out.set_drivetrain_traj_torque_lim_rr(torqueRequest);
    }
    // std::cout << "tq req "<<torqueRequest <<std::endl;
    return cmd_out;
}