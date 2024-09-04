#ifndef __VEHICLESTATE_H__
#define __VEHICLESTATE_H__
#include <Utils.hpp>
#include <variant>

// TODO this will need a one-to-one protobuf message  
namespace core
{

    struct DriverInput
    {
        float requested_accel; // float from 0 to 1 representing percent of accel pedal travel
        float requested_brake; // float from 0 to 1 representing pedal travel of brake
    };

    struct VehicleState
    {
        bool is_ready_to_drive;
        DriverInput input;
        veh_vec<float> current_rpms;
        xyz_vec<float> current_body_vel_ms;
        
    };

    struct SpeedControlOut
    {
        veh_vec<float> desired_rpms;
        veh_vec<float> positive_torque_lim_nm;
        veh_vec<float> negative_torque_lim_nm;
    };

    struct TorqueControlOut
    {
        veh_vec<float> desired_torques_nm;
    };

    // we will have both speed and torque control output controllers
    struct ControllerOutput 
    {
        std::variant<SpeedControlOut, TorqueControlOut, std::monostate> out;
    };

}
#endif // __VEHICLESTATE_H__