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
        xyz_vec<float> current_body_vel_ms;
        xyz_vec<float> current_body_accel_mss;
        xyz_vec<float> current_angular_rate_rads;
        ypr_vec<float> current_ypr_rad;
        veh_vec<float> current_rpms;
        bool state_is_valid;
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

    namespace control
    {
        enum class ControllerManagerStatus
        {
            NO_ERROR = 0,
            ERROR_CONTROLLER_INDEX_OUT_OF_RANGE = 1,
            ERROR_SPEED_DIFF_TOO_HIGH = 2,
            ERROR_TORQUE_DIFF_TOO_HIGH = 3,
            ERROR_DRIVER_ON_PEDAL = 3
        };
    }

}
#endif // __VEHICLESTATE_H__