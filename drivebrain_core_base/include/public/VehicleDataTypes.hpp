#ifndef __VEHICLESTATE_H__
#define __VEHICLESTATE_H__
#include <Utils.hpp>
#include <variant>
#include <Literals.hpp>
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
        veh_vec<velocity_rpm> current_rpms;
        xyz_vec<speed_m_s> current_body_vel_ms;
    };

    struct SpeedControlOut
    {
        veh_vec<velocity_rpm> desired_rpms;
        veh_vec<torque_nm> positive_torque_lim_nm;
        veh_vec<torque_nm> negative_torque_lim_nm;
    };

    struct TorqueControlOut
    {
        veh_vec<torque_nm> desired_torques_nm;
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
            NO_ERROR,
            ERROR_CONTROLLER_INDEX_OUT_OF_RANGE,
            ERROR_SPEED_DIFF_TOO_HIGH,
            ERROR_TORQUE_DIFF_TOO_HIGH,
            ERROR_DRIVER_ON_PEDAL,
            NUM_CONTROLLER_MANAGER_STATUSES
        };
    }

}
#endif // __VEHICLESTATE_H__