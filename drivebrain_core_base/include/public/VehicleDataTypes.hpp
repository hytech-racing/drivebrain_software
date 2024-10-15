#ifndef __VEHICLESTATE_H__
#define __VEHICLESTATE_H__
#include <Utils.hpp>
#include <variant>

// TODO this will need a one-to-one protobuf message
namespace core
{

    // TODO change all driver inputs to use this?
    // this is the struct that will contain the data that is already
    // being logged via the sensor interfaces and doesnt go directly
    // into the vehicle state
    struct RawInputData
    {
        veh_vec<float> raw_load_cell_values;
    };

    struct DriverInput
    {
        float requested_accel; // float from 0 to 1 representing percent of accel pedal travel
        float requested_brake; // float from 0 to 1 representing pedal travel of brake
    };
    struct ControllerTorqueOut
    {
        veh_vec<float> res_torque_lim_nm;
    };

    struct TireDynamics
    {
        veh_vec<xyz_vec<float>> tire_forces_n;
        veh_vec<xyz_vec<float>> tire_moments_nm;
        veh_vec<float> accel_saturation_nm;
        veh_vec<float> brake_saturation_nm;
        float v_y_lm;
        float psi_dot_lm_rad_s;
        TireDynamics()
        {
            tire_forces_n = {};
            tire_moments_nm = {};
            accel_saturation_nm = {};
            brake_saturation_nm = {};
            v_y_lm = 0;
            psi_dot_lm_rad_s = 0;
        }
    };

    struct TorqueVectoringStatus
    {
        veh_vec<float> torque_additional_nm;
        float additional_mz_moment_nm;
        float des_psi_dot;
        float psi_dot_err;
        float perceived_vx;
        float integral_yaw_rate_err;
        float perceived_psi_dot;
    };

    struct MatlabMathResult
    {
        TireDynamics tire_dynamics;
        TorqueVectoringStatus torque_vectoring_status;
    };

    struct SpeedControlOut
    {
        int64_t mcu_recv_millis;
        veh_vec<float> desired_rpms;
        veh_vec<float> torque_lim_nm;
    };

    struct TorqueControlOut
    {
        veh_vec<float> desired_torques_nm;
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
        int prev_MCU_recv_millis;
        float steering_angle_deg;
        SpeedControlOut prev_controller_output;
        TireDynamics tire_dynamics;
        veh_vec<float> driver_torque;
        ControllerTorqueOut matlab_math_temp_out;
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