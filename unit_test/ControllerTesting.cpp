#include "LoadCellVectoringTorqueController.hpp"
#include <Literals.hpp>
#include <gtest/gtest.h>
#include <LoadCellVectoringTorqueController.hpp>
#include <VehicleDataTypes.hpp>
#include <JsonFileHandler.hpp>
#include <Logger.hpp>
#include <Utils.hpp>

class ControllerTesting : public testing::Test {

protected:
    core::Logger logger;
    core::JsonFileHandler config;
    control::LoadCellVectoringTorqueController lctv;
    core::VehicleState in;

    ControllerTesting()
        : logger(core::LogLevel::INFO), 
        config("../config/drivebrain_config.json"),
        lctv(config),
        in()
    {
        in.is_ready_to_drive = true;
        in.current_rpms = { 0.0, 0.0, 0.0, 0.0 };
        in.current_body_vel_ms = { 0.0 , 0.0 , 0.0 };
        in.input = { 0.0, 0.0 };
    }

    void SetUp() override {
        lctv.init();
    }
};


TEST_F(ControllerTesting, FullBrakeRequestLCTV)
{
    in.input.requested_brake = 1.0;
    in.normalized_corner_load = {1.0, 1.0, 1.0, 1.0}; // simulate balanced weight

    auto cmd = lctv.step_controller(in);
    auto res = std::get_if<core::SpeedControlOut>(&cmd.out);

    // All RPMs should be 0
    ASSERT_NEAR(res->desired_rpms.FL, 0.0, 1e-3);
    ASSERT_NEAR(res->desired_rpms.FR, 0.0, 1e-3);
    ASSERT_NEAR(res->desired_rpms.RL, 0.0, 1e-3);
    ASSERT_NEAR(res->desired_rpms.RR, 0.0, 1e-3);

    float regen_torque_pool = 1.0 * std::get<float>(lctv.get_cached_param("max_regen_torque"));

    float front_scale = (2.0 - std::get<float>(lctv.get_cached_param("rear_torque_scale")));
    float rear_scale = std::get<float>(lctv.get_cached_param("rear_torque_scale"));

    ASSERT_NEAR(res->torque_lim_nm.FL, regen_torque_pool * 1.0 * front_scale, 0.01);
    ASSERT_NEAR(res->torque_lim_nm.FR, regen_torque_pool * 1.0 * front_scale, 0.01);
    ASSERT_NEAR(res->torque_lim_nm.RL, regen_torque_pool * 1.0 * rear_scale, 0.01);
    ASSERT_NEAR(res->torque_lim_nm.RR, regen_torque_pool * 1.0 * rear_scale, 0.01);
}

TEST_F(ControllerTesting, FullPositiveAccelProportionalToLoad)
{
    in.input.requested_accel = 1.0;
    in.normalized_corner_load = {1.2f, 0.8f, 1.2f, 0.8f}; // FL, FR, RL, RR

    auto cmd = lctv.step_controller(in);
    auto res = std::get_if<core::SpeedControlOut>(&cmd.out);
    ASSERT_NE(res, nullptr);

    float speed_set = std::get<float>(lctv.get_cached_param("positive_speed_set"));
    float max_torque = std::get<float>(lctv.get_cached_param("max_torque"));
    float rpm_expected = speed_set * constants::METERS_PER_SECOND_TO_RPM;

    // Confirm all wheels are set to the correct RPM
    ASSERT_NEAR(res->desired_rpms.FL, rpm_expected, 1.0);
    ASSERT_NEAR(res->desired_rpms.FR, rpm_expected, 1.0);
    ASSERT_NEAR(res->desired_rpms.RL, rpm_expected, 1.0);
    ASSERT_NEAR(res->desired_rpms.RR, rpm_expected, 1.0);

    // Total available torque across all wheels
    float torque_budget = max_torque * 4.0f;

    // Sum of normalized corner loads
    float total_load = 1.2f + 0.8f + 1.2f + 0.8f;

    // Expected torque = (normalized_load / total_load) * torque_budget
    float FL_torque = (1.2f / total_load) * torque_budget;
    float FR_torque = (0.8f / total_load) * torque_budget;
    float RL_torque = (1.2f / total_load) * torque_budget;
    float RR_torque = (0.8f / total_load) * torque_budget;

    ASSERT_NEAR(res->torque_lim_nm.FL, FL_torque, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.FR, FR_torque, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RL, RL_torque, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RR, RR_torque, 1.0);
}