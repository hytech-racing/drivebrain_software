#include <gtest/gtest.h>
#include <SimpleController.hpp>
#include <VehicleDataTypes.hpp>
#include <JsonFileHandler.hpp>
#include <Logger.hpp>
#include <Utils.hpp>

class SimpleControllerTest : public testing::Test {

protected:
    core::Logger logger;
    core::JsonFileHandler config;
    control::SimpleController simple_controller;
    control::SimpleController fail_controller;
    core::VehicleState in;

    SimpleControllerTest()
        : logger(core::LogLevel::INFO), 
        config("../config/drivebrain_config.json"),
        simple_controller(logger, config),
        fail_controller(logger, config, "no_config_here"),
        in()
    {
        in.is_ready_to_drive = true;
        in.current_rpms = { 0.0, 0.0, 0.0, 0.0 };
        in.current_body_vel_ms = { 0.0 , 0.0 , 0.0 };
        in.input = { 0.0, 0.0 };
    }

    void SetUp() override {
        simple_controller.init();
    }
};

TEST_F(SimpleControllerTest, ConstructorInitializesProperly) 
{
    EXPECT_NEAR(simple_controller.get_dt_sec(), 0.001, 0.01);
}

TEST_F(SimpleControllerTest, InitHasConfig)
{
    EXPECT_TRUE(simple_controller.init());
}

TEST_F(SimpleControllerTest, InitDoesNotHaveConfig)
{
    EXPECT_FALSE(fail_controller.init());
}

TEST_F(SimpleControllerTest, NoPedalInput)
{
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::SpeedControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_rpms.FL, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.FR, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.RL, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.RR, 1672.12, 1.0);

    ASSERT_NEAR(res->torque_lim_nm.FR, 0.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.FL, 0.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RR, 0.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RL, 0.0, 1.0); 
}

TEST_F(SimpleControllerTest, SmallPositiveAccelRequest)
{
    in.input.requested_accel = 0.2;
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::SpeedControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_rpms.FL, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.FR, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.RL, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.RR, 1672.12, 1.0);

    ASSERT_NEAR(res->torque_lim_nm.FR, 4.48, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.FL, 4.48, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RR, 4.48, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RL, 4.48, 1.0);
}

TEST_F(SimpleControllerTest, FullPositiveAccelRequest)
{
    in.input.requested_accel = 1;
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::SpeedControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_rpms.FL, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.FR, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.RL, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.RR, 1672.12, 1.0);

    ASSERT_NEAR(res->torque_lim_nm.FR, 22.4, 2.0);
    ASSERT_NEAR(res->torque_lim_nm.FL, 22.4, 2.0);
    ASSERT_NEAR(res->torque_lim_nm.RR, 22.4, 2.0);
    ASSERT_NEAR(res->torque_lim_nm.RL, 22.4, 2.0);
}

TEST_F(SimpleControllerTest, SmallNegativeAccelRequest)
{
    in.input.requested_accel = 0.2;
    in.input.requested_brake = 0.8;
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::SpeedControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_rpms.FL, 0.0, 1.0);
    ASSERT_NEAR(res->desired_rpms.FR, 0.0, 1.0);
    ASSERT_NEAR(res->desired_rpms.RL, 0.0, 1.0);
    ASSERT_NEAR(res->desired_rpms.RR, 0.0, 1.0);

    ASSERT_NEAR(res->torque_lim_nm.FL, 6.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.FR, 6.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RL, 6.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RR, 6.0, 1.0);
}

TEST_F(SimpleControllerTest, FullNegativeAccelRequest)
{
    in.input.requested_brake = 1;
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::SpeedControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_rpms.FL, 0.0, 1.0);
    ASSERT_NEAR(res->desired_rpms.FR, 0.0, 1.0);
    ASSERT_NEAR(res->desired_rpms.RL, 0.0, 1.0);
    ASSERT_NEAR(res->desired_rpms.RR, 0.0, 1.0);

    ASSERT_NEAR(res->torque_lim_nm.FL, 10.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.FR, 10.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RL, 10.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RR, 10.0, 1.0);
}

TEST_F(SimpleControllerTest, FullBrakeAndAccelRequest)
{
    in.input.requested_brake = 1;
    in.input.requested_accel = 1;
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::SpeedControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_rpms.FL, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.FR, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.RL, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.RR, 1672.12, 1.0);

    ASSERT_NEAR(res->torque_lim_nm.FR, 0.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.FL, 0.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RR, 0.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RL, 0.0, 1.0);
}

TEST_F(SimpleControllerTest, VariableRequests)
{
    in.input.requested_brake = 1;
    in.input.requested_accel = 1;
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::SpeedControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_rpms.FL, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.FR, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.RL, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.RR, 1672.12, 1.0);

    ASSERT_NEAR(res->torque_lim_nm.FR, 0.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.FL, 0.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RR, 0.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RL, 0.0, 1.0);

    in.input.requested_accel = 0;
    cmd = simple_controller.step_controller(in);

    ASSERT_NEAR(res->desired_rpms.FL, 0.0, 1.0);
    ASSERT_NEAR(res->desired_rpms.FR, 0.0, 1.0);
    ASSERT_NEAR(res->desired_rpms.RL, 0.0, 1.0);
    ASSERT_NEAR(res->desired_rpms.RR, 0.0, 1.0);

    ASSERT_NEAR(res->torque_lim_nm.FL, 10.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.FR, 10.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RL, 10.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RR, 10.0, 1.0);

    in.input.requested_accel = 1;
    in.input.requested_brake = 0;
    cmd = simple_controller.step_controller(in);

    ASSERT_NEAR(res->desired_rpms.FL, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.FR, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.RL, 1672.12, 1.0);
    ASSERT_NEAR(res->desired_rpms.RR, 1672.12, 1.0);

    ASSERT_NEAR(res->torque_lim_nm.FR, 22.4, 2.0);
    ASSERT_NEAR(res->torque_lim_nm.FL, 22.4, 2.0);
    ASSERT_NEAR(res->torque_lim_nm.RR, 22.4, 2.0);
    ASSERT_NEAR(res->torque_lim_nm.RL, 22.4, 2.0);
}
