#include <gtest/gtest.h>
#include <SimpleTorqueController.hpp>
#include <VehicleDataTypes.hpp>
#include <JsonFileHandler.hpp>
#include <Logger.hpp>
#include <Utils.hpp>

class SimpleTorqueControllerTest : public testing::Test {

protected:
    core::Logger logger;
    core::JsonFileHandler config;
    control::SimpleTorqueController simple_controller;
    control::SimpleTorqueController fail_controller;
    core::VehicleState in;

    SimpleTorqueControllerTest()
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

TEST_F(SimpleTorqueControllerTest, ConstructorInitializesProperly) 
{
    EXPECT_NEAR(simple_controller.get_dt_sec(), 0.001, 0.01);
}

TEST_F(SimpleTorqueControllerTest, InitHasConfig)
{
    EXPECT_TRUE(simple_controller.init());
}

TEST_F(SimpleTorqueControllerTest, InitDoesNotHaveConfig)
{
    EXPECT_FALSE(fail_controller.init());
}

TEST_F(SimpleTorqueControllerTest, NoPedalInput)
{
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::TorqueControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_torques_nm.FR, 0.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.FL, 0.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.RR, 0.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.RL, 0.0, 1.0); 
}

TEST_F(SimpleTorqueControllerTest, SmallPositiveAccelRequest)
{
    in.input.requested_accel = 0.2;
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::TorqueControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_torques_nm.FR, 4.48, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.FL, 4.48, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.RR, 4.48, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.RL, 4.48, 1.0);
}

TEST_F(SimpleTorqueControllerTest, FullPositiveAccelRequest)
{
    in.input.requested_accel = 1;
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::TorqueControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_torques_nm.FR, 21, 2.0);
    ASSERT_NEAR(res->desired_torques_nm.FL, 21, 2.0);
    ASSERT_NEAR(res->desired_torques_nm.RR, 21, 2.0);
    ASSERT_NEAR(res->desired_torques_nm.RL, 21, 2.0);
}

TEST_F(SimpleTorqueControllerTest, SmallNegativeAccelRequest)
{
    in.input.requested_accel = 0.2;
    in.input.requested_brake = 0.8;
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::TorqueControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_torques_nm.FL, 6.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.FR, 6.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.RL, 6.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.RR, 6.0, 1.0);
}

TEST_F(SimpleTorqueControllerTest, FullNegativeAccelRequest)
{
    in.input.requested_brake = 1;
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::TorqueControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_torques_nm.FL, 10.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.FR, 10.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.RL, 10.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.RR, 10.0, 1.0);
}

TEST_F(SimpleTorqueControllerTest, FullBrakeAndAccelRequest)
{
    in.input.requested_brake = 1;
    in.input.requested_accel = 1;
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::TorqueControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_torques_nm.FR, 0.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.FL, 0.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.RR, 0.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.RL, 0.0, 1.0);
}

TEST_F(SimpleTorqueControllerTest, VariableRequests)
{
    in.input.requested_brake = 1;
    in.input.requested_accel = 1;
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::TorqueControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_torques_nm.FR, 0.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.FL, 0.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.RR, 0.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.RL, 0.0, 1.0);

    in.input.requested_accel = 0;
    cmd = simple_controller.step_controller(in);

    ASSERT_NEAR(res->desired_torques_nm.FL, 10.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.FR, 10.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.RL, 10.0, 1.0);
    ASSERT_NEAR(res->desired_torques_nm.RR, 10.0, 1.0);

    in.input.requested_accel = 1;
    in.input.requested_brake = 0;
    cmd = simple_controller.step_controller(in);

    ASSERT_NEAR(res->desired_torques_nm.FR, 21, 2.0);
    ASSERT_NEAR(res->desired_torques_nm.FL, 21, 2.0);
    ASSERT_NEAR(res->desired_torques_nm.RR, 21, 2.0);
    ASSERT_NEAR(res->desired_torques_nm.RL, 21, 2.0);
}
