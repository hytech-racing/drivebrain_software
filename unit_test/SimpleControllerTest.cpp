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
        core::VehicleState in;

        SimpleControllerTest()
            : logger(core::LogLevel::INFO), 
            config("../config/test_config/can_driver.json"), // TODO probably want a better way to get param path
            simple_controller(logger, config),
            in(true, { 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 })
        {
        }

        void SetUp() override {
            simple_controller.init();
        }

        void TearDown() override {
            // Shouldn't need anything here
        }

};

TEST_F(SimpleControllerTest, SmallPositiveAccelRequest) {
    in.input.requested_accel = .2;
    auto res = simple_controller.step_controller(in);
    ASSERT_NEAR(res.desired_rpms.FL, 1672.12);
    ASSERT_NEAR(res.desired_rpms.FR, 1672.12);
    ASSERT_NEAR(res.desired_rpms.RL, 1672.12);
    ASSERT_NEAR(res.desired_rpms.RR, 1672.12);

    ASSERT_NEAR(res.torque_lim_nm.FR, 4.48);
    ASSERT_NEAR(res.torque_lim_nm.FL, 4.48);
    ASSERT_NEAR(res.torque_lim_nm.RR, 4.48);
    ASSERT_NEAR(res.torque_lim_nm.RL, 4.48);
}
TEST_F(SimpleControllerTest, FullPositiveAccelRequest) {
    in.input.requested_accel = 1;
    auto res = simple_controller.step_controller(in);
    ASSERT_NEAR(res.desired_rpms.FL, 1672.12);
    ASSERT_NEAR(res.desired_rpms.FR, 1672.12);
    ASSERT_NEAR(res.desired_rpms.RL, 1672.12);
    ASSERT_NEAR(res.desired_rpms.RR, 1672.12);

    ASSERT_NEAR(res.torque_lim_nm.FR, 22.4);
    ASSERT_NEAR(res.torque_lim_nm.FL, 22.4);
    ASSERT_NEAR(res.torque_lim_nm.RR, 22.4);
    ASSERT_NEAR(res.torque_lim_nm.RL, 22.4);
}

TEST_F(SimpleControllerTest, SmallNegativeAccelRequest)
{
    in.input.requested_accel = .2;
    in.input.requested_brake = .8;
    auto res = simple_controller.step_controller(in);

    ASSERT_NEAR(res.desired_rpms.FL, 6.0);
    ASSERT_NEAR(res.desired_rpms.FR, 6.0);
    ASSERT_NEAR(res.desired_rpms.RL, 6.0);
    ASSERT_NEAR(res.desired_rpms.RR, 6.0);
}
TEST_F(SimpleControllerTest, FullNegativeAccelRequest)
{
    in.input.requested_brake = 1;
    auto res = simple_controller.step_controller(in);

    ASSERT_NEAR(res.desired_rpms.FL, 10.0);
    ASSERT_NEAR(res.desired_rpms.FR, 10.0);
    ASSERT_NEAR(res.desired_rpms.RL, 10.0);
    ASSERT_NEAR(res.desired_rpms.RR, 10.0);
}



