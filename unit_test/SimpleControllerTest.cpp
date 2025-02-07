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

        SimpleControllerTest()
            : logger(core::LogLevel::INFO), 
            config("../config/test_config/can_driver.json"), // TODO probably want a better way to get param path
            simple_controller(logger, config) {
        }

        void SetUp() override {
            simple_controller.init();
        }

        void TearDown() override {
            // Shouldn't need anything here
        }

};

TEST_F(SimpleControllerTest, Outputs) {

    // --- Test Case 1: Simple Acceleration ---
    {
        core::VehicleState in;
        veh_vec<float> current_rpms; 
        current_rpms.FL = 100; 
        current_rpms.FR = 100;
        current_rpms.RL = 100;
        current_rpms.RR = 100;
        in.current_rpms = current_rpms;
        in.input.requested_accel = 10;
        in.input.requested_brake = 0;
        in.prev_MCU_recv_millis = 0;

        auto res = simple_controller.step_controller(in);
        ASSERT_EQ(res.desired_rpms.FL, 180);
        ASSERT_EQ(res.desired_rpms.FR, 180);
        ASSERT_EQ(res.desired_rpms.RL, 180);
        ASSERT_EQ(res.desired_rpms.RR, 180);
        ASSERT_EQ(res.torque_lim_nm.FL, 210);
        ASSERT_EQ(res.torque_lim_nm.FR, 210);
        ASSERT_EQ(res.torque_lim_nm.RL, 210);
        ASSERT_EQ(res.torque_lim_nm.RR, 210);
    }

    // --- Test Case 2: Full Braking ---
    {
        core::VehicleState in;
        veh_vec<float> current_rpms;
        current_rpms.FL = 100;
        current_rpms.FR = 100;
        current_rpms.RL = 100;
        current_rpms.RR = 100;
        in.current_rpms = current_rpms;
        in.input.requested_accel = 0;
        in.input.requested_brake = 10;
        in.prev_MCU_recv_millis = 0;

        auto res = simple_controller.step_controller(in);
        ASSERT_EQ(res.desired_rpms.FL, 0);
        ASSERT_EQ(res.desired_rpms.FR, 0);
        ASSERT_EQ(res.desired_rpms.RL, 0);
        ASSERT_EQ(res.desired_rpms.RR, 0);
        ASSERT_EQ(res.torque_lim_nm.FL, 60);
        ASSERT_EQ(res.torque_lim_nm.FR, 60);
        ASSERT_EQ(res.torque_lim_nm.RL, 60);
        ASSERT_EQ(res.torque_lim_nm.RR, 60);
    }

    // --- Test Case 3: Zero Acceleration and Zero Brake (Coasting) ---
    {
        core::VehicleState in;
        veh_vec<float> current_rpms;
        current_rpms.FL = 100;
        current_rpms.FR = 100;
        current_rpms.RL = 100;
        current_rpms.RR = 100;
        in.current_rpms = current_rpms;
        in.input.requested_accel = 0;
        in.input.requested_brake = 0;
        in.prev_MCU_recv_millis = 0;

        auto res = simple_controller.step_controller(in);
        ASSERT_EQ(res.desired_rpms.FL, 0);
        ASSERT_EQ(res.desired_rpms.FR, 0);
        ASSERT_EQ(res.desired_rpms.RL, 0);
        ASSERT_EQ(res.desired_rpms.RR, 0);
        ASSERT_EQ(res.torque_lim_nm.FL, 0);
        ASSERT_EQ(res.torque_lim_nm.FR, 0);
        ASSERT_EQ(res.torque_lim_nm.RL, 0);
        ASSERT_EQ(res.torque_lim_nm.RR, 0);
    }

    // --- Test Case 4: Power Limiting Scenario ---
    {
        core::VehicleState in;
        veh_vec<float> current_rpms;
        current_rpms.FL = 5000;
        current_rpms.FR = 5000;
        current_rpms.RL = 5000;
        current_rpms.RR = 5000;
        in.current_rpms = current_rpms;
        in.input.requested_accel = 10;
        in.input.requested_brake = 0;
        in.prev_MCU_recv_millis = 0;

        auto res = simple_controller.step_controller(in);
        ASSERT_LT(res.torque_lim_nm.FL, 210);  // Expect power limiting applied
        ASSERT_LT(res.torque_lim_nm.FR, 210);
        ASSERT_LT(res.torque_lim_nm.RL, 210);
        ASSERT_LT(res.torque_lim_nm.RR, 210);
    }
}
