#include <gtest/gtest.h>
#include <SimpleController.hpp>
#include <VehicleDataTypes.hpp>
#include <JsonFileHandler.hpp>
#include <Logger.hpp>
#include <Utils.hpp>

class SimpleControllerTest : public testing::Test {

    protected:
        SimpleControllerTest() : config(std::string("../config/drivebrain_config.json")){
            // config = core::JsonFileHandler();
            simple_controller = std::make_unique<control::SimpleController>(config);
        }
        
        core::JsonFileHandler config;
        
        // control::SimpleController simple_controller({}, {});

        std::unique_ptr<control::SimpleController> simple_controller;

        void SetUp() override {
            simple_controller->init();
        }

        void TearDown() override {
            // Shouldn't need anything here
        }

};

TEST_F(SimpleControllerTest, SimpleAccel) {

    // // --- Test Case 1: Simple Acceleration ---
    {
        core::VehicleState in;
        veh_vec<float> current_rpms; 
        current_rpms.FL = 100; 
        current_rpms.FR = 100;
        current_rpms.RL = 100;
        current_rpms.RR = 100;
        in.current_rpms = current_rpms;
        in.input.requested_accel = 0.1;
        in.input.requested_brake = 0;
        in.prev_MCU_recv_millis = 0;

        auto res = simple_controller->step_controller(in);
        std::cout <<"asdf: " << res.desired_rpms.FL <<std::endl;
        // assert 
        ASSERT_NEAR(res.desired_rpms.FL, 25082, 1);
        ASSERT_NEAR(res.desired_rpms.FR, 25082, 1);
        ASSERT_NEAR(res.desired_rpms.RL, 25082, 1);
        ASSERT_NEAR(res.desired_rpms.RR, 25082, 1);

        // std::cout << res.torque_lim_nm.FL <<std::endl;
        ASSERT_NEAR(res.torque_lim_nm.FL, 2.1, 0.01);
        ASSERT_NEAR(res.torque_lim_nm.FR, 2.1, 0.01);
        ASSERT_NEAR(res.torque_lim_nm.RL, 2.1, 0.01);
        ASSERT_NEAR(res.torque_lim_nm.RR, 2.1, 0.01);
    }
}

TEST_F(SimpleControllerTest, FullBraking)
{
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
        in.input.requested_brake = 1.0;
        in.prev_MCU_recv_millis = 0;

        auto res = simple_controller->step_controller(in);
        ASSERT_EQ(res.desired_rpms.FL, 0);
        ASSERT_EQ(res.desired_rpms.FR, 0);
        ASSERT_EQ(res.desired_rpms.RL, 0);
        ASSERT_EQ(res.desired_rpms.RR, 0);
        ASSERT_EQ(res.torque_lim_nm.FL, 10);
        ASSERT_EQ(res.torque_lim_nm.FR, 10);
        ASSERT_EQ(res.torque_lim_nm.RL, 10);
        ASSERT_EQ(res.torque_lim_nm.RR, 10);
    }
}

TEST_F(SimpleControllerTest, ZerlAccelZeroBrake)
{
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
        in.input.requested_brake = 0.0;
        in.prev_MCU_recv_millis = 0;

        auto res = simple_controller->step_controller(in);
        
        ASSERT_EQ(res.torque_lim_nm.FL, 0);
        ASSERT_EQ(res.torque_lim_nm.FR, 0);
        ASSERT_EQ(res.torque_lim_nm.RL, 0);
        ASSERT_EQ(res.torque_lim_nm.RR, 0);
    }
}

TEST_F(SimpleControllerTest, TestPowerLimit)
{
    // // --- Test Case 4: Power Limiting Scenario ---
    core::VehicleState in;
    veh_vec<float> current_rpms;
    current_rpms.FL = 10000;
    current_rpms.FR = 10000;
    current_rpms.RL = 10000;
    current_rpms.RR = 10000;
    in.current_rpms = current_rpms;
    in.input.requested_accel = 1.0;
    in.input.requested_brake = 0;
    in.prev_MCU_recv_millis = 0;

    auto res = simple_controller->step_controller(in);

    ASSERT_NEAR(res.desired_rpms.FL, 25082, 1);
    ASSERT_NEAR(res.desired_rpms.FR, 25082, 1);
    ASSERT_NEAR(res.desired_rpms.RL, 25082, 1);
    ASSERT_NEAR(res.desired_rpms.RR, 25082, 1);

    ASSERT_LT(res.torque_lim_nm.FL, 21);  // Expect power limiting applied
    ASSERT_LT(res.torque_lim_nm.FR, 21);
    ASSERT_LT(res.torque_lim_nm.RL, 21);
    ASSERT_LT(res.torque_lim_nm.RR, 21);


    constexpr long double RPM_TO_RAD_PER_SECOND = 2.0 * 3.1415 / 60.0;
    
    float net_power = 0;
    net_power += ::abs(res.torque_lim_nm.FL) * (current_rpms.FL * constants::RPM_TO_RAD_PER_SECOND);
    net_power += ::abs(res.torque_lim_nm.FR) * (current_rpms.FR * constants::RPM_TO_RAD_PER_SECOND);
    net_power += ::abs(res.torque_lim_nm.RL) * (current_rpms.RL * constants::RPM_TO_RAD_PER_SECOND);
    net_power += ::abs(res.torque_lim_nm.RR) * (current_rpms.RR * constants::RPM_TO_RAD_PER_SECOND);

    ASSERT_NEAR(net_power, 63000.0, 0.001);  // Expect power limiting applied and ensure near 63kw (hard-coded limit)
}
