#include <Literals.hpp>
#include <gtest/gtest.h>
#include <SimpleSpeedController.hpp>
#include <VehicleDataTypes.hpp>
#include <JsonFileHandler.hpp>
#include <Logger.hpp>
#include <Utils.hpp>

class SimpleSpeedControllerTest : public testing::Test {

protected:
    core::Logger logger;
    core::JsonFileHandler config;
    core::JsonFileHandler fail_config;
    control::SimpleSpeedController simple_controller;
    control::SimpleSpeedController fail_controller;
    core::VehicleState in;

    SimpleSpeedControllerTest()
        : logger(core::LogLevel::INFO), 
        config("../config/drivebrain_config.json"),
        fail_config("../config/fail_config.json"),
        simple_controller(config),
        fail_controller(fail_config),
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


TEST_F(SimpleSpeedControllerTest, InitHasConfig)
{
    EXPECT_TRUE(simple_controller.init());
}

TEST_F(SimpleSpeedControllerTest, InitDoesNotHaveConfig)
{
    EXPECT_FALSE(fail_controller.init());
}

TEST_F(SimpleSpeedControllerTest, NoPedalInput)
{

    in.input.requested_accel = 0.0;
    in.input.requested_brake = 0.0;   
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::SpeedControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_rpms.FL,std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.FR,std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.RL,std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.RR,std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);

    ASSERT_NEAR(res->torque_lim_nm.FR, 0.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.FL, 0.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RR, 0.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RL, 0.0, 1.0); 
}

TEST_F(SimpleSpeedControllerTest, SmallPositiveAccelRequest)
{
    in.input.requested_accel = 0.2;
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::SpeedControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_rpms.FL, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.FR, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.RL, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.RR, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);

    ASSERT_NEAR(res->torque_lim_nm.FR, 4.48, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.FL, 4.48, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RR, 4.48, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RL, 4.48, 1.0);
}

TEST_F(SimpleSpeedControllerTest, FullPositiveAccelRequest)
{
    in.input.requested_accel = 1;
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::SpeedControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_rpms.FL, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.FR, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.RL, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.RR, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);

    ASSERT_NEAR(res->torque_lim_nm.FR, 22.4, 2.0);
    ASSERT_NEAR(res->torque_lim_nm.FL, 22.4, 2.0);
    ASSERT_NEAR(res->torque_lim_nm.RR, 22.4, 2.0);
    ASSERT_NEAR(res->torque_lim_nm.RL, 22.4, 2.0);
}

TEST_F(SimpleSpeedControllerTest, SmallNegativeAccelRequest)
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

TEST_F(SimpleSpeedControllerTest, FullNegativeAccelRequest)
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

TEST_F(SimpleSpeedControllerTest, FullBrakeAndAccelRequest)
{
    in.input.requested_brake = 1;
    in.input.requested_accel = 1;
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::SpeedControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_rpms.FL, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.FR, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.RL, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.RR, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.FR, 0.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.FL, 0.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RR, 0.0, 1.0);
    ASSERT_NEAR(res->torque_lim_nm.RL, 0.0, 1.0);
}










// Ben's tests

TEST_F(SimpleSpeedControllerTest, SimpleAccel) {

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

        auto res = std::get<core::SpeedControlOut>(simple_controller.step_controller(in).out);
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

TEST_F(SimpleSpeedControllerTest, FullBraking)
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

        auto res = std::get<core::SpeedControlOut>(simple_controller.step_controller(in).out);
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

TEST_F(SimpleSpeedControllerTest, ZerlAccelZeroBrake)
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

        auto res = std::get<core::SpeedControlOut>(simple_controller.step_controller(in).out);
        
        ASSERT_EQ(res.torque_lim_nm.FL, 0);
        ASSERT_EQ(res.torque_lim_nm.FR, 0);
        ASSERT_EQ(res.torque_lim_nm.RL, 0);
        ASSERT_EQ(res.torque_lim_nm.RR, 0);
    }
}


TEST_F(SimpleSpeedControllerTest, VariableRequests)
{
    in.input.requested_brake = 1;
    in.input.requested_accel = 1;
    auto cmd = simple_controller.step_controller(in);
    auto res = std::get_if<core::SpeedControlOut>(&cmd.out);

    ASSERT_NEAR(res->desired_rpms.FL, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.FR, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.RL, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.RR, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);

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

    ASSERT_NEAR(res->desired_rpms.FL, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.FR, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.RL, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);
    ASSERT_NEAR(res->desired_rpms.RR, std::get<float>(simple_controller.get_cached_param("positive_speed_set")) * constants::METERS_PER_SECOND_TO_RPM, 1.0);

    ASSERT_NEAR(res->torque_lim_nm.FR, 22.4, 2.0);
    ASSERT_NEAR(res->torque_lim_nm.FL, 22.4, 2.0);
    ASSERT_NEAR(res->torque_lim_nm.RR, 22.4, 2.0);
    ASSERT_NEAR(res->torque_lim_nm.RL, 22.4, 2.0);
}



TEST_F(SimpleSpeedControllerTest, TestPowerLimit)
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

    auto res = std::get<core::SpeedControlOut>(simple_controller.step_controller(in).out);

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
