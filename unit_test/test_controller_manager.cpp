#include <gtest/gtest.h>
#include "ControllerManager.hpp"
#include "Controller.hpp"
#include "Controllers.hpp"
#include "SimpleSpeedController.hpp"
#include "SimpleTorqueController.hpp"

#include <VehicleDataTypes.hpp>

class ControllerManagerTest : public ::testing::Test {
protected:
    control::SimpleSpeedController simpleSpeedController1;
    control::SimpleTorqueController simpleTorqueController1;
    control::SimpleSpeedController simpleSpeedController2;
    control::SimpleTorqueController simpleTorqueController2;
    control::ControllerManager<control::Controller<core::ControllerOutput, core::VehicleState>, 2> controller_manager_2speed;
    control::ControllerManager<control::Controller<core::ControllerOutput, core::VehicleState>, 2> controller_manager_2torque;
    control::ControllerManager<control::Controller<core::ControllerOutput, core::VehicleState>, 2> controller_manager_diff;
    core::JsonFileHandler json_file_handler; 
    core::Logger logger; 

    core::VehicleState vehicle_state;
    core::ControllerOutput torque_controller_output;
    core::ControllerOutput speed_controller_output;

    ControllerManagerTest()
        : logger(core::LogLevel::NONE),
          json_file_handler("../config/drivebrain_config.json"),
          simpleSpeedController1(logger, json_file_handler),
          simpleSpeedController2(logger, json_file_handler),
          simpleTorqueController1(logger, json_file_handler),
          simpleTorqueController2(logger, json_file_handler),
          controller_manager_2speed(logger, json_file_handler, {&simpleSpeedController1, &simpleSpeedController2}),
          controller_manager_2torque(logger, json_file_handler, {&simpleTorqueController1, &simpleTorqueController2}),
          controller_manager_diff(logger, json_file_handler, {&simpleSpeedController1, &simpleTorqueController1})
    {
    }

    void SetUp() override {
        vehicle_state.is_ready_to_drive = true;
        vehicle_state.input.requested_accel = 0.0;
        vehicle_state.input.requested_brake = 0.0;
        vehicle_state.current_rpms = {1000, 1000, 1000, 1000};

        controller_manager_2speed.init();
        controller_manager_2torque.init();
        controller_manager_diff.init();
        simpleSpeedController1.init();
        simpleSpeedController2.init();
        simpleTorqueController1.init();
        simpleTorqueController2.init();
        
        // std::cout << "set up" << std::endl;
    }

    void TearDown() override {
        // Clean up after each test if necessary
    }
};

// Test the initialization
TEST_F(ControllerManagerTest, InitializationSuccess) {
    ASSERT_TRUE(controller_manager_2speed.init());
    ASSERT_TRUE(controller_manager_2torque.init());
    ASSERT_TRUE(controller_manager_diff.init());
}

// Test active controller timestep retrieval
TEST_F(ControllerManagerTest, GetActiveControllerTimestep) {
    EXPECT_EQ(controller_manager_2speed.get_active_controller_timestep(), 0.001f);
    EXPECT_EQ(controller_manager_2torque.get_active_controller_timestep(), 0.001f);
    EXPECT_EQ(controller_manager_diff.get_active_controller_timestep(), 0.001f);
}

// Test stepping the active controller
TEST_F(ControllerManagerTest, StepActiveTorqueController) {
    vehicle_state.input.requested_accel = 1.0;
    core::ControllerOutput output = controller_manager_2torque.step_active_controller(vehicle_state);
    
    ASSERT_TRUE(std::holds_alternative<core::TorqueControlOut>(output.out));
    auto torque_output = std::get<core::TorqueControlOut>(output.out);
    EXPECT_EQ(torque_output.desired_torques_nm.FL, 21.0);
    EXPECT_EQ(torque_output.desired_torques_nm.FR, 21.0);
    EXPECT_EQ(torque_output.desired_torques_nm.RL, 21.0);
    EXPECT_EQ(torque_output.desired_torques_nm.RR, 21.0);
}

TEST_F(ControllerManagerTest, StepActiveSpeedController) {
    vehicle_state.input.requested_accel = 1.0;
    core::ControllerOutput output = controller_manager_2speed.step_active_controller(vehicle_state);
    
    ASSERT_TRUE(std::holds_alternative<core::SpeedControlOut>(output.out));
    auto speed_output = std::get<core::SpeedControlOut>(output.out);
    EXPECT_EQ(speed_output.torque_lim_nm.FL, 21.0);
    EXPECT_EQ(speed_output.torque_lim_nm.FR, 21.0);
    EXPECT_EQ(speed_output.torque_lim_nm.RL, 21.0);
    EXPECT_EQ(speed_output.torque_lim_nm.RR, 21.0);
}

//swap between same controller outputs
TEST_F(ControllerManagerTest, SwapSameTypes) {
    vehicle_state.current_rpms = {100, 100, 100, 100};
    ASSERT_FALSE(controller_manager_diff.swap_active_controller(2, vehicle_state));

    core::ControllerOutput output = controller_manager_diff.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::SpeedControlOut>(output.out));
}

//swap between same controller outputs
TEST_F(ControllerManagerTest, SwapSameIndex) {
    vehicle_state.current_rpms = {100, 100, 100, 100};
    ASSERT_FALSE(controller_manager_diff.swap_active_controller(0, vehicle_state));

    core::ControllerOutput output = controller_manager_diff.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::SpeedControlOut>(output.out));
}

// Test switching controllers
TEST_F(ControllerManagerTest, SwapBetweenTypes) {
    vehicle_state.current_rpms = {100, 100, 100, 100};
    ASSERT_TRUE(controller_manager_diff.swap_active_controller(1, vehicle_state));

    core::ControllerOutput output = controller_manager_diff.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::TorqueControlOut>(output.out));

    ASSERT_TRUE(controller_manager_diff.swap_active_controller(0, vehicle_state));

    output = controller_manager_diff.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::SpeedControlOut>(output.out));
}

//switching where kachow
TEST_F(ControllerManagerTest, SwapSpeedTooHigh) {
    vehicle_state.current_rpms = {10000, 10000, 10000, 10000};
    ASSERT_FALSE(controller_manager_diff.swap_active_controller(1, vehicle_state));

    core::ControllerOutput output = controller_manager_diff.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::SpeedControlOut>(output.out));
}

// Test out of range index
TEST_F(ControllerManagerTest, SwapControllerFailure_OutOfRange) {
    ASSERT_FALSE(controller_manager_diff.swap_active_controller(2, vehicle_state));
    EXPECT_EQ(controller_manager_diff.get_current_ctr_manager_state().current_status, core::control::ControllerManagerStatus::ERROR_CONTROLLER_INDEX_OUT_OF_RANGE);

    ASSERT_FALSE(controller_manager_diff.swap_active_controller(-7, vehicle_state));
    EXPECT_EQ(controller_manager_diff.get_current_ctr_manager_state().current_status, core::control::ControllerManagerStatus::ERROR_CONTROLLER_INDEX_OUT_OF_RANGE);
}

// Test high RPM
TEST_F(ControllerManagerTest, SwapControllerFailure_HighRPM) {
    vehicle_state.current_rpms = {100000, 10000, 10000, 10000};

    ASSERT_FALSE(controller_manager_2speed.swap_active_controller(1, vehicle_state));
    EXPECT_EQ(controller_manager_2speed.get_current_ctr_manager_state().current_status, core::control::ControllerManagerStatus::ERROR_SPEED_TOO_HIGH);
}

TEST_F(ControllerManagerTest, SwapControllerFailure_HighRPM_onewheel) {
    vehicle_state.current_rpms = {100000, 0, 0, 0};

    ASSERT_FALSE(controller_manager_2speed.swap_active_controller(1, vehicle_state));
    EXPECT_EQ(controller_manager_2speed.get_current_ctr_manager_state().current_status, core::control::ControllerManagerStatus::ERROR_SPEED_TOO_HIGH);
}

//test foot on accelerator over/under threshold
TEST_F(ControllerManagerTest, SwapAccelerator) {
    vehicle_state.input.requested_accel = .3;
    ASSERT_FALSE(controller_manager_2speed.swap_active_controller(1, vehicle_state));
    EXPECT_EQ(controller_manager_2speed.get_current_ctr_manager_state().current_status, core::control::ControllerManagerStatus::ERROR_DRIVER_ON_PEDAL);

    vehicle_state.input.requested_accel = .1;
    ASSERT_TRUE(controller_manager_2speed.swap_active_controller(1, vehicle_state));
    EXPECT_EQ(controller_manager_2speed.get_current_ctr_manager_state().current_status, core::control::ControllerManagerStatus::NO_ERROR);
}

//real conroller stuff
TEST_F(ControllerManagerTest, StepSimpleSpeedController) {
    vehicle_state.input.requested_accel = .2;
    core::ControllerOutput output = controller_manager_2speed.step_active_controller(vehicle_state);
    
    ASSERT_TRUE(std::holds_alternative<core::SpeedControlOut>(output.out));
    auto _output = std::get<core::SpeedControlOut>(output.out);
    ASSERT_TRUE(_output.desired_rpms.FL - constants::METERS_PER_SECOND_TO_RPM * 3 < 10.0);
    ASSERT_TRUE(_output.desired_rpms.FR - constants::METERS_PER_SECOND_TO_RPM * 3 < 10.0);
    ASSERT_TRUE(_output.desired_rpms.RL - constants::METERS_PER_SECOND_TO_RPM * 3 < 10.0);
    ASSERT_TRUE(_output.desired_rpms.RR - constants::METERS_PER_SECOND_TO_RPM * 3 < 10.0);
}

TEST_F(ControllerManagerTest, SwapSimpleSpeedControllers) {
    vehicle_state.current_rpms = {100, 100, 100, 100};
    ASSERT_TRUE(controller_manager_2speed.swap_active_controller(1, vehicle_state));

    core::ControllerOutput output = controller_manager_2speed.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::SpeedControlOut>(output.out));
}

TEST_F(ControllerManagerTest, SwapDiffSimpleControllers) {
    vehicle_state.current_rpms = {100, 100, 100, 100};
    ASSERT_TRUE(controller_manager_diff.swap_active_controller(1, vehicle_state));

    core::ControllerOutput output = controller_manager_diff.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::TorqueControlOut>(output.out));
}