#include <gtest/gtest.h>
#include "ControllerManager.hpp"
#include <VehicleDataTypes.hpp>

class MockController {
public:
    MockController(float dt) : _dt_sec(dt), _output({}) {}

    float get_dt_sec() const {
        return _dt_sec;
    }

    core::ControllerOutput step_controller(const core::VehicleState& state) {
        return _output;
    }

    void set_output(const core::ControllerOutput& output) {
        _output = output;
    }

private:
    float _dt_sec;
    core::ControllerOutput _output;
};

class ControllerManagerTest : public ::testing::Test {
protected:
    MockController torquecontroller1; 
    MockController torquecontroller2; 
    MockController speedcontroller1;
    MockController speedcontroller2; 
    std::array<MockController*, 2> controllers;
    control::ControllerManager<MockController, 2> controller_manager; 
    core::JsonFileHandler json_file_handler; 
    core::Logger logger; 

    core::VehicleState vehicle_state;
    core::ControllerOutput torque_controller_output;
    core::ControllerOutput speed_controller_output;

    ControllerManagerTest()
        : torquecontroller1(0.01f),
          speedcontroller1(0.02f),
          controllers( { &controller1, &controller2 } ),
          logger(core::LogLevel::NONE),
          json_file_handler("../config/test_config/can_driver.json"),
          controller_manager(logger, json_file_handler, controllers) 
    {
    }

    void SetUp() override {
        vehicle_state.is_ready_to_drive = true;
        vehicle_state.input.requested_accel = 0.0;
        vehicle_state.input.requested_brake = 0.0;
        vehicle_state.current_rpms = {1000, 1000, 1000, 1000};

        torque_controller_output.out = core::TorqueControlOut{{10, 10, 10, 10}};
        torquecontroller1.set_output(torque_controller_output);
        torquecontroller2.set_output(torque_controller_output);
        speed_controller_output.out = core::SpeedControlOut{{4, 4, 4, 4}, {10, 10, 10, 10}, {10, 10, 10, 10}};
        speedcontroller1.set_output(speed_controller_output);
        speedcontroller2.set_output(speed_controller_output);

        controller_manager.init();
        // std::cout << "set up" << std::endl;
    }

    void TearDown() override {
        // Clean up after each test if necessary
    }
};

// Test the initialization
TEST_F(ControllerManagerTest, InitializationSuccess) {
    ASSERT_TRUE(controller_manager.init());
}

// Test active controller timestep retrieval
TEST_F(ControllerManagerTest, GetActiveControllerTimestep) {
    EXPECT_EQ(controller_manager.get_active_controller_timestep(), 0.01f);

    controller_manager.swap_active_controller(1, vehicle_state);
    EXPECT_EQ(controller_manager.get_active_controller_timestep(), 0.02f);
}

// Test stepping the active controller
TEST_F(ControllerManagerTest, StepActiveController) {
    core::ControllerOutput output = controller_manager.step_active_controller(vehicle_state);
    
    ASSERT_TRUE(std::holds_alternative<core::TorqueControlOut>(output.out));
    auto torque_output = std::get<core::TorqueControlOut>(output.out);
    EXPECT_EQ(torque_output.desired_torques_nm.FL, 10);
    EXPECT_EQ(torque_output.desired_torques_nm.FR, 10);
    EXPECT_EQ(torque_output.desired_torques_nm.RL, 10);
    EXPECT_EQ(torque_output.desired_torques_nm.RR, 10);
}

// Test switching controllers
TEST_F(ControllerManagerTest, SwapControllerSuccess) {
    vehicle_state.current_rpms = {100, 100, 100, 100};
    ASSERT_TRUE(controller_manager.swap_active_controller(1, vehicle_state));

    core::ControllerOutput output = controller_manager.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::SpeedControlOut>(output.out));
}

// Test out of range index
TEST_F(ControllerManagerTest, SwapControllerFailure_OutOfRange) {
    ASSERT_FALSE(controller_manager.swap_active_controller(2, vehicle_state));
    EXPECT_EQ(controller_manager.get_current_ctr_manager_state().current_status, core::control::ControllerManagerStatus::ERROR_CONTROLLER_INDEX_OUT_OF_RANGE);
}

// Test high RPM
TEST_F(ControllerManagerTest, SwapControllerFailure_HighRPM) {
    vehicle_state.current_rpms = {100000, 10000, 10000, 10000};

    ASSERT_FALSE(controller_manager.swap_active_controller(1, vehicle_state));
    EXPECT_EQ(controller_manager.get_current_ctr_manager_state().current_status, core::control::ControllerManagerStatus::ERROR_SPEED_DIFF_TOO_HIGH);
}

// Test pausing the controller stepping
TEST_F(ControllerManagerTest, PauseStepping) {
    ASSERT_TRUE(controller_manager.pause_evaluating());
    ASSERT_FALSE(controller_manager.pause_evaluating());

    core::ControllerOutput output = controller_manager.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::TorqueControlOut>(output.out)); // No output after pausing
}

// Test unpausing the controller stepping
TEST_F(ControllerManagerTest, UnpauseStepping) {
    controller_manager.pause_evaluating();
    ASSERT_TRUE(controller_manager.unpause_evaluating());
    ASSERT_FALSE(controller_manager.unpause_evaluating());

    core::ControllerOutput output = controller_manager.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::TorqueControlOut>(output.out));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}