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
    MockController controller1; 
    MockController controller2; 
    std::array<MockController*, 2> controllers;
    control::ControllerManager<MockController, 2> controller_manager; 
    core::JsonFileHandler json_file_handler; 
    core::Logger logger; 

    core::VehicleState vehicle_state;
    core::ControllerOutput controller_output;

    ControllerManagerTest()
        : controller1(0.01f),
          controller2(0.02f),
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

        controller_output.out = core::TorqueControlOut{{10, 10, 10, 10}};
        controller1.set_output(controller_output);

        controller_manager.init();
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
    ASSERT_TRUE(std::holds_alternative<core::TorqueControlOut>(output.out));
}

// Test out of range index
TEST_F(ControllerManagerTest, SwapControllerFailure_OutOfRange) {
    ASSERT_FALSE(controller_manager.swap_active_controller(2, vehicle_state));
    EXPECT_EQ(controller_manager.get_current_car_state().current_status, core::control::ControllerManagerStatus::ERROR_CONTROLLER_INDEX_OUT_OF_RANGE);
}

// Test high RPM
TEST_F(ControllerManagerTest, SwapControllerFailure_HighRPM) {
    vehicle_state.current_rpms = {10000, 10000, 10000, 10000};

    ASSERT_FALSE(controller_manager.swap_active_controller(1, vehicle_state));
    EXPECT_EQ(controller_manager.get_current_car_state().current_status, core::control::ControllerManagerStatus::ERROR_SPEED_DIFF_TOO_HIGH);
}

// Test pausing the controller stepping
TEST_F(ControllerManagerTest, PauseStepping) {
    ASSERT_TRUE(controller_manager.pause_stepping());
    ASSERT_FALSE(controller_manager.pause_stepping());

    core::ControllerOutput output = controller_manager.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<std::monostate>(output.out)); // No output after pausing
}

// Test unpausing the controller stepping
TEST_F(ControllerManagerTest, UnpauseStepping) {
    controller_manager.pause_stepping();
    ASSERT_TRUE(controller_manager.unpause_stepping());
    ASSERT_FALSE(controller_manager.unpause_stepping());

    core::ControllerOutput output = controller_manager.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::TorqueControlOut>(output.out));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}