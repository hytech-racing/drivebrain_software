#include <gtest/gtest.h>
#include "ControllerManager.hpp"
#include "Controller.hpp"
#include "Controllers.hpp"
#include "SimpleController.hpp"
#include <VehicleDataTypes.hpp>
#include "VehicleDataTypes.hpp"

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
    std::array<MockController*, 4> controllers4;
    control::ControllerManager<MockController, 4> controller_manager_4;
    control::SimpleController simpleController1;
    control::SimpleController simpleController2;
    std::array<control::Controller<core::ControllerOutput, core::VehicleState>*, 2> controllers_real;
    control::ControllerManager<control::Controller<core::ControllerOutput, core::VehicleState>, 2> controller_manager_real;
    core::JsonFileHandler json_file_handler; 
    core::Logger logger; 

    core::VehicleState vehicle_state;
    core::ControllerOutput torque_controller_output;
    core::ControllerOutput speed_controller_output;

    ControllerManagerTest()
        : torquecontroller1(0.01f),
          speedcontroller1(0.02f),
          torquecontroller2(0.03f),
          speedcontroller2(0.04f),
          controllers( { &torquecontroller1, &speedcontroller1 } ),
          controllers4( { &torquecontroller1, &speedcontroller1, &torquecontroller2, &speedcontroller2 } ),
          logger(core::LogLevel::NONE),
          json_file_handler("../config/drivebrain_config.json"),
          controller_manager(logger, json_file_handler, controllers),
          controller_manager_4(logger, json_file_handler, controllers4),
          simpleController1(logger, json_file_handler),
          simpleController2(logger, json_file_handler),
          controllers_real ({ &simpleController1, &simpleController2 }),
          controller_manager_real(logger, json_file_handler, controllers_real) 
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
        speed_controller_output.out = core::SpeedControlOut{0, {4, 4, 4, 4}, {10, 10, 10, 10}};
        speedcontroller1.set_output(speed_controller_output);
        speedcontroller2.set_output(speed_controller_output);

        controller_manager.init();
        controller_manager_4.init();
        controller_manager_real.init();
        simpleController1.init();
        simpleController2.init();
        // std::cout << "set up" << std::endl;
    }

    void set_torqueout_too_high(MockController* mock)
    {
        torque_controller_output.out = core::TorqueControlOut{{101, 101, 101, 101}};
        mock->set_output(torque_controller_output);
    }

    void set_speedout_too_high(MockController* mock)
    {
        speed_controller_output.out = core::SpeedControlOut{0, {3000, 3000, 3000, 3000}, {10, 10, 10, 10}};
        mock->set_output(speed_controller_output);
    }

    void TearDown() override {
        // Clean up after each test if necessary
    }
};

// Test the initialization
TEST_F(ControllerManagerTest, InitializationSuccess) {
    ASSERT_TRUE(controller_manager.init());
    ASSERT_TRUE(controller_manager_4.init());
    ASSERT_TRUE(controller_manager_real.init());
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

//swap between same controller outputs
TEST_F(ControllerManagerTest, SwapSameTypes) {
    vehicle_state.current_rpms = {100, 100, 100, 100};
    ASSERT_TRUE(controller_manager_4.swap_active_controller(2, vehicle_state));

    core::ControllerOutput output = controller_manager_4.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::TorqueControlOut>(output.out));
}

// Test switching controllers
TEST_F(ControllerManagerTest, SwapBetweenTypes) {
    vehicle_state.current_rpms = {100, 100, 100, 100};
    ASSERT_TRUE(controller_manager.swap_active_controller(1, vehicle_state));

    core::ControllerOutput output = controller_manager.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::SpeedControlOut>(output.out));

    ASSERT_TRUE(controller_manager.swap_active_controller(0, vehicle_state));

    output = controller_manager.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::TorqueControlOut>(output.out));
}

//switching with different controllers where current controller torque too high
TEST_F(ControllerManagerTest, SwapTorqueTooHigh) {
    vehicle_state.current_rpms = {100, 100, 100, 100};
    set_torqueout_too_high(&torquecontroller1);
    ASSERT_FALSE(controller_manager.swap_active_controller(1, vehicle_state));

    core::ControllerOutput output = controller_manager.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::TorqueControlOut>(output.out));
}

//switching with different controllers where desired 
TEST_F(ControllerManagerTest, SwapSpeedTooHigh) {
    vehicle_state.current_rpms = {100, 100, 100, 100};
    set_speedout_too_high(&speedcontroller1);
    ASSERT_FALSE(controller_manager.swap_active_controller(1, vehicle_state));

    core::ControllerOutput output = controller_manager.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::TorqueControlOut>(output.out));
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

//test foot on accelerator over/under threshold
TEST_F(ControllerManagerTest, SwapAccelerator) {
    vehicle_state.input.requested_accel = .3;
    ASSERT_FALSE(controller_manager.swap_active_controller(1, vehicle_state));
    EXPECT_EQ(controller_manager.get_current_ctr_manager_state().current_status, core::control::ControllerManagerStatus::ERROR_DRIVER_ON_PEDAL);

    vehicle_state.input.requested_accel = .1;
    ASSERT_TRUE(controller_manager.swap_active_controller(1, vehicle_state));
    EXPECT_EQ(controller_manager.get_current_ctr_manager_state().current_status, core::control::ControllerManagerStatus::NO_ERROR);
}

//real conroller stuff
TEST_F(ControllerManagerTest, StepSimpleController) {
    vehicle_state.input.requested_accel = .2;
    core::ControllerOutput output = controller_manager_real.step_active_controller(vehicle_state);
    
    ASSERT_TRUE(std::holds_alternative<core::SpeedControlOut>(output.out));
    auto _output = std::get<core::SpeedControlOut>(output.out);
    ASSERT_TRUE(_output.desired_rpms.FL - constants::METERS_PER_SECOND_TO_RPM * 3 < 10.0);
    ASSERT_TRUE(_output.desired_rpms.FR - constants::METERS_PER_SECOND_TO_RPM * 3 < 10.0);
    ASSERT_TRUE(_output.desired_rpms.RL - constants::METERS_PER_SECOND_TO_RPM * 3 < 10.0);
    ASSERT_TRUE(_output.desired_rpms.RR - constants::METERS_PER_SECOND_TO_RPM * 3 < 10.0);
}

TEST_F(ControllerManagerTest, SwapSimpleControllers) {
    vehicle_state.current_rpms = {100, 100, 100, 100};
    ASSERT_TRUE(controller_manager_real.swap_active_controller(1, vehicle_state));

    core::ControllerOutput output = controller_manager_real.step_active_controller(vehicle_state);
    ASSERT_TRUE(std::holds_alternative<core::SpeedControlOut>(output.out));
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}