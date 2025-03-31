#include "SimpleSpeedController.hpp"
#include "SimpleTorqueController.hpp"

#include <Controller.hpp>
#include <ControllerManager.hpp>
#include <JsonFileHandler.hpp>
#include <Logger.hpp>
#include <JsonFileHandler.hpp>
#include <MsgLogger.hpp>
#include <VehicleDataTypes.hpp>
#include <cstddef>
#include <hytech.pb.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <chrono>
core::JsonFileHandler _config("../config/test_tcmux_integration.json");
core::Logger _logger(core::LogLevel::INFO);
control::SimpleTorqueController controller1(_logger, _config);
control::SimpleSpeedController controller2(_logger, _config);

  // std::cout << _controllerManager.ma

         // for 1000 rpm
        // _max_switch_rpm = ((*max_switch_speed) * constants::METERS_PER_SECOND_TO_RPM);
        // METERS_PER_SECOND_TO_RPM    = 1.0 / RPM_TO_METERS_PER_SECOND;
        // RPM_TO_METERS_PER_SECOND    = WHEEL_DIAMETER * 3.1415 / GEARBOX_RATIO / 60.0;
        // GEARBOX_RATIO               = 11.86;
        // WHEEL_DIAMETER              = 0.4064;


int main(int argc, char **argv) {
    controller1.init();
    controller2.init();
    // important to init controllers. maybe put this in constructor?
    control::ControllerManager<control::Controller<core::ControllerOutput, core::VehicleState>, 2 > _controllerManager(
        _logger, _config, {&controller1, &controller2}
    );
    _controllerManager.init();





    core::VehicleState vehicle_state = {};
    vehicle_state.is_ready_to_drive = true;
    vehicle_state.input.requested_accel = 0.2;
    vehicle_state.input.requested_brake = 0.0;
    vehicle_state.state_is_valid = true;
    std::cout << "Testing torque controller (index 0)" << std::endl;
  
    vehicle_state.current_rpms = {1000, 1000, 1000, 1000};
    _controllerManager.swap_active_controller(1, vehicle_state);
    std::cout << "Expected output: 0(failure due to high rpms). Actual output: " << _controllerManager.get_active_controller_index() << std::endl;
    
  
    vehicle_state.current_rpms = {990, 990, 990, 990};
    _controllerManager.swap_active_controller(1, vehicle_state);
    std::cout << "Expected output: 0(failure due to high accel). Actual output: " << _controllerManager.get_active_controller_index() << std::endl;
  
    vehicle_state.input.requested_accel = 0.0;
  
    _controllerManager.swap_active_controller(0, vehicle_state);
    std::cout << "Expected output: 0(failure due to same controller). Actual output: " << _controllerManager.get_active_controller_index() << std::endl;
    // controller1.get_config();
    vehicle_state.current_rpms = {990, 990, 990, 990};
    _controllerManager.swap_active_controller(1, vehicle_state);
    std::cout << "Expected output: 1(success). Actual output: " << _controllerManager.get_active_controller_index() << std::endl;
  
  
  
  
    std::cout << "Current controller index: " << _controllerManager.get_active_controller_index() << std::endl;
    // vehicle_state.current_rpms = {0, 0, 0, 0};
  
    _controllerManager.swap_active_controller(0, vehicle_state);
    std::cout << "Expected output: 0(success). Actual output: " << _controllerManager.get_active_controller_index() << std::endl;
    std::cout << "Current controller index: " << _controllerManager.get_active_controller_index() << std::endl;

    return 0;
}