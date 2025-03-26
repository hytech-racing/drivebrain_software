#include "SimpleController.hpp"
#include <ControllerManager.hpp>
#include <JsonFileHandler.hpp>
#include <Logger.hpp>
#include <JsonFileHandler.hpp>
#include <MsgLogger.hpp>
#include <VehicleDataTypes.hpp>
#include <cstdio>
#include <memory>
#include <hytech.pb.h>
#include <chrono>
core::JsonFileHandler _config("../config/drivebrain_config.json");
core::Logger _logger(core::LogLevel::INFO);
control::SimpleController controller1(_logger, _config);
control::SimpleController controller2(_logger, _config);



int main(int argc, char **argv) {
    return 0;
  //  _state_estimator = std::make_unique<core::StateEstimator>(_logger, _message_logger);

    core::VehicleState vehicle_state = {};
    vehicle_state.is_ready_to_drive = true;
    vehicle_state.input.requested_accel = 0.0;
    vehicle_state.input.requested_brake = 0.0;
    vehicle_state.current_rpms = {1000, 1000, 1000, 1000};

    control::ControllerManager<control::Controller<core::ControllerOutput, core::VehicleState>, 2 > _controllerManager(
        _logger, _config, {&controller1, &controller2}
    );

    auto desired_rpm_msg = std::make_shared<hytech::drivebrain_speed_set_input>();
    auto torque_limit_msg = std::make_shared<hytech::drivebrain_torque_lim_input>();
    auto main_start_time = std::chrono::high_resolution_clock::now();
   
    while ((std::chrono::high_resolution_clock::now() - main_start_time) < std::chrono::seconds(5) ) {
     
        auto start_time = std::chrono::high_resolution_clock::now();

    //    core::ControllerOutput torque_cmd = _controllerManager.step_active_controller(vehicle_state);


       
        auto end_time = std::chrono::high_resolution_clock::now();
        auto delta = end_time - start_time;
    }



    return 0;
}