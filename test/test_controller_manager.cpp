#include <ControllerManager.hpp>
#include <JsonFileHandler.hpp>
#include <Logger.hpp>
#include "SimpleTorqueController.hpp"
#include <JsonFileHandler.hpp>
#include <MsgLogger.hpp>
#include <VehicleDataTypes.hpp>
#include <cstdio>
#include <memory>
#include <hytech.pb.h>
#include <chrono>
core::JsonFileHandler _config("../config/drivebrain_config.json");
core::Logger _logger(core::LogLevel::INFO);
control::SimpleTorqueController controller1(_logger, _config);
control::SimpleTorqueController controller2(_logger, _config);
std::shared_ptr<core::MsgLogger<std::string>> _message_logger;
//std::unique_ptr<core::StateEstimator> _state_estimator;



int main(int argc, char **argv) {
    _message_logger = std::make_shared<core::MsgLogger<std::string>>(
        ".txt", true, std::printf, std::printf, std::printf, std::printf
    );
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

        core::ControllerOutput torque_cmd = _controllerManager.step_active_controller(vehicle_state);
        auto speed_cmd_out = {0, std::get<core::TorqueControlOut>(torque_cmd.out).desired_torques_nm, std::get<core::TorqueControlOut>(torque_cmd.out).desired_torques_nm};
        auto temp_desired_torques = vehicle_state.matlab_math_temp_out;


        if(temp_desired_torques.res_torque_lim_nm.FL < 0) {
            desired_rpm_msg->set_drivebrain_set_rpm_fl(0);
        } else {
            desired_rpm_msg->set_drivebrain_set_rpm_fl(speed_cmd_out.desired_rpms.FL);
        }

        if(temp_desired_torques.res_torque_lim_nm.FR < 0) {
            desired_rpm_msg->set_drivebrain_set_rpm_fr(0);
        } else {
            desired_rpm_msg->set_drivebrain_set_rpm_fr(speed_cmd_out.desired_rpms.FR);
        }

        if(temp_desired_torques.res_torque_lim_nm.RL < 0) {
            desired_rpm_msg->set_drivebrain_set_rpm_rl(0);
        } else {
            desired_rpm_msg->set_drivebrain_set_rpm_rl(speed_cmd_out.desired_rpms.RL);
        }

        if(temp_desired_torques.res_torque_lim_nm.RR < 0) {
            desired_rpm_msg->set_drivebrain_set_rpm_rr(0);
        } else {
            desired_rpm_msg->set_drivebrain_set_rpm_rr(speed_cmd_out.desired_rpms.RR);
        }

        torque_limit_msg->set_drivebrain_torque_fl(::abs(temp_desired_torques.res_torque_lim_nm.FL));
        torque_limit_msg->set_drivebrain_torque_fr(::abs(temp_desired_torques.res_torque_lim_nm.FR));
        torque_limit_msg->set_drivebrain_torque_rl(::abs(temp_desired_torques.res_torque_lim_nm.RL));
        torque_limit_msg->set_drivebrain_torque_rr(::abs(temp_desired_torques.res_torque_lim_nm.RR));
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto delta = end_time - start_time;
    }
    _controllerManager.swap_active_controller(1, vehicle_state);



    return 0;
}