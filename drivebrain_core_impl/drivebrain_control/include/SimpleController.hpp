#pragma once
#include <Controller.hpp>
#include <Configurable.hpp>
#include <hytech.pb.h>
// ABOUT: this controller is an implementation of mode 0

// this controller will be reactionary 
namespace control
{

// TODO make the output CAN message for the drivetrain, rpms telem is just a standin for now
class SimpleController : Controller<drivetrain_command, mcu_pedal_readings>, public core::common::Configurable
{
public:
    struct config {
        float max_torque;
        float max_reg_torque;
    };
    SimpleController(core::JsonFileHandler &json_file_handler) : Configurable(json_file_handler, "SimpleController") {}

    bool init();
    drivetrain_command step_controller(const mcu_pedal_readings& in) override;
private:
    config _config;
};
}