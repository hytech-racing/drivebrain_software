#pragma once
#include <Controller.hpp>
#include <Configurable.hpp>
#include <Literals.hpp>
#include <hytech.pb.h>
#include <VehicleDataTypes.hpp>
#include <utility>

#include <chrono>
// ABOUT: this controller is an implementation of mode 0

// this controller will be reactionary for now 
namespace control
{

// TODO make the output CAN message for the drivetrain, rpms telem is just a standin for now
class SimpleController : Controller<std::pair<drivebrain_torque_lim_input, drivebrain_speed_set_input>, core::VehicleState>, public core::common::Configurable
{
public:
    
    // rear_torque_scale:
        // 0 to 2 scale on forward torque to rear wheels. 0 = FWD, 1 = Balanced, 2 = RWD
    
    // regen_torque_scale:
        // same as rear_torque_scale but applies to regen torque split. 0 = All regen 
        // torque on the front, 1 = 50/50, 2 = all regen torque on the rear
    struct config {
        torque_nm max_torque;
        torque_nm max_reg_torque;
        float rear_torque_scale;  
        float regen_torque_scale; 
        speed_m_s positive_speed_set;
    };

    SimpleController(core::JsonFileHandler &json_file_handler) : Configurable(json_file_handler, "SimpleController") {}

    
    float get_dt_sec() override { 
        using namespace std::literals;
        return (0.01s).count(); 
    }
    bool init();
    std::pair<drivebrain_torque_lim_input, drivebrain_speed_set_input> step_controller(const core::VehicleState &in) override;
private:
    void _handle_param_updates(const std::unordered_map<std::string, core::common::Configurable::ParamTypes> &new_param_map);
    config _config;
};
}