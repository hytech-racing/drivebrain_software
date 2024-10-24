#pragma once
#include <Controller.hpp>
#include <Configurable.hpp>
#include <Logger.hpp>
#include <hytech_msgs.pb.h>
#include <VehicleDataTypes.hpp>
#include <utility>
#include <mutex>

// ABOUT: this controller is an implementation of mode 0

// this controller will be reactionary for now
namespace control
{

    // TODO make the output CAN message for the drivetrain, rpms telem is just a standin for now
    class SimpleController : public Controller<core::ControllerOutput, core::VehicleState>, public core::common::Configurable
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
        SimpleController(core::Logger &logger, core::JsonFileHandler &json_file_handler) : Configurable(logger, json_file_handler, "SimpleController") {}
        SimpleController(core::Logger &logger, core::JsonFileHandler &json_file_handler, std::string config) : Configurable(logger, json_file_handler, config) {}
        float get_dt_sec() override { 
            return (0.001); 
        }
        bool init() override;
        core::ControllerOutput step_controller(const core::VehicleState &in) override;

    private:
        void _handle_param_updates(const std::unordered_map<std::string, core::common::Configurable::ParamTypes> &new_param_map);
    private:
        std::mutex _config_mutex;
        config _config;
    };
}