#pragma once

#include <drivebrain_core/Controller.hpp>
#include <drivebrain_core/Configurable.hpp>
#include <drivebrain_core/VehicleDataTypes.hpp>

namespace control {

class TorqueController : public core::common::Configurable, public core::control::Controller<core::ControllerOutput, core::VehicleState> {
public:
    TorqueController(core::Logger& logger, core::JsonFileHandler& config) 
        : Configurable(logger, config, "TorqueController") {}

    virtual ~TorqueController() = default;

    // Required by Controller base class
    virtual float get_dt_sec() override = 0;
    virtual core::ControllerOutput step_controller(const core::VehicleState& input) override = 0;

    // Required by Configurable base class
    virtual bool init() override = 0;
};

} // namespace control 