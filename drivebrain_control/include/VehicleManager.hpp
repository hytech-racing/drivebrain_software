#pragma once

#include <array>
#include <memory>

#include <Controller.hpp>
#include <Configurable.hpp>

// requirements:
// shall handle ability to be constructed to contain the controllers, handle providing them the inputs from state estimation and / or drivers
// controller manager shall not affect the output of any controller. 
// each controller shall be able to handle re-configuration at runtime, even while being the "active" controller, however the controller manager shall handle when its configuration gets updated
// vehicle manager shall handle stepping of each controller at their desired rates and be able to switch between
// vehicle manager shall be able to change the mode of the drivetrain dependent on what mode the desired controller wants the drivetrain to be in (speed / torque)

namespace control
{
    template <size_t NumControllers>
    class VehicleManager : common::Configurable
    {
    public:
        VehicleManager(std::array<std::unique_ptr<Controller>, NumControllers> controllers): _controllers(std::move(controllers)) {}
        ~VehicleManager() = default;

        // void stepActiveController();

    private:
        std::array<Controller, NumControllers> _controllers;
    };
}