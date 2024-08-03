#pragma once

#include <array>
#include <memory>

#include <Controller.hpp>

// shall handle ability to be constructed
namespace control
{
    class ControllerManager
    {

        //
        ControllerManager(std::shared_ptr<Controller> controller, ...);
        ~ControllerManager() = default;
    };
}