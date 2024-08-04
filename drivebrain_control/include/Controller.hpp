#pragma once
#include <Configurable.hpp>
// virtual base class for a controller

// we want to be able to have controllers that require the drivetrain to be in a certain state / control mode
// we want to be able to output different types of output from each controller.
// the controller should have access to the parameter server and be "Configurable"

namespace control
{
    template <typename ControllerResult, typename ControllerInput>
    class Controller : common::Configurable
    {
    public:
        virtual ControllerResult stepController(const ControllerInput &in) = 0;
    };
}