#include <Controller.hpp>

// ABOUT: this controller is an implementation of mode 0

// this controller will be reactionary 
namespace control
{

class SimpleController : Controller<SpeedControlOut, SpeedControlIn>
{

public:
    SpeedControlOut step_controller(const SpeedControlIn& in);
    
};
}