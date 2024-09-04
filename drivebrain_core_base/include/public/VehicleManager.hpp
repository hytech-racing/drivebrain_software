#ifndef __VEHICLEMANAGER_H__
#define __VEHICLEMANAGER_H__
#include <ControllerManager.hpp>


// super loop management:

// the drivers are ran within one asio context and update the generic state
    // the generic state will be an unordered map that has a string identifier and a state vector
    // the value in the unordered map will be a state vector; the state vector is a regular std::vector
    // that has state variables; the state variables will be variants of primitive types;
        // in theory i could have a vehicle state schema that is derived from all of the message definitions,
        // however I think this generic state will work for now even if it is a little in-efficient

class VehicleManager 
{
    public:
        VehicleManager() = default;
        ~VehicleManager() = default;
        

        
    private:

};
#endif // __VEHICLEMANAGER_H__