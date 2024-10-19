#ifndef __UTILS_H__
#define __UTILS_H__

// yoinked from: https://github.com/hytech-racing/MCU/blob/f36ebdafc33327b2d57ce9240225160c66afc9fa/lib/systems/include/Utility.h

// TODO turn these into functions 
namespace constants
{
    const float GEARBOX_RATIO               = 11.86;
    const float WHEEL_DIAMETER              = 0.4064; // meters
    const float RPM_TO_METERS_PER_SECOND    = WHEEL_DIAMETER * 3.1415 / GEARBOX_RATIO / 60.0;
    const float RPM_TO_KILOMETERS_PER_HOUR  = RPM_TO_METERS_PER_SECOND * 3600.0 / 1000.0;
    const float METERS_PER_SECOND_TO_RPM    = 1.0 / RPM_TO_METERS_PER_SECOND;

    const float RPM_TO_RAD_PER_SECOND = 2 * 3.1415 / 60.0;
    const float RAD_PER_SECOND_TO_RPM = 1 / RPM_TO_RAD_PER_SECOND;
}

/// @brief generic data vector type that can be used with tire and / or anything to do with 4 corners of the car.
template <typename T>
struct veh_vec
{
    T FL;
    T FR;
    T RL;
    T RR;
};

template <typename T>
struct xyz_vec
{
    T x;
    T y;
    T z;
};

template <typename T>
struct ypr_vec 
{   
    T yaw;
    T pitch;
    T roll;
};


#endif // __UTILS_H__