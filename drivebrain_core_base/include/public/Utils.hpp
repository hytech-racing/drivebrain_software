#ifndef __UTILS_H__
#define __UTILS_H__

// yoinked from: https://github.com/hytech-racing/MCU/blob/f36ebdafc33327b2d57ce9240225160c66afc9fa/lib/systems/include/Utility.h

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


#endif // __UTILS_H__