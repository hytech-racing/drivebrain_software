#ifndef __UTILS_H__
#define __UTILS_H__

    /// @brief generic data vector type that can be used with tire and / or anything to do with 4 corners of the car.
template <typename T>
struct veh_vec
{
    T FL;
    T FR;
    T RL;
    T RR;
};


#endif // __UTILS_H__