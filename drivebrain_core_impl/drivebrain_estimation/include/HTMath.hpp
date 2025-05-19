#ifndef __HTMATH_H__
#define __HTMATH_H__

#include <Utils.hpp>
namespace math
{
    template <typename T>
    float linear_approx(T x_val, T scale, T offset)
    {
        return ((static_cast<float>(x_val) * static_cast<float>(scale)) + (static_cast<float>(offset)));
    }
    template<typename T>
    float normalize_linear_scale(T input_value, T min_val, T max_val, float new_min, float new_max)
    {
        if (max_val == min_val) return 0; // avoid divide by zero
        float normalized = (static_cast<float>(input_value) - static_cast<float>(min_val)) / (static_cast<float>(max_val) - static_cast<float>(min_val));
        float scaled = (new_min + (normalized * (new_max - new_min)));
        return scaled;
    }
    template <typename T>
    float linear_fit_normal(T input_value, T min_val, T max_val)
    {
        return normalize_linear_scale(input_value, min_val, max_val, 0, 1);
    }
    
};

#endif // __HTMATH_H__