#ifndef __JSONUTILS_H__
#define __JSONUTILS_H__

#include <vector>

#define LOAD_PARAM_OR_FAIL(param_name, param_type, config_struct)                                  \
    {                                                                                              \
        auto opt = get_parameter_value<param_type>(#param_name);                                   \
        if (!opt)                                                                                  \
            return false;                                                                          \
        config_struct.param_name = *opt;                                                           \
    }

#endif // __JSONUTILS_H__