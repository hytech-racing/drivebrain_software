#include <Configurable.hpp>

using namespace core::common;
std::string Configurable::get_name()
{
    return _component_name;
}

std::vector<std::string> Configurable::get_param_names()
{

    std::vector<std::string> names;
    {
        std::unique_lock lk(_live_params.mtx);
        for (const auto &param : _live_params.param_vals)
        {
            names.push_back(param.first);
        }
    }
    return names;
}

std::unordered_map<std::string, Configurable::ParamTypes> Configurable::get_params_map()
{
    std::unique_lock lk(_live_params.mtx);
    return _live_params.param_vals;
}
