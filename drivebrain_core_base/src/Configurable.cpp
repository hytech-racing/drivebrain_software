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

Configurable::ParamTypes Configurable::get_cached_param(std::string id)
{
    std::unique_lock lk(_live_params.mtx);
    if(_live_params.param_vals.find(id) != _live_params.param_vals.end())
    {
        return _live_params.param_vals[id]; 
    } else {
        return std::monostate();
    }
}

std::unordered_map<std::string, Configurable::ParamTypes> Configurable::get_params_map()
{
    std::unique_lock lk(_live_params.mtx);
    return _live_params.param_vals;
}

void Configurable::handle_live_param_update(const std::string &key, Configurable::ParamTypes param_val)
{
    
    // TODO may want to handle this a little bit differently so we arent locking so much
    {
        std::unique_lock lk(_live_params.mtx);
        _live_params.param_vals[key] = param_val;
        param_update_handler_sig(_live_params.param_vals);
    }
    
}