#include <param_server.hpp>
#include <variant>

core::FoxgloveParameterServer::FoxgloveParameterServer(std::vector<core::common::Configurable *> configurable_components) : _components(configurable_components)
{
    _log_handler = [](foxglove::WebSocketLogLevel, char const *msg)
    {
        std::cout << msg << std::endl;
    };

    _server_options.capabilities.push_back("parameters");
    _server_options.capabilities.push_back("parametersSubscribe");

    _server = foxglove::ServerFactory::createServer<websocketpp::connection_hdl>(
        "beep boop", _log_handler, _server_options);

    foxglove::ServerHandlers<foxglove::ConnHandle> hdlrs;

    hdlrs.parameterRequestHandler = [this](const std::vector<std::string> &param_names, const std::optional<std::string> &request_id,
                                           foxglove::ConnHandle clientHandle)
    {
        auto params_vec = _get_current_params();
        _server->publishParameterValues(clientHandle, params_vec, request_id);
    };

    hdlrs.parameterChangeHandler = [&](const std::vector<foxglove::Parameter> &params, const std::optional<std::string> &request_id, foxglove::ConnHandle clientHandle)
    {
        // loop through all of the params we are trying to change
        for(const auto & param_to_change : params)
        {
            // sets the drivebrain parameters
            
            _set_db_param(param_to_change);
        }
        // get the updated params
        auto fxglove_params_vec = _get_current_params();
        _server->publishParameterValues(clientHandle, fxglove_params_vec, request_id);
    };

    _server->setHandlers(std::move(hdlrs));
    _server->start("0.0.0.0", 8765);
}

foxglove::Parameter core::FoxgloveParameterServer::_get_foxglove_param(const std::string &set_name, core::common::Configurable::ParamTypes param)
{
    if (std::holds_alternative<bool>(param))
    {
        std::cout << set_name << " Variant holds an bool: " << std::get<bool>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<bool>(param));
    }
    else if (std::holds_alternative<int>(param))
    {
        std::cout << set_name << " Variant holds a int: " << std::get<int>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<int>(param));
    }
    else if (std::holds_alternative<float>(param))
    {
        std::cout << set_name << " Variant holds a float: " << std::get<float>(param) << std::endl;
        return foxglove::Parameter(set_name, ((double)std::get<float>(param)));
    }
    else if (std::holds_alternative<double>(param))
    {
        std::cout << set_name << " Variant holds a float: " << std::get<float>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<double>(param));
    }
    else if (std::holds_alternative<std::string>(param))
    {
        std::cout << set_name << " Variant holds a string: " << std::get<std::string>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<std::string>(param));
    }
    else
    {
        std::cout << set_name << " unknown param variant type" << std::endl;
    }

    return {};
}

core::common::Configurable::ParamTypes core::FoxgloveParameterServer::_get_db_param(foxglove::Parameter param)
{
    
    if (param.getValue().getType() == foxglove::ParameterType::PARAMETER_BOOL)
    {
        return param.getValue().getValue<bool>();
    }
    else if (param.getValue().getType() == foxglove::ParameterType::PARAMETER_INTEGER)
    {
        return ((int)param.getValue().getValue<int64_t>());
    }
    else if (param.getValue().getType()== foxglove::ParameterType::PARAMETER_DOUBLE)
    {
        
        return ((float)param.getValue().getValue<double>());
    }
    else if (param.getValue().getType()== foxglove::ParameterType::PARAMETER_STRING)
    {
        return param.getValue().getValue<std::string>();
    }
    else
    {
        std::cout << "unsupported param type" << std::endl;
    }

    return std::monostate();
}


void core::FoxgloveParameterServer::_set_db_param(foxglove::Parameter param_update)
{
    auto split_pos = param_update.getName().find("/");

    auto param_name = param_update.getName().substr(split_pos + 1);
    auto component_name = param_update.getName().substr(0, split_pos);

    for(const auto component : _components)
    {
        if(component->get_name() == component_name)
        {
            
            auto val = _get_db_param(param_update);
            component->handle_live_param_update(param_name, val);
            return;
        }
    }
    std::cout <<"WARNING: could not find component " << component_name << std::endl;
}

std::vector<foxglove::Parameter> core::FoxgloveParameterServer::_get_current_params()
{
    std::vector<foxglove::Parameter> params;
    for (const auto component : _components)
    {
        auto params_map = component->get_params_map();
        auto param_parent = component->get_name();
        auto param_names = component->get_param_names();
        for (const auto &component_param_name : param_names)
        {
            auto foxglove_param_id = param_parent + "/" + component_param_name;
            auto fxglove_param = _get_foxglove_param(foxglove_param_id, params_map[component_param_name]);
            params.push_back(fxglove_param);
        }
    }
    return params;
}
