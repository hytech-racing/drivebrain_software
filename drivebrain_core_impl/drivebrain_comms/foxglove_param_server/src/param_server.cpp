#include <param_server.hpp>

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
}

foxglove::Parameter _get_foxglove_param(const std::string& set_name, core::common::Configurable::ParamTypes param)
{
    if (std::holds_alternative<bool>(param))
    {
        std::cout << "Variant holds an bool: " << std::get<bool>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<bool>(param));
    }
    else if (std::holds_alternative<int>(param))
    {
        std::cout << "Variant holds a int: " << std::get<int>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<int>(param));
    }
    else if (std::holds_alternative<float>(param))
    {
        std::cout << "Variant holds a float: " << std::get<float>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<float>(param));
    }
    else if (std::holds_alternative<std::string>(param))
    {
        std::cout << "Variant holds a string: " << std::get<std::string>(param) << std::endl;
        return foxglove::Parameter(set_name, std::get<std::string>(param));
    }
    else
    {
        std::cout << "unknown param variant type" << std::endl;
    }

    return {};
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
