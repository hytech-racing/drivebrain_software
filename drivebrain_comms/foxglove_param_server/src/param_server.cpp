#include <param_server.hpp>

template <size_t N>
core::FoxgloveParameterServer::FoxgloveParameterServer(std::array<core::common::Configurable*, N> configurable_components)
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
        auto params_vec = _get_current_params(param_names);
        _server->publishParameterValues(clientHandle, params_vec, request_id);
    };
}

std::vector<foxglove::Parameter> core::FoxgloveParameterServer::_get_current_params(const std::vector<std::string>& param_names)
{
    std::vector<foxglove::Parameter> params;

    // params.emplace_back()
    
    return params;
}

