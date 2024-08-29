#pragma once
#include <array>
#include <vector>
#include <functional>

// from drivebrain_core
#include <Configurable.hpp> 

#include <foxglove/websocket/base64.hpp>
#include <foxglove/websocket/server_factory.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>

namespace core
{
    class FoxgloveParameterServer
    {
    public:
        FoxgloveParameterServer() = delete;
        
        
        FoxgloveParameterServer(std::vector<core::common::Configurable*> configurable_components);
        ~FoxgloveParameterServer() = default;

    private:
        std::vector<foxglove::Parameter> _get_current_params();
    private:
        
        std::vector<core::common::Configurable*> _components;
        std::unique_ptr<foxglove::ServerInterface<websocketpp::connection_hdl>> _server;
        std::function<void(foxglove::WebSocketLogLevel, char const*)> _log_handler;
        foxglove::ServerOptions _server_options;
    };
}