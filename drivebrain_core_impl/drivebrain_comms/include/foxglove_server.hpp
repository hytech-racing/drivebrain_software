#pragma once
#include <array>
#include <vector>
#include <functional>
#include <variant>

// from drivebrain_core
#include <Configurable.hpp>
#include <websocket/base64.hpp>
#include <websocket/server_factory.hpp>
#include <websocket/websocket_notls.hpp>
#include <websocket/websocket_server.hpp>
#include <DriverBus.hpp>

#include <google/protobuf/message.h>


namespace core
{
    class FoxgloveWSServer
    {
    public:
        // FoxgloveWSServer() = delete;

        explicit FoxgloveWSServer(std::vector<core::common::Configurable *> configurable_components);
        void send_live_telem_msg(std::shared_ptr<google::protobuf::Message> msg);
        ~FoxgloveWSServer()
        {
            _server->stop();
        }

    private:
        std::vector<foxglove::Parameter> _get_current_params();
        foxglove::Parameter _get_foxglove_param(const std::string &set_name, core::common::Configurable::ParamTypes param);
        core::common::Configurable::ParamTypes _get_db_param(foxglove::Parameter param_update);
        void _set_db_param(foxglove::Parameter param_update);
        std::optional<foxglove::Parameter> _convert_foxglove_param(core::common::Configurable::ParamTypes curr_param_val, foxglove::Parameter incoming_param);
        void _handle_foxglove_send();
    private:
        std::vector<core::common::Configurable *> _components;
        std::unique_ptr<foxglove::ServerInterface<websocketpp::connection_hdl>> _server;
        std::function<void(foxglove::WebSocketLogLevel, char const *)> _log_handler;
        foxglove::ServerOptions _server_options;
        std::unordered_map<std::string, foxglove::ChannelId> _id_name_map;

    };
}
