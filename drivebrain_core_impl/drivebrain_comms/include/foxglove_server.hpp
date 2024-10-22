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


// TODO:

// - [x] look into renaming if using the same class for foxglove server for live data too
// - [ ] write tests for type conversion between foxgloe and drivebrain types and the conditions that they need to handle
// - [ ] write tests for properly handling namespaced cache of parameters
// - [x] figure out some way of logging to the mcap file the config changes. 
//         if we log directly from here we have multiple layers that can "log" as this class itself is a 
//         downstream consumer and upstream producer of messages to log which is wierd. however with 
//         the current method of binding functions for the message logger we actually dont care, we can just 
//         add a bound function here for handling the logging to the mcap of the json message. wont be too
//         bad / wierd.
// - [ ] add json logging function to be bound at construction
namespace core
{
    class FoxgloveWSServer
    {
    public:
        FoxgloveWSServer() = delete;

        FoxgloveWSServer(std::vector<core::common::Configurable *> configurable_components, std::function<void(const std::string)> param_log_func);
        void send_live_telem_msg(std::shared_ptr<google::protobuf::Message> msg);
        ~FoxgloveWSServer()
        {
            _server->stop();
        }

        /// @brief get the schema to give to the mcap logger that is the json representation of all of the params so we can log both the current params and when we change params
        /// @return string containing schema
        std::string get_json_parameter_schema();

    private:
        /// @brief gets a vector of foxglove parameters from all components, parameter name is name-spaced to each component
        /// @return vector of foxglove params
        std::vector<foxglove::Parameter> _get_current_params();
        
        /// @brief gets a converted to foxglove parameter from a drivebrain parameter type
        /// @param set_name the final name to set set the foxglove parameter to
        /// @param param the drivebrain parameter
        /// @return foxglove parameter type
        foxglove::Parameter _get_foxglove_param(const std::string& set_name, core::common::Configurable::ParamTypes param);
        
        /// @brief gets a drivebrain parameter from a foxglove parameter
        /// @param param the parameter to convert
        /// @return drivebrain param
        core::common::Configurable::ParamTypes _get_db_param(foxglove::Parameter param);

        /// @brief function to handle execution of the parameter updating internally for a single foxglove parameter
        /// @param param_update the new parameter value that has been properly converted to the expected type that is currently in the parameter cache
        void _set_db_param(foxglove::Parameter param_update);
        
        /// @brief handles conversion of input foxglove parameter as the foxglove parameters incoming from the server arent guaranteed to stay the same type (float -> int, int-> float, etc.)
        /// @param curr_param_val the internal parameter value of the drivebrain type
        /// @param incoming_param the new foxglove parameter value that may need conversion
        /// @return optional foxglove parameter, if the type is un-supported for conversion this is a nullopt and the parameter update should not occur
        std::optional<foxglove::Parameter> _convert_foxglove_param(core::common::Configurable::ParamTypes curr_param_val, foxglove::Parameter incoming_param);

    private:
        std::vector<core::common::Configurable *> _components;
        std::unique_ptr<foxglove::ServerInterface<websocketpp::connection_hdl>> _server;
        std::function<void(foxglove::WebSocketLogLevel, char const *)> _log_handler;
        foxglove::ServerOptions _server_options;
        std::unordered_map<std::string, foxglove::ChannelId> _id_name_map;
        std::function<void(const std::string &)> _log_param_change_func;
    };
}