#ifndef __DRIVEBRAINMCAPLOGGER_H__
#define __DRIVEBRAINMCAPLOGGER_H__

#include <DriverBus.hpp>

#include <nlohmann/json.hpp>

#include <google/protobuf/message.h>

#include <mcap.hpp>
#include <mcap/writer.hpp>
#include <mcap/mcap.hpp>

#include <types.hpp>
#include <unordered_map>
#include <memory>
#include <string>
#include <variant>
#include <mutex>
#include <thread>
#include <optional>

#include "Configurable.hpp" // core::common::Configurable
#include <functional>
// - [ ] add functionality for logging parameters json 


// USER STORIES
// - I want to add the ability to log json data for use with logging parameters
// REQUIREMENTS:

// EXTERNAL REQUIREMENTS:
    // - assert that all components have been initialized before registering their combined parameter set schema
// - json message logger function. needed to handle the serialization of the json data (json.dump())
// - add in schema registration for json message(s): at first there will only need to be one, but we can stay general for now
// 
namespace common
{
    class DrivebrainMCAPLogger 
    {
    public:
        struct RawMessage
        {
            std::string serialized_data;
            std::string message_name;
            uint64_t log_time;
        };

        DrivebrainMCAPLogger(const std::string &base_dir, std::vector<std::shared_ptr<core::common::Configurable>> configurable_components);
        ~DrivebrainMCAPLogger();

        /// @brief 
        /// @param out_msg 
        void log_msg(std::shared_ptr<google::protobuf::Message> out_msg);
        void log_params();
        void log_json_struct(const std::string & topic, const nlohmann::json & out_json );
        
        void open_new_mcap(const std::string &name);
        void close_current_mcap();

        bool init_param_schema();
        
    private:
        template <typename... Ts>
        void _get_params_as_json(const std::string& param_parent, const std::string& name, const core::common::Configurable::ParamTypes& var_val, nlohmann::json & params_all)
        {
            // Lambda to check and set parameter value
            // nlohmann::json & params_all
            auto set_if_holds = [&](auto type_tag) {
                using T = decltype(type_tag);
                if (std::holds_alternative<T>(var_val)) {
                    params_all[param_parent][name] = std::get<T>(var_val);
                }
            };

            (set_if_holds(Ts{}), ...);
        }

        void _handle_log_to_file();
        nlohmann::json _get_param_vals();
        std::optional<nlohmann::json> _get_params_schema();
        // how this funciton will work is if we have not written the param schema that was determined by the initialization of each of the components
        // we keep calling this function in the log_params function that is being run into the message logger instance.
    private:
        core::common::ThreadSafeDeque<RawMessage> _input_deque;
        std::thread _log_thread;
        bool _running = false;
        bool _param_schema_written = false;
        mcap::McapWriterOptions _options;
        mcap::McapWriter _writer;
        std::mutex _logger_mtx;
        std::unordered_map<std::string, uint32_t> _msg_name_id_map;
        
        // std::function<std::optional<nlohmann::json>()> _get_params_schema;
        // std::function<nlohmann::json()> _get_param_vals;
        std::vector<std::shared_ptr<core::common::Configurable>> _configurable_components;
    };
}

#endif // __DRIVEBRAINMCAPLOGGER_H__