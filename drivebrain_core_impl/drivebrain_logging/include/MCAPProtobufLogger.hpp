#ifndef __MCAPPROTOBUFLOGGER_H__
#define __MCAPPROTOBUFLOGGER_H__

#include <DriverBus.hpp>

#include <nlohmann/json.hpp>

#include <google/protobuf/message.h>

#include <mcap.hpp>
#include <mcap/writer.hpp>
#include <mcap/mcap.hpp>

#include <unordered_map>
#include <memory>
#include <string>
#include <variant>
#include <mutex>
#include <thread>

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
    class MCAPProtobufLogger 
    {
    public:
        struct RawMessage
        {
            std::string serialized_data;
            std::string message_name;
            uint64_t log_time;
        };

        MCAPProtobufLogger(const std::string &base_dir, std::function<nlohmann::json::object()> get_component_param_schemas);
        ~MCAPProtobufLogger();

        /// @brief 
        /// @param out_msg 
        void log_msg(std::shared_ptr<google::protobuf::Message> out_msg);
        void log_json_struct(const std::string & topic, const nlohmann::json::object & out_json );
        
        void open_new_mcap(const std::string &name);
        void close_current_mcap();

    private:
        void _handle_log_to_file();
    private:
        core::common::ThreadSafeDeque<RawMessage> _input_deque;
        std::thread _log_thread;
        bool _running = false;
        mcap::McapWriterOptions _options;
        mcap::McapWriter _writer;
        std::mutex _logger_mtx;
        std::unordered_map<std::string, uint32_t> _msg_name_id_map;

        std::function<nlohmann::json::object()> _get_params_schema;

    };
}

#endif // __MCAPPROTOBUFLOGGER_H__