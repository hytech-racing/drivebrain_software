#include <nlohmann/json_fwd.hpp>
#define MCAP_IMPLEMENTATION
#include <types.hpp>
#include <DrivebrainMCAPLogger.hpp>
#include <chrono>
#include <ProtobufUtils.hpp>
#include <mcap/writer.hpp>
#include <mcap.hpp>
#include <hytech_msgs.pb.h>
#include <mutex>
#include <versions.h>
#include <utility>
#include <spdlog/spdlog.h> 

namespace common
{
    DrivebrainMCAPLogger::DrivebrainMCAPLogger(const std::string &base_dir, std::vector<std::shared_ptr<core::common::Configurable>> configurable_components)
        : _options(mcap::McapWriterOptions("")), _configurable_components(configurable_components)
    {
        auto optional_map = util::generate_name_to_id_map({"hytech_msgs.proto", "hytech.proto"});
        if (optional_map)
        {
            spdlog::info("Opened MCAP"); 
            _msg_name_id_map = *optional_map;
        }
        else
        {
            spdlog::error("Error: No map generated"); 
        }
        {
            std::unique_lock lk(_input_deque.mtx);
            _running = true;
        }
        _options.noChunking = true;
        _log_thread = std::thread(&DrivebrainMCAPLogger::_handle_log_to_file, this);
    }

    DrivebrainMCAPLogger::~DrivebrainMCAPLogger()
    {
        {
            std::unique_lock lk(_input_deque.mtx);
            _running = false;
        }
        spdlog::info("attempting close log thread in mcap logger");
        _input_deque.cv.notify_all();
        spdlog::info("notif sent");
        _log_thread.join();
        spdlog::info("joined log thread in mcap logger");
    }

    // is guaranteed to be called before starting the logging to the mcap
    void DrivebrainMCAPLogger::open_new_mcap(const std::string &name)
    {
        spdlog::info("Open MCAP function called"); 

        const auto res = _writer.open(name.c_str(), _options);
        if (!res.ok())
        {
            spdlog::error("Failed to open {} for writing: {}", name, res.message);
        }
        // TODO handle message name de-confliction for messages of the same name
        // message-receiving .protos (non-base .proto files)
        auto receiving_descriptors = util::get_pb_descriptors({"hytech_msgs.proto", "hytech.proto"});
        // auto schema_only_descriptors = util::get_pb_descriptors({"base_msgs.proto"});

        auto add_protobuf_schema_func = [this](const std::vector<const google::protobuf::FileDescriptor *> &descriptors, bool skip_channel)
        {
            for (const auto &file_descriptor : descriptors)
            {
                for (int i = 0; i < file_descriptor->message_type_count(); ++i)
                {
                    const google::protobuf::Descriptor *message_descriptor = file_descriptor->message_type(i);
                    mcap::Schema schema(message_descriptor->full_name(), "protobuf",
                                        util::build_file_descriptor_set(message_descriptor).SerializeAsString());
                    _writer.addSchema(schema);
                    mcap::Channel channel(message_descriptor->name(), "protobuf", schema.id);
                    if (!skip_channel)
                    {
                        _writer.addChannel(channel);
                    }
                }
            }
        };

        // add_protobuf_schema_func(schema_only_descriptors, true);
        add_protobuf_schema_func(receiving_descriptors, false);

        spdlog::info("Added message descriptions to MCAP"); 
    }

    void DrivebrainMCAPLogger::close_current_mcap()
    {
        spdlog::info("Closing MCAP"); 
        _writer.close();
    }

    void DrivebrainMCAPLogger::_handle_log_to_file()
    {
        core::common::ThreadSafeDeque<RawMessage> q;

        while (true)
        {
            {
                std::unique_lock lk(_input_deque.mtx);
                _input_deque.cv.wait(lk, [this]()
                                    { return !_input_deque.deque.empty() || !_running; });

                // Ensure exit condition is checked immediately after waking up
                if (!_running && _input_deque.deque.empty())
                    break;  // Clean exit instead of return (better readability)

                q.deque = std::move(_input_deque.deque); // Move instead of copy
            }

            for (auto &msg : q.deque)
            {
                mcap::Message msg_to_log;
                msg_to_log.data = reinterpret_cast<const std::byte *>(msg.serialized_data.data());
                msg_to_log.dataSize = msg.serialized_data.size();

                msg_to_log.logTime = msg.log_time;
                msg_to_log.publishTime = msg.log_time;

                {
                    std::unique_lock lk(_logger_mtx);
                    msg_to_log.channelId = _msg_name_id_map[msg.message_name];
                    spdlog::info("logging at channel: {}", msg_to_log.channelId);
                    auto write_res = _writer.write(msg_to_log);

                    spdlog::info("Logging message: {}", msg.message_name);
                    spdlog::info("message size: {}", msg_to_log.dataSize);
                    spdlog::info(msg.serialized_data);
                }
            }

            q.deque.clear();
        }

        spdlog::info("Logger thread exiting...");

        // Ensure all resources are cleaned up
        {
            std::unique_lock lk(_input_deque.mtx);
            _input_deque.deque.clear();
        }

        close_current_mcap();
    }

    void DrivebrainMCAPLogger::log_msg(std::shared_ptr<google::protobuf::Message> msg_out)
    {
        mcap::Timestamp log_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        DrivebrainMCAPLogger::RawMessage msg_to_enque;
        msg_to_enque.serialized_data = msg_out->SerializeAsString();
        msg_to_enque.message_name = msg_out->GetDescriptor()->name();
        msg_to_enque.log_time = log_time;

        {
            std::unique_lock lk(_input_deque.mtx);
            _input_deque.deque.push_back(msg_to_enque);
            _input_deque.cv.notify_all();
        }
        
    }


    // NOTE: required to be called after all of the components have been initialized
    // TODO: make this thread safe by adding a writer mutex as this resource is being worked with in multiple threads:
    //       1: message logger param log thread (if the param schema cannot be written on construction of the msg logger)
    //       2: this class's log thread
    bool DrivebrainMCAPLogger::init_param_schema()
    {
        // param schema and channel creation
        auto params_schema_json = _get_params_schema();

        if(params_schema_json)
        {
            mcap::Schema config_schema("drivebrain_configuration", "jsonschema", (*params_schema_json).dump());
            _writer.addSchema(config_schema);
            mcap::Channel config_channel("drivebrain_configuration", "json", config_schema.id);
            _writer.addChannel(config_channel);
            _param_schema_written = true;
            // TODO this is a name that cannot be used as a component name. need to assert this in the configureable component
            _msg_name_id_map["drivebrain_configuration"] = config_channel.id;
            return true;
        } else {
            return false;
        }
    }
    
    void DrivebrainMCAPLogger::log_params()
    {
        if(_param_schema_written)
        {
            mcap::Timestamp log_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

            // TODO handle getting of all of the current config values from the components here
            auto contentJson = _get_param_vals();
            
            DrivebrainMCAPLogger::RawMessage msg_to_enque;
            msg_to_enque.serialized_data = contentJson.dump();
            msg_to_enque.message_name = "drivebrain_configuration";
            msg_to_enque.log_time = log_time;

            {
                std::unique_lock lk(_input_deque.mtx);
                _input_deque.deque.push_back(msg_to_enque);
                _input_deque.cv.notify_all();
            }
        }
        
        
    }
    nlohmann::json DrivebrainMCAPLogger::_get_param_vals() {
    
        nlohmann::json params_all;
        for(const auto cc: _configurable_components)
        {
            std::unordered_map params_map = cc->get_all_params_map();
            std::string param_parent = cc->get_name();
            std::cout << param_parent << std::endl;
            std::vector<std::string> param_names = cc->get_param_names();
            for (auto i = params_map.begin(); i != params_map.end(); i++)
            {
                auto name = i->first;
                auto var_val = i->second;
                
                _get_params_as_json<bool, int, float, double, std::string>(param_parent, name, var_val, params_all);
            }
        }
        std::cout << params_all.dump() <<std::endl;
        return params_all;
    }

    std::optional<nlohmann::json> DrivebrainMCAPLogger::_get_params_schema()
    {
        nlohmann::json top_level_schema;
        top_level_schema["type"] = "object";
        for(const auto component : _configurable_components )
        {
            // TODO handle multiple instances of the same component
            if(!component->is_configured())
            {
                return std::nullopt;
            }
            top_level_schema["properties"][component->get_name()] = component->get_schema();
        }
        return top_level_schema;
    }


}



