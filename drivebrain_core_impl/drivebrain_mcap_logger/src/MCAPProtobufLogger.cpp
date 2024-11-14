#define MCAP_IMPLEMENTATION
#include <MCAPProtobufLogger.hpp>
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
    MCAPProtobufLogger::MCAPProtobufLogger(const std::string &base_dir)
        : _options(mcap::McapWriterOptions(""))
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
        _log_thread = std::thread(&MCAPProtobufLogger::_handle_log_to_file, this);
    }
    MCAPProtobufLogger::~MCAPProtobufLogger()
    {
        {
            std::unique_lock lk(_input_deque.mtx);
            _running = false;
        }
        _input_deque.cv.notify_all();
        _log_thread.join();
    }

    void MCAPProtobufLogger::open_new_mcap(const std::string &name)
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
        auto schema_only_descriptors = util::get_pb_descriptors({"base_msgs.proto"});

        auto add_schema_func = [this](const std::vector<const google::protobuf::FileDescriptor *> &descriptors, bool skip_channel)
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
        add_schema_func(schema_only_descriptors, true);
        add_schema_func(receiving_descriptors, false);

        spdlog::info("Added message descriptions to MCAP"); 
    }

    void MCAPProtobufLogger::close_current_mcap()
    {
        spdlog::info("Closing MCAP"); 
        _writer.close();
    }

    void MCAPProtobufLogger::_handle_log_to_file()
    {
        core::common::ThreadSafeDeque<ProtobufRawMessage> q;

        // this will occasionally take a while (~200ms) to complete a loop iteration so this is in its own thread
        while (true)
        {
            {
                std::unique_lock lk(_input_deque.mtx);
                _input_deque.cv.wait(lk, [this]()
                                    { return !_input_deque.deque.empty() || !_running; });
                if (!_running)
                {
                    _input_deque.deque.clear();
                    return;
                }
                q.deque = _input_deque.deque;
                _input_deque.deque.clear();
            }
            for (auto &msg : q.deque)
            {
                mcap::Message msg_to_log;
                msg_to_log.data = reinterpret_cast<const std::byte *>(msg.serialized_data.data());
                msg_to_log.dataSize = msg.serialized_data.size();
                msg_to_log.logTime = msg.log_time;
                msg_to_log.publishTime = msg.log_time;

                // msg_to_log.sequence = 0; uh, idk https://github.com/foxglove/mcap/blob/main/cpp/mcap/include/mcap/types.hpp#L184

                {
                    std::unique_lock lk(_logger_mtx);
                    msg_to_log.channelId = _msg_name_id_map[msg.message_name]; // under the mutex we also lookup in the map
                    auto write_res = _writer.write(msg_to_log);

                    spdlog::info("Logging message: {}", msg.message_name); 
                }
            }
            q.deque.clear();
        }
    }

    void MCAPProtobufLogger::log_msg(std::shared_ptr<google::protobuf::Message> msg_out)
    {
        mcap::Timestamp log_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        MCAPProtobufLogger::ProtobufRawMessage msg_to_enque;
        msg_to_enque.serialized_data = msg_out->SerializeAsString();
        msg_to_enque.message_name = msg_out->GetDescriptor()->name();
        msg_to_enque.log_time = log_time;

        {
            std::unique_lock lk(_input_deque.mtx);
            _input_deque.deque.push_back(msg_to_enque);
            _input_deque.cv.notify_all();
        }

        
    }
}