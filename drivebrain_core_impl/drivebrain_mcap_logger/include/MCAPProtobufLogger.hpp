#ifndef __MCAPPROTOBUFLOGGER_H__
#define __MCAPPROTOBUFLOGGER_H__

#include <google/protobuf/message.h>
#include <mcap.hpp>
#include <memory>
#include <string>
#include <unordered_map>
#include <mcap/writer.hpp>
#include <mcap/mcap.hpp>
#include <mutex>
#include <DriverBus.hpp>

#include <thread>
namespace common
{
    class MCAPProtobufLogger 
    {
    public:
        struct ProtobufRawMessage
        {
            std::string serialized_data;
            std::string message_name;
            uint64_t log_time;
        };

        MCAPProtobufLogger(const std::string &base_dir, const std::string &parameter_json_schema);
        ~MCAPProtobufLogger();

        /// @brief 
        /// @param out_msg 
        void log_msg(std::shared_ptr<google::protobuf::Message> out_msg);
        void open_new_mcap(const std::string &name);
        void close_current_mcap();

    private:
        void _handle_log_to_file();
    private:
        core::common::ThreadSafeDeque<ProtobufRawMessage> _input_deque;
        std::thread _log_thread;
        bool _running = false;
        mcap::McapWriterOptions _options;
        mcap::McapWriter _writer;
        std::mutex _logger_mtx;
        std::unordered_map<std::string, uint32_t> _msg_name_id_map;
        std::string _parameter_json_schema;

    };
}

#endif // __MCAPPROTOBUFLOGGER_H__