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
namespace common
{
    class MCAPProtobufLogger 
    {
    public:
        MCAPProtobufLogger(const std::string &base_dir);
        void log_msg(std::shared_ptr<google::protobuf::Message> out_msg);
        void open_new_mcap(const std::string &name);
        void close_current_mcap();
    private:
        mcap::McapWriterOptions _options;
        mcap::McapWriter _writer;
        std::mutex _mtx;
        std::unordered_map<std::string, uint32_t> _msg_name_id_map;

    };
}

#endif // __MCAPPROTOBUFLOGGER_H__