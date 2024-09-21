#ifndef __MCAPPROTOBUFLOGGER_H__
#define __MCAPPROTOBUFLOGGER_H__

#include <google/protobuf/message.h>

#include <MsgLogger.hpp>

#include <memory>
#include <string>
#include <unordered_map>

namespace common
{
    class MCAPProtobufLogger : public MsgLogger<google::protobuf::Message>
    {
    public:
        MCAPProtobufLogger(const std::string &base_dir) : MsgLogger<google::protobuf::Message>(file_name);
        void log_msg(std::shared_ptr<google::protobuf::Message> out_msg);
        void open_new_mcap(const std::string &name);
        void close_current_mcap();
    private:
        mcap::McapWriterOptions _options;
        mcap::McapWriter _writer;
        std::unordered_map<std::string, uint32_t> _msg_name_id_map

    };
}

#endif // __MCAPPROTOBUFLOGGER_H__