#ifndef __MCAPPROTOBUFLOGGER_H__
#define __MCAPPROTOBUFLOGGER_H__

#include <google/protobuf/message.h>

#include <MsgLogger.hpp>

#include <string>

namespace common
{
    class MCAPProtobufLogger : public MsgLogger<google::protobuf::Message>
    {
    public:
        MCAPProtobufLogger(const std::string &file_name) : MsgLogger<google::protobuf::Message>(file_name);
        void log_msg(std::shared_ptr<google::protobuf::Message> out_msg);
    private:
          mcap::McapWriter _writer;

    };
}

#endif // __MCAPPROTOBUFLOGGER_H__