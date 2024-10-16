#ifndef __MESSAGELOGGER_H__
#define __MESSAGELOGGER_H__

#include <google/protobuf/message.h>

// message logger for handling the interface to both live data and file-recorded data

// requirements: 
// - a pointer to this object needs to be able to be given to 
//    other objects so that they can log their messages.

//  
namespace core
{
    class MessageLogger
    {
    public:
        MessageLogger() = default;
        void log_message(std::shared_ptr<google::protobuf::Message> message_to_log);
    private:
        
    };
    


}

#endif // __MESSAGELOGGER_H__