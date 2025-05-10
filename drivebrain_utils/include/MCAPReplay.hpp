#ifndef __MCAPREPLAY_H__
#define __MCAPREPLAY_H__

#include <memory>
#include <string>

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/descriptor_database.h>
#include <google/protobuf/dynamic_message.h>

#define MCAP_IMPLEMENTATION
#include "mcap/reader.hpp"


// this utility will be able to replay a car MCAP file to 
// ethernet socket and a virtual CAN bus

// [x] simple reading of mcap file
// [ ] read of mcap at real-time speed
    // starting with the timestamp of the first message, ensure that we wait until the next message was sent
    // before we "send" the next message on read of the message.
// [ ] use a CANDriver and send all CAN traffic from MCAP file to vcan
// [ ] use instances of (TODO) ETHSendComms to send the VCF/VCR/ACU data

namespace util
{
    class MCAPReplay
    {
        public:
            MCAPReplay(){}
            
            void start(std::string filename);
        private:
            bool _load_schema(const mcap::SchemaPtr schema, google::protobuf::SimpleDescriptorDatabase* proto_db);

        private:
            
            mcap::McapReader _mcap_reader;
    };

}
#endif // __MCAPREPLAY_H__