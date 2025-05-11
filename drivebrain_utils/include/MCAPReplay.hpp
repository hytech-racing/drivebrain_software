#ifndef __MCAPREPLAY_H__
#define __MCAPREPLAY_H__

#include <memory>
#include <string>

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/descriptor_database.h>
#include <google/protobuf/dynamic_message.h>



// Drivebrain impl components
#include <CANComms.hpp>
#include <ETHSendComms.hpp>


#define MCAP_IMPLEMENTATION
#include "mcap/reader.hpp"


// this utility will be able to replay a car MCAP file to 
// ethernet socket and a virtual CAN bus

// [x] simple reading of mcap file
// [x] read of mcap at real-time speed
    // starting with the timestamp of the first message, ensure that we wait until the next message was sent
    // before we "send" the next message on read of the message.
// [x] use a CANDriver and send all CAN traffic from MCAP file to vcan
// [ ] use instances of (TODO) ETHSendComms to send the VCF/VCR/ACU data
    // [ ] create ETHSendComms

namespace util
{
    class MCAPReplay
    {
        public:
            MCAPReplay(std::string param_file_path, std::string dbc_file_path);
            
            void start(std::string filename);
        private:
            bool _load_schema(const mcap::SchemaPtr schema, google::protobuf::SimpleDescriptorDatabase* proto_db);
            std::shared_ptr<google::protobuf::Message> _get_pb_msg(google::protobuf::DescriptorPool &protoPool,
                                                     google::protobuf::DynamicMessageFactory &protoFactory,
                                                     const mcap::SchemaPtr schema,
                                                     google::protobuf::SimpleDescriptorDatabase *protoDbPtr, mcap::Message mcap_msg);
        private:
             
            core::JsonFileHandler _config;
            core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> _primary_can_tx_queue;
            boost::asio::io_context _io_context;
            std::shared_ptr<comms::CANDriver> _driver_primary_can = nullptr;
            std::shared_ptr<comms::ETHSendComms> _acu_sender = nullptr;
            std::shared_ptr<comms::ETHSendComms> _vcr_sender = nullptr;
            std::shared_ptr<comms::ETHSendComms> _acu_core_sender = nullptr;
            std::shared_ptr<comms::ETHSendComms> _vn_sender = nullptr;
            mcap::McapReader _mcap_reader;
        
            std::thread _io_context_thread;
    };

}
#endif // __MCAPREPLAY_H__