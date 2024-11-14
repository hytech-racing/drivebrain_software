#define MCAP_IMPLEMENTATION
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/descriptor_database.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/reflection.h>
#include <google/protobuf/message.h>
#include "reader.hpp"
#include <memory>
#include <iostream>
#include <vector>
#include "socket.hpp"
#include "helper.hpp"

// Server exists to test the implementation of the mcap replay without putting it through drivebrain
int main(int argc, char** argv) {
    Socket socket(DEFAULT_IP, DEFAULT_PORT);
    socket.bind();

    const std::optional<std::string> inputFileName_o = utils::errorCheckInput(argc, argv);
    if (!inputFileName_o.has_value()) {
        return 1;
    }
    const std::vector<std::string> channelTopics = utils::getTopics("test/test_mcap_replay/res/test_input.json");

    gp::SimpleDescriptorDatabase protoDb;
    gp::DescriptorPool protoPool(&protoDb);
    gp::DynamicMessageFactory protoFactory(&protoPool);

    mcap::McapReader reader;
    const auto res = reader.open(*inputFileName_o);
    if (!res.ok()) {
        return 1;
    }
    
    // TODO: Need server to be able to get schema from socket receive for testing
    SchemaMap schemaMap = utils::generateSchemaMap(reader, channelTopics, &protoDb); 

    std::cout << "Listening..." << "\n\n";

    // Assume only specifying one schema
    // Server Loop
    while (true) {
        std::string received_message;
        if (socket.receive(received_message)) {
            const mcap::SchemaPtr schema = schemaMap[channelTopics[0]]; // only use one topic for testing
            const gp::Descriptor* descriptor = protoPool.FindMessageTypeByName(schema->name);
            if (descriptor == nullptr) {
                std::cerr << "SERVER ERROR: Message descriptor is null" << std::endl;
                continue;
            }

            std::unique_ptr<gp::Message> message(protoFactory.GetPrototype(descriptor)->New());
            if (!message->ParseFromString(received_message)) {
                std::cerr << "Failed to parse received message" << std::endl;
                continue;
            }

            std::cout << "SERVER received message: " << message->DebugString() << std::endl;
        }
    }

    socket.close();
    return 0;
}