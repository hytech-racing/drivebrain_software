#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/descriptor_database.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/reflection.h>
#include <google/protobuf/message.h>
#include "../mcap/reader.hpp"
#include <memory>
#include <iostream>
#include <vector>
#include "socket.hpp"
#include "helper.hpp"

int main() {
    Socket socket(DEFAULT_IP, DEFAULT_PORT);
    socket.bind();

    const char *inputFilename = "testdata.mcap"; // TODO: shouldn't be hardcoded
    const std::string channelName = "mcu_error_states_data"; // TODO shouldn't be hardcoded
    const std::string schemaName = "mcu_error_states"; // TODO shouldn't be hardcoded

    mcap::McapReader reader;
    const auto res = reader.open(inputFilename);
    if (!res.ok()) {
        return 1;
    }

    gp::SimpleDescriptorDatabase protoDb;
    gp::DescriptorPool protoPool(&protoDb);
    gp::DynamicMessageFactory protoFactory(&protoPool);
    
    // Determine relevant schema
    // TODO: For now only for one hardcoded channelName
    auto messageView = reader.readMessages();
    mcap::SchemaPtr schema;
    for (auto it = messageView.begin(); it != messageView.end(); it++) {
        if (it->schema->encoding != "protobuf") {
            continue;
        }
        if (it->channel->messageEncoding != "protobuf") {
            std::cerr << "expected channel with ID " << it->channel->id << " and schema ID "
                      << it->schema->id << "to use message encoding 'protobuf', but message encoding is "
                      << it->channel->messageEncoding << std::endl;
            reader.close();
            return 1;
        }
        if (it->channel->topic != channelName) {
            continue;
        }
        const gp::Descriptor* descriptor = protoPool.FindMessageTypeByName(it->schema->name);
        if (descriptor == nullptr) {
            if (!utils::LoadSchema(it->schema, &protoDb)) {
                reader.close();
                return 1;
            }
            descriptor = protoPool.FindMessageTypeByName(it->schema->name);
            if (descriptor == nullptr) {
                std::cerr << "failed to find descriptor after loading pool" << std::endl;
                reader.close();
                return 1;
            }
        }

        auto message = std::unique_ptr<gp::Message>(protoFactory.GetPrototype(descriptor)->New());
        if (!message->ParseFromArray(it->message.data, static_cast<int>(it->message.dataSize))) {
            std::cerr << "failed to parse message using included schema" << std::endl;
            reader.close();
            return 1;
        }
        schema = it->schema;
        break;
    }
    reader.close();



    std::cout << "Server intialized. Listening..." << "\n\n";

    // Server Loop
    while (true) {
        std::string received_message;
        if (socket.receive(received_message)) {
            std::cout << "Try SCHEMA: " << schemaName << " for descriptor" << std::endl;
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