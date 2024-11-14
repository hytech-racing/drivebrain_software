#define MCAP_IMPLEMENTATION
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/descriptor_database.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/reflection.h>
#include <google/protobuf/message.h>
#include <memory>
#include <iostream>
#include <vector>
#include "reader.hpp"
#include "socket.hpp"
#include "helper.hpp"
#include <chrono>
#include <iterator>
#include <thread>

#define SEC_TO_NANO 1'000'000'000
constexpr bool DEBUG_OUT = true;

void printTimePoint(const std::chrono::system_clock::time_point& time_point) {
    std::time_t time = std::chrono::system_clock::to_time_t(time_point);
    std::tm* local_time = std::localtime(&time);
    std::cout << std::put_time(local_time, "%Y-%m-%d %H:%M:%S");
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
        time_point.time_since_epoch()) % SEC_TO_NANO;
    std::cout << '.' << std::setfill('0') << std::setw(9) << nanoseconds.count() << std::endl;
}

void printTimePoint(mcap::Timestamp logTime) {
    printTimePoint(std::chrono::system_clock::time_point(std::chrono::nanoseconds(logTime)));
}


int main(int argc, char** argv) {
    Socket socket(DEFAULT_IP, DEFAULT_PORT);

    const std::optional<std::string> inputFileName_o = utils::errorCheckInput(argc, argv);
    if (!inputFileName_o.has_value()) {
        return 1;
    }

    std::vector<std::string> channelTopics = utils::getTopics("test/test_mcap_replay/res/test_input.json");
    
    gp::SimpleDescriptorDatabase protoDb;
    gp::DescriptorPool protoPool(&protoDb);
    gp::DynamicMessageFactory protoFactory(&protoPool);

    mcap::McapReader reader;
    const auto res = reader.open(*inputFileName_o);
    if (!res.ok()) {
        std::cerr << "ERROR: Failed to open MCAP reader" << std::endl;
        return 1;
    }

    int c = 0; 
    mcap::Timestamp curTime = 0;
    mcap::Timestamp lastTime = 0;

    std::vector<mcap::MessageView> messages;
    auto view = reader.readMessages();

    // Send Msgs Loop
    for (auto it = view.begin(); it != view.end(); ++it) {
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

        // Only send the msgs specified by the JSON
        if (std::find(channelTopics.begin(), channelTopics.end(), it->channel->topic) == channelTopics.end()) {
            continue;
        }

        const gp::Descriptor* descriptor = protoPool.FindMessageTypeByName(it->schema->name);
        if (descriptor == nullptr) {
            if (!utils::loadSchema(it->schema, &protoDb)) {
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
        if (!message->ParseFromArray(it->message.data, static_cast<int>(it->message.dataSize))) { // SEG FAULT HERE
            std::cerr << "ERROR: failed to parse message using included schema" << std::endl;
            reader.close();
            return 1;
        }


        // Time calculations
        if (c == 0) { // it == view.begin() is more fitting but it doesn't work probably bc weird mcap::LinearMessageView behavior
            curTime = it->message.logTime;
            lastTime = it->message.logTime;
        } else {
            lastTime = curTime;
            curTime = it->message.logTime;
        }
        if (lastTime > curTime) {
            std::cerr << "ERROR: Garbage MCAP data" << std::endl;
            return 1;
        }
        uint64_t waitTime = curTime - lastTime;

        // DEBUG OUT
        if (DEBUG_OUT) {
            std::cout << "DEBUG:\n";
            std::cout << "Channel Topic: " << it->channel->topic << std::endl; 
            std::cout << "Schema Name: " << it->schema->name<< std::endl; 
            std::cout << " CurTime: ";
            printTimePoint(curTime);
            std::cout << "waitTime: " << waitTime << std::endl;
            std::cout << "Message before serialization:" << std::endl;
            std::cout << message->DebugString() << std::endl;
        }

        // Serialize PB msg
        std::string serializedMessage;
        if (!message->SerializeToString(&serializedMessage)) {
            std::cerr << "Failed to serialize protobuf message" << std::endl;
            reader.close();
            return 1;
        }

        // Send the serialized PB message to the server using UDP
        std::this_thread::sleep_for(std::chrono::nanoseconds(waitTime));
        socket.send(serializedMessage, false);

        ++c;

        // DEBUG
        // Exit out early
        // if (c > 3) break;
    }

    reader.close();
    socket.close();
    return 0;
}
