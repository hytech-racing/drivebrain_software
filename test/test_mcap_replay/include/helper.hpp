#ifndef HELPER_HPP
#define HELPER_HPP

#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/descriptor_database.h>
#include <google/protobuf/dynamic_message.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/reflection.h>
#include <google/protobuf/message.h>
#include <memory>
#include <iostream>
#include <vector>
#include "socket.hpp"
#include <array>
#include <fstream>
#include <algorithm>
#include <unordered_map>
#include <mcap/reader.hpp>
#include <nlohmann/json.hpp>

namespace gp = google::protobuf;
using Json = nlohmann::json;
using SchemaMap = std::unordered_map<std::string, mcap::SchemaPtr>;

namespace utils {
    bool loadSchema(const mcap::SchemaPtr schema, gp::SimpleDescriptorDatabase* protoDb);
    std::optional<std::string> errorCheckInput(int argc, char** argv);
    std::vector<std::string> getTopics(std::string fileName);
    mcap::SchemaPtr topicToSchema(mcap::McapReader& reader, const std::string& channelTopic, gp::SimpleDescriptorDatabase* protoDb);
    SchemaMap generateSchemaMap(
        mcap::McapReader& reader,
        const std::vector<std::string>& channelTopics,
        gp::SimpleDescriptorDatabase* protoDb);
}

#endif