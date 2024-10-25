#ifndef HELPER_HPP
#define HELPER_HPP

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
#include <array>
#include <algorithm>
// #include <nlohmann/json.hpp>

namespace gp = google::protobuf;

namespace utils {
    bool LoadSchema(const mcap::SchemaPtr schema, gp::SimpleDescriptorDatabase *protoDb);
    // LoadTopics();
}

#endif