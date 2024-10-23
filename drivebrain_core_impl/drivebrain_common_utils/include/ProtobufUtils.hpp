#ifndef __PROTOBUFUTILS_H__
#define __PROTOBUFUTILS_H__

#include <google/protobuf/descriptor.h>
#include <google/protobuf/descriptor.pb.h>

#include <optional>
#include <unordered_set>
#include <algorithm>
#include <string>

// TODO figure out if this is the best place to include this or nah
#include <hytech_msgs.pb.h>
#include <tuple>
#include <vector>
#include <string>
#include <queue>

/// @brief grouping of common protobuf handling utility functions. as of right now they are used by both @ref core::FoxgloveWSServer and @ref common::MCAPProtobufLogger file recording class
namespace util
{
    /// @brief gets a vector of protobuf file descriptors for a vector of filenames of .proto files.
    /// @param filenames vector of file names
    /// @return descriptors
    std::vector<const google::protobuf::FileDescriptor *> get_pb_descriptors(const std::vector<std::string> &filenames);
    std::optional<std::unordered_map<std::string, uint32_t>> generate_name_to_id_map(const std::vector<std::string> &filenames);

    /// @brief helper function for creating a file descriptor set from a specific descriptor. 
    /// @param toplevelDescriptor 
    /// @return 
    google::protobuf::FileDescriptorSet build_file_descriptor_set(const google::protobuf::Descriptor *toplevelDescriptor);
}
#endif // __PROTOBUFUTILS_H__