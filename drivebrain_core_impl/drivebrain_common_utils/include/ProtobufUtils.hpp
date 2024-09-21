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

namespace util
{
    std::optional<std::unordered_map<std::string, uint32_t>> generate_name_to_id_map(const std::string &proto_file_name)
    {
        std::unordered_map<std::string, uint32_t> map;
        const google::protobuf::FileDescriptor *file_descriptor =
            google::protobuf::DescriptorPool::generated_pool()->FindFileByName(proto_file_name);

        if (!file_descriptor)
        {
            std::cerr << "File descriptor not found!" << std::endl;
            return std::nullopt;
        }

        for (int i = 0; i < file_descriptor->message_type_count(); ++i)
        {
            const google::protobuf::Descriptor* message_descriptor = file_descriptor->message_type(i);
            map[message_descriptor->name()] = i+1;
        }
        return map;
    }
}
#endif // __PROTOBUFUTILS_H__