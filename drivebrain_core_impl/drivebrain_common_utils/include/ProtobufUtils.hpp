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

#include <queue>
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
            const google::protobuf::Descriptor *message_descriptor = file_descriptor->message_type(i);
            map[message_descriptor->name()] = i + 1;
        }
        return map;
    }

    google::protobuf::FileDescriptorSet build_file_descriptor_set(
        const google::protobuf::Descriptor *toplevelDescriptor)
    {
        google::protobuf::FileDescriptorSet fdSet;
        std::queue<const google::protobuf::FileDescriptor *> toAdd;
        toAdd.push(toplevelDescriptor->file());
        std::unordered_set<std::string> seenDependencies;
        while (!toAdd.empty())
        {
            const google::protobuf::FileDescriptor *next = toAdd.front();
            toAdd.pop();
            next->CopyTo(fdSet.add_file());
            for (int i = 0; i < next->dependency_count(); ++i)
            {
                const auto &dep = next->dependency(i);
                if (seenDependencies.find(dep->name()) == seenDependencies.end())
                {
                    seenDependencies.insert(dep->name());
                    toAdd.push(dep);
                }
            }
        }
        return fdSet;
    }
}
#endif // __PROTOBUFUTILS_H__