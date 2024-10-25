#include "helper.hpp"

namespace utils {

    // Loads the FileDescriptorSet from a protobuf schema definition into a SimpleDescriptorDatabase.
    bool LoadSchema(const mcap::SchemaPtr schema, gp::SimpleDescriptorDatabase *protoDb) {
        gp::FileDescriptorSet fdSet;
        if (!fdSet.ParseFromArray(schema->data.data(), static_cast<int>(schema->data.size()))) {
            return false;
        }
        gp::FileDescriptorProto unused;
        for (int i = 0; i < fdSet.file_size(); ++i) {
            const auto &file = fdSet.file(i);
            if (!protoDb->FindFileByName(file.name(), &unused)) {
                if (!protoDb->Add(file)) {
                    return false;
                }
            }
        }
        return true;
    }
}
