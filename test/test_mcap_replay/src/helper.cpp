#include "helper.hpp"
#include <optional>

namespace utils {

    // Loads the FileDescriptorSet from a protobuf schema definition into a SimpleDescriptorDatabase.
    bool loadSchema(const mcap::SchemaPtr schema, gp::SimpleDescriptorDatabase* protoDb) {
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

    std::optional<std::string> errorCheckInput(int argc, char** argv) {
        if (argc != 2) {
            std::cerr << "ARG ERROR: Incorrect # of args. " << argc << " were given." << std::endl;
            return {};
        } else {
            std::string fileInput = argv[1];
            std::string ft = ".mcap";
            if (fileInput.size() > ft.size()
                && fileInput.rfind(ft) == fileInput.size() - ft.size()) {
                return {fileInput};
            } else {
                std::cerr << "ARG ERROR: Requires an .mcap file. \"" << fileInput << "\" was given." << std::endl;
                return {};
            }
        }
    }

    // Will assume that provided JSON is valid
    // Note: vector might not be great for this use case.
    std::vector<std::string> getTopics(std::string fileName) {
        std::ifstream f(fileName);
        Json jsonData = Json::parse(f);
        std::vector<std::string> channelTopics;
        const std::string key = "channelTopics";

        if (jsonData.contains(key) 
            && jsonData[key].is_array()) {
            size_t topicCount = jsonData[key].size();
            for (size_t i = 0; i < topicCount; ++i) {
                std::cout << i << std::endl;
                channelTopics.push_back(jsonData[key][i]);
            }
        } else {
            std::cerr << "JSON ERROR: channelTopics key is missing or not an array." << std::endl;
        }
        return channelTopics;
    }

    mcap::SchemaPtr topicToSchema(mcap::McapReader& reader, const std::string& channelTopic, gp::SimpleDescriptorDatabase* protoDb) {
        auto messageView = reader.readMessages();
        for (auto it = messageView.begin(); it != messageView.end(); it++) {
            if (it->schema->encoding != "protobuf"
                || it->channel->topic != channelTopic) {
                continue;
            }
            loadSchema(it->schema, protoDb);
            return it->schema;
        }
        return nullptr;
    }

    SchemaMap generateSchemaMap(
        mcap::McapReader& reader,
        const std::vector<std::string>& channelTopics,
        gp::SimpleDescriptorDatabase* protoDb) {

        SchemaMap schemaMap;
        for (const auto& channelTopic : channelTopics) {
            mcap::SchemaPtr schema = topicToSchema(reader, channelTopic, protoDb);
            if (schema) {
                schemaMap[channelTopic] = schema;
            }
        }
        return schemaMap;
    }


}
