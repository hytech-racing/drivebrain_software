#include "MCAPReplay.hpp"

#include <cmath>
#include <spdlog/spdlog.h>

namespace gp = google::protobuf;

namespace util {
bool MCAPReplay::_load_schema(const mcap::SchemaPtr schema,
                              google::protobuf::SimpleDescriptorDatabase *protoDb) {
    gp::FileDescriptorSet fdSet;
    if (!fdSet.ParseFromArray(schema->data.data(), static_cast<int>(schema->data.size()))) {
        spdlog::error("failed to parse schema data");
        return false;
    }
    gp::FileDescriptorProto unused;
    for (int i = 0; i < fdSet.file_size(); ++i) {
        const auto &file = fdSet.file(i);
        if (!protoDb->FindFileByName(file.name(), &unused)) {
            if (!protoDb->Add(file)) {
                spdlog::error("failed to add def {} to protoDB", file.name());
                // std::cerr << "failed to add definition " << file.name() << "to protoDB" <<
                // std::endl;
                return false;
            }
        }
    }
    return true;
}

void MCAPReplay::start(std::string filename) {
    const auto res = _mcap_reader.open(filename);
    if (!res.ok()) {

        spdlog::error("Failed to open {}", filename);
        return;
    }

    auto msg_view = _mcap_reader.readMessages();

    gp::SimpleDescriptorDatabase protoDb;
    gp::DescriptorPool protoPool(&protoDb);
    gp::DynamicMessageFactory protoFactory(&protoPool);

    for (auto it = msg_view.begin(); it != msg_view.end(); it++) {
        // skip any non-protobuf-encoded messages.
        if (it->schema->encoding != "protobuf") {
            continue;
        }

        spdlog::info("name: {}", it->schema->name);
        const gp::Descriptor *descriptor = protoPool.FindMessageTypeByName(it->schema->name);

        if (descriptor == nullptr) {
            if (!_load_schema(it->schema, &protoDb)) {
                spdlog::error("failed to load schema, exiting");
                _mcap_reader.close();
                return;
            }
            descriptor = protoPool.FindMessageTypeByName(it->schema->name);
            if (descriptor == nullptr) {
                spdlog::error("failed to find desc after loading pool, exiting");
                _mcap_reader.close();
                return;
            }
        }
    }

    _mcap_reader.close();
}
} // namespace util