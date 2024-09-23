#define MCAP_IMPLEMENTATION
#include <MCAPProtobufLogger.hpp>
#include <chrono>
#include <ProtobufUtils.hpp>
#include <mcap/writer.hpp>
#include <mcap.hpp>
#include <hytech_msgs.pb.h>
namespace common
{
    MCAPProtobufLogger::MCAPProtobufLogger(const std::string &base_dir)
        : _options(mcap::McapWriterOptions(""))
    {
        auto optional_map = util::generate_name_to_id_map("hytech_msgs.proto");
        if (optional_map)
        {
            std::cout << "opend mcap" << std::endl;
            _msg_name_id_map = *optional_map;
        }
        else
        {
            std::cout << "error: no map gend" << std::endl;
        }
    }

    void MCAPProtobufLogger::open_new_mcap(const std::string &name)
    {
        std::cout << "opend mcap ran func" << std::endl;

        const auto res = _writer.open(name.c_str(), _options);
        if (!res.ok())
        {
            std::cerr << "Failed to open " << name << " for writing: " << res.message
                      << std::endl;
        }

        const google::protobuf::FileDescriptor *file_descriptor =
            google::protobuf::DescriptorPool::generated_pool()->FindFileByName("hytech_msgs.proto");

        if (!file_descriptor)
        {
            std::cerr << "File descriptor not found!" << std::endl;
            _writer.close();
            return;
        }

        for (int i = 0; i < file_descriptor->message_type_count(); ++i)
        {
            const google::protobuf::Descriptor *message_descriptor = file_descriptor->message_type(i);
            mcap::Schema schema(message_descriptor->full_name(), "protobuf",
                                util::build_file_descriptor_set(message_descriptor).SerializeAsString());
            _writer.addSchema(schema);
            mcap::Channel channel(message_descriptor->name(), "protobuf", schema.id);
            _writer.addChannel(channel);
        }

        std::cout <<"added message descriptions to mcap" <<std::endl;
    }

    void MCAPProtobufLogger::close_current_mcap()
    {
        std::cout <<"closing" <<std::endl;
        _writer.close();
    }

    void MCAPProtobufLogger::log_msg(std::shared_ptr<google::protobuf::Message> msg_out)
    {
        mcap::Timestamp log_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        // std::cout << "logging msg " << msg_out->GetDescriptor()->name() << std::endl;
        if(msg_out->GetDescriptor()->name() == "MCUOutputData")
        {
            auto msg = std::static_pointer_cast<hytech_msgs::MCUOutputData>(msg_out);
            if (msg->brake_percent() == 0)
            {
                std::cout << "empty msg recvd" <<std::endl;
            }
        }
        mcap::Message msg_to_log;
        msg_to_log.logTime = log_time;
        msg_to_log.publishTime = log_time;
        // msg_to_log.sequence = 0; uh, idk https://github.com/foxglove/mcap/blob/main/cpp/mcap/include/mcap/types.hpp#L184
        std::string serialized = msg_out->SerializeAsString();
        msg_to_log.data = reinterpret_cast<const std::byte *>(serialized.data());
        msg_to_log.dataSize = serialized.size();

        msg_to_log.channelId = _msg_name_id_map[msg_out->GetDescriptor()->name()];

        auto write_res = _writer.write(msg_to_log);
        // std::cout << "status: " << write_res.message << std::endl;
        _writer.closeLastChunk();
    }

}