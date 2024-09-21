#include <MCAPProtobufLogger.hpp>
#include <chrono>
#include <ProtobufUtils.hpp>

namespace common
{
    MCAPProtobufLogger::MCAPProtobufLogger(const std::string &file_dir_name)
    {
        _options = mcap::McapWriterOptions("");
        _msg_name_id_map = util::generate_name_to_id_map("hytech_msgs.proto")
    }

    void MCAPProtobufLogger::open_new_mcap(const std::string & name)
    {
        const auto res = _writer.open(name.c_str(), _options);
        if (!res.ok()) {
            std::cerr << "Failed to open " << name << " for writing: " << res.message
                        << std::endl;
        }
    }

    void MCAPProtobufLogger::close_current_mcap()
    {
        _writer.close();
    }
    void MCAPProtobufLogger::log_msg(std::shared_ptr<google::protobuf::Message> msg_out)
    {
        mcap::Timestamp log_time = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        mcap::Message msg_to_log;
        msg_to_log.logTime = log_time;
        msg_to_log.publishTime = log_time;
        // msg_to_log.sequence = 0; uh, idk https://github.com/foxglove/mcap/blob/main/cpp/mcap/include/mcap/types.hpp#L184
        std::string serialized = msg_out->SerializeAsString();
        msg_to_log.data = reinterpret_cast<const std::byte*>(serialized.data());
        msg_to_log.dataSize = serialized.size();

        msg_to_log.channelId = _msg_name_id_map[msg->GetDescriptor()->name()];
        
        // _writer.
    }


}