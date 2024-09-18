#include <MCAPProtobufLogger.hpp>

namespace common
{
    MCAPProtobufLogger::MCAPProtobufLogger(const std::string &file_dir_name)
    {
        auto options = mcap::McapWriterOptions("");
        const auto res = _writer.open(file_name.c_str(), options);
        if (!res.ok()) {
        std::cerr << "Failed to open " << outputFilename << " for writing: " << res.message
                    << std::endl;
        }
    }


    MCAPProtobufLogger::log_msg(std::shared_ptr<google::protobuf::Message> msg_out)
    {

    }


}