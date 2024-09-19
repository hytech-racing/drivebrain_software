// Example code for writing Protobuf messages to an MCAP file. This executable
// writes a sequence of foxglove.PointCloud messages to an MCAP which should
// show an expanding sphere when viewed in Foxglove.
#define MCAP_IMPLEMENTATION
#include "writer.hpp"
#include "BuildFileDescriptorSet.h"
// #include "foxglove/PointCloud.pb.h"
#include <hytech_msgs.pb.h>


#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <random>

#define NS_PER_MS 1000000

int main(int argc, char** argv) {


  hytech_msgs::MCUOutputData out_data;
  out_data.mutable_rpm_data()->set_fl(6969.0);
  
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <output.mcap>" << std::endl;
    return 1;
  }
  const char* outputFilename = argv[1];

  mcap::McapWriter writer;
  {
    auto options = mcap::McapWriterOptions("");
    const auto res = writer.open(outputFilename, options);
    if (!res.ok()) {
      std::cerr << "Failed to open " << outputFilename << " for writing: " << res.message
                << std::endl;
      return 1;
    }
  }

  // Create a channel and schema for our messages.
  // A message's channel informs the reader on the topic those messages were published on.
  // A channel's schema informs the reader on how to interpret the messages' content.
  // MCAP follows a relational model, where:
  // * messages have a many-to-one relationship with channels (indicated by their channel_id)
  // * channels have a many-to-one relationship with schemas (indicated by their schema_id)
  mcap::ChannelId channelId;
  {
    // protobuf schemas in MCAP are represented as a serialized FileDescriptorSet.
    // You can use the method in BuildFileDescriptorSet to generate this at runtime,
    // or generate them ahead of time with protoc:
    //   protoc --include_imports --descriptor_set_out=filename your_type.proto
    mcap::Schema schema(
      out_data.GetTypeName(), "protobuf",
      foxglove::BuildFileDescriptorSet(hytech_msgs::MCUOutputData::descriptor()).SerializeAsString());
    writer.addSchema(schema);

    // choose an arbitrary topic name.
    mcap::Channel channel("mcu", "protobuf", schema.id);
    writer.addChannel(channel);
    channelId = channel.id;
  }

  mcap::Timestamp startTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::system_clock::now().time_since_epoch())
                                .count();
  // write 100 pointcloud messages into the output MCAP file.
  for (uint32_t frameIndex = 0; frameIndex < 100; ++frameIndex) {
    // Space these frames 100ms apart in time.
    mcap::Timestamp new_time = startTime + (static_cast<uint64_t>(frameIndex) * 100 * NS_PER_MS);
    // Slightly increase the size of the cloud on every frame.
    std::string serialized = out_data.SerializeAsString();
    // Include the pointcloud data in a message, then write it into the MCAP file.
    mcap::Message msg;
    msg.channelId = channelId;
    msg.sequence = frameIndex;
    msg.publishTime = new_time;
    msg.logTime = new_time;
    msg.data = reinterpret_cast<const std::byte*>(serialized.data());
    msg.dataSize = serialized.size();
    const auto res = writer.write(msg);
    if (!res.ok()) {
      std::cerr << "Failed to write message: " << res.message << "\n";
      writer.terminate();
      std::ignore = std::remove(outputFilename);
      return 1;
    }
  }
  // Write the index and footer to the file, then close it.
  writer.close();
  std::cerr << "wrote 100 messages to " << outputFilename << std::endl;
  return 0;
}