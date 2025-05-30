#include "MCAPReplay.hpp"

#include <JsonFileHandler.hpp>
#include <cmath>
#include <spdlog/spdlog.h>

#include <chrono>

namespace gp = google::protobuf;

namespace util {
MCAPReplay::MCAPReplay(std::string param_file_path, std::string dbc_file_path)
    : _config(param_file_path) {
    std::cout << param_file_path << std::endl;
    bool cf = false;
    _driver_primary_can =
        std::make_shared<comms::CANDriver>(_config, _primary_can_tx_queue, _io_context,
                                           dbc_file_path, cf, nullptr, "CANDriverPrimary");
    if(cf)
    {
        throw std::runtime_error("Failed to initialize can driver");
    }

    _acu_sender = std::make_shared<comms::ETHSendComms>(_io_context, 7766, "127.0.0.1", false);
    // _acu_core_sender = std::make_shared<comms::ETHSendComms>(nullptr, _io_context, 7777, "127.0.0.1");
    // _vcr_sender = std::make_shared<comms::ETHSendComms>(nullptr, _io_context, 9999, "127.0.0.1");
    _vn_sender = std::make_shared<comms::ETHSendComms>(_io_context, 13111, "127.0.0.1", false); // fake VN 
    

}

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
                return false;
            }
        }
    }
    return true;
}

std::shared_ptr<gp::Message> MCAPReplay::_get_pb_msg(gp::DescriptorPool &protoPool,
                                                     gp::DynamicMessageFactory &protoFactory,
                                                     const mcap::SchemaPtr schema,
                                                     google::protobuf::SimpleDescriptorDatabase *protoDbPtr, mcap::Message mcap_msg) {
    const gp::Descriptor *descriptor = protoPool.FindMessageTypeByName(schema->name);


    if (descriptor == nullptr) {
        if (!_load_schema(schema, protoDbPtr)) {
            std::cerr << "failed to load schema" << std::endl;
            return nullptr;
        }
        descriptor = protoPool.FindMessageTypeByName(schema->name);
        if (descriptor == nullptr) {
            std::cerr << "failed to find descriptor after loading pool" << std::endl;
            return nullptr;
        }
    }
    auto msg = std::shared_ptr<gp::Message>(protoFactory.GetPrototype(descriptor)->New());


    if (!msg->ParseFromArray(mcap_msg.data, static_cast<int>(mcap_msg.dataSize))) {
      std::cerr << "failed to parse message" << std::endl;
      return nullptr;
    }

    return msg;
}

void MCAPReplay::start(std::string filename) {
    const auto res = _mcap_reader.open(filename);
    if (!res.ok()) {

        spdlog::error("Failed to open {}", filename);
        return;
    }

    _io_context_thread = std::thread([this]() {
        try {
            _io_context.run();
        } catch (const std::exception &e) {
            spdlog::error("Error in io_context: {}", e.what());
        }
    });

    auto prev_ns = std::chrono::nanoseconds(-1);
    auto msg_view = _mcap_reader.readMessages();
    auto prev_start = std::chrono::high_resolution_clock::now();
    gp::SimpleDescriptorDatabase protoDb;
    gp::DescriptorPool protoPool(&protoDb);
    gp::DynamicMessageFactory protoFactory(&protoPool);

    for (auto it = msg_view.begin(); it != msg_view.end(); it++) {
        // skip any non-protobuf-encoded messages.

        auto loop_start = std::chrono::high_resolution_clock::now();
        if (it->schema->encoding != "protobuf") {
            continue;
        }

        auto next_ns = std::chrono::nanoseconds(it->message.logTime);

        // spdlog::info("time diff in ns: {}", std::chrono::duration_cast<std::chrono::nanoseconds>(
        // next_ns-prev_ns).count());
        // TODO add configurable divider for the sleep time

        if (prev_ns.count() > 0) {
            auto intended_duration = next_ns - prev_ns;
            auto actual_elapsed = (loop_start - prev_start);
            if (actual_elapsed < intended_duration) {
                std::this_thread::sleep_for(intended_duration - actual_elapsed);
            }
        }

        prev_ns = next_ns;

        auto msg = _get_pb_msg(protoPool, protoFactory, it->schema, &protoDb, it->message);

        auto msg_name = it->schema->name;
        // TODO make filter configure-able
        if ((!msg_name.rfind("hytech.", 0)) && 
            !(msg_name == "hytech.drivebrain_torque_lim_input") && 
            !(msg_name == "hytech.drivebrain_speed_set_input") &&
            !(msg_name == "hytech.drivebrain_desired_torque_input")
        ) // denotes CAN message that is not a drivebrain output
        {
            {
                std::unique_lock lk(_primary_can_tx_queue.mtx);
                _primary_can_tx_queue.deque.push_back(msg);
                _primary_can_tx_queue.cv.notify_all();
            }
        } 
        else if(it->schema->name == "hytech_msgs.ACUAllData")
        {
            spdlog::info("sending ACUALLData");
            _acu_sender->enqueue_msg_to_send(msg);
        } else if(it->schema->name == "hytech_msgs.VNData")
        {
            _vn_sender->enqueue_msg_to_send(msg);
        }

        prev_start = loop_start;
    }

    spdlog::info("halted message logger");
    _io_context.stop();
    if (_io_context_thread.joinable()) {
        _io_context_thread.join();
        spdlog::info("joined io context");
    }

    _mcap_reader.close();
}
} // namespace util