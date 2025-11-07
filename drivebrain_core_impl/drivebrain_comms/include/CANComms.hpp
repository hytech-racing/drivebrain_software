#pragma once
// drivebrain includes
#include <Configurable.hpp>
#include <Loggable.hpp>
#include <DriverBus.hpp>
#include <MsgLogger.hpp>
#include <StateEstimator.hpp>
#include <hytech.pb.h> // generated from CAN description

// system includes
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>

// protobuf
#include <google/protobuf/any.pb.h>
#include <google/protobuf/message.h>
#include <google/protobuf/dynamic_message.h>

// boost
#include <boost/asio.hpp>

// dbcppp
#include <Signal.h>
#include <Network.h>

// c++ stl includes
#include <memory>
#include <deque>
#include <variant>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <functional>
#include <optional>

#include <unistd.h>
#include <cstring>

namespace comms
{
    class CANDriver : public core::common::Loggable<std::shared_ptr<google::protobuf::Message>>,
                      public core::common::Configurable
    {
    public:
        using FieldVariant = std::variant<int32_t, int64_t, uint32_t, uint64_t, float, double, bool, std::string, std::monostate>;
        using deqtype = core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>>;

        CANDriver(core::JsonFileHandler &json_file_handler, boost::asio::io_context& io_context, std::optional<std::string> dbc_path, bool &construction_failed, std::shared_ptr<core::StateEstimator> state_estimator) : 
            Configurable(json_file_handler, "CANDriver"),
            _socket(io_context),
            _dbc_path(dbc_path),
            _state_estimator(state_estimator) {
            _running = true;
            construction_failed = !init();
        }

        CANDriver(core::JsonFileHandler &json_file_handler, boost::asio::io_context& io_context, std::optional<std::string> dbc_path, bool &construction_failed, std::shared_ptr<core::StateEstimator> state_estimator, std::string driver_name) : 
            Configurable(json_file_handler, driver_name),
            _socket(io_context),
            _dbc_path(dbc_path),
            _state_estimator(state_estimator)
        {
            _running = true;
            construction_failed = !init();
        }
        ~CANDriver();
        bool init();


        std::shared_ptr<google::protobuf::Message> pb_msg_recv(const can_frame &in_frame);
        void set_field_values_of_pb_msg(const std::unordered_map<std::string, FieldVariant> &field_values, std::shared_ptr<google::protobuf::Message> message);
        void send_message(std::shared_ptr<google::protobuf::Message> pb_msg);

        // TODO: move this into a util library?

        /// @brief get the value from a field within a protobuf message based on the name of the field
        /// @param message
        /// @param field_name
        /// @return variant of types
        FieldVariant get_field_value(std::shared_ptr<google::protobuf::Message> message, const std::string &field_name);

        
        // for exposing to the test framework directly
    protected:
        // socket operations
        bool _open_socket(const std::string& interface_name);
        void _do_read();

        void _handle_recv_CAN_frame(const struct can_frame& frame);

        std::shared_ptr<google::protobuf::Message> _get_pb_msg_by_name(const std::string &name);
        std::optional<can_frame> _get_CAN_msg(std::shared_ptr<google::protobuf::Message> msg);

    private:
        static std::string _to_lowercase(std::string s);

    private:
        std::condition_variable _cv;

        struct can_frame _frame;

        boost::asio::posix::stream_descriptor _socket;
        std::optional<std::string> _dbc_path;

        std::unordered_map<uint64_t, std::unique_ptr<dbcppp::IMessage>> _messages;
        std::unordered_map<std::string, uint64_t> _messages_names_and_ids;
        int _CAN_socket; // can socket bound to
        bool _running = false;
        std::shared_ptr<core::StateEstimator> _state_estimator;
        size_t _recv_process_count = 0;
        const size_t _recv_process_log_interval = 1000;
        long _recv_process_max_duration_us = 0;           // Microseconds
    };
}
