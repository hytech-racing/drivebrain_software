#ifndef __MSG_LOGGER__
#define __MSG_LOGGER__

#include <memory>
#include <string>

#include <deque>
#include <thread>
#include <functional>
#include <condition_variable>
#include <mutex>
#include <versions.h>

#include "hytech_msgs.pb.h"
#include <google/protobuf/message.h>

namespace core
{
    template <typename MsgType>
    class MsgLogger
    {
    public:
        struct ThreadSafeOutput
        {
            std::mutex mtx;
            std::condition_variable cv;
            std::deque<MsgType> deque;
        };

        MsgLogger(const std::string &log_file_extension,
                  bool init_logging,
                  std::function<void(MsgType)> logger_msg_func,
                  std::function<void()> stop_log_func,
                  std::function<void(const std::string &)> open_log_func,
                  std::function<void(MsgType)> live_msg_output_func) : _logger_msg_function(logger_msg_func),
                                                                       _live_msg_output_func(live_msg_output_func),
                                                                       _log_file_extension(log_file_extension),
                                                                       _stop_log_function(stop_log_func),
                                                                       _open_log_function(open_log_func)
        {
            _running = true;

            if (init_logging)
            {
                auto log_name = _generate_log_name(_log_file_extension);
                _current_log_name = log_name;
                _open_log_function(log_name);
                _logging = init_logging;
            }
        }

        ~MsgLogger()
        {
            // should be fine for the most part to not lock here, this is the only place we are writing these bools
            _running = false;
            _logging = false;
            _stop_log_function();
            std::cout << "safely exited" << std::endl;
        }

        void log_msg(MsgType msg)
        {
            // TODO maybe make this also more thread safe ... ?
            bool logging = true;
            {
                std::unique_lock lk(_mtx);
                logging = _logging;
            }

            if (logging)
            {
                auto out_msg = static_cast<std::shared_ptr<google::protobuf::Message>>(msg);
                {
                    std::unique_lock lk(_mtx);

                    _handle_output_messages(msg, _logger_msg_function);
                    _handle_output_messages(msg, _live_msg_output_func);

                }
            }
            else
            {
                std::unique_lock lk(_mtx);
                _handle_output_messages(msg, _live_msg_output_func);
            }
        }

        // will only open a new file for logging if we are not currently logging
        void start_logging_to_new_file()
        {
            std::unique_lock lk(_mtx);
            if (!_logging)
            {
                _logging = true;
                _current_log_name = _generate_log_name(_log_file_extension);
                _open_log_function(_current_log_name);
            }
        }
        std::tuple<const std::string &, bool> get_logger_status()
        {
            std::unique_lock lk(_mtx);
            return {_current_log_name, _logging};
        }

        void stop_logging_to_file()
        {
            std::cout << "attempting stop" << std::endl;
            {
                std::unique_lock lk(_mtx);
                if (_logging)
                {
                    _logging = false;
                    _stop_log_function();
                }
            }
            std::cout << "stopped" << std::endl;
        }

    private:
        std::string _generate_log_name(const std::string &extension)
        {
            // Get the current time
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);

            // Create a stringstream to format the date and time
            std::stringstream ss;

            ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")
               << extension;

            // Return the generated filename
            return ss.str();
        }

        void _handle_output_messages(MsgType msg, std::function<void(MsgType)> output_function)
        {
            auto out_msg = static_cast<std::shared_ptr<google::protobuf::Message>>(msg);
            if (out_msg->GetDescriptor()->name() == "MCUOutputData")
            {
                auto cast_msg = std::static_pointer_cast<hytech_msgs::MCUOutputData>(out_msg);
                if (cast_msg->brake_percent() == 0)
                {
                    std::cout << "empty msg recvd at output level" << std::endl;
                }
            }
            output_function(msg);
        }

    private:
        bool _running = true;
        bool _logging = false;
        std::mutex _mtx;

        std::string _log_file_extension;
        std::string _current_log_name = "NONE";
        std::function<void()> _stop_log_function;
        std::function<void(const std::string &)> _open_log_function;
        std::function<void(MsgType)> _logger_msg_function;
        std::function<void(MsgType)> _live_msg_output_func;
    };
}
#endif // __MSG_LOGGER__