#ifndef __MSG_LOGGER__
#define __MSG_LOGGER__

#include <memory>
#include <string>

#include <deque>
#include <thread>
#include <functional>
#include <condition_variable>
#include <mutex>

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
                  std::function<void(MsgType)> live_msg_output_func) : _log_file_extension(log_file_extension),
                                                                       _stop_log_function(stop_log_func),
                                                                       _open_log_function(open_log_func)
        {
            _running = true;

            if (init_logging)
            {
                auto log_name = _generate_log_name(_log_file_extension);
                _open_log_function(log_name);
                _logging = init_logging;
            }

            _logger_thread = std::thread(&MsgLogger::_handle_output_messages, this, std::ref(_thread_safe_log), logger_msg_func, true);
            _live_telem_thread = std::thread(&MsgLogger::_handle_output_messages, this, std::ref(_thread_safe_live_output), live_msg_output_func, false);
        }

        ~MsgLogger()
        {
            // should be fine for the most part to not lock here, this is the only place we are writing these bools
            _running = false;
            _logging = false;
            _thread_safe_log.cv.notify_all();
            _thread_safe_live_output.cv.notify_all();
            _logger_thread.join();
            _live_telem_thread.join();
            _stop_log_function();
            std::cout << "safely exited" << std::endl;
        }

        void log_msg(MsgType msg)
        {
            if (_logging)
            {
                _add_msg_to_queue(_thread_safe_log, msg);
            }
            _add_msg_to_queue(_thread_safe_live_output, msg);
        }

        // will only open a new file for logging if we are not currently logging
        void start_logging_to_new_file()
        {
            if (!_logging)
            {
                std::unique_lock lk(_thread_safe_log.mtx);
                _logging = true;
                _open_log_function(_generate_log_name(_log_file_extension));
            }
        }

        void stop_logging_to_file()
        {
            std::cout << "attempting stop" <<std::endl;
            if (_logging)
            {
                std::unique_lock lk(_thread_safe_log.mtx);
                _logging = false;
                _stop_log_function();
            }
            std::cout << "stopped" <<std::endl;
        }
        bool messages_still_need_output()
        {
            {
                std::unique_lock lk(_thread_safe_log.mtx);
                if (!_thread_safe_log.deque.empty())
                {
                    return true;
                }
            }
            {
                std::unique_lock lk(_thread_safe_live_output.mtx);
                if (!_thread_safe_live_output.deque.empty())
                {
                    return true;
                }
            }
            return false;
        }

    private:
        std::string _generate_log_name(const std::string &extension)
        {
            // Get the current time
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);

            // Create a stringstream to format the date and time
            std::stringstream ss;

            // Format: YYYY-MM-DD_HH-MM-SS (with custom prefix and extension)
            ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S")
               << extension;

            // Return the generated filename
            return ss.str();
        }

        void _add_msg_to_queue(ThreadSafeOutput &queue, MsgType msg)
        {
            {
                std::unique_lock lk(queue.mtx);
                queue.deque.push_back(msg);
                queue.cv.notify_all();
            }
        }

        void _handle_output_messages(ThreadSafeOutput &queue, std::function<void(MsgType)> output_function, bool is_logger)
        {
            while (_running)
            {
                std::deque<MsgType> local_q;
                {
                    std::unique_lock lk(queue.mtx);

                    // only the is_logger thread can "pause" while running
                    queue.cv.wait(lk, [is_logger, &local_q, &queue, this]()
                                  { 
                                    return ( (!queue.deque.empty()) || !_running); });
                    if(!queue.deque.empty())
                    {
                        local_q = queue.deque;
                        queue.deque.clear();
                    }
                }

                for (const auto &msg : local_q)
                {
                    output_function(msg);
                }
            }
        }

    private:
        bool _running = true;
        bool _logging = false;
        ThreadSafeOutput _thread_safe_log, _thread_safe_live_output;

        std::string _log_file_extension;
        std::function<void()> _stop_log_function;
        std::function<void(const std::string &)> _open_log_function;
        std::thread _logger_thread;
        std::thread _live_telem_thread;
    };
}
#endif // __MSG_LOGGER__