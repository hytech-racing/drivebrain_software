#ifndef __MSG_LOGGER__
#define __MSG_LOGGER__

#include <memory>
#include <string>

namespace core
{
    template <typename MsgType>
    class MsgLogger
    {
    public:
        MsgLogger(const std::string &log_output_file_dir, bool init_logging) : _output_file_dir(log_output_file_dir) {
            if(init_logging)
            {
                auto log_name = generate_log_name();
                start_new_log(log_name);
                _logging = init_logging;
            }
            
        }

        virtual std::string generate_log_name() = 0;
        virtual void log_msg(std::shared_ptr<MsgType> msg) = 0;
        virtual void stop_log() = 0;
        virtual void start_new_log(const std::string& log_file_dir) =0;

    private:
        bool _logging = false;
        std::string _output_file_dir;
    };
}
#endif // __MSG_LOGGER__