#include <ScaleComms.hpp>
#include <spdlog/spdlog.h>


int main()
{
    spdlog::set_level(spdlog::level::debug);
    core::JsonFileHandler config("config/test_scale_comms.json");
    boost::asio::io_context io_context;
    comms::ScaleComms scale_comms(config, io_context);
    (void)scale_comms.init();
    
    io_context.run();
    return 0;
}
