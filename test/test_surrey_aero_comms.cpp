#include <SurreyAeroComms.hpp>
#include <spdlog/spdlog.h>


int main()
{
    spdlog::set_level(spdlog::level::debug);
    core::JsonFileHandler config("config/test_surrey_aero_comms.json");
    boost::asio::io_context io_context;
    comms::SurreyAeroComms aero_comms(config, io_context);
    (void)aero_comms.init();
    
    io_context.run();
    return 0;
}
