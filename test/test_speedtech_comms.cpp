#include <SpeedTechComms.hpp>
#include <spdlog/spdlog.h>

int main()
{
    spdlog::set_level(spdlog::level::debug);
    core::JsonFileHandler config("config/test_speedtech_comms.json");
    boost::asio::io_context io_context;
    comms::SpeedTechComms speed_tech_lap_timer_driver(config, io_context);
    (void)speed_tech_lap_timer_driver.init();
    
    io_context.run();
    return 0;
}