#include <spdlog/spdlog.h>
#include <JsonFileHandler.hpp>

void core::JsonFileHandler::save_config() {
    std::ofstream configFile(_configFilePath);
    if (configFile.is_open())
    {
        configFile << _config.dump(4); // Pretty print with an indent of 4 spaces
    }
    else
    {
        //std::cerr << "Failed to open config file for writing.\n";
        spdlog::error("Failed to open config file for writing.");
    }
}

void core::JsonFileHandler::_load_config() {

    std::ifstream configFile(_configFilePath);
    if (configFile.is_open())
    {
        configFile >> _config;
    }
    else
    {
        //std::cerr << "Config file not found, creating new one.\n";
        spdlog::error("Config file not found, creating new one.");
    }
}
