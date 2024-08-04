#pragma once

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <string>
#include <iostream>
#include <functional>
#include <mutex>
#include <unordered_map>
namespace common
{
    class Configurable
    {
    public:
        Configurable() = default;
        ~Configurable() = default;

        // Initialize the configuration from a JSON file
        bool init(const std::string &configFile)
        {
            std::lock_guard<std::mutex> lock(_mutex);
            try
            {
                boost::property_tree::read_json(configFile, _configTree);
                return true;
            }
            catch (const boost::property_tree::json_parser_error &e)
            {
                std::cerr << "Error reading configuration file: " << e.what() << std::endl;
                return false;
            }
        }


        // we could integrate this with: https://protobuf.dev/reference/cpp/api-docs/google.protobuf.util.json_util/
        // to handle the parameter updating from foxglove over protobuf. imma look into the foxglove server stuff in python
        // for handling this.
        
        // Update configuration from a JSON string (not from a file)
        bool updateConfigFromJson(const std::string& jsonString)
        {
            std::lock_guard<std::mutex> lock(_mutex);
            try
            {
                std::istringstream jsonStream(jsonString);
                boost::property_tree::ptree updatedTree;
                boost::property_tree::read_json(jsonStream, updatedTree);

                for (const auto& item : updatedTree)
                {
                    _configTree.put(item.first, item.second.data());
                    notifyObservers(item.first);
                }
                return true;
            }
            catch (const boost::property_tree::json_parser_error& e)
            {
                std::cerr << "Error updating configuration from JSON: " << e.what() << std::endl;
                return false;
            }
        }
        
    protected:
        boost::property_tree::ptree _configTree;
        std::mutex _mutex;
    };
}