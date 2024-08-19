#pragma once

// TODO make this private 
#include <JsonFileHandler.hpp>

#include <string>
#include <iostream>
#include <functional>
#include <mutex>
#include <unordered_map>
#include <optional>
#include <variant>

// STORY:

// what I want to be able to have this do is be able to be part of every software component and
// it exposes functionality for handling ptree passed in parameters in a smart way.

// NOTES:

// we will need someway of ensuring that none of the component names are the same

// REQUIREMENTS:

// - [ ] configuration should be nested in the following way:
// 1. each component has its own single layer scope
// 2. each scope doesnt have any further scope (one layer deep only)
// - [ ] all parameter value getting has to return
// - [ ] there will only be ONE config file being edited by ONE thing at a time
// this is pertinent to the parameter server for saving the parameters, for accessing at init time we will initialize before kicking off threads

// this is also pertinent to construction of the core of drivebrain since we want the components to be able to update the in-memory version of the json, but only one thing should be able to write it out to the json file once updated

// need to force each configurable instance to have a handler for the runtime-updating of each parameter? or have a run-time callable config access function
// -> this would be hard to police and ensure that the runtime calling only gets called during runtime

// what if we made all the parameter handling be within a specific function for each component
    // then the initial init can call the same set param function that gets called at init time 

namespace core
{
    namespace common
    {
        /// @brief this is the class that configurable components inherit from to get parameter access to the top-level ptree
        class Configurable
        {
        public:
            Configurable(core::JsonFileHandler &json_file_handler, const std::string &component_name)
                : _json_file_handler(json_file_handler), _component_name(component_name) {}

        protected:

            using ParamTypes = std::variant<bool, int, float, std::string>;
            /// @brief Gets a parameter value within the component's scope, ensuring it exists with a default value and if it doesnt it will created it
            /// @tparam ParamType parameter type
            /// @param key the id of the parameter being requested
            /// @return the optional config value
            template <typename ParamType>
            ParamType get_parameter_value(const std::string &key)
            {
                // TODO assert that the template type is only of the specific types supported by nlohmann's json lib
                auto &config = _json_file_handler.get_config();

                // Ensure the component's section exists and if it doesnt we created it
                if (!config.contains(_component_name))
                {
                    std::cout << "WARNING: config file does not contain config for component: " << _component_name << std::endl;
                    return std::nullopt;
                }

                // Access the specific key within the component's section
                if (!config[_component_name].contains(key))
                {
                    std::cout << "WARNING: config file does not contain config: "<< key << " for component: " << _component_name << std::endl;
                    return std::nullopt;
                }
                return config[_component_name][key].get<ParamType>();
            }

            /// @brief TODO: component-scoped parameter setting call. this will be getting called by the parameter server within drivebrain
            /// @param key 
            /// @param parameter_val 
            /// @return true or false depending on if the key exists or not
            bool handle_update_parameter(const std::string &key, std::variant<bool, int, float, std::string> parameter_val)
            {
                auto &config = _json_file_handler.get_config();
                if (config[_component_name].contains(key))
                {
                    // config[_component_name][key] = parameter_val;
                    // set_parameter(key, parameter_value);
                    return true;
                }
                return false;
            }

            // /// @brief TODO: this is the handler that each component must implement that can take in the parameter ID
            // /// @tparam ParamType 
            // /// @param key 
            // /// @param param_value 
            // virtual void set_parameter(const std::string &key, std::variant<bool, int, float, std::string> param_value) = 0;

        private:
            std::string _component_name;
            core::JsonFileHandler &_json_file_handler;
        };

    }

}