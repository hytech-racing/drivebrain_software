#pragma once

// TODO make this private
#include <JsonFileHandler.hpp>
#include <Logger.hpp>

#include <boost/signals2.hpp>

#include <string>
#include <iostream>
#include <functional>
#include <mutex>
#include <unordered_map>
#include <optional>
#include <variant>
#include <mutex>

// STORY:

// what I want to be able to have this do is be able to be part of every software component and
// it exposes functionality for handling ptree passed in parameters in a smart way.

// NOTES:

// we will need someway of ensuring that none of the component names are the same

// REQUIREMENTS:

// - [x] configuration should be nested in the following way:
// 1. each component has its own single layer scope
// 2. each scope doesnt have any further scope (one layer deep only)
// - [x] all parameter value getting has to return
// - [x] there will only be ONE config file being edited by ONE thing at a time
// this is pertinent to the parameter server for saving the parameters, for accessing at init time we will initialize before kicking off threads

// this is also pertinent to construction of the core of drivebrain since we want the components to be able to update the in-memory version of the json, but only one thing should be able to write it out to the json file once updated

// need to force each configurable instance to have a handler for the runtime-updating of each parameter? or have a run-time callable config access function
// -> this would be hard to police and ensure that the runtime calling only gets called during runtime

// what if we made all the parameter handling be within a specific function for each component
// then the initial init can call the same set param function that gets called at init time

// live parameters:
// - [ ] add live parameter handling through use of boost signals

// the live parameter settings will be handled by having a true or false flag within the get_parameter.
// if this flag is set, the is gotten from the config file and then the map of live parameters gets it's
namespace core
{
    namespace common
    {
        /// @brief this is the class that configurable components inherit from to get parameter access to the top-level ptree
        class Configurable
        {

        public:
            using ParamTypes = std::variant<bool, int, float, double, std::string, std::monostate>;
            Configurable(core::Logger &logger, core::JsonFileHandler &json_file_handler, const std::string &component_name)
                : _logger(logger), _json_file_handler(json_file_handler), _component_name(component_name) {}

            std::string get_name();
            std::vector<std::string> get_param_names();

            std::unordered_map<std::string, ParamTypes> get_params_map();

            void handle_live_param_update(const std::string &key, ParamTypes param_val);

            Configurable::ParamTypes get_cached_param(std::string id);

        protected:
            boost::signals2::signal<void(const std::unordered_map<std::string, ParamTypes> &)> param_update_handler_sig;
            virtual bool init() = 0;

            template <typename ParamType>
            void _handle_assert()
            {
                static_assert(
                    std::is_same_v<ParamType, bool> ||
                        std::is_same_v<ParamType, int> ||
                        std::is_same_v<ParamType, double> ||
                        std::is_same_v<ParamType, float> ||
                        std::is_same_v<ParamType, std::string>,
                    "ParamType must be bool, int, double, float, or std::string");
            }
            /// @brief Gets a parameter value within the component's scope, ensuring it exists with a default value and if it doesnt it will created it
            /// @tparam ParamType parameter type
            /// @param key the id of the parameter being requested
            /// @return the optional config value
            template <typename ParamType>
            std::optional<ParamType> get_parameter_value(const std::string &key)
            {
                _handle_assert<ParamType>();

                // TODO assert that the template type is only of the specific types supported by nlohmann's json lib
                auto &config = _json_file_handler.get_config();

                // Ensure the component's section exists and if it doesnt we created it
                if (!config.contains(_component_name))
                {
                    auto log_str = std::string("config file does not contain config for component: ") + _component_name;

                    _logger.log_string(log_str, core::LogLevel::WARNING);
                    return std::nullopt;
                }

                // Access the specific key within the component's section
                if (!config[_component_name].contains(key))
                {
                    auto log_str = std::string("config file does not contain config: ") + key + std::string("for component: ") + _component_name;
                    _logger.log_string(log_str, core::LogLevel::WARNING);

                    return std::nullopt;
                }
                return config[_component_name][key].get<ParamType>();
            }

            template <typename ParamType>
            std::optional<ParamType> get_live_parameter(const std::string &key)
            {
                _handle_assert<ParamType>();
                auto res = get_parameter_value<ParamType>(key);

                if (!res)
                {
                    return std::nullopt;
                }
                else
                {
                    {
                        std::unique_lock lk(_live_params.mtx);
                        _live_params.param_vals[key] = *res;
                    }
                }
                return res;
            }

        private:
            core::Logger &_logger;
            std::string _component_name;
            core::JsonFileHandler &_json_file_handler;

            struct
            {
                std::unordered_map<std::string, ParamTypes> param_vals;
                std::mutex mtx;
            } _live_params;
        };

    }

}