#ifndef __PARAMLOGGER_H__
#define __PARAMLOGGER_H__

#include <Configurable.hpp>

#include <optional>
#include <vector>
#include <string>
// #include <thread>

// handles the logging of the params from all of the configurable components

// functions:
// 1. verifies that all components have been configured upon construction 
//      (NOTE: this boi needs to be constructed only after the components have been initialized for the first time)
// 2. creates the json schema given the map of parameters
// 3. gathers the maps of params contained within the components
 
namespace util
{
std::optional<std::string> get_schema(std::vector<core::common::Configurable *> configurable_components);
}
#endif // __PARAMLOGGER_H__