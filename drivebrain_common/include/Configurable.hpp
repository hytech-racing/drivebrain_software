#pragma once

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <string>
#include <iostream>
#include <functional>
#include <mutex>
#include <unordered_map>

// STORY:

// what I want to be able to have this do is be able to be part of every software component and it exposes functionality for handling ptree passed in parameters in a smart way.


// REQUIREMENTS:

// - [ ] configuration should be nested in the following way:
// 1. each component has its own single layer scope
// 2. each scope doesnt have any further scope (one layer deep only)



// - [ ] there will only be ONE config file being edited by ONE thing at a time
    // this is only pertinent to the parameter server for saving the parameters, for accessing at init time we will initialize before kicking off threads 


namespace common
{
    /// @brief this is the class that configurable components inherit from to get parameter access to the top-level ptree
    class Configurable
    {
    public:
        Configurable() = default;
        ~Configurable() = default;

        
    protected:

    };
}