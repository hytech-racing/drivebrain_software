#include <PerformanceTracker.hpp>

PerformanceTracker::PerformanceTracker(core::JsonFileHandler &json_file_handler) 
    : core::Configurable(json_file_handler, "PerformanceTracker")
{
    
}