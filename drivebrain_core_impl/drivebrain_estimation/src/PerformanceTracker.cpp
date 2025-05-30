#include "PerformanceTracker.hpp"
#include "JSONUtils.hpp"

#include <spdlog/spdlog.h>
#include <GeographicLib/Geodesic.hpp>
#include <ratio>

PerformanceTracker::PerformanceTracker(core::JsonFileHandler &json_file_handler) 
    : core::common::Configurable(json_file_handler, "PerformanceTracker")
{

}

void PerformanceTracker::_handle_param_updates(const std::unordered_map<std::string, core::common::Configurable::ParamTypes> &new_param_map)
{
    HANDLE_LIVE_PARAM_LOCK_AND_LOAD(start_lat, float, _config, _config_mutex, new_param_map);
    HANDLE_LIVE_PARAM_LOCK_AND_LOAD(start_lon, float, _config, _config_mutex, new_param_map);
    HANDLE_LIVE_PARAM_LOCK_AND_LOAD(start_alt, float, _config, _config_mutex, new_param_map);
    HANDLE_LIVE_PARAM_LOCK_AND_LOAD(finish_lat, float, _config, _config_mutex, new_param_map);
    HANDLE_LIVE_PARAM_LOCK_AND_LOAD(finish_lon, float, _config, _config_mutex, new_param_map);
    HANDLE_LIVE_PARAM_LOCK_AND_LOAD(finish_alt, float, _config, _config_mutex, new_param_map);
    HANDLE_LIVE_PARAM_LOCK_AND_LOAD(bubble_radius_m, float, _config, _config_mutex, new_param_map);
}

bool PerformanceTracker::init() {
    LOAD_LIVE_PARAM_OR_FAIL(start_lat, float, _config);
    LOAD_LIVE_PARAM_OR_FAIL(start_lon, float, _config);
    LOAD_LIVE_PARAM_OR_FAIL(start_alt, float, _config);
    LOAD_LIVE_PARAM_OR_FAIL(finish_lat, float, _config);
    LOAD_LIVE_PARAM_OR_FAIL(finish_lon, float, _config);
    LOAD_LIVE_PARAM_OR_FAIL(finish_alt, float, _config);
    LOAD_LIVE_PARAM_OR_FAIL(bubble_radius_m, float, _config);
    
    param_update_handler_sig.connect(boost::bind(&PerformanceTracker::_handle_param_updates, this, std::placeholders::_1));
    
    set_configured();
    return true;
}

void PerformanceTracker::activate()
{
    _performance_state.active = true;
}

void PerformanceTracker::reset_tracker()
{
    // TODO lap count reset
    _performance_state.current_lap_time_ms = 0;
    _performance_state.active = false;
    _performance_state.lap_count = 0;
}

void PerformanceTracker::deactivate()
{
    _performance_state.active = false;
}

using namespace GeographicLib;
Performance PerformanceTracker::update(const core::Position& current_position)
{
    // Calculate distance to finish position

    switch(_performance_state.state_machine_state)
    const Geodesic& geod = Geodesic::WGS84();
    double distance_from_finish_m, distance_from_start_m;
    double azi1, azi2;

    geod.Inverse(current_position.lat, current_position.lon,
                 _config.finish_lat, _config.finish_lon,
                 distance_from_finish_m, azi1, azi2);

    geod.Inverse(current_position.lat, current_position.lon,
                 _config.start_lat, _config.start_lon,
                 distance_from_start_m, azi1, azi2);
    
    // Get current time (steady clock for accurate time deltas)
    auto now = std::chrono::steady_clock::now();

    // If this is the first call, keep initializing the timestamp
    if (!_performance_state.active) {
        _performance_state.last_update_time = now;
        return _performance_state;
    }
    
    // next, we determine if we have actually left the starting bubble yet
    // bool timer_was_started = _performance_state.timer_started;
    _performance_state.timer_started = (distance_from_start_m > _config.bubble_radius_m);

    if(!_performance_state.timer_started)
    {
        _performance_state.last_update_time = std::chrono::steady_clock::now();
        return _performance_state;
    }

    // Calculate delta time since last update
    std::chrono::duration<double, std::milli> delta_time_ms = now - _performance_state.last_update_time;
    _performance_state.last_update_time = now;


    _performance_state.current_lap_time_ms += delta_time_ms.count();

    // Bubble entry detection (edge trigger)
    bool currently_in_bubble = (distance_from_finish_m <= _config.bubble_radius_m);
    
    // Update current lap time if are not in the completion bubble
    if (currently_in_bubble && !_performance_state.was_in_bubble_last_update)
    {
        // Just entered the bubble — complete the lap
        auto completed_lap_time_ms = _performance_state.current_lap_time_ms;

        // Store lap time, print, log, etc.
        std::cout <<"from cout " << _performance_state.current_lap_time_ms <<std::endl;
        spdlog::info("lap completed time: {}ms", completed_lap_time_ms);
        _performance_state.lap_count++;
        _performance_state.lap_times.push_back(completed_lap_time_ms);
    }

    // Update state for next call
    _performance_state.was_in_bubble_last_update = currently_in_bubble;
    return _performance_state;
}
