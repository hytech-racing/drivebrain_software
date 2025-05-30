#ifndef __PERFORMANCETRACKER_H__
#define __PERFORMANCETRACKER_H__
// TODO able to set the start / finish LLA points

// TODO able to detect when the lap has started when 
//      leaving a certain radius of the start point and / or when we have started moving

// design:


// loop lap timer state:
// when active, detect when we have left the starting bubble and start the timer. once crossing back into the start bubble

// start / finish timer state:
// when active, detect when we have left the starting bubble and start the timer, only once we have crossed into the finish bubble do we stop the time. 
// push back to the lap times the time that was reached. 


#include <Configurable.hpp>
#include <VehicleDataTypes.hpp>
#include <chrono>


struct TrackConfig
{
    core::Position start;
    core::Position finish;
};

struct Performance
{
    
    bool timer_started = false;
    double current_lap_time_ms = 0;
    bool was_in_bubble_last_update = false;
    std::chrono::time_point<std::chrono::steady_clock> last_update_time{};
    int lap_count=0;
    std::vector<double> lap_times;
};

enum class PerformanceTrackerState
{
    NOT_ACTIVE = 0,
    LAP_TIMING = 1,

};

class PerformanceTracker : public core::common::Configurable
{
    private:
        std::mutex _config_mutex; 
        struct config
        {
            float start_lat;
            float start_lon;
            float start_alt;
            float finish_lat;
            float finish_lon;
            float finish_alt;
            float bubble_radius_m; 
        } _config{};

    public:
        PerformanceTracker(core::JsonFileHandler &json_file_handler);
        bool init();
    
    public: 
        // functions that will get called by the grpc service
        void reset_tracker();
        void activate();
        void deactivate();
    public: // functions that will be called by the state estimator

        // Update current position check for lap start/finish detection and return the performance state
        Performance update(const core::Position& current_position);
    private:
        void _handle_param_updates(const std::unordered_map<std::string, core::common::Configurable::ParamTypes> &new_param_map);
    private:
        
        Performance _performance_state{};
};

#endif // __PERFORMANCETRACKER_H__