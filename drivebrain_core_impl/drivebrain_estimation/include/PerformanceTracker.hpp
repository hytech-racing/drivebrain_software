#ifndef __PERFORMANCETRACKER_H__
#define __PERFORMANCETRACKER_H__
// TODO able to set the start / finish LLA points

// TODO able to detect when the lap has started when 
//      leaving a certain radius of the start point and / or when we have started moving

#include <Configurable.hpp>
#include <VehicleDataTypes.hpp>

struct LLA {
    double latitude;
    double longitude;
    double altitude;
};

struct TrackConfig
{
    LLA start;
    LLA finish;
};

struct Performance
{
    bool active = false;
    double current_lap_time = -1;
};

class PerformanceTracker : public core::common::Configurable
{
    private:
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
        void reset_timer();
    
    public: // functions that will be called by the state estimator

        // Update current position and speed, check for lap start/finish detection
        void update(const lla_t& current_position, double speed);

    private: 

};

#endif // __PERFORMANCETRACKER_H__