#ifndef __PERFORMANCETRACKER_H__
#define __PERFORMANCETRACKER_H__

#include <optional>
#include <vector>
#include <chrono>

#include <Literals.hpp>
#include <VehicleDataTypes.hpp>
// goals:
// - [ ] be able to track lap times
//      - skid pad
//      - accel
//      - autox
//      - endurance

// - be able to create the lap visualizations
//      - current position on track
//      - display the skidpad track in foxglove (create the foxglove schema protobuf
//        messages containing the associated viz)

// TODOs:

/*
// - [ ] handle geodesic / geographiclib calculations for associated events
//      - [ ] get geopoints for finish line for accel

            for the accel event I want to create the accel event bounding
            box as well with the left and right side of the track estimated from the resting
position of the car itself, assuming it to be in the middle of left and right barriers.

            accel event definition:

            D.9.2.3 "The foremost part of the vehicle will be staged at 0.30 m behind the starting
line"
                - this tells us where in front of the vehicle the start is W.R.T the resting
position of the car.
                    - would be interesting to see how fast we are going when we actually are
starting on the start line

            D.9.1.2 "Course width will be minimum 4.9 m wide as measured between the inner edges of
the bases of the course edge cones"
                - 4.9m is the width

            - [ ] get ecef from GEOID forward for the accel-local coordinate system, use the
starting rest position for the 0,0 in x/y. assume flat ground, z=0 and / or same altitude. This
position will be the ecef offset for the translation.
            - [ ] get the online transformation of the current position of the car for accel-centric
position for visualization and timing purposes.
            - [ ] timing shall start the moment having crossed the starting line
                - our vectornav only has the accuracy ~1m, we will need to see how good it compares
to the regular lap timer
                    - if it is not nearly as good we will need to incorporate the lap timer as a
remote input potentially
        - [ ] handle skidpad event tracking w/ lap time estimates and visualization of estimated
track

//      - [ ] lap completion logic

// - [ ] create the foxglove schema messages with reference overlays for accel / autox / endurance /

// - [ ] track the ackermann radius of the curvature 
// skidpad
*/


// - [ ] TODO derive from configurable base class
/**
 * @class PerformanceTracker
 * @brief
 *
 */
class PerformanceTracker {
  public:
    enum class TrackerMode { NOT_TRACKING, TRACKING_ACCEL, TRACKING_LAP_MANUAL, TRACKING_LAP_DETECT_START };
    
    struct AccelSettings {
        meters track_length;
        meters track_width;
        meters starting_line_lead_in_dist;
    };
    
    struct LapTrackingSettings
    {
        /// @brief // minimum distance between points to be considered two different points on the driven path in meters.
        meters sample_distance; 
    };

    struct Settings {
        AccelSettings accel_settings;
        LapTrackingSettings lap_tracking_settings
    };

    /**
     * @class EventTime
     * @brief returned from the evaluation function of the performance tracker. contains the entire
     * state of the tracker
     *
     */
    struct EventTime {
        TrackerMode mode;
        /// @brief signifies if an event is currently being timed or not.
        bool timing_event;
        /// @brief the current event time in seconds
        float current_event_time;

        /// @brief optional previous times recorded event times. for endurance this
        //         will contain previous lap times and for skid pad it will contain the previous
        //         completion time
        std::optional<std::vector<float>> previous_event_times;
    };
    
    PerformanceTracker(Settings tracker_settings);

    EventTime evaluate_tracker(const core::VehicleState &state);

    /**
     * @brief mode setter for the performance tracker. will be used by the grpc service
     *
     * @param mode the mode to change to.
     */
    void set_tracker_mode(TrackerMode mode);

  private:
    
    enum class InternalState {
        NOT_ATTEMPTING_LAP,
        START_LINE_NOT_CROSSED,
        TIMER_STARTED,
        FINISHED
    };

    std::chrono::time_point<std::chrono::high_resolution_clock> _lap_time;
    Settings _settings;
    EventTime _current_event_timing;
    InternalState _state;

    std::vector<core::GeoPose> _driven_path;
};

#endif // __PERFORMANCETRACKER_H__
