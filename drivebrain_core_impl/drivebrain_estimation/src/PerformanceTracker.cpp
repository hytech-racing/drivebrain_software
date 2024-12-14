#include <PerformanceTracker.hpp>

#include <chrono>

PerformanceTracker::PerformanceTracker(PerformanceTracker::settings tracker_settings)
    : _settings(tracker_settings) {}

PerformanceTracker::EventTime
PerformanceTracker::evaluate_tracker(const core::VehicleState &state) {
    // skidpad (comp-spec):
    // 1. we are driving towards the center of the skidpad for the first time and we have yet to
    // start a time.
    //      watching for significant right turn. once we have started the significant right turn,
    //      set the pos as the start / stop line
    // 2. we are in the start lap of skidpad, watching for the first cross of the start/stop line.
    // 3. we have crossed the start stop line and are in lap 2, starting the right-hand turn skidpad
    // time.
    // 4. we have crossed the start / stop line completing lap 2, immediately stop the timer.
    // 5. we are in the lap 3 warm up lap and need to wait to cross the START/stop line again to
    // start the left hand skidpad lap, start the second lap time
    // 6. we have completed lap 3 and are in lap 4. wait for final cross of the start/stop line.
    // stop left-hand time

    // IN THEORY:
    // skidpad could be an automatically "kicked off" lap-timer where when the start condition has
    // been reached, we start the timer and stop / begin times as soon as the start/stop line has
    // been cross consecutive times

    // states to handle:
    // accel:
    // 1. we are at the accel starting line and we need to detect when we cross the start line
    // 2. we are in accel and we need to continue to monitoring if we have crossed the finish line
    // 3. we have crossed the accel finish line and we need to immediately stop the timer and switch
    // to the resting state

    // autox / endurance timing: -> may be also able to be used for skidpad timing
    // 1. start of lap 1 begins when the start line is crossed.
    //      -> this can either be determined by crossing a parameterized (manual) start line OR
    //          by when the has crossed back over where the car has been before (both sets the
    //          position of the starting line AND starts the lap time)(un-timed cold lap)
    // 2. finish of lap time occurs when the car has crossed back over the start / stop line
    // 3. finish of lap timing occurs when the car has finished a certain number of laps

    switch (_current_event_timing.mode) {
    case (TrackerMode::TRACKING_LAP_DETECT_START): {
        // only add to the path once we have traveled a certain amount of distance. (prevents)
        auto prev_point = _sampled_driven_path.back().position;
        auto cur_point = state.current_pose.position;

        double distance_from_last_path_point, dummy_az1, dummy_az2;
        Geodesic::WGS84.Inverse(cur_point.lat, cur_point.lon, prev_point.lat, prev_point.lon, // ins
                                distance_from_last_path_point, dummy_az1, dummy_az2); // outs

        // only do this if
        if (distance_from_last_path_point > _settings.lap_tracking_settings.sample_distance) {
            _sampled_driven_path.push_back(state.current_pose);
        }
        float current_lap_time =
            _handle_lap_timing_with_start_detection(_sampled_driven_path, state.current_pose);
    }
    default: {
        break;
    }
    }
}

float PerformanceTracker::_handle_lap_timing_with_start_detection(
    const std::vector<core::GeoPose> &driven_path, const core::GeoPose &current_pose) {
    const auto now = std::chrono::high_resolution_clock::now();

    // detect if we have driven back over a point on the path.
    // if any two positions on the driven path have a distance between them that is less than the
    // sample distance, we can infer that we have finished a lap. upon detection

    if (_detect_lap_start_point(driven_path, current_pose).first) {
        _lap_start_time = now;
    }

    // we can re-use the potential lap start time here.
    float duration_us = static_cast_<float>(
        std::chrono::duration_cast<std::chrono::microseconds>(now - _lap_start_time).count());

    // TODO detect finish of lap by checking if we have crossed over the start line
    return (duration_us / 1000000.0f);
}

std::pair<bool, std::optional<std::size_t>>
PerformanceTracker::_detect_lap_start_point(const std::vector<core::GeoPose> &driven_path,
                                            core::GeoPoint current_position) {
    // TODO check to see if the latest position is less than the sample distance away, inferring
    // that we have crossed back over a previous point

    // return
    for (std::size_t i = 0; i < driven_path.size(); i++) {
        auto path_pos = driven_path[i];
        double distance_from_path_point, dummy_az1, dummy_az2;
        Geodesic::WGS84.Inverse(current_position.lat, current_position.lon, path_pos.position.lat,
                                path_pos.position.lon,                           // ins
                                distance_from_path_point, dummy_az1, dummy_az2); // outs

        // if this is true, we know that we have detected a lap being completed as we have traveled
        // to within range of a previous point
        if (distance_from_path_point < settings.lap_tracking_settings.sample_distance) {
            return {true, i};
        }
    }

    return {false, std::nullopt};
}
