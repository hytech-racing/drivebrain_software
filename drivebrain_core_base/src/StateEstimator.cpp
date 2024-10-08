#include <StateEstimator.hpp>
#include <chrono>
#include <algorithm>
#include "hytech_msgs.pb.h"
using namespace core;


bool StateEstimator::init()
{
    // shared threshold for both maximum jitter and maximum time for received state variables for valid state estimate 
    std::optional threshold_microseconds = get_live_parameter<int>("threshold_microseconds");

    if (!(threshold_microseconds))
    {
        return false;
    }

    _config = {*threshold_microseconds};
    return true;
}

void StateEstimator::handle_recv_process(std::shared_ptr<google::protobuf::Message> message)
{
    if (message->GetTypeName() == "hytech_msgs.MCUOutputData")
    {
        auto in_msg = std::static_pointer_cast<hytech_msgs::MCUOutputData>(message);
        core::DriverInput input = {(in_msg->accel_percent()), (in_msg->brake_percent())};
        veh_vec<float> rpms = {in_msg->rpm_data().fl(), in_msg->rpm_data().fr(), in_msg->rpm_data().rl(), in_msg->rpm_data().rr()};
        {
            std::unique_lock lk(_state_mutex);
            _timestamp_array[0] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
            _vehicle_state.input = input;
            _vehicle_state.current_rpms = rpms;
        }
    }
}

// TODO parameterize the timeout threshold
template <size_t arr_len>
bool StateEstimator::_validate_stamps(const std::array<std::chrono::microseconds, arr_len> &timestamp_arr)
{
    std::array<std::chrono::microseconds, arr_len> timestamp_array_to_sort;
    {
        std::unique_lock lk(_state_mutex);
        timestamp_array_to_sort = timestamp_arr;
    }
    const std::chrono::microseconds threshold(_config.threshold_microseconds); 
    // Sort the array
    std::sort(timestamp_array_to_sort.begin(), timestamp_array_to_sort.end());

    // Check if the range between the smallest and largest timestamps is within the threshold
    auto min_stamp = timestamp_array_to_sort.front();
    auto max_stamp = timestamp_array_to_sort.back();

    bool within_threshold = (max_stamp - min_stamp) <= threshold;

    auto curr_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
    bool all_members_received = min_stamp.count() > 0;
    bool last_update_recent_enough = (std::chrono::duration_cast<std::chrono::microseconds>(curr_time - max_stamp)) < threshold;
    return within_threshold && all_members_received && last_update_recent_enough;
}

std::pair<core::VehicleState, bool> StateEstimator::get_latest_state_and_validity()
{
    bool state_is_valid = _validate_stamps(_timestamp_array);
    {
        std::unique_lock lk(_state_mutex);
        
        return {_vehicle_state, state_is_valid};
    }
}