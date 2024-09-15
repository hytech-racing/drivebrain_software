#include <StateEstimator.hpp>
#include <chrono>
#include <algorithm>
#include <drivebrain_core_msgs/v1/drivebrain_vehicle_manager.pb.h>

using namespace core;

void StateEstimator::handle_recv_process(std::shared_ptr<google::protobuf::Message> message)
{
    if (message->GetTypeName() == "MCUData")
    {
        auto in_msg = std::static_pointer_cast<drivebrain_core_msgs::MCUData>(message);
        core::DriverInput input = {(in_msg->accel_percent() / 100.0f), (in_msg->brake_percent() / 100.0f)};
        veh_vec<float> rpms = {in_msg->rpm_data().fl(), in_msg->rpm_data().fr(), in_msg->rpm_data().rl(), in_msg->rpm_data().rr()};
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.input = input;
            _vehicle_state.current_rpms = rpms;
        }
    }
    // if( message->GetTypeName() == "mcu_pedal_readings")
    // {
    //     auto in_msg = std::static_pointer_cast<mcu_pedal_readings>(message);
    //     core::DriverInput input = { (in_msg->accel_percent_float() / 100.0f), (in_msg->brake_percent_float() / 100.0f)};
    //     {
    //         std::unique_lock lk(_state_mutex);
    //         _timestamp_array[0] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
    //         _vehicle_state.input = input;
    //     }
    // }

    // else if(message->GetTypeName() == "vn_vel")
    // {
    //     auto in_msg = std::static_pointer_cast<vn_vel>(message);
    //     xyz_vec<float> body_vel = {in_msg->vn_body_vel_x(), in_msg->vn_body_vel_y(), in_msg->vn_body_vel_z()};
    //     {
    //         std::unique_lock lk(_state_mutex);
    //         _timestamp_array[1] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
    //         _vehicle_state.current_body_vel_ms = body_vel;
    //     }
    // }

    // else if(message->GetTypeName() == "drivetrain_rpms_telem")
    // {
    //     auto in_msg = std::static_pointer_cast<drivetrain_rpms_telem>(message);
    //     veh_vec<float> rpms = {(float)in_msg->fl_motor_rpm(), (float)in_msg->fr_motor_rpm(), (float)in_msg->rl_motor_rpm(), (float)in_msg->rr_motor_rpm()};
    //     {
    //         std::unique_lock lk(_state_mutex);
    //         _timestamp_array[2] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
    //         _vehicle_state.current_rpms = rpms;
    //     }
    // } else {
    //     return;
    // }
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
    const std::chrono::microseconds threshold(30000); // 30 milliseconds in microseconds

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
    auto state_is_valid = _validate_stamps(_timestamp_array);
    {
        std::unique_lock lk(_state_mutex);

        return {_vehicle_state, state_is_valid};
    }
}