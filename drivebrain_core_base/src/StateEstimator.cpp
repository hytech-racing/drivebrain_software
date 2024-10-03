#include <StateEstimator.hpp>
#include <chrono>
#include <algorithm>

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

void StateEstimator::_start_recv_thread()
{
    _run_recv_thread = true;
    // create the receive thread by giving it a lambda function that it runs

    // general flow: 
    // while running, wait on a message to enter the queue, then when a message is in the queue, get it and check what type it is.
    // next, cast the shared_ptr to it's proper type and then access it's data and update it's associated vehicle state vars
    // if we dont have a handler for the specific data, simply just continue, no need to do anything with it
    _recv_thread = std::thread([this]()
                               {
        while(_run_recv_thread)
        {
            std::shared_ptr<google::protobuf::Message> input_msg;
            {
                std::unique_lock lk(_msg_in_queue.mtx);
                _msg_in_queue.cv.wait(lk, [this]()
                                        { return !_msg_in_queue.deque.empty(); });
                input_msg = _msg_in_queue.deque.back();
                _msg_in_queue.deque.pop_back();
            }
                // check by the input names what type to cast the input pointer to
                if( input_msg->GetTypeName() == "mcu_pedal_readings")
                {
                    auto in_msg = std::static_pointer_cast<mcu_pedal_readings>(input_msg);
                    core::DriverInput input = { (in_msg->accel_percent_float() / 100.0f), (in_msg->brake_percent_float() / 100.0f)};
                    {
                        std::unique_lock lk(_state_mutex);
                        _timestamp_array[0] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
                        _vehicle_state.input = input;
                    }
                }
                else if(input_msg->GetTypeName() == "vn_vel")
                {
                    auto in_msg = std::static_pointer_cast<vn_vel>(input_msg);
                    xyz_vec<float> body_vel = {in_msg->vn_body_vel_x(), in_msg->vn_body_vel_y(), in_msg->vn_body_vel_z()};
                    {
                        std::unique_lock lk(_state_mutex);
                        _timestamp_array[1] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
                        _vehicle_state.current_body_vel_ms = body_vel;
                    }
                }
                else if(input_msg->GetTypeName() == "drivetrain_rpms_telem")
                {
                    auto in_msg = std::static_pointer_cast<drivetrain_rpms_telem>(input_msg);
                    veh_vec<float> rpms = {(float)in_msg->fl_motor_rpm(), (float)in_msg->fr_motor_rpm(), (float)in_msg->rl_motor_rpm(), (float)in_msg->rr_motor_rpm()}; 
                    {
                        std::unique_lock lk(_state_mutex);
                        _timestamp_array[2] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
                        _vehicle_state.current_rpms = rpms;
                    }
                } else {
                    continue;
                }
        } });
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
    bool last_update_recent_enough = (std::chrono::duration_cast<std::chrono::microseconds>(curr_time - min_stamp)) < threshold;
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