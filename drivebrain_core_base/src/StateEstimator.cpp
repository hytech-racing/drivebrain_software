#include <StateEstimator.hpp>
#include <chrono>
#include <algorithm>
#include "hytech_msgs.pb.h"
using namespace core;

void StateEstimator::handle_recv_process(std::shared_ptr<google::protobuf::Message> message)
{
    if (message->GetTypeName() == "hytech_msgs.MCUOutputData")
    {
        auto in_msg = std::static_pointer_cast<hytech_msgs::MCUOutputData>(message);
        core::DriverInput input = {(in_msg->accel_percent()), (in_msg->brake_percent())};
        int prev_MCU_recv_millis = in_msg->mcu_recv_millis();
        veh_vec<float> rpms = {in_msg->rpm_data().fl(), in_msg->rpm_data().fr(), in_msg->rpm_data().rl(), in_msg->rpm_data().rr()};
        {
            std::unique_lock lk(_state_mutex);
            _timestamp_array[0] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
            _vehicle_state.input = input;
            _vehicle_state.current_rpms = rpms;
            _vehicle_state.prev_MCU_recv_millis = prev_MCU_recv_millis;
            _vehicle_state.steering_angle_deg = in_msg->steering_angle_deg();
        }
    }

    else if (message->GetTypeName() == "hytech_msgs.VNData")
    {
        auto in_msg = std::static_pointer_cast<hytech_msgs::VNData>(message);
        xyz_vec<float> body_vel_ms = {
            (in_msg->vn_vel_m_s().x()),
            (in_msg->vn_vel_m_s().y()),
            (in_msg->vn_vel_m_s().z())};

        xyz_vec<float> body_accel_mss = {
            (in_msg->vn_linear_accel_m_ss().x()),
            (in_msg->vn_linear_accel_m_ss().y()),
            (in_msg->vn_linear_accel_m_ss().z())};

        xyz_vec<float> angular_rate_rads = {
            (in_msg->vn_angular_rate_rad_s().x()),
            (in_msg->vn_angular_rate_rad_s().y()),
            (in_msg->vn_angular_rate_rad_s().z())};

        ypr_vec<float> ypr_rad = {
            (in_msg->vn_ypr_rad().yaw()),
            (in_msg->vn_ypr_rad().pitch()),
            (in_msg->vn_ypr_rad().roll())};

        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.current_body_vel_ms = body_vel_ms;
            _vehicle_state.current_body_accel_mss = body_accel_mss;
            _vehicle_state.current_angular_rate_rads = angular_rate_rads;
            _vehicle_state.current_ypr_rad = ypr_rad;
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
    

    core::VehicleState current_state;
    {
        _vehicle_state.state_is_valid = state_is_valid;
        std::unique_lock lk(_state_mutex);
        current_state = _vehicle_state;
    }
    // TODO add in state estimation calculations from simulink code gen

    // Create the proto message to send
    std::shared_ptr<hytech_msgs::VehicleData> msg_out = std::make_shared<hytech_msgs::VehicleData>();

    msg_out->set_is_ready_to_drive(true);

    hytech_msgs::SpeedControlIn *current_inputs = msg_out->mutable_current_inputs();
    current_inputs->set_accel_percent(current_state.input.requested_accel);
    current_inputs->set_brake_percent(current_state.input.requested_brake);

    hytech_msgs::xyz_vector *current_body_vel_ms = msg_out->mutable_current_body_vel_ms();
    current_body_vel_ms->set_x(current_state.current_body_vel_ms.x);
    current_body_vel_ms->set_y(current_state.current_body_vel_ms.y);
    current_body_vel_ms->set_z(current_state.current_body_vel_ms.z);

    hytech_msgs::xyz_vector *current_body_accel_mss = msg_out->mutable_current_body_accel_mss();
    current_body_accel_mss->set_x(current_state.current_body_accel_mss.x);
    current_body_accel_mss->set_y(current_state.current_body_accel_mss.y);
    current_body_accel_mss->set_z(current_state.current_body_accel_mss.z);

    hytech_msgs::xyz_vector *current_angular_rate_rads = msg_out->mutable_current_angular_rate_rads();
    current_angular_rate_rads->set_x(current_state.current_angular_rate_rads.x);
    current_angular_rate_rads->set_y(current_state.current_angular_rate_rads.y);
    current_angular_rate_rads->set_z(current_state.current_angular_rate_rads.z);

    hytech_msgs::ypr_vector *current_ypr_rad = msg_out->mutable_current_ypr_rad();
    current_ypr_rad->set_yaw(current_state.current_ypr_rad.yaw);
    current_ypr_rad->set_pitch(current_state.current_ypr_rad.pitch);
    current_ypr_rad->set_roll(current_state.current_ypr_rad.roll);

    hytech_msgs::veh_vec_float *curr_rpms = msg_out->mutable_current_rpms();
    curr_rpms->set_fl(current_state.current_rpms.FL);
    curr_rpms->set_fr(current_state.current_rpms.FR);
    curr_rpms->set_rl(current_state.current_rpms.RL);
    curr_rpms->set_rr(current_state.current_rpms.RR);

    msg_out->set_state_is_valid(state_is_valid);
    msg_out->set_steering_angle_deg(current_state.steering_angle_deg);

    _message_logger->log_msg(static_cast<std::shared_ptr<google::protobuf::Message>>(msg_out));

    return {current_state, state_is_valid};
}