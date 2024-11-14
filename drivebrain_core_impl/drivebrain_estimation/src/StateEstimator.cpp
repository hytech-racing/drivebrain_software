#include <StateEstimator.hpp>
#include <chrono>
#include <algorithm>
#include "hytech_msgs.pb.h"
#include <Tire_Model_Codegen_MatlabModel.hpp>
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
            _raw_input_data.raw_load_cell_values = {
                in_msg->load_cell_data().fl(),
                in_msg->load_cell_data().fr(),
                in_msg->load_cell_data().rl(),
                in_msg->load_cell_data().rr(),
            };
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

void StateEstimator::set_previous_control_output(SpeedControlOut prev_control_output)
{
    std::unique_lock lk(_state_mutex);
    _vehicle_state.prev_controller_output = prev_control_output;
}

std::pair<core::VehicleState, bool> StateEstimator::get_latest_state_and_validity()
{
    // auto state_is_valid = _validate_stamps(_timestamp_array);
    auto state_estim_start = std::chrono::high_resolution_clock::now();
    core::VehicleState current_state;
    core::RawInputData current_raw_data;
    auto state_mutex_start = std::chrono::high_resolution_clock::now();
    {
        std::unique_lock lk(_state_mutex);
        current_state = _vehicle_state;
        current_raw_data = _raw_input_data;
    }
    auto state_mutex_end = std::chrono::high_resolution_clock::now();

    // Create the proto message to send
    std::shared_ptr<hytech_msgs::VehicleData> msg_out = std::make_shared<hytech_msgs::VehicleData>();

    ////////////////////////////////////////////////////////////////////////////
    /// matlab math ////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    auto matlab_math_start = std::chrono::high_resolution_clock::now();

    // Convert Vehicle and Raw Data to Tire Model Input
    bool sign = (current_state.input.requested_accel - current_state.input.requested_brake) >= 0;
    estimation::Tire_Model_Codegen_MatlabModel::inputs model_inputs = {
        current_state.prev_controller_output.torque_lim_nm.FL * (sign ? 1.0f : -1.0f),
        current_state.prev_controller_output.torque_lim_nm.FR * (sign ? 1.0f : -1.0f),
        current_state.prev_controller_output.torque_lim_nm.RL * (sign ? 1.0f : -1.0f),
        current_state.prev_controller_output.torque_lim_nm.RR * (sign ? 1.0f : -1.0f),
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        current_raw_data.raw_load_cell_values.FL,
        current_raw_data.raw_load_cell_values.FR,
        current_raw_data.raw_load_cell_values.RL,
        current_raw_data.raw_load_cell_values.RR,
        current_state.current_rpms.FL,
        current_state.current_rpms.RL,
        current_state.current_rpms.FR,
        current_state.current_rpms.RR,
        current_state.steering_angle_deg,
        current_state.current_body_vel_ms.x,
        current_state.current_angular_rate_rads.z,
        current_state.current_body_vel_ms.y
    };

    
    auto res = _matlab_estimator.evaluate_estimator(model_inputs);
    auto matlab_math_end = std::chrono::high_resolution_clock::now();
    auto state_mutex_2_start = std::chrono::high_resolution_clock::now();
    {
        std::unique_lock lk(_state_mutex);
        _vehicle_state.matlab_math_temp_out = {res.torq_req_FL,res.torq_req_FR,res.torq_req_RL,res.torq_req_RR};
    }
    auto state_mutex_2_end = std::chrono::high_resolution_clock::now();

    hytech_msgs::TireDynamics *current_tire_dynamics = msg_out->mutable_tire_dynamics();
    auto current_tire_forces = current_tire_dynamics->mutable_tire_forces_n();


    auto fl_xyz = current_tire_forces->mutable_fl();
    fl_xyz->set_x(res.FXFL);
    fl_xyz->set_y(res.FYFL);
    fl_xyz->set_z(res.FZFL);

    auto fr_xyz = current_tire_forces->mutable_fr();
    fr_xyz->set_x(res.FXFR);
    fr_xyz->set_y(res.FYFR);
    fr_xyz->set_z(res.FZFR);

    auto rl_xyz = current_tire_forces->mutable_rl();
    rl_xyz->set_x(res.FXRL);
    rl_xyz->set_y(res.FYRL);
    rl_xyz->set_z(res.FZRL);

    auto rr_xyz = current_tire_forces->mutable_rr();
    rr_xyz->set_x(res.FXRR);
    rr_xyz->set_y(res.FYRR);
    rr_xyz->set_z(res.FZRR);

    auto current_tire_moments_nm = current_tire_dynamics->mutable_tire_moments_nm();

    auto fl_xyz_moments = current_tire_moments_nm->mutable_fl();
    fl_xyz_moments->set_x(0);
    fl_xyz_moments->set_y(0);
    fl_xyz_moments->set_z(res.MZFL);

    auto fr_xyz_moments = current_tire_moments_nm->mutable_fr();
    fr_xyz_moments->set_x(0);
    fr_xyz_moments->set_y(0);
    fr_xyz_moments->set_z(res.MZFR);

    auto rl_xyz_moments = current_tire_moments_nm->mutable_rl();
    rl_xyz_moments->set_x(0);
    rl_xyz_moments->set_y(0);
    rl_xyz_moments->set_z(res.MZRL);

    auto rr_xyz_moments = current_tire_moments_nm->mutable_rr();
    rr_xyz_moments->set_x(0);
    rr_xyz_moments->set_y(0);
    rr_xyz_moments->set_z(res.MZRR);

    auto current_accel_saturation_nm = current_tire_dynamics->mutable_accel_saturation_nm();

    current_accel_saturation_nm->set_fl(res.satAccelTFL);
    current_accel_saturation_nm->set_fr(res.satAccelTFR);
    current_accel_saturation_nm->set_rl(res.satAccelTRL);
    current_accel_saturation_nm->set_rr(res.satAccelTRR);

    auto current_brake_saturation_nm = current_tire_dynamics->mutable_brake_saturation_nm();

    current_brake_saturation_nm->set_fl(res.satBrakeTFL);
    current_brake_saturation_nm->set_fr(res.satBrakeTFR);
    current_brake_saturation_nm->set_rl(res.satBrakeTRL);
    current_brake_saturation_nm->set_rr(res.satBrakeTRR);

    msg_out->set_v_y_lm(res.Vy_LM);
    msg_out->set_psi_dot_lm_rad_s(res.Psi_dot_LMrads);

    hytech_msgs::TorqueVectoringStatus *current_tv_status = msg_out->mutable_tv_status();

    auto torque_add = current_tv_status->mutable_torque_additional_nm();
    torque_add->set_fl(res.Torq_Add_FL);
    torque_add->set_fr(res.Torq_Add_FR);
    torque_add->set_rl(res.Torq_Add_RL);
    torque_add->set_rr(res.Torq_Add_RR);
    current_tv_status->set_additional_mz_moment_nm(res.AdditionalMzNm);
    current_tv_status->set_des_psi_dot(res.DesiredYawRaterads);
    current_tv_status->set_psi_dot_err(res.Yaw_Rate_Err);
    current_tv_status->set_perceived_vx(res.Perceived_Vx);
    current_tv_status->set_integral_yaw_rate_err(res.Integral_Yaw_Rate_Err);
    current_tv_status->set_perceived_psi_dot(res.perceived_psi_dot);
    current_tv_status->set_vy_vn_gain(res.vy_vn_gain);
    current_tv_status->set_psi_dot_gain(res.psi_dot_gain);
    current_tv_status->set_perceived_vy(res.perceived_vy);


    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

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

    msg_out->set_state_is_valid(true);
    msg_out->set_steering_angle_deg(current_state.steering_angle_deg);

    auto prev_driver_torque_req = msg_out->mutable_driver_torque();
    prev_driver_torque_req->set_fl(current_state.prev_controller_output.torque_lim_nm.FL);
    prev_driver_torque_req->set_fr(current_state.prev_controller_output.torque_lim_nm.FR);
    prev_driver_torque_req->set_rl(current_state.prev_controller_output.torque_lim_nm.RL);
    prev_driver_torque_req->set_rr(current_state.prev_controller_output.torque_lim_nm.RR);

    auto log_start = std::chrono::high_resolution_clock::now();
    _message_logger->log_msg(static_cast<std::shared_ptr<google::protobuf::Message>>(msg_out));
    auto log_end = std::chrono::high_resolution_clock::now();
    
    auto state_estim_end = std::chrono::high_resolution_clock::now();

    // auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(state_estim_end - state_estim_start);
    // if (elapsed > std::chrono::microseconds(6000))
    // {
    //     // std::cout << "WARNING: timing" << std::endl;
    //     // std::cout << "total: " << (static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count())) << " us\n";
    //     // std::cout << "state mutex: " << (static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(state_mutex_end - state_mutex_start).count())) << " us\n";
    //     // std::cout << "state mutex 2: " << (static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(state_mutex_2_end - state_mutex_2_start).count())) << " us\n";
    //     // std::cout << "matlab math: " << (static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(matlab_math_end - matlab_math_start).count())) << " us\n";
    //     // std::cout << "log time: " << (static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(log_end - log_start).count())) << " us\n";

    // }

    return {current_state, true};
}