#include <StateEstimator.hpp>
#include <Tire_Model_Codegen.h>
#include <VehicleDataTypes.hpp>
#include <base_msgs.pb.h>
#include <chrono>
#include <algorithm>
#include "hytech_msgs.pb.h"
#include "hytech.pb.h" // from HT_CAN
#include <Tire_Model_Codegen_MatlabModel.hpp>
#include <google/protobuf/message.h>
#include <memory>
#include <mutex>
using namespace core;

void StateEstimator::handle_recv_process(std::shared_ptr<google::protobuf::Message> message)
{
    if (message->GetTypeName() == "hytech_msgs.VNData")
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
    else {
        _recv_low_level_state(message);
    }
}

void StateEstimator::_recv_low_level_state(std::shared_ptr<google::protobuf::Message> message)
{
    if (message->GetTypeName() == "hytech.rear_suspension") {
        auto in_msg = std::static_pointer_cast<hytech::rear_suspension>(message);        
        {
            std::unique_lock lk(_state_mutex);
            _timestamp_array[0] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
            _raw_input_data.raw_load_cell_values.RL = in_msg->rl_load_cell();
            _raw_input_data.raw_load_cell_values.RR = in_msg->rr_load_cell();
            _raw_input_data.raw_shock_pot_values.RL = in_msg->rl_shock_pot();
            _raw_input_data.raw_shock_pot_values.RR = in_msg->rr_shock_pot();
        }
    } else if(message->GetTypeName() == "hytech.front_suspension")
    {
        auto in_msg = std::static_pointer_cast<hytech::front_suspension>(message);
        {
            std::unique_lock lk(_state_mutex);
            _timestamp_array[1] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
            _raw_input_data.raw_load_cell_values.FL = in_msg->fl_load_cell();
            _raw_input_data.raw_load_cell_values.FR = in_msg->fr_load_cell();
            _raw_input_data.raw_shock_pot_values.FL = in_msg->fl_shock_pot();
            _raw_input_data.raw_shock_pot_values.FR = in_msg->fr_shock_pot();
        }
    } else if(message->GetTypeName() == "hytech.pedals_system_data")
    {
        auto in_msg = std::static_pointer_cast<hytech::pedals_system_data>(message);
        core::DriverInput input = {(in_msg->accel_pedal()), (in_msg->brake_pedal())};
        {
            std::unique_lock lk(_state_mutex);
            _timestamp_array[2] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
            _vehicle_state.input = input;
        }
    } else if(message->GetTypeName() == "hytech.steering_data")
    {
        auto in_msg = std::static_pointer_cast<hytech::steering_data>(message);
        {
            std::unique_lock lk(_state_mutex);
            _timestamp_array[3] = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
            _raw_input_data.raw_steering_analog = in_msg->steering_analog_raw();
            _raw_input_data.raw_steering_digital = in_msg->steering_digital_raw();
        }
    } else {
        _recv_inverter_states(message);
    }
}

// INV1_STATUS INV1_TEMPS INV1_DYNAMICS INV1_POWER INV1_FEEDBACK
void StateEstimator::_recv_inverter_states(std::shared_ptr<google::protobuf::Message> msg)
{
    auto name = msg->GetTypeName();
    if( name == "hytech.inv1_status" )
    {

    } else if(name == "hytech.inv2_status")
    {

    } else if(name == "hytech.inv1_dynamics")
    {
        _handle_set_inverter_dynamics<0, hytech::inv1_dynamics>(msg);
    } else if(name == "hytech.inv2_dynamics")
    {
        _handle_set_inverter_dynamics<1, hytech::inv2_dynamics>(msg);
    } else if(name == "hytech.inv3_dynamics")
    {
        _handle_set_inverter_dynamics<2, hytech::inv3_dynamics>(msg);
    } else if(name == "hytech.inv4_dynamics")
    {
        _handle_set_inverter_dynamics<3, hytech::inv4_dynamics>(msg);
    }
}

template <size_t ind, typename inverter_dynamics_msg>
void StateEstimator::_handle_set_inverter_dynamics(std::shared_ptr<google::protobuf::Message> msg) {
    auto in_msg = std::static_pointer_cast<inverter_dynamics_msg>(msg);
    {
        std::unique_lock lk(_state_mutex);
        _raw_input_data.raw_inverter_torques.set_from_index<ind>(in_msg->actual_torque_nm());
        _raw_input_data.raw_inverter_power.set_from_index<ind>(in_msg->actual_power_w());
        _vehicle_state.current_rpms.set_from_index<ind>(in_msg->actual_speed_rpm());
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

Tire_Model_Codegen::ExtY_Tire_Model_Codegen_T StateEstimator::_eval_estimator(core::VehicleState vehicle_state, core::RawInputData raw_input_data)
{
    // Convert Vehicle and Raw Data to Tire Model Input
    bool sign = (vehicle_state.input.requested_accel - vehicle_state.input.requested_brake) >= 0;
    estimation::Tire_Model_Codegen_MatlabModel::inputs model_inputs = {
        vehicle_state.prev_controller_output.torque_lim_nm.FL * (sign ? 1.0f : -1.0f),
        vehicle_state.prev_controller_output.torque_lim_nm.FR * (sign ? 1.0f : -1.0f),
        vehicle_state.prev_controller_output.torque_lim_nm.RL * (sign ? 1.0f : -1.0f),
        vehicle_state.prev_controller_output.torque_lim_nm.RR * (sign ? 1.0f : -1.0f),
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        raw_input_data.raw_load_cell_values.FL,
        raw_input_data.raw_load_cell_values.FR,
        raw_input_data.raw_load_cell_values.RL,
        raw_input_data.raw_load_cell_values.RR,
        vehicle_state.current_rpms.FL,
        vehicle_state.current_rpms.RL,
        vehicle_state.current_rpms.FR,
        vehicle_state.current_rpms.RR,
        vehicle_state.steering_angle_deg,
        vehicle_state.current_body_vel_ms.x,
        vehicle_state.current_angular_rate_rads.z,
        vehicle_state.current_body_vel_ms.y
    };

    
    auto res = _matlab_estimator.evaluate_estimator(model_inputs);

    return res;
}

std::shared_ptr<hytech_msgs::VehicleData> StateEstimator::_set_tire_dynamics(std::shared_ptr<hytech_msgs::VehicleData> msg_out, Tire_Model_Codegen::ExtY_Tire_Model_Codegen_T res)
{
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

    return msg_out;
}

std::shared_ptr<hytech_msgs::VehicleData> StateEstimator::_set_tv_status(std::shared_ptr<hytech_msgs::VehicleData> msg_out, Tire_Model_Codegen::ExtY_Tire_Model_Codegen_T res)
{
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
    return msg_out;

}

std::shared_ptr<hytech_msgs::VehicleData> StateEstimator::_set_ins_state_data(core::VehicleState current_state, std::shared_ptr<hytech_msgs::VehicleData> msg_out)
{
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
    return msg_out;
}

std::pair<core::VehicleState, bool> StateEstimator::get_latest_state_and_validity()
{
    auto state_is_valid = _validate_stamps(_timestamp_array);
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

    auto res = _eval_estimator(current_state, current_raw_data);
    
    auto matlab_math_end = std::chrono::high_resolution_clock::now();

    auto state_mutex_2_start = std::chrono::high_resolution_clock::now();
    {
        std::unique_lock lk(_state_mutex);
        _vehicle_state.matlab_math_temp_out = {static_cast<float>(res.torq_req_FL),static_cast<float>(res.torq_req_FR), static_cast<float>(res.torq_req_RL),static_cast<float>(res.torq_req_RR)};
    }

    auto state_mutex_2_end = std::chrono::high_resolution_clock::now();

    msg_out = _set_tire_dynamics(msg_out, res);
    msg_out = _set_tire_dynamics(msg_out, res);

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    msg_out->set_is_ready_to_drive(true);

    hytech_msgs::SpeedControlIn *current_inputs = msg_out->mutable_current_inputs();
    current_inputs->set_accel_percent(current_state.input.requested_accel);
    current_inputs->set_brake_percent(current_state.input.requested_brake);

    msg_out = _set_ins_state_data(current_state, msg_out);

    hytech_msgs::veh_vec_float *curr_rpms = msg_out->mutable_current_rpms();
    curr_rpms->set_fl(current_state.current_rpms.FL);
    curr_rpms->set_fr(current_state.current_rpms.FR);
    curr_rpms->set_rl(current_state.current_rpms.RL);
    curr_rpms->set_rr(current_state.current_rpms.RR);

    msg_out->set_state_is_valid(state_is_valid);
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

    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(state_estim_end - state_estim_start);
    
    constexpr bool debug = false;
    if ( (elapsed > std::chrono::microseconds(6000)) && debug ) // 6ms
    {
        std::cout << "WARNING: timing" << std::endl;
        std::cout << "total: " << (static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count())) << " us\n";
        std::cout << "state mutex: " << (static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(state_mutex_end - state_mutex_start).count())) << " us\n";
        std::cout << "state mutex 2: " << (static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(state_mutex_2_end - state_mutex_2_start).count())) << " us\n";
        std::cout << "matlab math: " << (static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(matlab_math_end - matlab_math_start).count())) << " us\n";
        std::cout << "log time: " << (static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(log_end - log_start).count())) << " us\n";
    }

    return {current_state, state_is_valid};
}