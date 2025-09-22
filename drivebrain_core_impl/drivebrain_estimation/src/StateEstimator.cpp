#include <StateEstimator.hpp>

#include <VehicleDataTypes.hpp>

#include <algorithm>
#include <chrono>

#include <google/protobuf/message.h>
#include <memory>
#include <mutex>

#include "HTMath.hpp"
#include "hytech.pb.h"      // from HT_CAN
#include "hytech_msgs.pb.h" // from HT_proto
#include <base_msgs.pb.h>   // from HT_proto
#include <spdlog/spdlog.h>

#include "JSONUtils.hpp"

using namespace core;

StateEstimator::~StateEstimator()
{
    spdlog::info("destructed StateEstimator");
}

// INV1_STATUS INV1_TEMPS INV1_DYNAMICS INV1_POWER INV1_FEEDBACK
void StateEstimator::_recv_inverter_states(std::shared_ptr<google::protobuf::Message> msg) {
    auto name = msg->GetTypeName();
    if (name == "hytech.inv1_dynamics") {
        _handle_set_inverter_dynamics<0, hytech::inv1_dynamics>(msg);
    } else if (name == "hytech.inv2_dynamics") {
        _handle_set_inverter_dynamics<1, hytech::inv2_dynamics>(msg);
    } else if (name == "hytech.inv3_dynamics") {
        _handle_set_inverter_dynamics<2, hytech::inv3_dynamics>(msg);
    } else if (name == "hytech.inv4_dynamics") {
        _handle_set_inverter_dynamics<3, hytech::inv4_dynamics>(msg);
    } else if (name == "hytech.inv1_temps") {
        _handle_set_inverter_temps<0, hytech::inv1_temps>(msg);
    } else if (name == "hytech.inv2_temps") {
        _handle_set_inverter_temps<1, hytech::inv2_temps>(msg);
    } else if (name == "hytech.inv3_temps") {
        _handle_set_inverter_temps<2, hytech::inv3_temps>(msg);
    } else if (name == "hytech.inv4_temps") {
        _handle_set_inverter_temps<3, hytech::inv4_temps>(msg);
    } else if (name == "hytech.inv1_overload") {
        auto in_msg = std::static_pointer_cast<hytech::inv1_overload>(msg);
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.motor_overload_percentages.FL = in_msg->motor_overload_percentage();
        }
    } else if (name == "hytech.inv2_overload") {
        auto in_msg = std::static_pointer_cast<hytech::inv2_overload>(msg);
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.motor_overload_percentages.FR = in_msg->motor_overload_percentage();
        }
    } else if (name == "hytech.inv3_overload") {
        auto in_msg = std::static_pointer_cast<hytech::inv3_overload>(msg);
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.motor_overload_percentages.RL = in_msg->motor_overload_percentage();
        }
    } else if (name == "hytech.inv4_overload") {
        auto in_msg = std::static_pointer_cast<hytech::inv4_overload>(msg);
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.motor_overload_percentages.RR = in_msg->motor_overload_percentage();
        }
    } else if(name == "hytech.inv1_status") {
        
        auto in_msg = std::static_pointer_cast<hytech::inv1_status>(msg);
        auto voltage = in_msg->dc_bus_voltage();
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.acc_data.pack_voltage = static_cast<float>(voltage);
        }
    }
}

void StateEstimator::handle_recv_process(std::shared_ptr<google::protobuf::Message> message) {
    if (message->GetTypeName() == "hytech_msgs.VNData") {
        auto in_msg = std::static_pointer_cast<hytech_msgs::VNData>(message);
        xyz_vec<float> body_vel_ms = {(in_msg->vn_vel_m_s().x()), (in_msg->vn_vel_m_s().y()),
                                      (in_msg->vn_vel_m_s().z())};

        xyz_vec<float> body_accel_mss = {(in_msg->vn_linear_accel_m_ss().x()),
                                         (in_msg->vn_linear_accel_m_ss().y()),
                                         (in_msg->vn_linear_accel_m_ss().z())};

        xyz_vec<float> angular_rate_rads = {(in_msg->vn_angular_rate_rad_s().x()),
                                            (in_msg->vn_angular_rate_rad_s().y()),
                                            (in_msg->vn_angular_rate_rad_s().z())};

        ypr_vec<float> ypr_rad = {(in_msg->vn_ypr_rad().yaw()), (in_msg->vn_ypr_rad().pitch()),
                                  (in_msg->vn_ypr_rad().roll())};

        auto ins_mode_int = in_msg->status().ins_mode_int();
        auto vel_u = in_msg->status().ins_vel_u();

        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.current_body_vel_ms = body_vel_ms;
            _vehicle_state.current_body_accel_mss = body_accel_mss;
            _vehicle_state.current_angular_rate_rads = angular_rate_rads;
            _vehicle_state.current_ypr_rad = ypr_rad;
            _vehicle_state.ins_status.status_mode = ins_mode_int;
            _vehicle_state.ins_status.vel_uncertainty = vel_u;
        }
    } else if (message->GetTypeName() == "hytech_msgs.VCRData_s") {
        auto in_msg = std::static_pointer_cast<hytech_msgs::VCRData_s>(message);
        bool is_rtd = (in_msg->status().vehicle_state() == hytech_msgs::VehicleState_e::READY_TO_DRIVE);
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.is_ready_to_drive = is_rtd;
        }
    } else if (message->GetTypeName() == "hytech_msgs.ACUAllData") {
        auto in_msg = std::static_pointer_cast<hytech_msgs::ACUAllData>(message);
        // std::cout << "min cell: " << in_msg->core_data().min_cell_voltage() << std::endl;
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.acc_data.min_cell_voltage = in_msg->core_data().min_cell_voltage();
        }
    }
    else {
        _recv_low_level_state(message);
    }
}

void StateEstimator::_recv_low_level_state(std::shared_ptr<google::protobuf::Message> message) {
    if (message->GetTypeName() == "hytech.rear_suspension") {
        auto in_msg = std::static_pointer_cast<hytech::rear_suspension>(message);
        {
            std::unique_lock lk(_state_mutex);
            _timestamp_array[0] = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch());
            _raw_input_data.raw_load_cell_values.RL = in_msg->rl_load_cell();
            _raw_input_data.raw_load_cell_values.RR = in_msg->rr_load_cell();
            _raw_input_data.raw_shock_pot_values.RL = in_msg->rl_shock_pot();
            _raw_input_data.raw_shock_pot_values.RR = in_msg->rr_shock_pot();
        }
    } else if (message->GetTypeName() == "hytech.front_suspension") {
        auto in_msg = std::static_pointer_cast<hytech::front_suspension>(message);
        {
            std::unique_lock lk(_state_mutex);
            _timestamp_array[1] = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch());
            _raw_input_data.raw_load_cell_values.FL = in_msg->fl_load_cell();
            _raw_input_data.raw_load_cell_values.FR = in_msg->fr_load_cell();
            _raw_input_data.raw_shock_pot_values.FL = in_msg->fl_shock_pot();
            _raw_input_data.raw_shock_pot_values.FR = in_msg->fr_shock_pot();
        }
    } else if (message->GetTypeName() == "hytech.pedals_system_data") {
        auto in_msg = std::static_pointer_cast<hytech::pedals_system_data>(message);
        core::DriverInput input = {(in_msg->accel_pedal()), (in_msg->brake_pedal())};
        {
            std::unique_lock lk(_state_mutex);
            _timestamp_array[2] = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch());
            _vehicle_state.input = input;
        }
    } else if (message->GetTypeName() == "hytech.steering_data") {
        auto in_msg = std::static_pointer_cast<hytech::steering_data>(message);
        {
            std::unique_lock lk(_state_mutex);
            _timestamp_array[3] = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch());
            _raw_input_data.raw_steering_analog = in_msg->steering_analog_raw();
            _raw_input_data.raw_steering_digital = in_msg->steering_digital_raw();
        }
    } else if (message->GetTypeName() == "hytech.em_measurement") {
        auto in_msg = std::static_pointer_cast<hytech::em_measurement>(message);
        float em_voltage = in_msg->em_voltage();
        float em_current = in_msg->em_current();
        {
            std::unique_lock lk(_state_mutex);
            _vehicle_state.old_energy_meter_kw = (-1.0f * em_voltage * em_current / 1000.0f); // on ht09 the energy meter is backwards as of right now
        }
    } 
    else {
        _recv_inverter_states(message);
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
        _vehicle_state.current_torques_nm = _raw_input_data.raw_inverter_torques;
    }
}

template <size_t ind, typename inverter_temps_msg>
void StateEstimator::_handle_set_inverter_temps(std::shared_ptr<google::protobuf::Message> msg) {
    
    
    auto in_msg = std::static_pointer_cast<inverter_temps_msg>(msg);
    core::DrivetrainData dt_data = {};
    dt_data.inverter_igbt_temps_c.set_from_index<ind>(in_msg->igbt_temp());
    dt_data.inverter_temps_c.set_from_index<ind>(in_msg->inverter_temp());
    dt_data.inverter_motor_temps_c.set_from_index<ind>(in_msg->motor_temp());
    {
        std::unique_lock lk(_state_mutex);
        _vehicle_state.dt_data = dt_data;
    }
}

// TODO parameterize the timeout threshold
template <size_t arr_len>
bool StateEstimator::_validate_stamps(
    const std::array<std::chrono::microseconds, arr_len> &timestamp_arr) {
    std::array<std::chrono::microseconds, arr_len> timestamp_array_to_sort;
    {
        std::unique_lock lk(_state_mutex);
        timestamp_array_to_sort = timestamp_arr;
    }

    auto debug_copy = timestamp_array_to_sort;
    const std::chrono::microseconds threshold(500000); // 500 milliseconds in microseconds

    // Sort the array
    std::sort(timestamp_array_to_sort.begin(), timestamp_array_to_sort.end());

    // Check if the range between the smallest and largest timestamps is within the threshold
    auto min_stamp = timestamp_array_to_sort.front();
    auto max_stamp = timestamp_array_to_sort.back();

    bool within_threshold = (max_stamp - min_stamp) <= threshold;

    auto curr_time = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch());

    constexpr std::chrono::seconds debug_print_period(1);
    
    bool all_members_received = min_stamp.count() > 0; // count here is the count in microseconds
    bool last_update_recent_enough =
        (std::chrono::duration_cast<std::chrono::microseconds>(curr_time - max_stamp)) < threshold;

    if(std::chrono::duration_cast<std::chrono::microseconds>(curr_time - max_stamp) < _debug_maxtime_diff || (_debug_maxtime_diff.count() < 0))
    {
        _debug_maxtime_diff = std::chrono::duration_cast<std::chrono::microseconds>(curr_time - max_stamp);
    }
    if(std::chrono::duration_cast<std::chrono::microseconds>(max_stamp - min_stamp) < _debug_maxjitter_diff || (_debug_maxjitter_diff.count() < 0))
    {
        _debug_maxjitter_diff = std::chrono::duration_cast<std::chrono::microseconds>(max_stamp - min_stamp);
    }

    if((curr_time - _last_debug_veh_state_print) > debug_print_period)
    {
        _last_debug_veh_state_print = std::chrono::duration_cast<std::chrono::seconds>(curr_time);
        spdlog::info("_debug_maxtime_diff: {}", _debug_maxtime_diff.count());
        spdlog::info("_debug_maxjitter_diff: {}", _debug_maxjitter_diff.count());
    }

    if(!within_threshold)
    {
        spdlog::warn("data not recvd within time window theshold: {}", (max_stamp - min_stamp).count());
    } else if(!all_members_received)
    {
        spdlog::warn("not all data recvd yet for state to be valid");
    } else if(!last_update_recent_enough)
    {
        spdlog::warn("max timestamp message has been been recvd in time window {}", (curr_time - max_stamp).count());
    }
    

    if(!(within_threshold && all_members_received && last_update_recent_enough))
    {
        spdlog::info("vcr suspension val: {}", debug_copy[0].count());
        spdlog::info("vcf suspension val: {}", debug_copy[1].count());
        spdlog::info("vcf pedals val: {}", debug_copy[2].count());
        spdlog::info("steering data val: {}", debug_copy[3].count());
    }
    return within_threshold && all_members_received && last_update_recent_enough;
}

core::VehicleState
StateEstimator::append_state_variables_from_raw_inputs(core::VehicleState vs,
                                                       core::RawInputData raw_data) {
    auto vehicle_state = vs;
    vehicle_state.suspension_potentiometers_mm.FL = raw_data.raw_shock_pot_values.FL;
    vehicle_state.suspension_potentiometers_mm.FR = raw_data.raw_shock_pot_values.FR;
    vehicle_state.suspension_potentiometers_mm.RL = raw_data.raw_shock_pot_values.RL;
    vehicle_state.suspension_potentiometers_mm.RR = raw_data.raw_shock_pot_values.RR;
    
    vehicle_state.loadcells.FL = raw_data.raw_load_cell_values.FL;
    vehicle_state.loadcells.FR = raw_data.raw_load_cell_values.FR;
    vehicle_state.loadcells.RL = raw_data.raw_load_cell_values.RL;
    vehicle_state.loadcells.RR = raw_data.raw_load_cell_values.RR;

    vehicle_state.steering_angle_deg = raw_data.raw_steering_analog;
    return vehicle_state;
}

void StateEstimator::set_previous_control_output(core::ControllerOutput prev_control_output) {
    std::unique_lock lk(_state_mutex);
    _vehicle_state.prev_controller_output = prev_control_output;
}

std::shared_ptr<hytech_msgs::VehicleData>
StateEstimator::_set_ins_state_data(core::VehicleState current_state,
                                    std::shared_ptr<hytech_msgs::VehicleData> msg_out) {
    hytech_msgs::xyz_vector *current_body_vel_ms = msg_out->mutable_current_body_vel_ms();
    current_body_vel_ms->set_x(current_state.current_body_vel_ms.x);
    current_body_vel_ms->set_y(current_state.current_body_vel_ms.y);
    current_body_vel_ms->set_z(current_state.current_body_vel_ms.z);

    hytech_msgs::xyz_vector *current_body_accel_mss = msg_out->mutable_current_body_accel_mss();
    current_body_accel_mss->set_x(current_state.current_body_accel_mss.x);
    current_body_accel_mss->set_y(current_state.current_body_accel_mss.y);
    current_body_accel_mss->set_z(current_state.current_body_accel_mss.z);

    hytech_msgs::xyz_vector *current_angular_rate_rads =
        msg_out->mutable_current_angular_rate_rads();
    current_angular_rate_rads->set_x(current_state.current_angular_rate_rads.x);
    current_angular_rate_rads->set_y(current_state.current_angular_rate_rads.y);
    current_angular_rate_rads->set_z(current_state.current_angular_rate_rads.z);

    hytech_msgs::ypr_vector *current_ypr_rad = msg_out->mutable_current_ypr_rad();
    current_ypr_rad->set_yaw(current_state.current_ypr_rad.yaw);
    current_ypr_rad->set_pitch(current_state.current_ypr_rad.pitch);
    current_ypr_rad->set_roll(current_state.current_ypr_rad.roll);
    return msg_out;
}

std::pair<core::VehicleState, bool> StateEstimator::get_latest_state_and_validity() {
    auto state_is_valid = _validate_stamps(_timestamp_array);
    auto state_estim_start = std::chrono::high_resolution_clock::now();
    core::VehicleState current_state = {};
    core::RawInputData current_raw_data = {};

    auto state_mutex_start = std::chrono::high_resolution_clock::now();
    {
        std::unique_lock lk(_state_mutex);
        _vehicle_state = append_state_variables_from_raw_inputs(_vehicle_state, _raw_input_data);
        current_state = _vehicle_state;
        current_raw_data = _raw_input_data;
    }

    auto state_mutex_end = std::chrono::high_resolution_clock::now();

    // Create the proto message to send
    std::shared_ptr<hytech_msgs::VehicleData> msg_out =
        std::make_shared<hytech_msgs::VehicleData>();

    msg_out->set_is_ready_to_drive(current_state.is_ready_to_drive);
    
    hytech_msgs::SpeedControlIn *current_inputs = msg_out->mutable_current_inputs();
    current_inputs->set_accel_percent(current_state.input.requested_accel);
    current_inputs->set_brake_percent(current_state.input.requested_brake);

    msg_out = _set_ins_state_data(current_state, msg_out);
    msg_out = _set_computed_states(current_state, msg_out);
    hytech_msgs::veh_vec_float *curr_rpms = msg_out->mutable_current_rpms();
    curr_rpms->set_fl(current_state.current_rpms.FL);
    curr_rpms->set_fr(current_state.current_rpms.FR);
    curr_rpms->set_rl(current_state.current_rpms.RL);
    curr_rpms->set_rr(current_state.current_rpms.RR);

    hytech_msgs::veh_vec_float *curr_torqs = msg_out->mutable_current_torques_nm();
    curr_torqs->set_fl(current_state.current_torques_nm.FL);
    curr_torqs->set_fr(current_state.current_torques_nm.FR);
    curr_torqs->set_rl(current_state.current_torques_nm.RL);
    curr_torqs->set_rr(current_state.current_torques_nm.RR);


    msg_out->set_state_is_valid(state_is_valid);
    msg_out->set_steering_angle_deg(current_state.steering_angle_deg);
    auto prev_driver_torque_req = msg_out->mutable_driver_torque();

    if (const core::TorqueControlOut *torqueControl =
            std::get_if<core::TorqueControlOut>(&current_state.prev_controller_output.out)) {
        // TODO: REPLACE THIS CODE BLOCK TO PUSH A TORQUE CONTROLLER STRUCT IN THE PROTOBUF MESSAGE,
        // NOT A SPEED CONTROLLER STRUCT RIGHT NOW IT TREATS DESIRED TORQUE AS A TORQUE LIMIT
        msg_out->set_is_using_torque_controller(
            true); // add to message that we are using desired torque commands, not torque limits
        _set_float_veh_vec_message_member(torqueControl->desired_torques_nm,
                                          prev_driver_torque_req);

    } else if (const core::SpeedControlOut *speedControl = std::get_if<core::SpeedControlOut>(
                   &current_state.prev_controller_output
                        .out)) { // assuming no monostate possible here
        msg_out->set_is_using_torque_controller(false);
        _set_float_veh_vec_message_member(speedControl->torque_lim_nm, prev_driver_torque_req);
    }

    msg_out->set_old_energy_meter_kw(current_state.old_energy_meter_kw);

    auto log_start = std::chrono::high_resolution_clock::now();
    
    this->log(msg_out);

    auto log_end = std::chrono::high_resolution_clock::now();

    auto state_estim_end = std::chrono::high_resolution_clock::now();

    auto elapsed =
        std::chrono::duration_cast<std::chrono::microseconds>(state_estim_end - state_estim_start);

    constexpr bool debug = true;
    if ((elapsed > std::chrono::microseconds(4000)) && debug) // 4ms
    {
        auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
        auto state_mutex_us = std::chrono::duration_cast<std::chrono::microseconds>(state_mutex_end - state_mutex_start).count();
        auto log_time_us = std::chrono::duration_cast<std::chrono::microseconds>(log_end - log_start).count();

        spdlog::warn("WARNING: timing");
        spdlog::warn("total: {} us", total_us);
        spdlog::warn("state mutex: {} us", state_mutex_us);
        spdlog::warn("log time: {} us", log_time_us);
    } 

    return {current_state, state_is_valid};
}

bool StateEstimator::init() {
    // SUS POT PARAMS
    LOAD_PARAM_OR_FAIL(fl_sus_pot_min, float, _config);
    LOAD_PARAM_OR_FAIL(fl_sus_pot_min_mm, float, _config);
    LOAD_PARAM_OR_FAIL(fl_sus_pot_max, float, _config);
    LOAD_PARAM_OR_FAIL(fl_sus_pot_max_mm, float, _config);
    LOAD_PARAM_OR_FAIL(fr_sus_pot_min, float, _config);
    LOAD_PARAM_OR_FAIL(fr_sus_pot_min_mm, float, _config);
    LOAD_PARAM_OR_FAIL(fr_sus_pot_max, float, _config);
    LOAD_PARAM_OR_FAIL(fr_sus_pot_max_mm, float, _config);
    LOAD_PARAM_OR_FAIL(rl_sus_pot_min, float, _config);
    LOAD_PARAM_OR_FAIL(rl_sus_pot_min_mm, float, _config);
    LOAD_PARAM_OR_FAIL(rl_sus_pot_max, float, _config);
    LOAD_PARAM_OR_FAIL(rl_sus_pot_max_mm, float, _config);
    LOAD_PARAM_OR_FAIL(rr_sus_pot_min, float, _config);
    LOAD_PARAM_OR_FAIL(rr_sus_pot_min_mm, float, _config);
    LOAD_PARAM_OR_FAIL(rr_sus_pot_max, float, _config);
    LOAD_PARAM_OR_FAIL(rr_sus_pot_max_mm, float, _config);
    
    // LOAD CELL CONFIGS
    LOAD_PARAM_OR_FAIL(fl_load_cell_offset, float, _config);
    LOAD_PARAM_OR_FAIL(fl_load_cell_scale, float, _config);
    LOAD_PARAM_OR_FAIL(fr_load_cell_offset, float, _config);
    LOAD_PARAM_OR_FAIL(fr_load_cell_scale, float, _config);
    LOAD_PARAM_OR_FAIL(rl_load_cell_offset, float, _config);
    LOAD_PARAM_OR_FAIL(rl_load_cell_scale, float, _config);
    LOAD_PARAM_OR_FAIL(rr_load_cell_offset, float, _config);
    LOAD_PARAM_OR_FAIL(rr_load_cell_scale, float, _config);
    set_configured();
    return true;
}

std::shared_ptr<hytech_msgs::VehicleData>
StateEstimator::_set_computed_states(core::VehicleState current_state,
                                     std::shared_ptr<hytech_msgs::VehicleData> msg_out) {
    auto suspension_potentiometers_mm = msg_out->mutable_suspension_potentiometers_mm();
    _set_float_veh_vec_message_member(current_state.suspension_potentiometers_mm,
                                      suspension_potentiometers_mm);
    return msg_out;
}

void StateEstimator::_set_float_veh_vec_message_member(veh_vec<float> from,
                                                       hytech_msgs::veh_vec_float *to_set) {
    to_set->set_fl(from.FL);
    to_set->set_fr(from.FR);
    to_set->set_rl(from.RL);
    to_set->set_rr(from.RR);
}