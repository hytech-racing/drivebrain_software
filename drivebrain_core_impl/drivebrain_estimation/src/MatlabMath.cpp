#include <MatlabMath.hpp>

namespace estimation
{
    void MatlabMath::_handle_param_updates(const std::unordered_map<std::string, core::common::Configurable::ParamTypes> &new_param_map)
    {
        if (auto pval = std::get_if<float>(&new_param_map.at("lmux_fl")))
        {
            std::unique_lock lk(_config_mutex);
            _config.lmux_fl = *pval;
            std::cout << "setting new lmux_fl " << _config.lmux_fl << std::endl;
        }
        if (auto pval = std::get_if<float>(&new_param_map.at("lmuy_fl")))
        {
            std::unique_lock lk(_config_mutex);
            _config.lmuy_fl = *pval;
            std::cout << "setting new lmuy_fl " << _config.lmuy_fl << std::endl;
        }
        if (auto pval = std::get_if<float>(&new_param_map.at("lmux_fr")))
        {
            std::unique_lock lk(_config_mutex);
            _config.lmux_fr = *pval;
            std::cout << "setting new lmux_fr " << _config.lmux_fr << std::endl;
        }
        if (auto pval = std::get_if<float>(&new_param_map.at("lmuy_fr")))
        {
            std::unique_lock lk(_config_mutex);
            _config.lmuy_fr = *pval;
            std::cout << "setting new lmuy_fr " << _config.lmuy_fr << std::endl;
        }
        if (auto pval = std::get_if<float>(&new_param_map.at("lmux_rl")))
        {
            std::unique_lock lk(_config_mutex);
            _config.lmux_rl = *pval;
            std::cout << "setting new lmux_rl " << _config.lmux_rl << std::endl;
        }
        if (auto pval = std::get_if<float>(&new_param_map.at("lmuy_rl")))
        {
            std::unique_lock lk(_config_mutex);
            _config.lmuy_rl = *pval;
            std::cout << "setting new lmuy_rl " << _config.lmuy_rl << std::endl;
        }
        if (auto pval = std::get_if<float>(&new_param_map.at("lmux_rr")))
        {
            std::unique_lock lk(_config_mutex);
            _config.lmux_rr = *pval;
            std::cout << "setting new lmux_rr " << _config.lmux_rr << std::endl;
        }
        if (auto pval = std::get_if<float>(&new_param_map.at("lmuy_rr")))
        {
            std::unique_lock lk(_config_mutex);
            _config.lmuy_rr = *pval;
            std::cout << "setting new lmuy_rr " << _config.lmuy_rr << std::endl;
        }
        if (auto pval = std::get_if<bool>(&new_param_map.at("use_fake_data")))
        {
            std::unique_lock lk(_config_mutex);
            _config.use_fake_data = *pval;
            std::cout << "setting new use_fake_data " << _config.use_fake_data << std::endl;
        }
        if (auto pval = std::get_if<float>(&new_param_map.at("Fake_Vx")))
        {
            std::unique_lock lk(_config_mutex);
            _config.Fake_Vx = *pval;
            std::cout << "setting new Fake_Vx " << _config.Fake_Vx << std::endl;
        }
        if (auto pval = std::get_if<float>(&new_param_map.at("DriveBiasFront")))
        {
            std::unique_lock lk(_config_mutex);
            _config.DriveBiasFront = *pval;
            std::cout << "setting new DriveBiasFront " << _config.DriveBiasFront << std::endl;
        }
        if (auto pval = std::get_if<float>(&new_param_map.at("BrakeBiasFront")))
        {
            std::unique_lock lk(_config_mutex);
            _config.BrakeBiasFront = *pval;
            std::cout << "setting new BrakeBiasFront " << _config.BrakeBiasFront << std::endl;
        }
    }

    MatlabMath::MatlabMath(core::Logger &logger, core::JsonFileHandler &json_file_handler, bool &construction_failed)
        : Configurable(logger, json_file_handler, "MatlabMath")
    {
        construction_failed = !init();
        _inputs = {};
    }

    bool MatlabMath::init()
    {
        auto lmux_fl = get_live_parameter<float>("lmux_fl");
        auto lmuy_fl = get_live_parameter<float>("lmuy_fl");
        auto lmux_fr = get_live_parameter<float>("lmux_fr");
        auto lmuy_fr = get_live_parameter<float>("lmuy_fr");
        auto lmux_rl = get_live_parameter<float>("lmux_rl");
        auto lmuy_rl = get_live_parameter<float>("lmuy_rl");
        auto lmux_rr = get_live_parameter<float>("lmux_rr");
        auto lmuy_rr = get_live_parameter<float>("lmuy_rr");

        auto use_fake_data = get_live_parameter<bool>("use_fake_data");
        auto Fake_Vx = get_live_parameter<float>("Fake_Vx");
        auto DriveBiasFront = get_live_parameter<float>("DriveBiasFront");
        auto BrakeBiasFront = get_live_parameter<float>("BrakeBiasFront");

        auto x1_fl = get_parameter_value<float>("x1_fl");
        auto x2_fl = get_parameter_value<float>("x2_fl");
        auto x3_fl = get_parameter_value<float>("x3_fl");

        auto y1_fl = get_parameter_value<float>("y1_fl");
        auto y2_fl = get_parameter_value<float>("y2_fl");
        auto y3_fl = get_parameter_value<float>("y3_fl");

        auto x1_fr = get_parameter_value<float>("x1_fr");
        auto x2_fr = get_parameter_value<float>("x2_fr");
        auto x3_fr = get_parameter_value<float>("x3_fr");

        auto y1_fr = get_parameter_value<float>("y1_fr");
        auto y2_fr = get_parameter_value<float>("y2_fr");
        auto y3_fr = get_parameter_value<float>("y3_fr");

        auto x1_rl = get_parameter_value<float>("x1_rl");
        auto x2_rl = get_parameter_value<float>("x2_rl");
        auto x3_rl = get_parameter_value<float>("x3_rl");

        auto y1_rl = get_parameter_value<float>("y1_rl");
        auto y2_rl = get_parameter_value<float>("y2_rl");
        auto y3_rl = get_parameter_value<float>("y3_rl");

        auto x1_rr = get_parameter_value<float>("x1_rr");
        auto x2_rr = get_parameter_value<float>("x2_rr");
        auto x3_rr = get_parameter_value<float>("x3_rr");

        auto y1_rr = get_parameter_value<float>("y1_rr");
        auto y2_rr = get_parameter_value<float>("y2_rr");
        auto y3_rr = get_parameter_value<float>("y3_rr");

        

        if (!(lmux_fl && lmuy_fl && lmux_fr && lmuy_fr && lmux_rl && lmuy_rl && lmux_rr && lmuy_rr &&
              x1_fl && x2_fl && x3_fl && y1_fl && y2_fl && y3_fl &&
              x1_fr && x2_fr && x3_fr && y1_fr && y2_fr && y3_fr &&
              x1_rl && x2_rl && x3_rl && y1_rl && y2_rl && y3_rl &&
              x1_rr && x2_rr && x3_rr && y1_rr && y2_rr && y3_rr))
        {
            return false;
        }
        {
            std::unique_lock lk(_config_mutex);
            _config = {
                *use_fake_data, *Fake_Vx, *DriveBiasFront, *BrakeBiasFront,
                *lmux_fl, *lmuy_fl, *lmux_fr, *lmuy_fr, *lmux_rl, *lmuy_rl, *lmux_rr, *lmuy_rr, // live configs
                *x1_fl, *x2_fl, *x3_fl, *y1_fl, *y2_fl, *y3_fl,                                 // file loaded configs (FL)
                *x1_fr, *x2_fr, *x3_fr, *y1_fr, *y2_fr, *y3_fr,                                 // file loaded configs (FR)
                *x1_rl, *x2_rl, *x3_rl, *y1_rl, *y2_rl, *y3_rl,                                 // file loaded configs (RL)
                *x1_rr, *x2_rr, *x3_rr, *y1_rr, *y2_rr, *y3_rr                                  // file loaded configs (RR)
            };

        }
        param_update_handler_sig.connect(boost::bind(&MatlabMath::_handle_param_updates, this, std::placeholders::_1));

        return true;
    }

    std::pair<core::MatlabMathResult, core::ControllerTorqueOut> MatlabMath::evaluate_estimator(const core::VehicleState &current_state, const core::RawInputData &raw_input)
    {
        config cur_config;
        {
            std::unique_lock lk(_config_mutex);
            cur_config = _config;
        }

        // TODO: erm i forgor
        _inputs.FL_b = 0.0f; // '<Root>/SA FL'
        _inputs.FR_b = 0.0f; // '<Root>/SA FR'
        _inputs.RL_g = 0.0f; // '<Root>/SA RL'
        _inputs.RR_a = 0.0f; // '<Root>/SA RR'

        _inputs.LCFL = raw_input.raw_load_cell_values.FL; // '<Root>/LC FL'
        _inputs.LCFR = raw_input.raw_load_cell_values.FR; // '<Root>/LC FR'
        _inputs.LCRL = raw_input.raw_load_cell_values.RL; // '<Root>/LC RL'
        _inputs.LCRR = raw_input.raw_load_cell_values.RR; // '<Root>/LC RR'
        _inputs.FL = 0;                                   // '<Root>/SL FL'
        _inputs.FR = 0;                                   // '<Root>/SL FR'
        _inputs.RL = 0;                                   // '<Root>/SL RL'
        _inputs.RR = 0;                                   // '<Root>/SL RR'

        // TODO fix, this is assuming that we are in mode 0. need to get actual torque request that isnt supremely dumb
        if ((current_state.input.requested_accel - current_state.input.requested_brake) >= 0)
        {
            _inputs.InitialTorqReqFL = current_state.prev_controller_output.torque_lim_nm.FL; // '<Root>/Initial Torq Req FL'
            _inputs.InitialTorqReqFR = current_state.prev_controller_output.torque_lim_nm.FR; // '<Root>/Initial Torq Req FR'
            _inputs.InitialTorqReqRL = current_state.prev_controller_output.torque_lim_nm.RL; // '<Root>/Initial Torq Req RL'
            _inputs.InitialTorqReqRR = current_state.prev_controller_output.torque_lim_nm.RR; // '<Root>/Initial Torq Req RR'
        }
        else
        {
            _inputs.InitialTorqReqFL = -1.0f * current_state.prev_controller_output.torque_lim_nm.FL; // '<Root>/Initial Torq Req FL'
            _inputs.InitialTorqReqFR = -1.0f * current_state.prev_controller_output.torque_lim_nm.FR; // '<Root>/Initial Torq Req FR'
            _inputs.InitialTorqReqRL = -1.0f * current_state.prev_controller_output.torque_lim_nm.RL; // '<Root>/Initial Torq Req RL'
            _inputs.InitialTorqReqRR = -1.0f * current_state.prev_controller_output.torque_lim_nm.RR; // '<Root>/Initial Torq Req RR'
        }

        _inputs.MotorRPMFL = current_state.current_rpms.FL; // '<Root>/Motor RPM FL'
        _inputs.MotorRPMFR = current_state.current_rpms.FR; // '<Root>/Motor RPM FR'
        _inputs.MotorRPMRL = current_state.current_rpms.RL; // '<Root>/Motor RPM RL'
        _inputs.MotorRPMRR = current_state.current_rpms.RR; // '<Root>/Motor RPM RR'

        _inputs.LMUXFL = cur_config.lmux_fl; // '<Root>/LMUX FL'
        _inputs.LMUXFR = cur_config.lmux_fr; // '<Root>/LMUX FR'
        _inputs.LMUXRL = cur_config.lmux_rl; // '<Root>/LMUX RL'
        _inputs.LMUXRR = cur_config.lmux_rr; // '<Root>/LMUX RR'
        _inputs.LMUYFL = cur_config.lmuy_fl; // '<Root>/LMUY FL'
        _inputs.LMUYFR = cur_config.lmuy_fr; // '<Root>/LMUY FR'
        _inputs.LMUYRL = cur_config.lmuy_rl; // '<Root>/LMUY RL'
        _inputs.LMUYRR = cur_config.lmuy_rr; // '<Root>/LMUY RR'

        _inputs.Interp_x1_FL = cur_config.x1_fl; // '<Root>/Interp_x1_FL'
        _inputs.Interp_x2_FL = cur_config.x2_fl; // '<Root>/Interp_x2_FL'
        _inputs.Interp_x3_FL = cur_config.x3_fl; // '<Root>/Interp_x3_FL'
        _inputs.interp_y1_FL = cur_config.y1_fl; // '<Root>/interp_y1_FL'
        _inputs.interp_y2_FL = cur_config.y2_fl; // '<Root>/interp_y2_FL'
        _inputs.interp_y3_FL = cur_config.y3_fl; // '<Root>/interp_y3_FL'
        _inputs.Interp_x1_FR = cur_config.x1_fr; // '<Root>/Interp_x1_FR'
        _inputs.Interp_x2_FR = cur_config.x2_fr; // '<Root>/Interp_x2_FR'
        _inputs.Interp_x3_FR = cur_config.x3_fr; // '<Root>/Interp_x3_FR'
        _inputs.interp_y1_FR = cur_config.y1_fr; // '<Root>/interp_y1_FR'
        _inputs.interp_y2_FR = cur_config.y2_fr; // '<Root>/interp_y2_FR'
        _inputs.interp_y3_FR = cur_config.y3_fr; // '<Root>/interp_y3_FR'
        _inputs.Interp_x1_RL = cur_config.x1_rl; // '<Root>/Interp_x1_RL'
        _inputs.Interp_x2_RL = cur_config.x2_rl; // '<Root>/Interp_x2_RL'
        _inputs.Interp_x3_RL = cur_config.x3_rl; // '<Root>/Interp_x3_RL'
        _inputs.interp_y1_RL = cur_config.y1_rl; // '<Root>/interp_y1_RL'
        _inputs.interp_y2_RL = cur_config.y2_rl; // '<Root>/interp_y2_RL'
        _inputs.interp_y3_RL = cur_config.y3_rl; // '<Root>/interp_y3_RL'
        _inputs.Interp_x1_RR = cur_config.x1_rr; // '<Root>/Interp_x1_RR'
        _inputs.Interp_x2_RR = cur_config.x2_rr; // '<Root>/Interp_x2_RR'
        _inputs.Interp_x3_RR = cur_config.x3_rr; // '<Root>/Interp_x3_RR'
        _inputs.interp_y1_RR = cur_config.y1_rr; // '<Root>/interp_y1_RR'
        _inputs.interp_y2_RR = cur_config.y2_rr; // '<Root>/interp_y2_RR'
        _inputs.interp_y3_RR = cur_config.y3_rr; // '<Root>/interp_y3_RR'

        _inputs.SteeringWheelAngleDeg = current_state.steering_angle_deg;
        _inputs.Vx_VN = current_state.current_body_vel_ms.x;
        
        _inputs.useFakeData = cur_config.use_fake_data;
        _inputs.Fake_Vx = cur_config.Fake_Vx;
        _inputs.Psi_dot_VNrads = current_state.current_angular_rate_rads.z;
        _inputs.DriveBiasFront = cur_config.DriveBiasFront;
        _inputs.BrakeBiasFront = cur_config.BrakeBiasFront;

        _model.setExternalInputs(&_inputs);
        _model.step();
        Tire_Model_Codegen::ExtY_Tire_Model_Codegen_T outputs = _model.getExternalOutputs();

        core::TireDynamics tire_dynamics_status;
        core::TorqueVectoringStatus torque_vectoring_status;        
        core::ControllerTorqueOut control_res;

        tire_dynamics_status.tire_forces_n.FL.x = outputs.FXFL;
        tire_dynamics_status.tire_forces_n.FR.x = outputs.FXFR;
        tire_dynamics_status.tire_forces_n.RL.x = outputs.FXRL;
        tire_dynamics_status.tire_forces_n.RR.x = outputs.FXRR;

        tire_dynamics_status.tire_forces_n.FL.y = outputs.FYFL;
        tire_dynamics_status.tire_forces_n.FR.y = outputs.FYFR;
        tire_dynamics_status.tire_forces_n.RL.y = outputs.FYRL;
        tire_dynamics_status.tire_forces_n.RR.y = outputs.FYRR;

        tire_dynamics_status.tire_forces_n.FL.z = outputs.FZFL;
        tire_dynamics_status.tire_forces_n.FR.z = outputs.FZFR;
        tire_dynamics_status.tire_forces_n.RL.z = outputs.FZRL;
        tire_dynamics_status.tire_forces_n.RR.z = outputs.FZRR;

        tire_dynamics_status.accel_saturation_nm.FL = outputs.satAccelTFL;
        tire_dynamics_status.accel_saturation_nm.FR = outputs.satAccelTFR;
        tire_dynamics_status.accel_saturation_nm.RL = outputs.satAccelTRL;
        tire_dynamics_status.accel_saturation_nm.RR = outputs.satAccelTRR;

        tire_dynamics_status.brake_saturation_nm.FL = outputs.satBrakeTFL;
        tire_dynamics_status.brake_saturation_nm.FR = outputs.satBrakeTFR;
        tire_dynamics_status.brake_saturation_nm.RL = outputs.satBrakeTRL;
        tire_dynamics_status.brake_saturation_nm.RR = outputs.satBrakeTRR;

        tire_dynamics_status.v_y_lm = outputs.Vy_LM;
        tire_dynamics_status.psi_dot_lm_rad_s = outputs.Psi_dot_LMrads;

        torque_vectoring_status.additional_mz_moment_nm = outputs.AdditionalMzNm;
        torque_vectoring_status.torque_additional_nm = {outputs.Torq_Add_FL, outputs.Torq_Add_FR, outputs.Torq_Add_RL, outputs.Torq_Add_RR};
        torque_vectoring_status.des_psi_dot = outputs.Des_Psi_dot;
        torque_vectoring_status.psi_dot_err = outputs.Psi_dot_err;

        control_res = {outputs.torq_req_FL, outputs.torq_req_FR, outputs.torq_req_RL, outputs.torq_req_RR}; // '<Root>/torq_req_FL'


        return {{tire_dynamics_status, torque_vectoring_status}, control_res};
    }

}