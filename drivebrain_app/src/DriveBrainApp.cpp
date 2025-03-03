// DriveBrainApp.cpp
#include "DriveBrainApp.hpp"

std::atomic<bool> DriveBrainApp::_stop_signal{false};

DriveBrainApp::DriveBrainApp(const std::string& param_path, const std::string& dbc_path, const DriveBrainSettings& settings)
    : _param_path(param_path)
    , _dbc_path(dbc_path)
    , _logger(core::LogLevel::INFO)
    , _config(_param_path)
    , _settings(settings)
    , _db_service_thread([this]() {
        if (!_settings.run_db_service) return;
        spdlog::info("started db service thread");
        try {
            while (!_stop_signal.load()) {
                _db_service->run_server();
            }
        } catch (const std::exception& e) {
            spdlog::error("Error in drivebrain service thread: {}", e.what());
        }
    })
    , _io_context_thread([this]() {
        if (!_settings.run_io_context) return;
        spdlog::info("Started io context thread");
        try {
            _io_context.run();
        } catch (const std::exception& e) {
            spdlog::error("Error in io_context: {}", e.what());
        }
    })
    , _process_thread([this]() {
        if (!_settings.run_process_loop) return;
        _process_loop();
    })
{

    spdlog::set_level(spdlog::level::warn);

    _mcap_logger = std::make_unique<common::MCAPProtobufLogger>("temp");
    
    _controller = std::make_unique<control::SimpleController>(_logger, _config);
    _configurable_components.push_back(_controller.get());
    
    bool matlab_construction_failed = false;
    _matlab_math = std::make_unique<estimation::Tire_Model_Codegen_MatlabModel>(
        _logger, _config, matlab_construction_failed);
    
    _configurable_components.push_back(_matlab_math.get());
    
    _foxglove_server = std::make_unique<core::FoxgloveWSServer>(_configurable_components);
    
    _message_logger = std::make_shared<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>>(
        ".mcap", true,
        std::bind(&common::MCAPProtobufLogger::log_msg, std::ref(*_mcap_logger), std::placeholders::_1),
        std::bind(&common::MCAPProtobufLogger::close_current_mcap, std::ref(*_mcap_logger)),
        std::bind(&common::MCAPProtobufLogger::open_new_mcap, std::ref(*_mcap_logger), std::placeholders::_1),
        std::bind(&core::FoxgloveWSServer::send_live_telem_msg, std::ref(*_foxglove_server), std::placeholders::_1));
    
    _state_estimator = std::make_unique<core::StateEstimator>(_logger, _message_logger, *_matlab_math);
    
    bool construction_failed = false;
    _driver = std::make_unique<comms::CANDriver>(
        _config, _logger, _message_logger, _tx_queue, _io_context, 
        _dbc_path, construction_failed, *_state_estimator);
    
    if (construction_failed) {
        throw std::runtime_error("Failed to construct CAN driver");
    }
    
    _configurable_components.push_back(_driver.get());
    
    _eth_driver = std::make_unique<comms::MCUETHComms>(
        _logger, _eth_tx_queue, _message_logger, *_state_estimator,
        _io_context, "192.168.1.30", 2001, 2000);
    
    _vn_driver = std::make_unique<comms::VNDriver>(
        _config, _logger, _message_logger, *_state_estimator, _io_context, construction_failed);

    if (construction_failed) {
        throw std::runtime_error("Failed to construct VN driver");
    }
    
    _db_service = std::make_unique<DBInterfaceImpl>(_message_logger);
    
    if (!_controller->init()) {
        throw std::runtime_error("Failed to initialize controller");
    }

    // TODO add here the creation of the config logger
}

DriveBrainApp::~DriveBrainApp() {
    _stop_signal.store(true);
    
    if (_process_thread.joinable()) {
        _process_thread.join();
    }
    spdlog::info("joined main process");

    _io_context.stop();
    if (_io_context_thread.joinable()) {
        _io_context_thread.join();
    }
    
    if (_db_service) {
        _db_service->stop_server();
    }
    if ( _db_service_thread.joinable()) {
        _db_service_thread.join();
    }
    spdlog::info("joined io context");
}

void DriveBrainApp::_process_loop() {
    auto out_msg = std::make_shared<hytech_msgs::MCUCommandData>();
    auto loop_time = _controller->get_dt_sec();
    auto loop_time_micros = (int)(loop_time * 1000000.0f);
    std::chrono::microseconds loop_chrono_time(loop_time_micros);

    while (!_stop_signal.load()) {
        auto start_time = std::chrono::high_resolution_clock::now();

        auto state_and_validity = _state_estimator->get_latest_state_and_validity();
        auto out_struct = _controller->step_controller(state_and_validity.first);
        auto temp_desired_torques = state_and_validity.first.matlab_math_temp_out;
        _state_estimator->set_previous_control_output(out_struct);

        out_msg->set_prev_mcu_recv_millis(out_struct.mcu_recv_millis);

        if(temp_desired_torques.res_torque_lim_nm.FL < 0) {
            out_msg->mutable_desired_rpms()->set_fl(0);
        } else {
            out_msg->mutable_desired_rpms()->set_fl(out_struct.desired_rpms.FL);
        }

        if(temp_desired_torques.res_torque_lim_nm.FR < 0) {
            out_msg->mutable_desired_rpms()->set_fr(0);
        } else {
            out_msg->mutable_desired_rpms()->set_fr(out_struct.desired_rpms.FR);
        }

        if(temp_desired_torques.res_torque_lim_nm.RL < 0) {
            out_msg->mutable_desired_rpms()->set_rl(0);
        } else {
            out_msg->mutable_desired_rpms()->set_rl(out_struct.desired_rpms.RL);
        }

        if(temp_desired_torques.res_torque_lim_nm.RR < 0) {
            out_msg->mutable_desired_rpms()->set_rr(0);
        } else {
            out_msg->mutable_desired_rpms()->set_rr(out_struct.desired_rpms.RR);
        }

        out_msg->mutable_torque_limit_nm()->set_fl(::abs(temp_desired_torques.res_torque_lim_nm.FL));
        out_msg->mutable_torque_limit_nm()->set_fr(::abs(temp_desired_torques.res_torque_lim_nm.FR));
        out_msg->mutable_torque_limit_nm()->set_rl(::abs(temp_desired_torques.res_torque_lim_nm.RL));
        out_msg->mutable_torque_limit_nm()->set_rr(::abs(temp_desired_torques.res_torque_lim_nm.RR));

        {
            std::unique_lock lk(_eth_tx_queue.mtx);
            _eth_tx_queue.deque.push_back(out_msg);
            _eth_tx_queue.cv.notify_all();
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        if (loop_chrono_time > elapsed) {
            std::this_thread::sleep_for(loop_chrono_time - elapsed);
        }
    }
}

void DriveBrainApp::run() {
    while (!_stop_signal.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
