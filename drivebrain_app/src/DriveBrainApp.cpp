// DriveBrainApp.cpp
#include "DriveBrainApp.hpp"

#include "hytech.pb.h"
#include <mutex>
#include <thread>

std::atomic<bool> DriveBrainApp::_stop_signal{false};

DriveBrainApp::DriveBrainApp(const std::string& param_path, const std::string& dbc_path, const DriveBrainSettings& settings)
    : _param_path(param_path)
    , _dbc_path(dbc_path)
    , _logger(core::LogLevel::INFO)
    , _config(_param_path)
    , _settings(settings)
    , controller1(control::SimpleSpeedController(_logger, _config))
    , controller2(control::SimpleTorqueController(_logger, _config))
    , _controllerManager(_logger, _config, {&controller1, &controller2})  // Initialize correctly
{

    spdlog::set_level(spdlog::level::warn);

    _mcap_logger = std::make_unique<common::MCAPProtobufLogger>("temp");


    //control::SimpleSpeedController controller1(_logger, _config);
    //control::SimpleTorqueController controller2(_logger, _config);
    _configurable_components.push_back(&controller1);
    _configurable_components.push_back(&controller2);
    //_controllerManager = control::ControllerManager<control::Controller<core::ControllerOutput, core::VehicleState>, 2 >(_logger, _config, {&controller1 , &controller2});
    _configurable_components.push_back(&_controllerManager);

    bool successful_controller1_init = controller1.init();
    bool successful_controller2_init = controller2.init();
    bool successful_manager_init = _controllerManager.init();
    
    // bool matlab_construction_failed = false;
    // _matlab_math = std::make_unique<estimation::Tire_Model_Codegen_MatlabModel>(
    //     _logger, _config, matlab_construction_failed);
    
    // _configurable_components.push_back(_matlab_math.get());
    
    _foxglove_server = std::make_unique<core::FoxgloveWSServer>(_configurable_components);
    
    _message_logger = std::make_shared<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>>(
        ".mcap", true,
        std::bind(&common::MCAPProtobufLogger::log_msg, std::ref(*_mcap_logger), std::placeholders::_1),
        std::bind(&common::MCAPProtobufLogger::close_current_mcap, std::ref(*_mcap_logger)),
        std::bind(&common::MCAPProtobufLogger::open_new_mcap, std::ref(*_mcap_logger), std::placeholders::_1),
        std::bind(&core::FoxgloveWSServer::send_live_telem_msg, std::ref(*_foxglove_server), std::placeholders::_1));
    
    _state_estimator = std::make_unique<core::StateEstimator>(_logger, _message_logger);
    
    bool construction_failed = false;
    _driver = std::make_unique<comms::CANDriver>(
        _config, _logger, _message_logger,_can_tx_queue, _io_context, 
        _dbc_path, construction_failed, *_state_estimator);
    
    if (construction_failed) {
        throw std::runtime_error("Failed to construct CAN driver");
    }
    
    _configurable_components.push_back(_driver.get());
    
    _eth_driver = std::make_unique<comms::MCUETHComms>(
        _logger, _eth_tx_queue, _message_logger, *_state_estimator,
        _io_context, "192.168.1.30", 2001, 2000);
    if(_settings.use_vectornav)
    {
        _vn_driver = std::make_unique<comms::VNDriver>(_config, _logger, _message_logger, *_state_estimator, _io_context);
    }
    
    if (!successful_controller1_init || !successful_controller2_init) {
        throw std::runtime_error("Failed to initialize a controller");
    }
    if (!successful_manager_init) {
        throw std::runtime_error("Failed to initialize controller manager");
    }
    switch_modes = 
    [this](size_t mode) -> bool {
        return _controllerManager.swap_active_controller(mode, _state_estimator->get_latest_state_and_validity().first);
    };
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
    // auto out_msg = std::make_shared<hytech_msgs::MCUCommandData>();
    auto desired_rpm_msg = std::make_shared<hytech::drivebrain_speed_set_input>();
    auto torque_limit_msg = std::make_shared<hytech::drivebrain_torque_lim_input>();
    auto loop_time = _controllerManager.get_active_controller_timestep();
    auto loop_time_micros = (int)(loop_time * 1000000.0f);
    std::chrono::microseconds loop_chrono_time(loop_time_micros);

    while (!_stop_signal.load()) {
        auto start_time = std::chrono::high_resolution_clock::now();

        auto state_and_validity = _state_estimator->get_latest_state_and_validity();
        // TODO handle invalid state. need tc mux
        auto out_struct = _controllerManager.step_active_controller(state_and_validity.first);

        //logic for retrieving whichever type is currently in the variant, i dont think we need to check if it has monostate
        core::SpeedControlOut speed_cmd_out;
        core::TorqueControlOut torque_cmd_out;
        if(std::holds_alternative<core::SpeedControlOut>(out_struct.out))
        {
            speed_cmd_out = std::get<core::SpeedControlOut>(out_struct.out);
        }
        else if(std::holds_alternative<core::TorqueControlOut>(out_struct.out))
        {
            torque_cmd_out = std::get<core::TorqueControlOut>(out_struct.out);
        }
        //for when we have both controllers the vision is if(speed_cmd_out) -> else 
        auto temp_desired_torques = state_and_validity.first.matlab_math_temp_out;


        //_state_estimator->set_previous_control_output(out_struct);
        // TCMUX code works with ControllerOutput, but the state estimator only works with speed controls it looks like
        // so we grab the speed if that is what the variant contains
        // this means that we actuall can only use one Controller, the speed one, so we gotta re-write state estimator :/
        if (auto speedOut = std::get_if<core::SpeedControlOut>(&out_struct.out)) {
            _state_estimator->set_previous_control_output(*speedOut);
        } else {
            // if the std::variant does not contain SpeedControlOut(we're so cooked)
            std::cerr << "Error: ControllerOutput is monostate or TorqueControlOut" << std::endl;
        }


        if(temp_desired_torques.res_torque_lim_nm.FL < 0) {
            desired_rpm_msg->set_drivebrain_set_rpm_fl(0);
        } else {
            desired_rpm_msg->set_drivebrain_set_rpm_fl(speed_cmd_out.desired_rpms.FL);
        }

        if(temp_desired_torques.res_torque_lim_nm.FR < 0) {
            desired_rpm_msg->set_drivebrain_set_rpm_fr(0);
        } else {
            desired_rpm_msg->set_drivebrain_set_rpm_fr(speed_cmd_out.desired_rpms.FR);
        }

        if(temp_desired_torques.res_torque_lim_nm.RL < 0) {
            desired_rpm_msg->set_drivebrain_set_rpm_rl(0);
        } else {
            desired_rpm_msg->set_drivebrain_set_rpm_rl(speed_cmd_out.desired_rpms.RL);
        }

        if(temp_desired_torques.res_torque_lim_nm.RR < 0) {
            desired_rpm_msg->set_drivebrain_set_rpm_rr(0);
        } else {
            desired_rpm_msg->set_drivebrain_set_rpm_rr(speed_cmd_out.desired_rpms.RR);
        }

        torque_limit_msg->set_drivebrain_torque_fl(::abs(temp_desired_torques.res_torque_lim_nm.FL));
        torque_limit_msg->set_drivebrain_torque_fl(::abs(temp_desired_torques.res_torque_lim_nm.FR));
        torque_limit_msg->set_drivebrain_torque_fl(::abs(temp_desired_torques.res_torque_lim_nm.RL));
        torque_limit_msg->set_drivebrain_torque_fl(::abs(temp_desired_torques.res_torque_lim_nm.RR));

        {
            std::unique_lock lk(_can_tx_queue.mtx);
            _can_tx_queue.deque.push_back(desired_rpm_msg);
            _can_tx_queue.deque.push_back(torque_limit_msg);
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        if (loop_chrono_time > elapsed) {
            std::this_thread::sleep_for(loop_chrono_time - elapsed);
        }
    }
}


std::atomic<bool> stop_signal{false};
void signal_handler(int signal)
{
    spdlog::info("Interrupt signal ({}) received. Cleaning up...", signal);
    stop_signal.store(true); // Set running to false to exit the main loop or gracefully terminate
}

void DriveBrainApp::run() {

    std::signal(SIGINT, signal_handler);
    _db_service_thread = std::thread([this]() {
        
        if (!_settings.run_db_service) return;
        
        _db_service = std::make_unique<DBInterfaceImpl>(_message_logger, switch_modes);
        spdlog::info("started db service thread");
        try {
            while (!stop_signal.load()) {
                _db_service->run_server();
            }
        } catch (const std::exception& e) {
            spdlog::error("Error in drivebrain service thread: {}", e.what());
        }
    });

    _io_context_thread = std::thread([this]() {
        if (!_settings.run_io_context) return;
        spdlog::info("Started io context thread");
        try {
            _io_context.run();
        } catch (const std::exception& e) {
            spdlog::error("Error in io_context: {}", e.what());
        }
    });

    _process_thread = std::thread([this]() {
        if (!_settings.run_process_loop) return;
        _process_loop();
    });

    
    while (!stop_signal.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
