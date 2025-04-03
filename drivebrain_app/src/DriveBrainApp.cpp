// DriveBrainApp.cpp
#include "DriveBrainApp.hpp"

#include "SimpleSpeedController.hpp"
#include "SimpleTorqueController.hpp"
#include "hytech.pb.h"
#include <memory>
#include <mutex>
#include <spdlog/spdlog.h>
#include <thread>

std::atomic<bool> DriveBrainApp::_stop_signal{false};

DriveBrainApp::DriveBrainApp(const std::string& param_path, const std::string& dbc_path, const DriveBrainSettings& settings)
    : _param_path(param_path)
    , _dbc_path(dbc_path)
    , _logger(core::LogLevel::INFO)
    , _config(_param_path)
    , _settings(settings)
    , controller1(std::make_shared<control::SimpleSpeedController>(_config))
    , controller2(std::make_shared<control::SimpleTorqueController>(_config))
    , _controllerManager(_config, {controller1, controller2})  // Initialize correctly
{
    // spdlog::info("top o");
    std::vector<std::shared_ptr<core::common::Configurable>> configurable_components;
    spdlog::set_level(spdlog::level::info);

    // TODO make this function that can get the config schemas from the configureable components. it also needs to join all of the schemas together 
    
    auto get_schema = []() -> nlohmann::json
    {
        return nlohmann::json();
    };

    
    controller1 = std::make_shared<control::SimpleSpeedController>(_config);
    if (!controller1->init()) {
        throw std::runtime_error("Failed to initialize controller");
    }
    configurable_components.push_back(std::static_pointer_cast<core::common::Configurable>(controller1));
    spdlog::info("made controller");

    
    
    
    _state_estimator = std::make_unique<core::StateEstimator>(_logger, _message_logger);
    spdlog::info("made state estimator");
    bool construction_failed = false;
    // this also calls init() in the constructor
    _driver = std::make_shared<comms::CANDriver>(
        _config, _logger, _message_logger,_can_tx_queue, _io_context, 
        _dbc_path, construction_failed, *_state_estimator);
    
    if (construction_failed) {
        throw std::runtime_error("Failed to construct CAN driver");
    }
    configurable_components.push_back(std::static_pointer_cast<core::common::Configurable>(_driver));
    spdlog::info("made CAN driver");
    _eth_driver = std::make_unique<comms::MCUETHComms>(
        _logger, _eth_tx_queue, _message_logger, *_state_estimator,
        _io_context, "192.168.1.30", 2001, 2000);
    
    spdlog::info("eth driver");

    auto switch_modes = 
    [this](size_t mode) -> bool {
        return _controllerManager.swap_active_controller(mode, _state_estimator->get_latest_state_and_validity().first);
    };
    _db_service = std::make_unique<DBInterfaceImpl>(_message_logger, switch_modes);
    spdlog::info("made db service");
    if(_settings.use_vectornav)
    {
        // on creation calls init()
        _vn_driver = std::make_shared<comms::VNDriver>(_config, _logger, _message_logger, *_state_estimator, _io_context, construction_failed);
        if (construction_failed) {
           throw std::runtime_error("Failed to construct VN driver");
        }
        configurable_components.push_back(_vn_driver);
    }

    
    
    
    // - [x] TODO figure out how im going to get the parameter schemas for each of the configureable components into the mcap logger if 
    // the mcap logger is needed by the message logger but I wont know the schemas until the components have been created and the each
    // component is given the message logger on construction. 
    //   if I just have an initialize method that calls the schema get function and sets a member var to store that schema
    //   that could work. 
    
    // - [x] TODO add in function for getting the current config values periodically of all of the configureable components,
    //       or, just give the vector of configureable components that gets given to the foxglove webserver instance
    //       and make the logger also handle the getting of all of the configs of the components (imma do dis way)
    _mcap_logger = std::make_shared<common::DrivebrainMCAPLogger>("temp", configurable_components);
    _foxglove_server = std::make_shared<core::FoxgloveWSServer>(configurable_components);
    
    spdlog::info("made mcap logger and foxglove server");

    // all things must be initialized before this gets constructed due to logging on init needing the schemas determined by the init 
    // functions of the configurable components
    _message_logger = std::make_shared<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>>(
        ".mcap", true,
        std::bind(&common::DrivebrainMCAPLogger::log_msg, _mcap_logger, std::placeholders::_1),
        std::bind(&common::DrivebrainMCAPLogger::close_current_mcap, _mcap_logger),
        std::bind(&common::DrivebrainMCAPLogger::open_new_mcap, _mcap_logger, std::placeholders::_1),
        std::bind(&core::FoxgloveWSServer::send_live_telem_msg, _foxglove_server, std::placeholders::_1),
        std::bind(&common::DrivebrainMCAPLogger::init_param_schema, _mcap_logger),
        std::bind(&common::DrivebrainMCAPLogger::log_params, _mcap_logger));

    if(_driver)
    {
        _driver->update_msg_logger(_message_logger);
    }
    if(_vn_driver)
    {
        _vn_driver->update_msg_logger(_message_logger);
    }
    if(_state_estimator)
    {
        _state_estimator->update_msg_logger(_message_logger);
    }

    if(_db_service)
    {
        _db_service->update_msg_logger(_message_logger);
    }
    if(_eth_driver)
    {
        _eth_driver->update_msg_logger(_message_logger);
    }

    spdlog::info("constructed app");
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
    auto desired_rpm_msg = std::make_shared<hytech::drivebrain_speed_set_input>();
    auto torque_limit_msg = std::make_shared<hytech::drivebrain_torque_lim_input>();
    auto desired_torque_msg = std::make_shared<hytech::drivebrain_desired_torque_input>();
    auto loop_time = _controllerManager.get_active_controller_timestep();
    auto loop_time_micros = (int)(loop_time * 1000000.0f);
    std::chrono::microseconds loop_chrono_time(loop_time_micros);

    while (!_stop_signal.load()) {
        auto start_time = std::chrono::high_resolution_clock::now();

        auto state_and_validity = _state_estimator->get_latest_state_and_validity();

        auto out_struct = _controllerManager.step_active_controller(state_and_validity.first);

        // get current command
        std::variant<core::SpeedControlOut, core::TorqueControlOut, std::monostate> cmd_out = out_struct.out;

        // push current command for next state estimator call
        _state_estimator->set_previous_control_output(out_struct);

        if (const core::SpeedControlOut* speedControl = std::get_if<core::SpeedControlOut>(&cmd_out)) { // speed controller, set RPM

            // set RPMs in message to the RPMS given from the controller
            desired_rpm_msg->set_drivebrain_set_rpm_fl(speedControl->desired_rpms.FL);
            desired_rpm_msg->set_drivebrain_set_rpm_fr(speedControl->desired_rpms.FR);
            desired_rpm_msg->set_drivebrain_set_rpm_rl(speedControl->desired_rpms.RL);
            desired_rpm_msg->set_drivebrain_set_rpm_rr(speedControl->desired_rpms.RR);

            // same with torque limits
            torque_limit_msg->set_drivebrain_torque_fl(::abs(speedControl->torque_lim_nm.FL));
            torque_limit_msg->set_drivebrain_torque_fr(::abs(speedControl->torque_lim_nm.FR));
            torque_limit_msg->set_drivebrain_torque_rl(::abs(speedControl->torque_lim_nm.RL));
            torque_limit_msg->set_drivebrain_torque_rr(::abs(speedControl->torque_lim_nm.RR));
            {
                std::unique_lock lk(_can_tx_queue.mtx);
                _can_tx_queue.deque.push_back(desired_rpm_msg);
                _can_tx_queue.deque.push_back(torque_limit_msg);
                _can_tx_queue.cv.notify_all(); // notify the CAN thread to send the messages
                spdlog::info("sent can");
            }
            
        } else if (const core::TorqueControlOut* torqueControl = std::get_if<core::TorqueControlOut>(&cmd_out)){ // if it is a torque controller:
            // set desired torque
            desired_torque_msg->set_drivebrain_torque_fl(::abs(torqueControl->desired_torques_nm.FL));
            desired_torque_msg->set_drivebrain_torque_fr(::abs(torqueControl->desired_torques_nm.FR));
            desired_torque_msg->set_drivebrain_torque_rl(::abs(torqueControl->desired_torques_nm.RL));
            desired_torque_msg->set_drivebrain_torque_rr(::abs(torqueControl->desired_torques_nm.RR));
            {
                std::unique_lock lk(_can_tx_queue.mtx);
                _can_tx_queue.deque.push_back(desired_torque_msg); // use new protobuf struct
                _can_tx_queue.cv.notify_all(); // notify the CAN thread to send the messages
            }
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
