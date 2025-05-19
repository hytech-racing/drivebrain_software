// DriveBrainApp.cpp
#include "DriveBrainApp.hpp"

#include "SimpleSpeedController.hpp"
#include "SurreyAeroComms.hpp"
#include "hytech.pb.h"
#include <hytech_msgs.pb.h>
#include <memory>
#include <mutex>
#include <spdlog/common.h>
#include <spdlog/spdlog.h>
#include <thread>

std::atomic<bool> stop_signal{false};

DriveBrainApp::DriveBrainApp(const std::string& param_path, const std::string& dbc_path, const DriveBrainSettings& settings)
    : _param_path(param_path)
    , _dbc_path(dbc_path)
    , _logger(core::LogLevel::INFO)
    , _config(_param_path)
    , _settings(settings)
    , controller1(std::make_shared<control::SimpleSpeedController>(_config))
    , _controllerManager(_config, {controller1})  // Initialize correctly
{
    // spdlog::info("top o");
    std::vector<std::weak_ptr<core::common::Configurable>> configurable_components;
    spdlog::set_level(spdlog::level::info);

    
    controller1 = std::make_shared<control::SimpleSpeedController>(_config);
    if (!controller1->init()) {
        throw std::runtime_error("Failed to initialize controller");
    }
    // TODO make this required for the controller manager and remove use of raii for this shared ptrs to the controllers for construction of cm
    _controllerManager.update_controllers({controller1});
    if(!_controllerManager.init()){
        throw std::runtime_error("Failed to initialize controller manager");
    }

    configurable_components.push_back(controller1);
    spdlog::info("made controller");

    
    
    
    _state_estimator = std::make_unique<core::StateEstimator>(_config, _message_logger);
    if(!_state_estimator->init())
    {
        throw std::runtime_error("Failed to initialize state estimator");
    }
    configurable_components.push_back(_state_estimator);
    spdlog::info("made state estimator");
    
    bool construction_failed = false;
    // this also calls init() in the constructor
    
    _driver_primary_can = std::make_shared<comms::CANDriver>(
        _config, _message_logger, _primary_can_tx_queue, _io_context, 
        _dbc_path, construction_failed, _state_estimator, "CANDriverPrimary");
    
    if (construction_failed) {
        throw std::runtime_error("Failed to construct CAN driver");
    }
    
    _driver_secondary_can = std::make_shared<comms::CANDriver>(
        _config,  _message_logger, _secondary_can_tx_queue, _io_context_secondary_can, 
        _dbc_path, construction_failed, _state_estimator, "CANDriverSecondary");
    
    if (construction_failed) {
        throw std::runtime_error("Failed to construct CAN driver");
    }
    configurable_components.push_back(_driver_primary_can);
    configurable_components.push_back(_driver_secondary_can);
    spdlog::info("made CAN driver");
    _acu_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::ACUAllData>>(_io_context, 7766);
    _vcr_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::VCRData_s>>(_io_context, 9999);
    _vcf_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::VCFData_s>>(_io_context, 4444);
    
    spdlog::info("eth drivers");

    auto switch_modes = 
    [this](size_t mode) -> bool {
        return _controllerManager.swap_active_controller(mode, _state_estimator->get_latest_state_and_validity().first);
    };
    _db_service = std::make_unique<DBInterfaceImpl>(_message_logger, switch_modes);
    spdlog::info("made db service");

    nlohmann::json &config_json = _config.get_config();
    if(config_json.contains("use_vectornav") && config_json["use_vectornav"])
    {
        spdlog::info("using vectornav");
        // on creation calls init()
        _vn_driver = std::make_shared<comms::VNDriver>(_config, _logger, _message_logger, _state_estimator, _io_context, construction_failed);
        if (construction_failed) {
           throw std::runtime_error("Failed to construct VN driver");
        }
        configurable_components.push_back(_vn_driver);
    } else if(config_json.contains("use_fake_vn") && config_json["use_fake_vn"])
    {
        _fake_vn = std::make_unique<comms::ETHRecvComms<hytech_msgs::VNData>>( _io_context, 13111, _state_estimator);
    } 
    
    if(config_json.contains("use_surrey_aero") && config_json["use_surrey_aero"])
    {
        spdlog::info("making surrey aero sensor");
        _aero_sensor_driver = std::make_shared<comms::SurreyAeroComms>(_config, _aero_usb_io_context);
        if(!_aero_sensor_driver->init()){
            throw std::runtime_error("failed to init aero sensor driver");

        }
        spdlog::info("made surrey aero sensor");
    }

    if(config_json.contains("use_scale_comms") && config_json["use_scale_comms"])
    {
        spdlog::info("making scale comms driver");
        _scale_comms = std::make_shared<comms::ScaleComms>(_config, _scale_usb_io_context);
        if(!_scale_comms->init()){
            throw std::runtime_error("failed to init scale comms");

        }
        spdlog::info("made scale comms driver");
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
    // // functions of the configurable components
    _message_logger = std::make_shared<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>>(
        ".mcap", true,
        std::bind(&common::DrivebrainMCAPLogger::log_msg, _mcap_logger, std::placeholders::_1),
        std::bind(&common::DrivebrainMCAPLogger::close_current_mcap, _mcap_logger),
        std::bind(&common::DrivebrainMCAPLogger::open_new_mcap, _mcap_logger, std::placeholders::_1),
        std::bind(&core::FoxgloveWSServer::send_live_telem_msg, _foxglove_server, std::placeholders::_1),
        std::bind(&common::DrivebrainMCAPLogger::init_param_schema, _mcap_logger),
        std::bind(&common::DrivebrainMCAPLogger::log_params, _mcap_logger));

    if(_driver_primary_can)
    {
        _driver_primary_can->update_msg_logger(_message_logger);
    }
    if(_driver_secondary_can)
    {
        _driver_secondary_can->update_msg_logger(_message_logger);
    }
    
    if(_vn_driver)
    {
        _vn_driver->update_msg_logger(_message_logger);
    }
    if(_fake_vn)
    {
        _fake_vn->update_msg_logger(_message_logger);
    }
    if(_state_estimator)
    {
        _state_estimator->update_msg_logger(_message_logger);
    }

    if(_db_service)
    {
        _db_service->update_msg_logger(_message_logger);
    }
    if(_acu_eth_driver)
    {
        _acu_eth_driver->update_msg_logger(_message_logger);
    }
    if(_vcr_eth_driver)
    {
        _vcr_eth_driver->update_msg_logger(_message_logger);
    }
    if(_vcf_eth_driver)
    {
        _vcf_eth_driver->update_msg_logger(_message_logger);
    }
    if(_aero_sensor_driver)
    {
        _aero_sensor_driver->set_msg_logger(_message_logger);
    }
    if(_scale_comms)
    {
        _scale_comms->set_msg_logger(_message_logger);
    }

    _message_logger->start_logging_params();

    spdlog::info("constructed app");
}

DriveBrainApp::~DriveBrainApp() {
    stop_signal.store(true);
    
    if(_message_logger)
    {
        _message_logger->halt();
    }
    

    if (_process_thread.joinable()) {
        _process_thread.join();
        spdlog::info("joined main process");
    }
    
    
    spdlog::info("halted message logger");
    _io_context.stop();
    if (_io_context_thread.joinable()) {
        _io_context_thread.join();
        spdlog::info("joined io context 1");
    }
    
    _io_context_secondary_can.stop();
    if (_io_context_secondary_thread.joinable()) {
        _io_context_secondary_thread.join();
        spdlog::info("joined io context 2");
    }

    if(_aero_sensor_driver) {
        _aero_usb_io_context.stop();
        if(_aero_usb_io_context_thread.joinable()) {
            _aero_usb_io_context_thread.join();
        }
    }
    if(_scale_comms)
    {
        _scale_usb_io_context.stop();
        if(_scale_usb_io_context_thread.joinable())
        {
            _scale_usb_io_context_thread.join();
        }
    }
    
    
    if (_db_service) {
        _db_service->stop_server();
    }
    if ( _db_service_thread.joinable()) {
        _db_service_thread.join();
        spdlog::info("joined db service");
    }
    spdlog::info("destructed db app");
}

void DriveBrainApp::_process_loop() {
    auto desired_rpm_msg = std::make_shared<hytech::drivebrain_speed_set_input>();
    auto torque_limit_msg = std::make_shared<hytech::drivebrain_torque_lim_input>();
    auto desired_torque_msg = std::make_shared<hytech::drivebrain_desired_torque_input>();
    auto loop_time = 0.005;
    auto loop_time_micros = (int)(loop_time * 1000000.0f);
    std::chrono::microseconds loop_chrono_time(loop_time_micros);

    while (!stop_signal.load()) {
        spdlog::debug("looping _process_loop");
        auto start_time = std::chrono::high_resolution_clock::now();

        auto state_and_validity = _state_estimator->get_latest_state_and_validity();
        
        auto out_struct = _controllerManager.step_active_controller(state_and_validity.first);

        // get current command
        std::variant<core::SpeedControlOut, core::TorqueControlOut, std::monostate> cmd_out = out_struct.out;

        // push current command for next state estimator call
        _state_estimator->set_previous_control_output(out_struct);
        bool state_is_valid = state_and_validity.second;
        if(state_is_valid)
        {

            if (const core::SpeedControlOut* speedControl = std::get_if<core::SpeedControlOut>(&cmd_out)) { // speed controller, set RPM

                // set RPMs in message to the RPMS given from the controller
            
                desired_rpm_msg->set_drivebrain_set_rpm_fl(speedControl->desired_rpms.FL);
                desired_rpm_msg->set_drivebrain_set_rpm_fr(speedControl->desired_rpms.FR);
                desired_rpm_msg->set_drivebrain_set_rpm_rl(speedControl->desired_rpms.RL);
                desired_rpm_msg->set_drivebrain_set_rpm_rr(speedControl->desired_rpms.RR);
            
                if(_message_logger)
                {
                    _message_logger->log_msg(static_cast<std::shared_ptr<google::protobuf::Message>>(desired_rpm_msg));
                }
                
                // same with torque limits
                torque_limit_msg->set_drivebrain_torque_fl(::abs(speedControl->torque_lim_nm.FL));
                torque_limit_msg->set_drivebrain_torque_fr(::abs(speedControl->torque_lim_nm.FR));
                torque_limit_msg->set_drivebrain_torque_rl(::abs(speedControl->torque_lim_nm.RL));
                torque_limit_msg->set_drivebrain_torque_rr(::abs(speedControl->torque_lim_nm.RR));
            
                if(_message_logger)
                {
                    _message_logger->log_msg(static_cast<std::shared_ptr<google::protobuf::Message>>(torque_limit_msg));
                }
                
                {
                    std::unique_lock lk(_primary_can_tx_queue.mtx);
                    _primary_can_tx_queue.deque.push_back(desired_rpm_msg);
                    _primary_can_tx_queue.deque.push_back(torque_limit_msg);
                    _primary_can_tx_queue.cv.notify_all(); // notify the CAN thread to send the messages
                    // spdlog::info("sent can");
                }
                
            } else if (const core::TorqueControlOut* torqueControl = std::get_if<core::TorqueControlOut>(&cmd_out)){ // if it is a torque controller:
                // set desired torque
                
                desired_torque_msg->set_drivebrain_torque_fl(torqueControl->desired_torques_nm.FL);
                desired_torque_msg->set_drivebrain_torque_fr(torqueControl->desired_torques_nm.FR);
                desired_torque_msg->set_drivebrain_torque_rl(torqueControl->desired_torques_nm.RL);
                desired_torque_msg->set_drivebrain_torque_rr(torqueControl->desired_torques_nm.RR);
                
                if(_message_logger)
                {
                    _message_logger->log_msg(static_cast<std::shared_ptr<google::protobuf::Message>>(desired_torque_msg));
                }
                
                {
                    std::unique_lock lk(_primary_can_tx_queue.mtx);
                    _primary_can_tx_queue.deque.push_back(desired_torque_msg); // use new protobuf struct
                    _primary_can_tx_queue.cv.notify_all(); // notify the CAN thread to send the messages
                }
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        if (loop_chrono_time > elapsed) {
            std::this_thread::sleep_for(loop_chrono_time - elapsed);
        }
    }
}



void signal_handler(int signal)
{
    spdlog::info("Interrupt signal db app({}) received. Cleaning up...", signal);
    stop_signal.store(true); // Set running to false to exit the main loop or gracefully terminate
}

void DriveBrainApp::run() {

    std::signal(SIGINT, signal_handler);
    _db_service_thread = std::thread([this]() {
        
        if (!_settings.run_db_service) return;
        
        spdlog::info("started db service thread");
        try {
            while (!stop_signal.load()) {
                spdlog::debug("looping db service thread");
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

    _io_context_secondary_thread = std::thread([this]() {
        if (!_settings.run_io_context) return;
        spdlog::info("Started io context 2 thread");
        try {
            _io_context_secondary_can.run();
        } catch (const std::exception& e) {
            spdlog::error("Error in io_context 2: {}", e.what());
        }
    });

    if(_aero_sensor_driver)
    {
        _aero_usb_io_context_thread = std::thread([this]() -> void {
            spdlog::info("Started _aero_usb_io_context_thread");
            try {
                _aero_usb_io_context.run();
            } catch (const std::exception& e) {
                spdlog::error("Error in _aero_usb_io_context: {}", e.what());
            }
        });
    }
    if(_scale_comms)
    {
        _scale_usb_io_context_thread = std::thread([this]() -> void {
            spdlog::info("Started _scale_usb_io_context_thread");
            try {
                _scale_usb_io_context.run();
            } catch (const std::exception& e) {
                spdlog::error("Error in _scale_usb_io_context: {}", e.what());
            }
        });
    }
    

    _process_thread = std::thread([this]() {
        if (!_settings.run_process_loop) return;
        _process_loop();
    });

    
    while (!stop_signal.load()) {
        spdlog::debug("looping DBAPP run");
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
