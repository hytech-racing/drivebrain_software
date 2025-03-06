// DriveBrainApp.hpp
#pragma once

#include <JsonFileHandler.hpp>
#include <CANComms.hpp>
#include <SimpleController.hpp>
#include <StateEstimator.hpp>
#include <MCUETHComms.hpp>
#include <VNComms.hpp>
#include <MsgLogger.hpp>
#include <MCAPProtobufLogger.hpp>
#include <mcap/writer.hpp>
#include <DrivebrainBase.hpp>
#include <foxglove_server.hpp>
#include <DBServiceImpl.hpp>

#include <thread>
#include <chrono>
#include <condition_variable>
#include <cassert>
#include <boost/program_options.hpp>
#include <boost/asio.hpp>
#include <memory>
#include <optional>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <spdlog/spdlog.h>
#include <atomic>

#include <drivebrain_core/ControllerManager.hpp>

struct DriveBrainSettings {
    bool run_db_service{true};
    bool run_io_context{true};
    bool run_process_loop{true};
    bool use_vectornav{true};
};

class DriveBrainApp {
public:
    DriveBrainApp(const std::string& param_path, const std::string& dbc_path,  const DriveBrainSettings& settings = DriveBrainSettings{});
    ~DriveBrainApp();

    void run();
    void stop();

private:
    // Private member functions
    void _process_loop();
    void _signal_handler(int signal);
private:
    // Private member variables
    static std::atomic<bool> _stop_signal;
    const std::string _param_path;
    core::Logger _logger;
    core::JsonFileHandler _config;
    std::optional<std::string> _dbc_path;
    boost::asio::io_context _io_context;
    
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> _rx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> _can_tx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> _eth_tx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> _live_telem_queue;

    std::vector<core::common::Configurable*> _configurable_components;
    std::unique_ptr<common::MCAPProtobufLogger> _mcap_logger;
     // std::unique_ptr<control::SimpleController> _controller;
    std::unique_ptr<control::ControllerManager> _controller_manager;
    
    // std::unique_ptr<estimation::Tire_Model_Codegen_MatlabModel> _matlab_math;
    std::unique_ptr<core::FoxgloveWSServer> _foxglove_server;
    std::shared_ptr<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>> _message_logger;
    std::unique_ptr<core::StateEstimator> _state_estimator;
    std::unique_ptr<comms::CANDriver> _driver;
    std::unique_ptr<comms::MCUETHComms> _eth_driver;
    std::unique_ptr<comms::VNDriver> _vn_driver;
    std::unique_ptr<DBInterfaceImpl> _db_service;
    
    std::thread _process_thread;
    std::thread _io_context_thread;
    std::thread _db_service_thread;

    const DriveBrainSettings _settings;
};