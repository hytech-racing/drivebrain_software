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

#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::seconds
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



class DriveBrainApp {
public:
    DriveBrainApp(int argc, char* argv[]);
    ~DriveBrainApp();

    bool initialize();
    void run();
    void stop();

private:
    
    std::string getParamPathFromArgs(int argc, char* argv[]);
    void parseCommandLine(int argc, char* argv[]);
    void setupSignalHandler();
    void startThreads();
    void processLoop();
    void ioContextLoop();
    void dbServiceLoop();

    
    const std::string param_path_;
    core::Logger logger_;
    core::JsonFileHandler config_;
    std::optional<std::string> dbc_path_;
    boost::asio::io_context io_context_;
    static std::atomic<bool> stop_signal_;
    bool construction_failed_{false};
    
    
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> rx_queue_;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> tx_queue_;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> eth_tx_queue_;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> live_telem_queue_;

    
    std::vector<core::common::Configurable*> configurable_components_;
    std::unique_ptr<common::MCAPProtobufLogger> mcap_logger_;
    std::unique_ptr<control::SimpleController> controller_;
    std::unique_ptr<estimation::Tire_Model_Codegen_MatlabModel> matlab_math_;
    std::unique_ptr<core::FoxgloveWSServer> foxglove_server_;
    std::shared_ptr<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>> message_logger_;
    std::unique_ptr<core::StateEstimator> state_estimator_;
    std::unique_ptr<comms::CANDriver> driver_;
    std::unique_ptr<comms::MCUETHComms> eth_driver_;
    std::unique_ptr<comms::VNDriver> vn_driver_;
    std::unique_ptr<DBInterfaceImpl> db_service_;
    
    
    std::unique_ptr<std::thread> process_thread_;
    std::unique_ptr<std::thread> io_context_thread_;
    std::unique_ptr<std::thread> db_service_thread_;
};
