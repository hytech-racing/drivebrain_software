// DriveBrainApp.cpp
#include "DriveBrainApp.hpp"

std::atomic<bool> DriveBrainApp::stop_signal_{false};

std::string DriveBrainApp::getParamPathFromArgs(int argc, char* argv[]) {
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    std::string param_path;
    
    desc.add_options()
        ("param-path,p", po::value<std::string>(&param_path)->default_value("config/drivebrain_config.json"), 
         "Path to the parameter JSON file");

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).allow_unregistered().run(), vm);
    po::notify(vm);
    
    return param_path;
}

void DriveBrainApp::parseCommandLine(int argc, char* argv[]) {
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    
    desc.add_options()
        ("help,h", "produce help message")
        ("dbc-path,d", po::value<std::string>(), "Path to the DBC file (optional)");

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).allow_unregistered().run(), vm);
    po::notify(vm);
    
    if (vm.count("help")) {
        std::stringstream ss;
        ss << desc;
        spdlog::info("{}", ss.str());
        throw std::runtime_error("Help requested");
    }
    
    if (vm.count("dbc-path")) {
        dbc_path_ = vm["dbc-path"].as<std::string>();
    }
}

DriveBrainApp::DriveBrainApp(int argc, char* argv[])
    : param_path_(getParamPathFromArgs(argc, argv))
    , logger_(core::LogLevel::INFO)
    , config_(param_path_)
{
    parseCommandLine(argc, argv);
    setupSignalHandler();
    spdlog::set_level(spdlog::level::warn);
}

DriveBrainApp::~DriveBrainApp() {
    stop();
}

void DriveBrainApp::setupSignalHandler() {
    std::signal(SIGINT, [](int signal) {
        spdlog::info("Interrupt signal ({}) received. Cleaning up...", signal);
        DriveBrainApp::stop_signal_.store(true);
    });
}

bool DriveBrainApp::initialize() {
    mcap_logger_ = std::make_unique<common::MCAPProtobufLogger>("temp");
    
    controller_ = std::make_unique<control::SimpleController>(logger_, config_);
    configurable_components_.push_back(controller_.get());
    
    matlab_math_ = std::make_unique<estimation::Tire_Model_Codegen_MatlabModel>(
        logger_, config_, construction_failed_);
    
    if (construction_failed_) {
        stop_signal_.store(true);
        return false;
    }
    
    configurable_components_.push_back(matlab_math_.get());
    
    // live foxglove server server that relies upon having pointers to configurable components for 
    // getting and handling updates of the registered live parameters
    foxglove_server_ = std::make_unique<core::FoxgloveWSServer>(configurable_components_);
    
    message_logger_ = std::make_shared<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>>(
        ".mcap", true,
        std::bind(&common::MCAPProtobufLogger::log_msg, std::ref(*mcap_logger_), std::placeholders::_1),
        std::bind(&common::MCAPProtobufLogger::close_current_mcap, std::ref(*mcap_logger_)),
        std::bind(&common::MCAPProtobufLogger::open_new_mcap, std::ref(*mcap_logger_), std::placeholders::_1),
        std::bind(&core::FoxgloveWSServer::send_live_telem_msg, std::ref(*foxglove_server_), std::placeholders::_1));
    
    state_estimator_ = std::make_unique<core::StateEstimator>(logger_, message_logger_, *matlab_math_);
    
    driver_ = std::make_unique<comms::CANDriver>(config_, logger_, message_logger_, tx_queue_, io_context_, dbc_path_, construction_failed_, *state_estimator_);
    
    if (construction_failed_) {
        stop_signal_.store(true);
        return false;
    }
    
    configurable_components_.push_back(driver_.get());
    
    eth_driver_ = std::make_unique<comms::MCUETHComms>(logger_, eth_tx_queue_, message_logger_, *state_estimator_,io_context_, "192.168.1.30", 2001, 2000);
    
    vn_driver_ = std::make_unique<comms::VNDriver>(config_, logger_, message_logger_, *state_estimator_, io_context_);
    
    db_service_ = std::make_unique<DBInterfaceImpl>(message_logger_);
    
    // required init, maybe want to call this in the constructor instead
    bool successful_controller_init = controller_->init();

    return true;
}

void DriveBrainApp::processLoop() {
    auto out_msg = std::make_shared<hytech_msgs::MCUCommandData>();
    auto loop_time = controller_->get_dt_sec();
    auto loop_time_micros = (int)(loop_time * 1000000.0f);
    std::chrono::microseconds loop_chrono_time(loop_time_micros);

    while (!stop_signal_.load()) {
        auto start_time = std::chrono::high_resolution_clock::now();
        // samples internal state set by the handle recv functions

        auto state_and_validity = state_estimator_->get_latest_state_and_validity();
        auto out_struct = controller_->step_controller(state_and_validity.first);
        auto temp_desired_torques = state_and_validity.first.matlab_math_temp_out;
        // feedback
        state_estimator_->set_previous_control_output(out_struct);
        // output
        // if(state_and_validity.second)
        // {
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
            std::unique_lock lk(eth_tx_queue_.mtx);
            eth_tx_queue_.deque.push_back(out_msg);
            eth_tx_queue_.cv.notify_all();
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        if (loop_chrono_time > elapsed) {
            std::this_thread::sleep_for(loop_chrono_time - elapsed);
        }
    }
}

void DriveBrainApp::startThreads() {
    db_service_thread_ = std::make_unique<std::thread>([this]() {
        spdlog::info("started db service thread");
        try {
            while (!stop_signal_.load()) {
                // Run the io_context as long as stop_signal is false
                db_service_->run_server(); // Run at least one handler, or return immediately if none
            }
        } catch (const std::exception& e) {
            spdlog::error("Error in drivebrain service thread: {}", e.what());
        }
    });

    // what we will do here is have a temporary super-loop.
    // in this thread we will block on having anything in the rx queue, everything by default goes into the foxglove server (TODO)
    // if we receive the pedals message, we step the controller and get its output to put intot he tx queue
    
    io_context_thread_ = std::make_unique<std::thread>([this]() {
        spdlog::info("Started io context thread");
        try {
            io_context_.run();
        } catch (const std::exception& e) {
            spdlog::error("Error in io_context: {}", e.what());
        }
    });

    process_thread_ = std::make_unique<std::thread>([this]() { processLoop(); });
}

void DriveBrainApp::run() {
    startThreads();
    
    while (!stop_signal_.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    stop();
}

void DriveBrainApp::stop() {
    stop_signal_.store(true);
    
    if (process_thread_ && process_thread_->joinable()) {
        process_thread_->join();
    }
    spdlog::info("joined main process");
    io_context_.stop();
    if (io_context_thread_ && io_context_thread_->joinable()) {
        io_context_thread_->join();
    }
    
    if (db_service_) {
        db_service_->stop_server();
    }
    if (db_service_thread_ && db_service_thread_->joinable()) {
        db_service_thread_->join();
    }
    spdlog::info("joined io context");
}