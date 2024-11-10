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
#include <array>

#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::seconds
#include <condition_variable>

#include <cassert>

#include <boost/program_options.hpp>
#include <boost/asio.hpp>

#include <memory>
#include <optional>

#include <mcap/mcap.hpp>
#include <thread> // For std::this_thread::sleep_for
#include <chrono> // For std::chrono::seconds

#include <csignal>
#include <cstdlib>

#include <iostream>
#include <sstream>

#include <spdlog/spdlog.h>

// TODO first application will have

// - [x] message queue that can send messages between the CAN driver and the controller
// - [x] CAN driver that can receive the pedals messages
// - [ ] fix the CAN messages that cant currently be encoded into the protobuf messages
// - [x] simple controller

std::atomic<bool> stop_signal{false};
// Signal handler function
void signalHandler(int signal)
{
    // std::cout << "Interrupt signal (" << signal << ") received. Cleaning up..." << std::endl;
    spdlog::info("Interrupt signal ({}) received. Cleaning up...", signal);
    stop_signal.store(true); // Set running to false to exit the main loop or gracefully terminate
}

int main(int argc, char *argv[])
{

    // io context for boost async io. gets given to the drivers working with all system peripherals
    boost::asio::io_context io_context;
    auto logger = core::Logger(core::LogLevel::INFO);
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> rx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> tx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> eth_tx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> live_telem_queue;

    std::vector<core::common::Configurable *> configurable_components;

    // transmit queue for data going from components to the CAN driver



    // vector of pointers to configurable components to be given to the param handler. 
    // these are the pointers to components that can be configured
    
    std::string param_path;    
    std::optional<std::string> dbc_path = std::nullopt;
    // program option parsing 
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "produce help message")
        ("param-path,p", po::value<std::string>(&param_path)->default_value("config/drivebrain_config.json"), "Path to the parameter JSON file")
        ("dbc-path,d", po::value<std::string>(), "Path to the DBC file (optional)");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help")) {
        // std::cout << desc << "\n";
        std::stringstream ss;
	ss << desc;
        spdlog::info("{}", ss.str());
        return 1;
    }
    if (vm.count("dbc-path")) {
        dbc_path = vm["dbc-path"].as<std::string>();
    }

    // config file handler that gets given to all configurable components.
    core::JsonFileHandler config(param_path);

    auto mcap_logger = common::MCAPProtobufLogger("temp");

    control::SimpleController controller(logger, config);
    configurable_components.push_back(&controller);
    bool construction_failed = false;
    estimation::MatlabMath matlab_math(logger, config, construction_failed);

    if (construction_failed)
    {
        stop_signal.store(true);
    }
    configurable_components.push_back(&matlab_math);

    // live foxglove server server that relies upon having pointers to configurable components for 
    // getting and handling updates of the registered live parameters
    core::FoxgloveWSServer foxglove_server(configurable_components);
    std::function<void(void)> close_file_temp = []() {};
    std::function<void(const std::string &)> open_file_temp = [](const std::string &) {};

    auto message_logger = std::make_shared<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>>(".mcap", true,
                                                                                                        std::bind(&common::MCAPProtobufLogger::log_msg, std::ref(mcap_logger), std::placeholders::_1),
                                                                                                        std::bind(&common::MCAPProtobufLogger::close_current_mcap, std::ref(mcap_logger)),
                                                                                                        std::bind(&common::MCAPProtobufLogger::open_new_mcap, std::ref(mcap_logger), std::placeholders::_1),
                                                                                                        std::bind(&core::FoxgloveWSServer::send_live_telem_msg, std::ref(foxglove_server), std::placeholders::_1));

    core::StateEstimator state_estimator(logger, message_logger, matlab_math);
    comms::CANDriver driver(config, logger, message_logger, tx_queue, io_context, dbc_path, construction_failed, state_estimator);

    // std::cout << "driver init " << driver.init() << std::endl;
    if (construction_failed)
    {
        stop_signal.store(true);
    }

    configurable_components.push_back(&driver);
    
    comms::MCUETHComms eth_driver(logger, eth_tx_queue, message_logger, state_estimator, io_context, "192.168.1.30", 2001, 2000);
    comms::VNDriver vn_driver(config, logger, message_logger, state_estimator, io_context);

    // required init, maybe want to call this in the constructor instead
    bool successful_controller_init = controller.init();

    DBInterfaceImpl db_service_inst(message_logger);
    std::thread db_service_thread([&db_service_inst]()
                                  {
            // std::cout <<"started db service thread" <<std::endl;
            spdlog::info("started db service thread");

            try {
                while (!stop_signal.load()) {
                    // Run the io_context as long as stop_signal is false
                    db_service_inst.run_server();  // Run at least one handler, or return immediately if none
                }
            } catch (const std::exception& e) {
                //std::cerr << "Error in drivebrain service thread: " << e.what() << std::endl;
                spdlog::error("Error in drivebrain service thread: {}", e.what());
            }
        }); 
    // what we will do here is have a temporary super-loop.
    // in this thread we will block on having anything in the rx queue, everything by default goes into the foxglove server (TODO)
    // if we receive the pedals message, we step the controller and get its output to put intot he tx queue
    std::thread io_context_thread([&io_context]()
                                  {
            // std::cout <<"started io context thread" <<std::endl;
            spdlog::info("Started io context thread");
            try {
                io_context.run();
            } catch (const std::exception& e) {
                //std::cerr << "Error in io_context: " << e.what() << std::endl;
                spdlog::error("Error in io_context: {}", e.what());
            } });

    std::thread process_thread([&rx_queue, &eth_tx_queue, &controller, &state_estimator]()
                               {
        auto out_msg = std::make_shared<hytech_msgs::MCUCommandData>();

        auto loop_time = controller.get_dt_sec();
        auto loop_time_micros = (int)(loop_time * 1000000.0f);
        std::chrono::microseconds loop_chrono_time(loop_time_micros);
        while(!stop_signal.load())
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            // samples internal state set by the handle recv functions
            
            auto state_and_validity = state_estimator.get_latest_state_and_validity();
            auto out_struct = controller.step_controller(state_and_validity.first);
            auto temp_desired_torques = state_and_validity.first.matlab_math_temp_out;
            // feedback
            state_estimator.set_previous_control_output(out_struct);
            // output
            // if(state_and_validity.second)
            // {
            out_msg->set_prev_mcu_recv_millis(out_struct.mcu_recv_millis);

            if(temp_desired_torques.res_torque_lim_nm.FL < 0)
            {
                out_msg->mutable_desired_rpms()->set_fl(0);
            } else {
                out_msg->mutable_desired_rpms()->set_fl(out_struct.desired_rpms.FL);
            }

            if(temp_desired_torques.res_torque_lim_nm.FR < 0)
            {
                out_msg->mutable_desired_rpms()->set_fr(0);
            } else {
                out_msg->mutable_desired_rpms()->set_fr(out_struct.desired_rpms.FR);
            }

            if(temp_desired_torques.res_torque_lim_nm.RL < 0)
            {
                out_msg->mutable_desired_rpms()->set_rl(0);
            } else {
                out_msg->mutable_desired_rpms()->set_rl(out_struct.desired_rpms.RL);
            }

            if(temp_desired_torques.res_torque_lim_nm.RR < 0)
            {
                out_msg->mutable_desired_rpms()->set_rr(0);
            } else {
                out_msg->mutable_desired_rpms()->set_rr(out_struct.desired_rpms.RR);
            }
            

            out_msg->mutable_torque_limit_nm()->set_fl(::abs(temp_desired_torques.res_torque_lim_nm.FL));
            out_msg->mutable_torque_limit_nm()->set_fr(::abs(temp_desired_torques.res_torque_lim_nm.FR));
            out_msg->mutable_torque_limit_nm()->set_rl(::abs(temp_desired_torques.res_torque_lim_nm.RL));
            out_msg->mutable_torque_limit_nm()->set_rr(::abs(temp_desired_torques.res_torque_lim_nm.RR));

            {
                std::unique_lock lk(eth_tx_queue.mtx);
                eth_tx_queue.deque.push_back(out_msg);
                eth_tx_queue.cv.notify_all();
            }

            auto end_time = std::chrono::high_resolution_clock::now();

            auto elapsed = 
                std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            if(loop_chrono_time > elapsed)
            {
                std::this_thread::sleep_for(loop_chrono_time - elapsed);
            } else {
                // std::cout <<"WARNING: missed timing" <<std::endl;
            }
            
            
            
        } });

    std::signal(SIGINT, signalHandler);

    while (!stop_signal.load())
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    process_thread.join();
    // std::cout << "joined main process" << std::endl;
    spdlog::info("joined main process");
    io_context.stop();
    io_context_thread.join();
    db_service_inst.stop_server();
    db_service_thread.join();
    // std::cout << "joined io context" << std::endl;
    spdlog::info("joined io context");
    return 0;
}
