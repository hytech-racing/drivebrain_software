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

#include <boost/asio.hpp>
#include <memory>
#include <optional>

#include <mcap/mcap.hpp>
#include <thread> // For std::this_thread::sleep_for
#include <chrono> // For std::chrono::seconds

#include <csignal>
#include <cstdlib>

// TODO first application will have

// - [x] message queue that can send messages between the CAN driver and the controller
// - [x] CAN driver that can receive the pedals messages
// - [ ] fix the CAN messages that cant currently be encoded into the protobuf messages
// - [x] simple controller

std::atomic<bool> stop_signal{false};
// Signal handler function
void signalHandler(int signal)
{
    std::cout << "Interrupt signal (" << signal << ") received. Cleaning up..." << std::endl;
    stop_signal.store(true); // Set running to false to exit the main loop or gracefully terminate
}

int main(int argc, char *argv[])
{

    boost::asio::io_context io_context;
    auto logger = core::Logger(core::LogLevel::INFO);
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> rx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> tx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> eth_tx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> live_telem_queue;

    std::vector<core::common::Configurable *> configurable_components;

    std::string param_path = "config/test_config/can_driver.json";
    std::optional<std::string> dbc_path = std::nullopt;
    if (argc == 3)
    {
        param_path = argv[1];
        dbc_path = argv[2];
    }

    core::JsonFileHandler config(param_path);

    auto mcap_logger = common::MCAPProtobufLogger("temp");

    core::StateEstimator state_estimator(logger);

    

    control::SimpleController controller(logger, config);
    configurable_components.push_back(&controller);

    auto foxglove_server = core::FoxgloveWSServer(configurable_components);

    std::function<void(void)> close_file_temp = []() {};
    std::function<void(const std::string &)> open_file_temp = [](const std::string &) {};

    auto message_logger = std::make_shared<core::MsgLogger<std::shared_ptr<google::protobuf::Message>>>(".mcap", true,
                                                                                                        std::bind(&common::MCAPProtobufLogger::log_msg, std::ref(mcap_logger), std::placeholders::_1),
                                                                                                        std::bind(&common::MCAPProtobufLogger::close_current_mcap, std::ref(mcap_logger)),
                                                                                                        std::bind(&common::MCAPProtobufLogger::open_new_mcap, std::ref(mcap_logger), std::placeholders::_1),
                                                                                                        std::bind(&core::FoxgloveWSServer::send_live_telem_msg, std::ref(foxglove_server), std::placeholders::_1));
    comms::CANDriver driver(config, logger, message_logger, tx_queue, rx_queue, io_context, dbc_path);
    std::cout << "driver init " << driver.init() << std::endl;
    configurable_components.push_back(&driver);
    comms::MCUETHComms eth_driver(logger, eth_tx_queue, message_logger, state_estimator, io_context, "192.168.1.30", 2001, 2000);
    comms::VNDriver vn_driver(config, logger, message_logger, state_estimator);

    auto _ = controller.init();

    DBInterfaceImpl db_service_inst(message_logger);
    std::thread db_service_thread([&db_service_inst]()
        {
            std::cout <<"started io context thread" <<std::endl;
            try {
                while (!stop_signal.load()) {
                    // Run the io_context as long as stop_signal is false
                    db_service_inst.run_server();  // Run at least one handler, or return immediately if none
                }
            } catch (const std::exception& e) {
                std::cerr << "Error in io_context: " << e.what() << std::endl;
            }
        }); 
    // what we will do here is have a temporary super-loop.
    // in this thread we will block on having anything in the rx queue, everything by default goes into the foxglove server (TODO)
    // if we receive the pedals message, we step the controller and get its output to put intot he tx queue
    std::thread io_context_thread([&io_context, &stop_signal]()
        {
            std::cout <<"started io context thread" <<std::endl;
            try {
                while (!stop_signal.load()) {
                    // Run the io_context as long as stop_signal is false
                    io_context.run_one();  // Run at least one handler, or return immediately if none
                }
            } catch (const std::exception& e) {
                std::cerr << "Error in io_context: " << e.what() << std::endl;
            }
        });

    std::thread process_thread([&stop_signal, &rx_queue, &eth_tx_queue, &controller, &state_estimator]()
                               {
        auto out_msg = std::make_shared<hytech_msgs::MCUCommandData>();

        auto loop_time = controller.get_dt_sec();
        auto loop_time_micros = (int)(loop_time * 1000000.0f);
        std::chrono::microseconds loop_chrono_time(loop_time_micros);
        while(!stop_signal.load())
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            auto state_and_validity = state_estimator.get_latest_state_and_validity();
            if(state_and_validity.second)
            {
                auto out = controller.step_controller(state_and_validity.first);
                out_msg->CopyFrom(out);
                {
                    std::unique_lock lk(eth_tx_queue.mtx);
                    eth_tx_queue.deque.push_back(out_msg);
                    eth_tx_queue.cv.notify_all();
                }
            } else {
            }
            auto end_time = std::chrono::high_resolution_clock::now();

            auto elapsed = 
                std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

            std::this_thread::sleep_for(loop_chrono_time - elapsed);
            
        } });

    std::signal(SIGINT, signalHandler);

    while (!stop_signal.load())
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    process_thread.join();
    std::cout << "joined main process" << std::endl;
    io_context.stop();
    io_context_thread.join();
    db_service_inst.stop_server();
    db_service_thread.join(); 
    std::cout << "joined io context" << std::endl;
    return 0;
}
