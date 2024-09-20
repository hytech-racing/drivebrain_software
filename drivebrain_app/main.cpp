#include <JsonFileHandler.hpp>
#include <CANComms.hpp>
#include <SimpleController.hpp>
#include <StateEstimator.hpp>
#include <MCUETHComms.hpp>

#include <DrivebrainBase.hpp>
#include <foxglove_server.hpp>
#include <array>

#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::seconds
#include <condition_variable>

#include <cassert>

#include <boost/asio.hpp>
#include <memory>
#include <optional>
// TODO first application will have

// - [x] message queue that can send messages between the CAN driver and the controller
// - [x] CAN driver that can receive the pedals messages
// - [ ] fix the CAN messages that cant currently be encoded into the protobuf messages
// - [x] simple controller

int main(int argc, char *argv[])
{

    boost::asio::io_context io_context;
    auto logger = core::Logger(core::LogLevel::INFO);
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> rx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> tx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> eth_tx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> live_telem_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> mcap_log_queue;


    std::vector<core::common::Configurable *> configurable_components;

    std::string param_path = "config/test_config/can_driver.json";
    std::optional<std::string> dbc_path = std::nullopt;
    if (argc == 3)
    {
        param_path = argv[1];
        dbc_path = argv[2];
    }
    core::JsonFileHandler config(param_path);
    comms::CANDriver driver(config, logger, tx_queue, rx_queue, io_context, dbc_path);

    core::StateEstimator state_estimator(logger);
    comms::MCUETHComms eth_driver(logger, eth_tx_queue, live_telem_queue, mcap_log_queue, state_estimator, io_context, "192.168.1.30", 2001, 2000);

    std::cout << "driver init " << driver.init() << std::endl;
    configurable_components.push_back(&driver);

    control::SimpleController controller(logger, config);
    configurable_components.push_back(&controller);

    auto foxglove_server = core::FoxgloveWSServer(configurable_components, live_telem_queue);

    auto _ = controller.init();

    // what we will do here is have a temporary super-loop.
    // in this thread we will block on having anything in the rx queue, everything by default goes into the foxglove server (TODO)
    // if we receive the pedals message, we step the controller and get its output to put intot he tx queue
    std::thread io_context_thread([&io_context]()
                                  {
        std::cout <<"started io context thread" <<std::endl;
        io_context.run(); });

    std::thread process_thread([&rx_queue, &eth_tx_queue, &controller, &state_estimator]()
                               {
        auto out_msg = std::make_shared<hytech_msgs::MCUCommandData>();

        auto loop_time = controller.get_dt_sec();
        auto loop_time_micros = (int)(loop_time * 1000000.0f);
        std::chrono::microseconds loop_chrono_time(loop_time_micros);
        while(true)
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            auto state_and_validity = state_estimator.get_latest_state_and_validity();
            if(state_and_validity.second)
            {
                // std::cout << state_and_validity.first.input.requested_accel <<std::endl;
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

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
