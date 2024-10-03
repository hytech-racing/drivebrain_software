#include <JsonFileHandler.hpp>
#include <CANComms.hpp>
#include <SimpleController.hpp>
#include <StateEstimator.hpp>
#include <MCUETHComms.hpp>

#include <DrivebrainBase.hpp>
#include <param_server.hpp>
#include <array>

#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::seconds
#include <condition_variable>

#include <cassert>

#include <boost/program_options.hpp>
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

    // io context for boost async io. gets given to the drivers working with all system peripherals
    boost::asio::io_context io_context;
    auto logger = core::Logger(core::LogLevel::INFO);
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> rx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> tx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> eth_tx_queue;

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
        ("param-path,p", po::value<std::string>(&param_path)->default_value("config/test_config/can_driver.json"), "Path to the parameter JSON file")
        ("dbc-path,d", po::value<std::string>(), "Path to the DBC file (optional)");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    
    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }
    if (vm.count("dbc-path")) {
        dbc_path = vm["dbc-path"].as<std::string>();
    }

    // config file handler that gets given to all configurable components.
    core::JsonFileHandler config(param_path);
    comms::CANDriver driver(config, logger, tx_queue, rx_queue, io_context, dbc_path);

    core::StateEstimator state_estimator(config, logger);
    comms::MCUETHComms eth_driver(logger, eth_tx_queue, state_estimator, io_context, "192.168.1.30", 2001, 2000);

    std::cout << "driver init " << driver.init() << std::endl;
    configurable_components.push_back(&driver);

    control::SimpleController controller(logger, config);
    configurable_components.push_back(&controller);

    // live parameter server that relies upon having pointers to configurable components for 
    // getting and handling updates of the registered live parameters
    core::FoxgloveParameterServer param_server(configurable_components);

    // required init, maybe want to call this in the constructor instead
    bool successful_controller_init = controller.init();

    // what we will do here is have a temporary super-loop.
    // in this thread we will block on having anything in the rx queue, everything by default goes into the foxglove server (TODO)
    // if we receive the pedals message, we step the controller and get its output to put intot he tx queue
    std::thread io_context_thread([&io_context]()
                                  {
        std::cout <<"started io context thread" <<std::endl;
        io_context.run(); });
    
    // the main loop that handles evaluation of the state estimator to sample the vehicle state and give it to the controller to act upon and 
    // result in a controller output. this controller output is then out into the transmit queue to be sent out. since the CAN driver is threaded the queue
    // is already waiting for the messages to come accross and as soon as they are put into the queue they will send.
    std::thread process_thread([&rx_queue, &eth_tx_queue, &controller, &state_estimator]()
                               {
        auto out_msg = std::make_shared<hytech_msgs::MCUCommandData>();

        auto loop_time = controller.get_dt_sec();
        auto loop_time_micros = (int)(loop_time * 1000000.0f);
        std::chrono::microseconds loop_chrono_time(loop_time_micros);
        while(true)
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            std::pair<core::VehicleState, bool> state_and_validity = state_estimator.get_latest_state_and_validity();
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
            // make sure our loop rate is what we expect it to be
            std::this_thread::sleep_for(loop_chrono_time - elapsed);
            
        } });

    // keep-alive loop for the main thread
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
