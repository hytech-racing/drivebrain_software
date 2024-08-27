#include <JsonFileHandler.hpp>
#include <CANComms.hpp>
#include <SimpleController.hpp>

#include <DrivebrainBase.hpp>

#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::seconds
#include <condition_variable>

#include <cassert>

#define assertm(exp, msg) assert(((void)msg, exp))

#include <boost/asio.hpp>
#include <memory>
// TODO first application will have

// - [ ] driver bus message queue https://chatgpt.com/share/03297278-0346-40ba-8ce7-7a75e919ee5c
// - [ ] CAN driver
// - [ ] simple controller

// - [ ] message queue manager for ensuring that everything is getting the data it needs
// - [ ] foxglove live telem and parameter server

int main()
{
    boost::asio::io_context io_context;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> rx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> tx_queue;
    core::JsonFileHandler config("config/test_config/can_driver.json");
    comms::CANDriver driver(config, tx_queue, rx_queue, io_context);

    assertm(driver.init(), "ERROR: driver did not initialize");

    control::SimpleController controller(config);
    assertm(controller.init(), "ERROR: controler did not initialize");
    // what we will do here is have a temporary super-loop.
    // in this thread we will block on having anything in the rx queue, everything by default goes into the foxglove server (TODO)
    // if we receive the pedals message, we step the controller and get its output to put intot he tx queue
    std::thread io_context_thread([&io_context]()
                                  { io_context.run(); });
    std::thread receive_thread([&rx_queue, &tx_queue, &controller]()
                               {

        
        while(true)
        {
            std::shared_ptr<google::protobuf::Message> input_msg;
            {
                std::unique_lock lk(rx_queue.mtx);
                rx_queue.cv.wait(lk, [&rx_queue]()
                                        { return !rx_queue.deque.empty(); });
                
                auto m = rx_queue.deque.back();
                input_msg = m; 
                rx_queue.deque.pop_back();
            }
            if(input_msg->GetTypeName() == "mcu_pedal_readings")
            {
                auto in_msg = std::static_pointer_cast<mcu_pedal_readings>(input_msg);
                auto to_send = std::make_shared<drivetrain_command>();
                to_send->CopyFrom(controller.step_controller(*in_msg));

                {
                    std::unique_lock lk(tx_queue.mtx);
                    tx_queue.deque.push_back(to_send);
                    tx_queue.cv.notify_all();
                }

            } else {
                std::cout << input_msg->GetTypeName() <<std::endl;
            }
            
        } });

    // io_context.run();
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
