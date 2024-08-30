#include <JsonFileHandler.hpp>
#include <CANComms.hpp>
#include <SimpleController.hpp>

#include <DrivebrainBase.hpp>
#include <param_server.hpp>

#include <thread> // std::this_thread::sleep_for
#include <chrono> // std::chrono::seconds
#include <condition_variable>

#include <cassert>


#include <boost/asio.hpp>
#include <memory>
// TODO first application will have

// - [x] message queue that can send messages between the CAN driver and the controller
// - [x] CAN driver that can receive the pedals messages
    // - [ ] fix the CAN messages that cant currently be encoded into the protobuf messages
// - [x] simple controller

int main()
{

    boost::asio::io_context io_context;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> rx_queue;
    core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> tx_queue;
    std::vector<core::common::Configurable*> configurable_components;

    
    core::JsonFileHandler config("config/test_config/can_driver.json");
    
    comms::CANDriver driver(config, tx_queue, rx_queue, io_context);
    std::cout << "driver init "<< driver.init() << std::endl;
    configurable_components.push_back(&driver);
    
    control::SimpleController controller(config);
    configurable_components.push_back(&controller);
    
    auto param_server = core::FoxgloveParameterServer(configurable_components);

    auto _ = controller.init();
    // what we will do here is have a temporary super-loop.
    // in this thread we will block on having anything in the rx queue, everything by default goes into the foxglove server (TODO)
    // if we receive the pedals message, we step the controller and get its output to put intot he tx queue
    std::thread io_context_thread([&io_context]()
                                  {
                                    std::cout <<"started io context thread" <<std::endl;
                                    io_context.run(); });
    std::thread receive_thread([&rx_queue, &tx_queue, &controller]()
                               {

        
        while(true)
        {
            // std::cout <<"started recv thread" <<std::endl;
            std::shared_ptr<google::protobuf::Message> input_msg;
            {
                // std::cout <<"waiting on rx"<<std::endl;
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

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
