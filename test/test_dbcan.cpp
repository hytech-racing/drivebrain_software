// #include <gtest/gtest.h>

// #include <CANComms.hpp>
// #include <JsonFileHandler.hpp>
// #include <hytech.pb.h>
// #include <variant>
// #include <memory>

// void print_can_frame(const struct can_frame &frame) {
//     // Print the CAN ID
//     printf("CAN ID: 0x%08X ", frame.can_id & CAN_EFF_MASK);

//     // Print frame flags
//     if (frame.can_id & CAN_EFF_FLAG) {
//         printf("(Extended Frame Format) ");
//     } else {
//         printf("(Standard Frame Format) ");
//     }

//     if (frame.can_id & CAN_RTR_FLAG) {
//         printf("(Remote Transmission Request) ");
//     }

//     if (frame.can_id & CAN_ERR_FLAG) {
//         printf("(Error Frame) ");
//     }

//     printf("\n");

//     // Print Data Length Code (DLC)
//     printf("DLC: %d\n", frame.can_dlc);

//     // Print data bytes
//     printf("Data: ");
//     for (int i = 0; i < frame.can_dlc; ++i) {
//         printf("%02X ", frame.data[i]);
//     }
//     printf("\n");
// }


// int main()
// {
//     bool construction_failed = false;
//     core::common::ThreadSafeDeque<std::shared_ptr<google::protobuf::Message>> _can_tx_queue;

//     std::unique_ptr<comms::CANDriver> _driver = std::make_unique<comms::CANDriver>(
//         _config, _logger, _message_logger,_can_tx_queue, _io_context, 
//         _dbc_path, construction_failed, *_state_estimator);
    
//     if (construction_failed) {
//         throw std::runtime_error("Failed to construct CAN driver");
//     }
// // }