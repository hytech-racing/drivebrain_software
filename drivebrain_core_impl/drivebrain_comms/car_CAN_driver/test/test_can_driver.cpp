#include <gtest/gtest.h>

#include <CANComms.hpp>
#include <JsonFileHandler.hpp>
#include <hytech.pb.h>
#include <variant>
#include <memory>



void print_can_frame(const struct can_frame &frame) {
    // Print the CAN ID
    printf("CAN ID: 0x%08X ", frame.can_id & CAN_EFF_MASK);

    // Print frame flags
    if (frame.can_id & CAN_EFF_FLAG) {
        printf("(Extended Frame Format) ");
    } else {
        printf("(Standard Frame Format) ");
    }

    if (frame.can_id & CAN_RTR_FLAG) {
        printf("(Remote Transmission Request) ");
    }

    if (frame.can_id & CAN_ERR_FLAG) {
        printf("(Error Frame) ");
    }

    printf("\n");

    // Print Data Length Code (DLC)
    printf("DLC: %d\n", frame.can_dlc);

    // Print data bytes
    printf("Data: ");
    for (int i = 0; i < frame.can_dlc; ++i) {
        printf("%02X ", frame.data[i]);
    }
    printf("\n");
}

TEST(CANDriver, test_construction) {
    core::JsonFileHandler test_file("config/test_config/can_driver.json");
    comms::CANDriver driver(test_file);
    EXPECT_TRUE(driver.init());
}

TEST(CANDriver, test_CAN_creation) {
  core::JsonFileHandler test_file("config/test_config/can_driver.json");
  comms::CANDriver driver(test_file);
  auto _ = driver.init();
  auto ht_pb_test = std::make_shared<drivetrain_rpms_telem>();

  auto motor_rpm = 100;
  ht_pb_test->set_fr_motor_rpm(motor_rpm);
  ht_pb_test->set_fl_motor_rpm(motor_rpm);
  auto res = driver._get_CAN_msg(ht_pb_test);
  EXPECT_TRUE((res.data[0] |= res.data[1])== motor_rpm);
  print_can_frame(res);

  auto ht_pb_bool_test = std::make_shared<drivetrain_status_telem>();
  ht_pb_bool_test->set_mc1_dc_on(true);
  auto res2 = driver._get_CAN_msg(ht_pb_bool_test);
  print_can_frame(res2);
}

TEST(CANDriver, test_CAN_recv)
{
    core::JsonFileHandler test_file("config/test_config/can_driver.json");
    comms::CANDriver driver(test_file);
    auto _ = driver.init();
    auto ht_pb_test = std::make_shared<drivetrain_rpms_telem>();

    auto motor_rpm = 100;
    ht_pb_test->set_fr_motor_rpm(motor_rpm);
    ht_pb_test->set_fl_motor_rpm(motor_rpm);
    auto res = driver._get_CAN_msg(ht_pb_test);
    EXPECT_TRUE((res.data[0] |= res.data[1])== motor_rpm);
    
    auto msg_res = driver.pb_msg_recv(res);

    auto msg_res_cast = std::dynamic_pointer_cast<drivetrain_rpms_telem>(msg_res);
    EXPECT_EQ(msg_res_cast->fl_motor_rpm(), 100);
}

// TEST(CANDriver, )