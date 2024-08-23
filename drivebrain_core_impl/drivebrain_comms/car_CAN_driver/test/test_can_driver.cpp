#include <gtest/gtest.h>

#include <CANComms.hpp>
#include <JsonFileHandler.hpp>

TEST(CANDriver, test_construction) {
    core::JsonFileHandler test_file("config/test_config/can_driver.json");
    comms::CANDriver driver(test_file);
    EXPECT_TRUE(driver.init());
    auto msg = driver._get_pb_msg_by_name("acu_shunt_measurements");
    std::cout << "Packed Any Message: " << msg.DebugString() << std::endl;

  // Expect two strings not to be equal.
//   EXPECT_STRNE("hello", "world");
  // Expect equality.
//   EXPECT_EQ(7 * 6, 42);

}