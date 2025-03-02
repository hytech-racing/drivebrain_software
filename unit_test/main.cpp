#include <gtest/gtest.h>

#include <gtest/gtest.h>
#include <SimpleController.hpp>
#include <VehicleDataTypes.hpp>
#include <JsonFileHandler.hpp>
#include <Logger.hpp>
#include <Utils.hpp>

// class SimpleControllerTest : public testing::Test {

//     protected:
//         core::Logger logger;
//         core::JsonFileHandler config;
//         std::unique_ptr<control::SimpleController> simple_controller;

//         SimpleControllerTest()
//         :
//         logger(core::LogLevel::INFO),
//         config(std::string("../config/drivebrain_config.json"))
//         {
//             simple_controller = std::make_unique<control::SimpleController>(logger, config);
//         }
        

//         void SetUp() override {
//             ASSERT_NE(simple_controller, nullptr);
//             simple_controller->init();
//         }
// };

TEST(SimpleControllerTest, SimpleAccel) {
    core::Logger logger(core::LogLevel::INFO);
    core::JsonFileHandler config("../config/drivebrain_config.json");
    control::SimpleController simple_controller(logger, config);
    core::VehicleState in{};
    veh_vec<float> current_rpms{}; 

    current_rpms.FL = 100.0f; 
    current_rpms.FR = 100.0f;
    current_rpms.RL = 100.0f;
    current_rpms.RR = 100.0f;

    in.current_rpms = current_rpms;
    in.input.requested_accel = 1.0f;
    in.input.requested_brake = 0.0f;
    in.prev_MCU_recv_millis = 0;

    // Debugging Checks
    ASSERT_TRUE(std::isfinite(in.input.requested_accel));
    ASSERT_TRUE(std::isfinite(in.input.requested_brake));

    auto res = simple_controller.step_controller(in);


    // EXPECT_FLOAT_EQ(res.desired_rpms.FL, 180.0f);
    // ASSERT_EQ(res.desired_rpms.FR, 180);
    // ASSERT_EQ(res.desired_rpms.RL, 180);
    // ASSERT_EQ(res.desired_rpms.RR, 180);
    // ASSERT_EQ(res.torque_lim_nm.FL, 210);
    // ASSERT_EQ(res.torque_lim_nm.FR, 210);
    // ASSERT_EQ(res.torque_lim_nm.RL, 210);
    // ASSERT_EQ(res.torque_lim_nm.RR, 210);

}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}