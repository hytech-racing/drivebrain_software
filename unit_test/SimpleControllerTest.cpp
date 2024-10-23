#include <gtest/gtest.h>
#include <SimpleController.hpp>
#include <VehicleDataTypes.hpp>
#include <JsonFileHandler.hpp>
#include <Logger.hpp>
#include <Utils.hpp>

class SimpleControllerTest : public testing::Test {

    protected:
        core::Logger logger;
        core::JsonFileHandler config;
        control::SimpleController simple_controller;

        SimpleControllerTest()
            : logger(core::LogLevel::INFO), 
            config("../config/test_config/can_driver.json"), // TODO probably want a better way to get param path
            simple_controller(logger, config) {
        }

        void SetUp() override {
            simple_controller.init();
        }

        void TearDown() override {
            // Shouldn't need anything here
        }

};

TEST_F(SimpleControllerTest, MaxRPM) {
    core::VehicleState in;
    auto res = simple_controller.step_controller(in);
    ASSERT_LT(res.desired_rpms.FL, 20000);
}
