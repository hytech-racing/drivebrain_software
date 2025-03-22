#include <gtest/gtest.h>

#include "DriveBrainApp.hpp"

class DrivebrainAppTest : public testing::Test {

  protected:
    core::Logger logger;
    core::JsonFileHandler config;
    DriveBrainApp app;
    DrivebrainAppTest()
        : logger(core::LogLevel::INFO),
          config("../config/test_config/can_driver.json"), // TODO probably want a better way to get
                                                           // param path
          app("../config/drivebrain_config.json", "../config/hytech.dbc",
              {.run_db_service = true,
               .run_io_context = true,
               .run_process_loop = true,
               .use_vectornav = false}) {

        std::cout << "constructed" << std::endl;
    }

    void SetUp() override {
        // simple_controller.init();
    }

    void TearDown() override {
        // Shouldn't need anything here
    }
};

TEST_F(DrivebrainAppTest, construction) {
    ASSERT_TRUE(true);
}
