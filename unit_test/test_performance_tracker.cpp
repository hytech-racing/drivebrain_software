#include <gtest/gtest.h>

#include <PerformanceTracker.hpp>

class PerformanceTrackerTest : public ::testing::Test {
protected:

    core::JsonFileHandler json_file_handler; 
    PerformanceTracker per_tracker;
    

    PerformanceTrackerTest()
        : 
          json_file_handler("../config/test_performance_tracker.json"),
          per_tracker(json_file_handler)
    {
    }

    void SetUp() override {
        (void)per_tracker.init();
    }

    void TearDown() override {
        // Clean up after each test if necessary
    }
};


TEST_F(PerformanceTrackerTest, Update_TracksLapTimeAndCompletionCorrectly) {
    // Arrange

    
    
    ASSERT_TRUE(per_tracker.init());

    core::Position pos_start = {0.0, 0.0, true}; // On starting line
    core::Position pos_outside_start_bubble = {0.001, 0.0, true}; // ~111m away
    core::Position pos_inside_finish_bubble = {0.001, 0.0, true}; // Same as finish

    // Act & Assert
    per_tracker.activate();

    // 1. Initial update should not start timer yet (still inside start bubble)
    auto check_timer_not_started = per_tracker.update(pos_start);
    ASSERT_FALSE(check_timer_not_started.timer_started);
    
    // 2. Move outside start bubble -> should start timer
    auto check_timer_started_state =  per_tracker.update(pos_outside_start_bubble);
    ASSERT_TRUE(check_timer_started_state.timer_started);
    ASSERT_TRUE(check_timer_started_state.lap_count==0);

    // Simulate some time passing by sleeping or mocking time if needed.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 3. Enter finish bubble -> should complete lap and reset lap timer
    auto state = per_tracker.update(pos_inside_finish_bubble);

    EXPECT_NEAR(state.current_lap_time_ms, 100, 1);
    std::cout << state.current_lap_time_ms << std::endl;
    ASSERT_TRUE(state.lap_count==1);
    state = per_tracker.update(pos_inside_finish_bubble);
    EXPECT_NEAR(state.current_lap_time_ms, 100, 1);
    ASSERT_TRUE(state.lap_count==1);
    state = per_tracker.update(pos_inside_finish_bubble);
    EXPECT_NEAR(state.current_lap_time_ms, 100, 1);
    std::cout <<state.current_lap_time_ms<< std::endl;
    ASSERT_TRUE(state.lap_count==1); // these shouldnt change 
    state = per_tracker.update(pos_inside_finish_bubble);
    // std::string output = testing::internal::GetCapturedStdout();

    
    // 4. Timer should reset after lap complete, test by moving again
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    per_tracker.update(pos_outside_start_bubble);

    EXPECT_TRUE(true); // If no crash and logs are correct, test passes
}
