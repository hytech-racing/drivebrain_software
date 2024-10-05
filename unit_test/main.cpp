#include <gtest/gtest.h>

// Random test code 
int Add(int a, int b) {
    return a + b;
}

TEST(AddTest, PositiveNumbers) {
    EXPECT_EQ(Add(2, 3), 5);
}

TEST(AddTest, NegativeNumbers) {
    EXPECT_EQ(Add(-2, -3), -5);
}

TEST(AddTest, MixedNumbers) {
    EXPECT_EQ(Add(-2, 3), 1);
}

// Main function for Google Test
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}