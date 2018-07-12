/**
 * Testing mathematical functions 
 * 
*/
#include "../include/catmull_ros/math_simple.hpp"

#include <gtest/gtest.h>

using namespace ros_math_simple;

TEST(CommonMathTest, clampSimpleTest0)
{
    ASSERT_EQ(0.5, clamp(0.5, -1, 1));
}

TEST(CommonMathTest, clampSimpleTest1)
{
    ASSERT_EQ(2.5, clamp(2.5, -1, 3));
}

TEST(CommonMathTest, clampMinTest0)
{
    ASSERT_EQ(-1, clamp(-1.5, -1, 1));
}

TEST(CommonMathTest, clampMinTest1)
{
    ASSERT_EQ(-1.5, clamp(-1.5, -2.5, 1));
}

TEST(CommonMathTest, clampMaxTest1)
{
    ASSERT_EQ(1, clamp(2, -2.5, 1));
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}