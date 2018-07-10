#include "../include/catmull_ros/catmull.hpp"

#include <chrono>
#include <gtest/gtest.h>

using namespace catmull_ros;


TEST(CatmullRomBasic, HermiteInterpolationTestTrivial)
{
    Vector3 p(0.0, 0.0, 0.0);
    double t = 1.0;
    ControlVertex cv0(p,t);
    cv0.initHermite();
    
    ASSERT_DOUBLE_EQ(0.0, cv0.getA0().x);
    ASSERT_DOUBLE_EQ(0.0, cv0.getA0().y);
    ASSERT_DOUBLE_EQ(0.0, cv0.getA0().z);
    ASSERT_DOUBLE_EQ(0.0, cv0.getA1().x);
    ASSERT_DOUBLE_EQ(0.0, cv0.getA1().y);
    ASSERT_DOUBLE_EQ(0.0, cv0.getA1().z);
    ASSERT_DOUBLE_EQ(0.0, cv0.getA2().x);
    ASSERT_DOUBLE_EQ(0.0, cv0.getA2().y);
    ASSERT_DOUBLE_EQ(0.0, cv0.getA2().z);
    ASSERT_DOUBLE_EQ(0.0, cv0.getA3().x);
    ASSERT_DOUBLE_EQ(0.0, cv0.getA2().y);
    ASSERT_DOUBLE_EQ(0.0, cv0.getA2().z);
}


TEST(CatmullRomBasic, LinearFunction)
{
    
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}