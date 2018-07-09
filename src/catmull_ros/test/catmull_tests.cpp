#include "../include/catmull_ros/catmull.hpp"

#include <chrono>
#include <gtest/gtest.h>

using namespace catmull_ros;


TEST(CatmullRomBasic, HermiteInterpolationTests)
{
    Vector3 p(0.0, 0.0, 0.0);
    double t = 1.0;
    ControlVertex cv0(p,t);
    
}


TEST(CatmullRomBasic, LinearFunction)
{
    
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}