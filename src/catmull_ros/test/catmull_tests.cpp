#include "../include/catmull_ros/catmull.hpp"

#include <gtest/gtest.h>

using namespace catmull_ros;


TEST(CatmullRomBasic, HermiteInterpolationTestTrivial)
{
    
    Vector3 p(0.0, 0.0, 0.0);
    double t = 1.0;
    
    ControlVertex cv0(p,t);
    
    cv0.InitHermite();
    
    ASSERT_DOUBLE_EQ(0.0, cv0.A0().x);
    ASSERT_DOUBLE_EQ(0.0, cv0.A0().y);
    ASSERT_DOUBLE_EQ(0.0, cv0.A0().z);
    ASSERT_DOUBLE_EQ(0.0, cv0.A1().x);
    ASSERT_DOUBLE_EQ(0.0, cv0.A1().y);
    ASSERT_DOUBLE_EQ(0.0, cv0.A1().z);
    ASSERT_DOUBLE_EQ(0.0, cv0.A2().x);
    ASSERT_DOUBLE_EQ(0.0, cv0.A2().y);
    ASSERT_DOUBLE_EQ(0.0, cv0.A2().z);
    ASSERT_DOUBLE_EQ(0.0, cv0.A3().x);
    ASSERT_DOUBLE_EQ(0.0, cv0.A2().y);
    ASSERT_DOUBLE_EQ(0.0, cv0.A2().z);
    Vector3 hermite_res = cv0.Hermite(2.0);
    ASSERT_DOUBLE_EQ(0.0, hermite_res.x);
    ASSERT_DOUBLE_EQ(0.0, hermite_res.y);
    ASSERT_DOUBLE_EQ(0.0, hermite_res.z);
}


TEST(CatmullRomBasic, TwoPointsCatmull)
{
    
    Vector3 p0(0.0, 0.0, 0.0);
    double t0 = 0.0;
    Vector3 p1(1.0, 1.0, 1.0);
    double t1 = 1.0;
    
    std::shared_ptr<ControlVertex> cv0(new ControlVertex(p0,t0));
    std::shared_ptr<ControlVertex> cv1(new ControlVertex(p1,t1));
    cv0->Initialize(cv0, cv1);
    cv1->InitHermite();
    
    
}

TEST(CatmullRomBasic, LinearFunction)
{
    Vector3 p0(0.0, 0.0, 0.0);
    Vector3 p1(1.0, 1.0, 1.0);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}