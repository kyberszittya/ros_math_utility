/*
* Testing the Catmull-Rom functions
*/
#include "../include/catmull_ros/catmull.hpp"

#include <gtest/gtest.h>

using namespace catmull_ros;


TEST(CatmullRomBasic, HermiteInterpolationTestTrivial)
{
    
    Vector3 p(0.0, 0.0, 0.0);
    double t = 1.0;
    
    ControlVertex cv0(p,t);
    
    cv0.InitHermite();
    
    ASSERT_DOUBLE_EQ(0.0, cv0.A0().X());
    ASSERT_DOUBLE_EQ(0.0, cv0.A0().Y());
    ASSERT_DOUBLE_EQ(0.0, cv0.A0().Z());
    ASSERT_DOUBLE_EQ(0.0, cv0.A1().X());
    ASSERT_DOUBLE_EQ(0.0, cv0.A1().Y());
    ASSERT_DOUBLE_EQ(0.0, cv0.A1().Z());
    ASSERT_DOUBLE_EQ(0.0, cv0.A2().X());
    ASSERT_DOUBLE_EQ(0.0, cv0.A2().Y());
    ASSERT_DOUBLE_EQ(0.0, cv0.A2().Z());
    ASSERT_DOUBLE_EQ(0.0, cv0.A3().X());
    ASSERT_DOUBLE_EQ(0.0, cv0.A2().Y());
    ASSERT_DOUBLE_EQ(0.0, cv0.A2().Z());
    Vector3 hermite_res = cv0.Hermite(2.0);
    ASSERT_DOUBLE_EQ(0.0, hermite_res.X());
    ASSERT_DOUBLE_EQ(0.0, hermite_res.Y());
    ASSERT_DOUBLE_EQ(0.0, hermite_res.Z());
}

TEST(CatmullRomBasic, SinglePointSpline)
{
    Vector3 p0(0.0, 0.0, 0.0);
    CatmullSpline cspline;
    cspline.AddControlVertex(p0);
    cspline.Construct();
    ASSERT_EQ(0.0,cspline.GetControlVertex(0)->A0().GetSquaredNorm());
    ASSERT_EQ(0.0,cspline.GetControlVertex(0)->A1().GetSquaredNorm());
    ASSERT_EQ(0.0,cspline.GetControlVertex(0)->A2().GetSquaredNorm());
    ASSERT_EQ(0.0,cspline.GetControlVertex(0)->A3().GetSquaredNorm());
}

TEST(CatmullRomBasic, SinglePointSpline0)
{
    Vector3 p0(1.0, 1.0, 1.0);
    CatmullSpline cspline;
    cspline.AddControlVertex(p0);
    cspline.Construct();
    ASSERT_DOUBLE_EQ(3.0,cspline.GetControlVertex(0)->A0().GetSquaredNorm());
    ASSERT_DOUBLE_EQ(0.0,cspline.GetControlVertex(0)->A1().GetSquaredNorm());
    ASSERT_DOUBLE_EQ(0.0,cspline.GetControlVertex(0)->A2().GetSquaredNorm());
    ASSERT_DOUBLE_EQ(0.0,cspline.GetControlVertex(0)->A3().GetSquaredNorm());
    ASSERT_DOUBLE_EQ(3.0, cspline.r(0.0).GetSquaredNorm());
    ASSERT_DOUBLE_EQ(0.0, cspline.r(90.0).GetSquaredNorm());
}

TEST(CatmullRomBasic, TwoPointFunction)
{
    Vector3 p0(0.0, 0.0, 0.0);
    Vector3 p1(1.0, 1.0, 1.0);
    CatmullSpline cspline;
    cspline.AddControlVertex(p0);
    cspline.AddControlVertex(p1);
    cspline.Construct();
    ASSERT_DOUBLE_EQ(0.0, cspline.GetControlVertex(0)->T());
    ASSERT_DOUBLE_EQ(sqrt(3), cspline.GetControlVertex(1)->T());
    ASSERT_DOUBLE_EQ(0.0, cspline.r(0).X());
    ASSERT_DOUBLE_EQ(0.0, cspline.r(0).Y());
    ASSERT_DOUBLE_EQ(0.0, cspline.r(0).Z());
    
    ASSERT_DOUBLE_EQ(1.0, cspline.r(sqrt(3)).X());
    ASSERT_DOUBLE_EQ(1.0, cspline.r(sqrt(3)).Y());
    ASSERT_DOUBLE_EQ(1.0, cspline.r(sqrt(3)).Z());
}

TEST(CatmullRomBasic, ThreePointFunction)
{
    Vector3 p0(0.0, 0.0, 0.0);
    Vector3 p1(1.0, 1.0, 1.0);
    Vector3 p2(3.0, 4.0, 5.0);
    CatmullSpline cspline;
    cspline.AddControlVertex(p0);
    cspline.AddControlVertex(p1);
    cspline.AddControlVertex(p2);
    cspline.Construct();
    ASSERT_DOUBLE_EQ(0.0, cspline.GetControlVertex(0)->T());
    ASSERT_DOUBLE_EQ(0.0, cspline.r(0).X());
    ASSERT_DOUBLE_EQ(0.0, cspline.r(0).Y());
    ASSERT_DOUBLE_EQ(0.0, cspline.r(0).Z());    
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}