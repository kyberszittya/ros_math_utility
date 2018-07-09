#include "../include/catmull_ros/catmull.hpp"

#include <chrono>
#include <random>
#include <gtest/gtest.h>
#include <cmath>

using namespace catmull_ros;

class VectorAdditionParameter: public testing::TestWithParam<
    std::tr1::tuple<double, double, double, double, double, double>>
{

};

TEST(VectorAdditionBasic, VectorOperationTestAddition)
{
    Vector3 v0(0.0, 0.0, 0.0);    
    Vector3 v1(0.0, 0.0, 0.0);
    auto start = std::chrono::system_clock::now();
    Vector3 res = v0+v1;
    auto end = std::chrono::system_clock::now();
    ASSERT_EQ(0.0, res.x);
    ASSERT_EQ(0.0, res.y);
    ASSERT_EQ(0.0, res.z);
    std::cout << "Total time spent: " << (end-start).count()/10e9 << std::endl;
}

TEST_P(VectorAdditionParameter, VectorAdditionP)
{
    Vector3 v0(
        std::tr1::get<0>(GetParam()), 
        std::tr1::get<1>(GetParam()), 
        std::tr1::get<2>(GetParam())
    );
    Vector3 v1(
        std::tr1::get<3>(GetParam()), 
        std::tr1::get<4>(GetParam()), 
        std::tr1::get<5>(GetParam())
    );
    auto start = std::chrono::system_clock::now();
    Vector3 res = v0+v1;
    auto end = std::chrono::system_clock::now();
    ASSERT_EQ(
        std::tr1::get<0>(GetParam())+std::tr1::get<3>(GetParam()), res.x);
    ASSERT_EQ(
        std::tr1::get<1>(GetParam())+std::tr1::get<4>(GetParam()), res.y);
    ASSERT_EQ(
        std::tr1::get<2>(GetParam())+std::tr1::get<5>(GetParam()), res.z);
    ASSERT_LE((end-start).count()/10e9,2e-8);
    //std::cout << "Total time spent: " << (end-start).count()/10e9 << std::endl;
}

INSTANTIATE_TEST_CASE_P(VectorAdditionSuite, VectorAdditionParameter,
        testing::Combine(
            testing::Range<double>(-10, 10, 10),
            testing::Range<double>(-10, 10, 10),
            testing::Range<double>(-10, 10, 10),
            testing::Range<double>(-10, 10, 10),
            testing::Range<double>(-10, 10, 10),
            testing::Range<double>(-10, 10, 10)
        )
);

double dRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

TEST(VectorAdditionBasic, VectorOperationTestAdditionRandom)
{
    std::random_device r;
    std::default_random_engine e1(r());
    std::uniform_real_distribution<double> uniform_dist(-100, 100);
    int mean = uniform_dist(e1);
    std::seed_seq seed2{r(), r(), r(), r(), r(), r(), r(), r()}; 
    std::mt19937 e2(seed2);
    std::normal_distribution<> normal_dist(mean, 2);
    int steps = 100;
    for (int n = 0; n < steps; n++)
    {
        const double x0 = normal_dist(e2);
        const double x1 = normal_dist(e2);
        const double y0 = normal_dist(e2);
        const double y1 = normal_dist(e2);
        const double z0 = normal_dist(e2);
        const double z1 = normal_dist(e2);
        Vector3 v0(x0, y0, z0);
        Vector3 v1(x1, y1, z1);
        auto start = std::chrono::system_clock::now();
        Vector3 res = v0+v1;
        auto end = std::chrono::system_clock::now();
        ASSERT_EQ(x0 + x1, res.x);
        ASSERT_EQ(y0 + y1, res.y);
        ASSERT_EQ(z0 + z1, res.z);
    }
    
    //std::cout << "Total time spent: " << (end-start).count()/10e9 << std::endl;
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}