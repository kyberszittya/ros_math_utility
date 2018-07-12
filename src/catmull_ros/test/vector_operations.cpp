#include "../include/catmull_ros/catmull.hpp"

#include <chrono>
#include <random>
#include <gtest/gtest.h>
#include <cmath>

using namespace catmull_ros;

const double T_NANOSECS = 1e9;
const double T_OPERATION_THRESHOLD = 3e-7;
const double LIMIT_RANDOM_TEST_MAX = 100;
const double LIMIT_RANDOM_TEST_MIN = -100;
const double LIMIT_TEST_MIN = -10;
const double LIMIT_TEST_MAX = 10;
const double TEST_STEP = 10;

const int RAND_TEST_CYCLES = 100;

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
    ASSERT_EQ(0.0, res.X());
    ASSERT_EQ(0.0, res.Y());
    ASSERT_EQ(0.0, res.Z());
    ASSERT_LE((end-start).count()/T_NANOSECS,T_OPERATION_THRESHOLD);
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
        std::tr1::get<0>(GetParam())+std::tr1::get<3>(GetParam()), res.X());
    ASSERT_EQ(
        std::tr1::get<1>(GetParam())+std::tr1::get<4>(GetParam()), res.Y());
    ASSERT_EQ(
        std::tr1::get<2>(GetParam())+std::tr1::get<5>(GetParam()), res.Z());
    ASSERT_LE((end-start).count()/T_NANOSECS,T_OPERATION_THRESHOLD);
}

INSTANTIATE_TEST_CASE_P(VectorAdditionSuite, VectorAdditionParameter,
        testing::Combine(
            testing::Range<double>(LIMIT_TEST_MIN, LIMIT_TEST_MAX, TEST_STEP),
            testing::Range<double>(LIMIT_TEST_MIN, LIMIT_TEST_MAX, TEST_STEP),
            testing::Range<double>(LIMIT_TEST_MIN, LIMIT_TEST_MAX, TEST_STEP),
            testing::Range<double>(LIMIT_TEST_MIN, LIMIT_TEST_MAX, TEST_STEP),
            testing::Range<double>(LIMIT_TEST_MIN, LIMIT_TEST_MAX, TEST_STEP),
            testing::Range<double>(LIMIT_TEST_MIN, LIMIT_TEST_MAX, TEST_STEP)
        )
);

static const std::vector<double> GenerateTwoVectors()
{
    std::vector<double> nums(6);
    static std::random_device r;
    static std::default_random_engine e1(r());
    static std::uniform_real_distribution<double> uniform_dist(-100, 100);
    int mean = uniform_dist(e1);
    std::seed_seq seed2{r(), r(), r(), r(), r(), r(), r(), r()}; 
    std::mt19937 e2(seed2);
    std::normal_distribution<> normal_dist(mean, 2);

    nums[0] = normal_dist(e2);
    nums[1] = normal_dist(e2);
    nums[2] = normal_dist(e2);
    nums[3] = normal_dist(e2);
    nums[4] = normal_dist(e2);
    nums[5] = normal_dist(e2);

    return nums;
}

static const std::vector<double> GenerateVectorScalar()
{
    std::vector<double> nums(4);
    static std::random_device r;
    static std::default_random_engine e1(r());
    static std::uniform_real_distribution<double> uniform_dist(
        LIMIT_RANDOM_TEST_MIN, LIMIT_RANDOM_TEST_MAX);
    int mean = uniform_dist(e1);
    std::seed_seq seed2{r(), r(), r(), r(), r(), r(), r(), r()}; 
    std::mt19937 e2(seed2);
    std::normal_distribution<> normal_dist(mean, 2);

    nums[0] = normal_dist(e2);
    nums[1] = normal_dist(e2);
    nums[2] = normal_dist(e2);
    nums[3] = normal_dist(e2);

    return nums;
}

TEST(VectorAdditionBasic, VectorOperationTestAdditionRandom)
{    
    for (int n = 0; n < RAND_TEST_CYCLES; n++)
    {
        std::vector<double> nums = GenerateTwoVectors();
        Vector3 v0(nums[0], nums[1], nums[2]);
        Vector3 v1(nums[3], nums[4], nums[5]);
        auto start = std::chrono::system_clock::now();
        Vector3 res = v0 + v1;
        auto end = std::chrono::system_clock::now();
        ASSERT_EQ(nums[0] + nums[3], res.X());
        ASSERT_EQ(nums[1] + nums[4], res.Y());
        ASSERT_EQ(nums[2] + nums[5], res.Z());
        ASSERT_LE((end-start).count()/T_NANOSECS,T_OPERATION_THRESHOLD);
    }
    
    
}

TEST(VectorMultiplyBasic, VectorOperationTestMultiplyRandom)
{
    for (int n = 0; n < RAND_TEST_CYCLES; n++)
    {
        std::vector<double> nums = GenerateTwoVectors();
        Vector3 v0(nums[0], nums[1], nums[2]);
        Vector3 v1(nums[3], nums[4], nums[5]);
        auto start = std::chrono::system_clock::now();
        Vector3 res = v0 * v1;
        auto end = std::chrono::system_clock::now();
        ASSERT_EQ(nums[0] * nums[3], res.X());
        ASSERT_EQ(nums[1] * nums[4], res.Y());
        ASSERT_EQ(nums[2] * nums[5], res.Z());
        ASSERT_LE((end-start).count()/T_NANOSECS,T_OPERATION_THRESHOLD);
    }
    
    
}

TEST(VectorMultiplyBasic, VectorOperationTestMultiplyDoubleRandom2)
{
    
    for (int n = 0; n < RAND_TEST_CYCLES; n++)
    {
        std::vector<double> nums = GenerateVectorScalar();
        Vector3 v0(nums[0], nums[1], nums[2]);
        
        auto start = std::chrono::system_clock::now();
        Vector3 res = nums[3] * v0;
        auto end = std::chrono::system_clock::now();
        ASSERT_EQ(nums[0] * nums[3], res.X());
        ASSERT_EQ(nums[1] * nums[3], res.Y());
        ASSERT_EQ(nums[2] * nums[3], res.Z());
        ASSERT_LE((end-start).count()/T_NANOSECS,T_OPERATION_THRESHOLD);
    }
    
    
}

TEST(VectorSubtractionBasic, VectorOperationTestSubtractionRandom)
{
    for (int n = 0; n < RAND_TEST_CYCLES; n++)
    {
        std::vector<double> nums = GenerateTwoVectors();
        const Vector3 v0(nums[0], nums[1], nums[2]);
        const Vector3 v1(nums[3], nums[4], nums[5]);
        auto start = std::chrono::system_clock::now();
        Vector3 res = v0 - v1;
        auto end = std::chrono::system_clock::now();
        ASSERT_EQ(nums[0] - nums[3], res.X());
        ASSERT_EQ(nums[1] - nums[4], res.Y());
        ASSERT_EQ(nums[2] - nums[5], res.Z());
        ASSERT_LE((end-start).count()/T_NANOSECS,T_OPERATION_THRESHOLD);
    }
    
    
}

TEST(VectorMultiplyBasic, VectorOperationTestMultiplyDoubleRandom)
{
    for (int n = 0; n < RAND_TEST_CYCLES; n++)
    {
        std::vector<double> nums = GenerateVectorScalar();
        Vector3 v0(nums[0], nums[1], nums[2]);
        auto start = std::chrono::system_clock::now();
        Vector3 res = v0 * nums[3];
        auto end = std::chrono::system_clock::now();
        ASSERT_EQ(nums[0] * nums[3], res.X());
        ASSERT_EQ(nums[1] * nums[3], res.Y());
        ASSERT_EQ(nums[2] * nums[3], res.Z());
        ASSERT_LE((end-start).count()/T_NANOSECS,T_OPERATION_THRESHOLD);
    }
    
    
}

TEST(VectorDivideBasic, VectorOperationTestDivideDoubleRandom)
{
    for (int n = 0; n < RAND_TEST_CYCLES; n++)
    {
        std::vector<double> nums = GenerateVectorScalar();
        Vector3 v0(nums[0], nums[1], nums[2]);
        auto start = std::chrono::system_clock::now();
        Vector3 res = v0 / nums[3];
        auto end = std::chrono::system_clock::now();
        ASSERT_EQ(nums[0] / nums[3], res.X());
        ASSERT_EQ(nums[1] / nums[3], res.Y());
        ASSERT_EQ(nums[2] / nums[3], res.Z());
        ASSERT_LE((end-start).count()/T_NANOSECS,T_OPERATION_THRESHOLD);
    }
    
    
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}