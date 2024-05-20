#include <gtest/gtest.h>
#include <roboost/utils/callback_scheduler.hpp>
#include <roboost/utils/controllers.hpp>
#include <roboost/utils/filters.hpp>
#include <roboost/utils/logging.hpp>
#include <thread>

using namespace roboost::controllers;
using namespace roboost::filters;
using namespace roboost::logging;
using namespace roboost::timing;

constexpr int32_t K_P = 1000;
constexpr int32_t K_I = 100;
constexpr int32_t K_D = 10;
constexpr int32_t MAX_INTEGRAL = 1000;
constexpr int32_t SETPOINT = 10000;

class PIDControllerTest : public ::testing::Test
{
protected:
    Logger& logger = Logger::get_instance<ConsoleLogger>();
    CallbackScheduler& timing_service = CallbackScheduler::get_instance();
    NoFilter<int32_t> derivative_filter;
    FastPIDController<NoFilter<int32_t>>* pid;

    virtual void SetUp()
    {
        timing_service.update();
        pid = new FastPIDController<NoFilter<int32_t>>(K_P, K_I, K_D, MAX_INTEGRAL, derivative_filter);
    }

    virtual void TearDown() { delete pid; }
};

TEST_F(PIDControllerTest, InitialConditions)
{
    // Test to ensure that the PID controller starts with zero integral and previous error
    std::cout << "Expected update output 0, got: " << pid->update(0, 0) << std::endl;
    ASSERT_EQ(pid->update(0, 0), 0);

    std::cout << "Expected kp: " << K_P << ", got: " << pid->get_kp() << std::endl;
    ASSERT_EQ(pid->get_kp(), K_P);

    std::cout << "Expected ki: " << K_I << ", got: " << pid->get_ki() << std::endl;
    ASSERT_EQ(pid->get_ki(), K_I);

    std::cout << "Expected kd: " << K_D << ", got: " << pid->get_kd() << std::endl;
    ASSERT_EQ(pid->get_kd(), K_D);

    std::cout << "Expected max integral: " << MAX_INTEGRAL << ", got: " << pid->get_max_integral() << std::endl;
    ASSERT_EQ(pid->get_max_integral(), MAX_INTEGRAL);
}

TEST_F(PIDControllerTest, ResponseToError)
{
    int32_t output = pid->update(SETPOINT, 0);
    std::cout << "Expected output 1386782, got: " << output << std::endl;
    ASSERT_EQ(output, 1386782);
}

TEST_F(PIDControllerTest, IntegralWindup)
{
    // Test that integral wind-up is handled correctly
    for (int i = 0; i < 1000; ++i)
    { // Simulate a long-running error accumulation
        pid->update(SETPOINT, 0);
    }
    int32_t output = pid->update(SETPOINT, 0);
    ASSERT_LE(pid->get_integral(), MAX_INTEGRAL); // Ensure the output does not exceed the max integral limit
}