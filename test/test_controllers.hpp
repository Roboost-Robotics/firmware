#include <gtest/gtest.h>
#include <roboost/utils/controllers.hpp>
#include <roboost/utils/timing.hpp>
#include <thread>

using namespace roboost::controllers;
using namespace roboost::timing;

class PIDControllerTest : public ::testing::Test
{
protected:
    PIDController* pid;
    TimingService& timing_service = TimingService::get_instance(); // Corrected access to TimingService

    virtual void SetUp()
    {
        // Initialize PID with the singleton instance of TimingService // TODO: Change timing service to a mock
        pid = new PIDController(1.0, 0.1, 0.01, 0.1, 100.0, timing_service);

        // Reset the timing service
        timing_service.reset();

        timing_service.update();
    }

    virtual void TearDown() { delete pid; }
};

TEST_F(PIDControllerTest, InitialConditions)
{
    // Test to ensure that the PID controller starts with zero integral and previous error
    std::cout << "Expected update output 0.0, got: " << pid->update(0.0, 0.0) << std::endl;
    ASSERT_EQ(pid->update(0.0, 0.0), 0.0);

    std::cout << "Expected kp 1.0, got: " << pid->get_kp() << std::endl;
    ASSERT_EQ(pid->get_kp(), 1.0);

    std::cout << "Expected ki 0.1, got: " << pid->get_ki() << std::endl;
    ASSERT_EQ(pid->get_ki(), 0.1);

    std::cout << "Expected kd 0.01, got: " << pid->get_kd() << std::endl;
    ASSERT_EQ(pid->get_kd(), 0.01);
}

TEST_F(PIDControllerTest, ResponseToError)
{
    double output = pid->update(10.0, 0.0);
    ASSERT_NEAR(output, 10.0, 1e-5) << "Expected output close to 10.0, got: " << output;
}

TEST_F(PIDControllerTest, IntegralWindup)
{
    // Test that integral wind-up is handled correctly
    for (int i = 0; i < 1000; ++i)
    { // Simulate a long-running error accumulation
        pid->update(10.0, 0.0);
    }
    double output = pid->update(10.0, 0.0);
    ASSERT_LE(output, 100.0); // Ensure the output does not exceed the max integral limit
}