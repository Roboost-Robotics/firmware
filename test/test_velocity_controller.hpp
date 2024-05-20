#include <gtest/gtest.h>
#include <math.h>
#include <roboost/motor_control/encoders/encoder.hpp>
#include <roboost/motor_control/motor_controllers/velocity_motor_controller.hpp>
#include <roboost/motor_control/motor_drivers/motor_driver.hpp>
#include <roboost/utils/controllers.hpp>
#include <roboost/utils/filters.hpp>

using namespace roboost::motor_control;
using namespace roboost::controllers;
using namespace roboost::filters;

class MockMotorDriver
{
public:
    MockMotorDriver() : control_value(0) {}

    void set_motor_control(int32_t value) { control_value = value; }

    int32_t get_motor_control() const { return control_value; }

private:
    int32_t control_value;
};

class MockEncoder
{
public:
    MockEncoder(int64_t velocity = 0) : velocity_(velocity), position_(0) {}

    void update() { position_ += velocity_; }

    int64_t get_velocity() const { return velocity_; }

    void set_velocity(int64_t velocity) { velocity_ = velocity; }

    int64_t get_position() const { return position_; }

    void set_position(int64_t position) { position_ = position; }

    float get_step_increment() const
    {
        return 2.0 * M_PI / 1024; // Assuming 1024 ticks per revolution
    }

    float get_velocity_radians_per_second() const { return velocity_ * get_step_increment(); }

private:
    int64_t velocity_;
    int64_t position_;
};

class VelocityControllerTest : public ::testing::Test
{
protected:
    MockMotorDriver motor_driver;
    MockEncoder encoder;
    LowPassFilter<float>* derivative_filter;
    PIDController<float, LowPassFilter<float>>* pid_controller;
    MovingAverageFilter<float>* input_filter;
    MovingAverageFilter<float>* output_filter;
    RateLimitingFilter<float>* rate_limiting_filter;
    VelocityController<MockMotorDriver, MockEncoder, PIDController<float, LowPassFilter<float>>, MovingAverageFilter<float>, MovingAverageFilter<float>, RateLimitingFilter<float>>*
        velocity_controller;

    void SetUp() override
    {
        derivative_filter = new LowPassFilter<float>(100000, 1);
        pid_controller = new PIDController<float, LowPassFilter<float>>(1.0, 0.0, 0.0, 50.0, *derivative_filter);
        input_filter = new MovingAverageFilter<float>(100);
        output_filter = new MovingAverageFilter<float>(100);
        rate_limiting_filter = new RateLimitingFilter<float>(6.0, 10000);

        velocity_controller =
            new VelocityController<MockMotorDriver, MockEncoder, PIDController<float, LowPassFilter<float>>, MovingAverageFilter<float>, MovingAverageFilter<float>, RateLimitingFilter<float>>(
                motor_driver, encoder, *pid_controller, *input_filter, *output_filter, *rate_limiting_filter, 10, 50);
    }

    void TearDown() override
    {
        delete derivative_filter;
        delete pid_controller;
        delete input_filter;
        delete output_filter;
        delete rate_limiting_filter;
        delete velocity_controller;
    }
};

TEST_F(VelocityControllerTest, SetTargetZero)
{
    velocity_controller->set_target(0);
    ASSERT_EQ(motor_driver.get_motor_control(), 0);
}

TEST_F(VelocityControllerTest, SetTargetPositive)
{
    encoder.set_velocity(0);
    velocity_controller->set_target(1.0);
    ASSERT_GT(motor_driver.get_motor_control(), 0);
}

TEST_F(VelocityControllerTest, SetTargetNegative)
{
    encoder.set_velocity(0);
    velocity_controller->set_target(-1.0);
    ASSERT_LT(motor_driver.get_motor_control(), 0);
}

TEST_F(VelocityControllerTest, MaintainSetpoint)
{
    encoder.set_velocity(1);
    velocity_controller->set_target(1.0);
    velocity_controller->set_target(1.0);                 // Set again to maintain
    ASSERT_NEAR(motor_driver.get_motor_control(), 0, 10); // Allow some tolerance
}