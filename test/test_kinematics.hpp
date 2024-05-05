#include <gtest/gtest.h>
#include <roboost/kinematics/kinematics.hpp>

using namespace roboost::kinematics;

class MecanumKinematicsTest : public ::testing::Test
{
protected:
    MecanumKinematics4W* kinematics;

    virtual void SetUp()
    {
        // Setting typical values for radius, wheel base, and track width
        kinematics = new MecanumKinematics4W(0.05, 0.4, 0.3);
    }

    virtual void TearDown() { delete kinematics; }
};

TEST_F(MecanumKinematicsTest, CalculateWheelVelocity)
{
    // clang-format off
    roboost::math::Vector robot_velocity = {   0.05,
                                0.0,
                                0.0};
    roboost::math::Vector expected_wheel_velocity = {  1.0, -1.0,
                                        1.0, -1.0};
    // clang-format on

    roboost::math::Vector calculated_velocity = kinematics->calculate_wheel_velocity(robot_velocity);
    ASSERT_EQ(calculated_velocity.size(), expected_wheel_velocity.size()); // Ensure size matches
    for (size_t i = 0; i < calculated_velocity.size(); ++i)
    {
        EXPECT_NEAR(calculated_velocity[i], expected_wheel_velocity[i], 1e-5);
    }
}

TEST_F(MecanumKinematicsTest, CalculateRobotVelocity)
{
    // clang-format off
    roboost::math::Vector wheel_velocity = {   1.0, -1.0,
                                1.0, -1.0}; // Example wheel velocities

    roboost::math::Vector expected_robot_velocity = {  0.05,   // vx
                                        0,      // vy
                                        0};     // omega
    // clang-format on
    roboost::math::Vector calculated_velocity = kinematics->calculate_robot_velocity(wheel_velocity);
    ASSERT_EQ(calculated_velocity.size(), expected_robot_velocity.size()); // Ensure size matches

    // Expected vel is
}