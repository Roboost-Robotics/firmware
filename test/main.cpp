#include "test_controllers.hpp"
#include "test_kinematics.hpp"
#include "test_velocity_controller.hpp"
#include <gtest/gtest.h>

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
