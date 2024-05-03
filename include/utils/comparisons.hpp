/**
 * @file comparisons.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Utility functions for comparisons.
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef COMPARISONS_H
#define COMPARISONS_H

#include <cmath>

namespace roboost
{
    namespace comparisons
    {
        // Method to check if two floats are approximately equal
        bool approx_equal(float a, float b, float epsilon) { return std::abs(a - b) < epsilon; }
    } // namespace comparisons
} // namespace roboost

#endif // COMPARISONS_H