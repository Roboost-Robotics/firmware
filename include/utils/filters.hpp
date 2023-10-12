/**
 * @file filters.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Utility functions and classes for filtering.
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef FILTERS_H
#define FILTERS_H

#include "utils/constants.h"
#include <algorithm>
#include <deque>

/**
 * @brief Abstract base class for filters.
 *
 */
class Filter
{
public:
    /**
     * @brief Update the filter.
     *
     * @param input The input value.
     * @return float The filtered value.
     */
    virtual float update(float input) = 0;
};

/**
 * @brief Implementation and definiton of low pass filter class.
 *
 */

class LowPassFilter : public Filter
{
public:
    /**
     * @brief Construct a new Low Pass Filter object
     *
     * @param cutoff_frequency The cutoff frequency of the filter.
     * @param sampling_time The sampling time of the filter.
     */
    LowPassFilter(float cutoff_frequency, float sampling_time);

    /**
     * @brief Update the filter.
     *
     * @param input The input value.
     * @return float The filtered value.
     */
    float update(float input);

private:
    float cutoff_frequency_;
    float sampling_time_;
    float alpha_;
    float output_;
};

/**
 * @brief Implementation and definiton of moving average filter class.
 *
 */
class MovingAverageFilter : public Filter
{
public:
    /**
     * @brief Construct a new Moving Average Filter object
     *
     * @param window_size The size of the filter window.
     */
    MovingAverageFilter(int window_size);

    /**
     * @brief Update the filter.
     *
     * @param input The input value.
     * @return float The filtered value.
     */
    float update(float input);

private:
    std::deque<float> input_history_;
    int window_size_;
    float output_;
};

/**
 * @brief Implementation and definiton of median filter class.
 *
 */
class MedianFilter : public Filter
{
public:
    /**
     * @brief Construct a new Median Filter object
     *
     * @param window_size The size of the filter window.
     */
    MedianFilter(int window_size);

    /**
     * @brief Update the filter.
     *
     * @param input The input value.
     * @return float The filtered value.
     */
    float update(float input);

private:
    std::deque<float> input_history_;
    int window_size_;
    float output_;
};

/**
 * @brief Implementation and definiton of exponential moving average filter
 * class.
 *
 */
class ExponentialMovingAverageFilter : public Filter
{
public:
    /**
     * @brief Construct a new Exponential Moving Average Filter object
     *
     * @param alpha The alpha value of the filter.
     */
    ExponentialMovingAverageFilter(float alpha);

    /**
     * @brief Update the filter.
     *
     * @param input The input value.
     * @return float The filtered value.
     */
    float update(float input);

private:
    float alpha_;
    float output_;
};

#endif // FILTERS_H