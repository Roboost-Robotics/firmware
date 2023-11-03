/**
 * @file filters.cpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Utility function and class definitions for filtering.
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "utils/filters.hpp"

LowPassFilter::LowPassFilter(float cutoff_frequency, float sampling_time)
    : cutoff_frequency_(cutoff_frequency), sampling_time_(sampling_time)
{
    alpha_ = sampling_time_ /
             (sampling_time_ + 1.0f / (2.0f * PI * cutoff_frequency_));
}

float LowPassFilter::update(float input)
{
    output_ = alpha_ * input + (1.0f - alpha_) * output_;
    return output_;
}

void LowPassFilter::reset() { output_ = 0.0f; }

float LowPassFilter::get_cutoff_frequency() { return cutoff_frequency_; }

float LowPassFilter::get_sampling_time() { return sampling_time_; }

void LowPassFilter::set_cutoff_frequency(float cutoff_frequency)
{
    cutoff_frequency_ = cutoff_frequency;
    alpha_ = sampling_time_ /
             (sampling_time_ + 1.0f / (2.0f * PI * cutoff_frequency_));
}

void LowPassFilter::set_sampling_time(float sampling_time)
{
    sampling_time_ = sampling_time;
    alpha_ = sampling_time_ /
             (sampling_time_ + 1.0f / (2.0f * PI * cutoff_frequency_));
}

MovingAverageFilter::MovingAverageFilter(int window_size)
    : window_size_(window_size)
{
}

float MovingAverageFilter::update(float input)
{
    input_history_.push_front(input);

    if (input_history_.size() > window_size_)
        input_history_.pop_back();

    float sum = 0.0f;
    for (float value : input_history_)
        sum += value;

    output_ = sum / input_history_.size();
    return output_;
}

MedianFilter::MedianFilter(int window_size) : window_size_(window_size) {}

float MedianFilter::update(float input)
{
    input_history_.push_front(input);

    if (input_history_.size() > window_size_)
        input_history_.pop_back();

    std::sort(input_history_.begin(), input_history_.end());

    if (input_history_.size() % 2 == 0)
        output_ = (input_history_[input_history_.size() / 2 - 1] +
                   input_history_[input_history_.size() / 2]) /
                  2.0f;
    else
        output_ = input_history_[input_history_.size() / 2];

    return output_;
}

ExponentialMovingAverageFilter::ExponentialMovingAverageFilter(float alpha)
    : alpha_(alpha)
{
}

float ExponentialMovingAverageFilter::update(float input)
{
    output_ = alpha_ * input + (1.0f - alpha_) * output_;
    return output_;
}