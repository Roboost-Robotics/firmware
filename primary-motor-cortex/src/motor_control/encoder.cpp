/**
 * @file encoder.cpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Implementation of Encoder class.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "motor-control/encoder.hpp"

Encoder::Encoder(const int pin_A, const int pin_B, const int resolution)
    : pin_A_(pin_A), pin_B_(pin_B), resolution_(resolution)
{
    pinMode(pin_A_, INPUT);
    pinMode(pin_B_, INPUT);
    last_state_A_ = digitalRead(pin_A_);
    last_state_B_ = digitalRead(pin_B_);
    last_time_ = micros();
}

float Encoder::read_velocity()
{
    int current_state_A = digitalRead(pin_A_);
    int current_state_B = digitalRead(pin_B_);

    if (current_state_A != last_state_A_)
    {
        int current_time = micros();
        int dt = current_time - last_time_;
        last_time_ = current_time;
        last_state_A_ = current_state_A;
        last_state_B_ = current_state_B;

        int direction =
            (current_state_B == HIGH) ? 1 : -1; // Clockwise or counterclockwise

        float velocity = (2.0 * PI * direction * resolution_) /
                         (float)dt; // Velocity in rad/s
        return velocity;
    }
    return 0.0; // No change in state, so no velocity
}