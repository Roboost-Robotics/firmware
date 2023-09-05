#include "motor-control/encoder.hpp"

HalfQuadEncoder::HalfQuadEncoder(const int pin_A, const int pin_B, const int resolution) : resolution_(resolution)
{
    ESP32Encoder::useInternalWeakPullResistors = DOWN;
    encoder_.attachHalfQuad(pin_A, pin_B);
}

float HalfQuadEncoder::get_position() { return position_; }

float HalfQuadEncoder::get_velocity() { return velocity_; }

void HalfQuadEncoder::update()
{
    unsigned long current_time = micros();
    float elapsed_time = (current_time - last_time_) / 1000000.0; // Convert to seconds

    velocity_ = encoder_.getCount() / (resolution_ * elapsed_time);
    encoder_.clearCount();

    position_ += velocity_ * elapsed_time;
    if (position_ > 2 * PI)
    {
        position_ -= 2 * PI;
    }
    else if (position_ < 0)
    {
        position_ += 2 * PI;
    }

    last_time_ = current_time;
}