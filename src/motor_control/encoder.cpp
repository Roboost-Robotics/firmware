#include "motor-control/encoder.hpp"

HalfQuadEncoder::HalfQuadEncoder(const u_int8_t& pin_A, const u_int8_t& pin_B,
                                 const u_int16_t& resolution,
                                 const bool reverse)
    : resolution_(resolution), reverse_(reverse)
{
    ESP32Encoder::useInternalWeakPullResistors = DOWN;
    encoder_.attachHalfQuad(pin_A, pin_B);
}

double HalfQuadEncoder::get_angle() { return position_; }

double HalfQuadEncoder::get_velocity() { return velocity_; }

void HalfQuadEncoder::update()
{
    unsigned long current_time = micros();
    double elapsed_time =
        (current_time - last_time_) / 1000000.0; // Convert to seconds

    double position_change = (encoder_.getCount() * (2.0 * PI / resolution_)) *
                             (reverse_ ? -1.0 : 1.0);

    position_ += position_change;

    velocity_ = position_change / elapsed_time;

    encoder_.clearCount();

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