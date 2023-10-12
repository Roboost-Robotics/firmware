#include "motor-control/encoder.hpp"

HalfQuadEncoder::HalfQuadEncoder(const u_int8_t& pin_A, const u_int8_t& pin_B,
                                 const u_int16_t& resolution,
                                 const bool reverse)
    : resolution_(resolution), reverse_(reverse)
{
    ESP32Encoder::useInternalWeakPullResistors = DOWN;
    encoder_.attachSingleEdge(pin_A, pin_B);
    step_increment_ = 2.0 * PI / resolution_;
}

double HalfQuadEncoder::get_angle() { return position_; }

double HalfQuadEncoder::get_velocity() { return velocity_; }

void HalfQuadEncoder::update()
{
    unsigned long current_time = micros();
    double elapsed_time = (current_time - last_time_) * 1e-6;

    int64_t count = encoder_.getCount();

    double position_change =
        ((count - prev_count_) * step_increment_) * (reverse_ ? -1.0 : 1.0);

    position_ = count * step_increment_;

    velocity_ = position_change / elapsed_time;

    last_time_ = current_time;
    prev_count_ = count;
}