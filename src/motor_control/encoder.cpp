#include "motor-control/encoder.hpp"

using namespace roboost::motor_control;

#ifdef ESP32 // TODO: Add Teensyduino support
double HalfQuadEncoder::get_angle() { return position_; }

double HalfQuadEncoder::get_velocity() { return velocity_; }

void HalfQuadEncoder::update()
{

    double dt = timing_service_.getDeltaTime();

    int64_t count = encoder_.getCount();

    double position_change = ((count - prev_count_) * step_increment_) * (reverse_ ? -1.0 : 1.0);

    position_ = count * step_increment_;

    velocity_ = position_change / dt;

    prev_count_ = count;
}
#endif // ESP32