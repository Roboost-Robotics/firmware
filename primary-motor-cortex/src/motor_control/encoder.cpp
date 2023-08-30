#include "motor-control/encoder.hpp"

Encoder* Encoder::instance_ptr = nullptr;

Encoder::Encoder(const int pin_A, const int pin_B, const int resolution)
    : pin_A_(pin_A), pin_B_(pin_B), resolution_(resolution)
{
    pinMode(pin_A_, INPUT_PULLUP);
    pinMode(pin_B_, INPUT_PULLUP);
    last_state_A_ = digitalRead(pin_A_);
    last_state_B_ = digitalRead(pin_B_);
    last_time_ = micros();
}

float Encoder::read_velocity()
{
    noInterrupts();
    int count_A_local = count_A;
    int count_B_local = count_B;
    count_A = 0;
    count_B = 0;
    interrupts();

    int direction = (count_B_local >= 0) ? 1 : -1;
    int dt = micros() - last_time_;
    last_time_ = micros();

    float velocity = (direction * resolution_ * count_A_local) / (float)dt;
    return velocity;
}

void IRAM_ATTR Encoder::function_ISR_EC_A() { count_A++; }

void IRAM_ATTR Encoder::function_ISR_EC_B() { count_B++; }
