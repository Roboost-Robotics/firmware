/**
 * @file encoder.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Defines the Encoder class, which provides rotational velocity reading
 * from an encoder.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder
{
public:
    Encoder(const int pin_A, const int pin_B, const int resolution);

    float read_velocity();

    const inline int get_pin_A() const { return pin_A_; }
    const inline int get_pin_B() const { return pin_B_; }

private:
    void IRAM_ATTR function_ISR_EC_A();
    void IRAM_ATTR function_ISR_EC_B();

    const int pin_A_, pin_B_;
    const int resolution_;
    volatile uint16_t count_A = 0;
    volatile uint16_t count_B = 0;
    int last_state_A_;
    int last_state_B_;
    unsigned long last_time_;
};

#endif // ENCODER_H
