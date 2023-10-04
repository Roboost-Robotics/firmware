/**
 * @file l298n_motor_driver.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Implementation of MotorDriver, which sets the control output
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef L298N_MOTOR_DRIVER_H
#define L298N_MOTOR_DRIVER_H

#include "motor-control/motor-drivers/motor_driver.hpp"
#include <stdint.h>

/**
 * @brief Implementation of MotorDriver, which sets the control output
 * directy to the L298N motor driver.
 *
 */
class L298NMotorDriver : public MotorDriver
{
public:
    /**
     * @brief Construct a new L298NMotorDriver object
     *
     * @param pin_in1 The pin number of the first input pin.
     * @param pin_in2 The pin number of the second input pin.
     * @param pin_ena The pin number of the enable pin.
     * @param pwm_channel The PWM channel to be used.
     */
    L298NMotorDriver(const uint8_t& pin_in1, const uint8_t& pin_in2, const uint8_t& pin_ena,
                     const uint8_t& pwm_channel);

    /**
     * @brief Set the motor control object
     *
     * @param control_value The control value to be set. Should be between -1 and 1.
     */
    void set_motor_control(float control_value);

private:
    const uint8_t pin_in1_;
    const uint8_t pin_in2_;
    const uint8_t pin_ena_;
    const uint8_t pwm_channel_;
};

#endif // L298N_MOTOR_DRIVER_H