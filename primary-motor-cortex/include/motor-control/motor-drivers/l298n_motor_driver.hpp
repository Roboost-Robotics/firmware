/**
 * @file l298n_motor_driver.hpp //todo
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef L298N_MOTOR_DRIVER_H
#define L298N_MOTOR_DRIVER_H

#include "motor-control/motor-drivers/motor_driver.hpp"

/**
 * @brief //todo
 *
 */
class L298NMotorDriver : public MotorDriver
{
public:
    /**
     * @brief Construct a new L298NMotorDriver object //todo
     *
     * @param pin_in1
     * @param pin_in2
     * @param pin_ena
     * @param pwm_channel
     */
    L298NMotorDriver(unsigned int pin_in1, unsigned int pin_in2,
                     unsigned int pin_ena, unsigned int pwm_channel);

    /**
     * @brief Set the motor control object //todo
     *
     * @param control_value
     */
    void set_motor_control(float control_value);

private:
    const unsigned int pin_in1_;
    const unsigned int pin_in2_;
    const unsigned int pin_ena_;
    const unsigned int pwm_channel_;
};

#endif // L298N_MOTOR_DRIVER_H