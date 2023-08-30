/**
 * @file voltage_measureing.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Definitions for handling voltage sensing.
 * @version 1.1
 * @date 2023-08-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef VOLTAGE_MEASUREING_H
#define VOLTAGE_MEASUREING_H

#include <Arduino.h>

/**
 * @brief Class handling voltage sensing with a voltage devider.
 *
 */
class VoltageDevider
{
public:
    /**
     * @brief Construct a new Voltage Devider object
     *
     * @param sensing_pin Pin on which devider is connected.
     * @param ground_resistor Resistance of the resistor connected to ground in
     * Ohm.
     * @param sensing_resistor Resistance of the resistor connected to the
     * voltage to measure in Ohm.
     */
    VoltageSensor(const int sensing_pin, const float ground_resistor,
                  const float sensing_resistor);

    /**
     * @brief Reads the voltage of the sensor.
     *
     * @return float Measured voltage.
     */
    float read();

private:
    const int rx_pin_, tx_pin_, motor_pin_;
    const std_msgs__msg__Header::frame_id frame_id_;
};

#endif // VOLTAGE_MEASUREING_H