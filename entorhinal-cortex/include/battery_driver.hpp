/**
 * @file battery_driver.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Definitions for monitoring and controlling batteries.
 * @version 1.1
 * @date 2023-08-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef BATTERY_DRIVER_H
#define BATTERY_DRIVER_H

#include "sensors/voltage_measureing.hpp"
#include <Arduino.h>

/**
 * @brief Class for monitoring and controlling a battery.
 *
 * @note // TODO: Implement once more advanced batteries are used.
 *
 */
class BatteryDriver
{
public:
    /**
     * @brief Construct a new Battery Controller object
     *
     * @param battery_driver
     */
    BatteryDriver();

    //? What methods would be relevant?
};

#endif // BATTERY_DRIVER_H