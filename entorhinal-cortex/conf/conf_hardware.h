/**
 * @file conf_hardware.h
 * @author Friedl Jakob (friedl.jak@gmail.com)
 * @brief Configuration file for the Roboost firmware.
 * @version 0.1
 * @date 2023-03-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef CONF_HARDWARE_H
#define CONF_HARDWARE_H

/**
 * @brief ESP32 specific configs
 *
 */
#define LED_BUILTIN 2

/**
 * @brief RP-LiDAR specific configs
 *
 */
#define RPLIDAR_MOTOR 18
#define RPLIDAR_RX 16
#define RPLIDAR_TX 17

/**
 * @brief Battery specific configs
 *
 */
#define PWR_IN 36
#define PWR_LED 13
#define PWR_MAX 12.6

#endif // CONF_HARDWARE_H