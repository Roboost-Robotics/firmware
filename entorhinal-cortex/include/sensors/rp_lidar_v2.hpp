/**
 * @file rp_lidar_v2.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Implementation of RP LiDAR V2 communication.
 * @version 1.1
 * @date 2023-08-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef RP_LIDAR_V2_H
#define RP_LIDAR_V2_H

#include "lidar.hpp"
#include <Arduino.h>
#include <RPLidar.h>

/**
 * @brief Class handling RP LiDAR V2 communication.
 *
 */
class RP_LiDAR_V2 : public LiDAR
{
public:
    /**
     * @brief Construct a new RP LiDAR V2 object
     *
     * @param rx_pin RX pin for serial communication.
     * @param tx_pin TX pin for serial communication.
     * @param motor_pin Pin for motor control.
     * @param frame_id Frame ID for ROS scan message.
     */
    RP_LiDAR_V2(const int rx_pin, const int tx_pin, const int motor_pin,
                const std_msgs__msg__Header::frame_id frame_id);

    /**
     * @brief Get the scan object
     *
     * @return sensor_msgs__msg__LaserScan the latest scan in ROS LaserMessage
     * formate.
     */
    sensor_msgs__msg__LaserScan get_scan() override;

private:
    const int rx_pin_, tx_pin_, motor_pin_;
    const std_msgs__msg__Header::frame_id frame_id_;
};

#endif // RP_LIDAR_V2_H