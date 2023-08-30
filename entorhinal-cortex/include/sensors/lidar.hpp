/**
 * @file lidar.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief LiDAR base class definitions.
 * @version 1.1
 * @date 2023-08-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef LIDAR_H
#define LIDAR_H

#include <sensor_msgs/msg/laser_scan.h>

/**
 * @brief Abstract base class for defining LiDAR communication.
 */
class LiDAR
{
public:
    /**
     * @brief Gets the most recent scan.
     *
     * @return sensor_msgs__msg__LaserScan Full scan in form of ROS LaserScan
     * message.
     */
    virtual sensor_msgs__msg__LaserScan get_scan() = 0;

    /**
     * @brief Updates LiDAR status and retreives scan.
     *
     * @note //! Might be blocking (outsource into separate thread if possible)
     */
    virtual void update() = 0;
};

#endif // LIDAR_H
