/**
 * @file core.cpp
 * @author Friedl Jakob (friedl.jak@gmail.com)
 * @brief //todo
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <Arduino.h>
#include <RPLidar.h>
#include <micro_ros_platformio.h>

#include "rcl_checks.h"
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <sensor_msgs/msg/laser_scan.h>

#include "conf_network.h"
#include "conf_robot.h"

#define RPLIDAR_MOTOR 18
#define RPLIDAR_RX 16
#define RPLIDAR_TX 17

HardwareSerial RPLidarSerial(2); // Using HardwareSerial 2
RPLidar lidar;

rcl_publisher_t publisher;
sensor_msgs__msg__LaserScan scan;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

void set_scan_parameters_and_publish(std::vector<float>& ranges,
                                     std::vector<float>& intensities,
                                     std::vector<float>& angles)
{
    scan.angle_min = *min_element(angles.begin(), angles.end());
    scan.angle_max = *max_element(angles.begin(), angles.end());
    scan.angle_increment =
        angles.size() > 1
            ? (scan.angle_max - scan.angle_min) / (angles.size() - 1)
            : 0;
    scan.range_min = *min_element(ranges.begin(), ranges.end());
    scan.range_max = *max_element(ranges.begin(), ranges.end());

    scan.ranges.size = scan.ranges.capacity = ranges.size();
    scan.ranges.data = reinterpret_cast<float*>(
        allocator.allocate(sizeof(float) * ranges.size(), allocator.state));

    scan.intensities.size = scan.intensities.capacity = intensities.size();
    scan.intensities.data = reinterpret_cast<float*>(allocator.allocate(
        sizeof(float) * intensities.size(), allocator.state));

    std::copy(ranges.begin(), ranges.end(), scan.ranges.data);
    std::copy(intensities.begin(), intensities.end(), scan.intensities.data);

    RCSOFTCHECK(rcl_publish(&publisher, &scan, NULL));

    allocator.deallocate(scan.ranges.data, allocator.state);
    allocator.deallocate(scan.intensities.data, allocator.state);
}

void setup()
{
    // Initialize serial and lidar
    Serial.begin(115200);
    RPLidarSerial.begin(115200, SERIAL_8N1, RPLIDAR_RX, RPLIDAR_TX);

    // Start the lidar's motor
    pinMode(RPLIDAR_MOTOR, OUTPUT);
    digitalWrite(RPLIDAR_MOTOR, HIGH);

    lidar.begin(RPLidarSerial);
    lidar.startScan();

    // Configure micro-ROS
    IPAddress agent_ip(AGENT_IP);
    uint16_t agent_port = AGENT_PORT;

    set_microros_wifi_transports(SSID, SSID_PW, agent_ip, agent_port);
    delay(2000);

    allocator = rcl_get_default_allocator();

    while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    {
        delay(1000);
    }

    while (rclc_node_init_default(&node, "lidar_node", "", &support) !=
           RCL_RET_OK)
    {
        delay(1000);
    }

    while (rclc_publisher_init_default(
               &publisher, &node,
               ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
               "scan") != RCL_RET_OK)
    {
        delay(1000);
    }

    while (rclc_executor_init(&executor, &support.context, 1, &allocator) !=
           RCL_RET_OK)
    {
        delay(1000);
    }

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // Set frame_id
    scan.header.frame_id.data = (char*)"lidar";
    scan.header.frame_id.size = strlen(scan.header.frame_id.data);
    scan.header.frame_id.capacity = scan.header.frame_id.size + 1;
}

void loop()
{
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

    if (lidar.isOpen())
    {
        std::vector<float> ranges, intensities, angles;
        RPLidarMeasurement measurement;
        float start_angle = -1.0;
        bool start_collecting = false;

        while (lidar.waitPoint() == RESULT_OK)
        {
            measurement = lidar.getCurrentPoint();

            // If we're just starting, record the first angle and start
            // collecting
            if (start_angle < 0.0)
            {
                start_angle = measurement.angle;
                start_collecting = true;
            }
            // Stop collecting after a full rotation
            else if ((start_angle <= measurement.angle + 1) &&
                     (start_angle >= measurement.angle - 1))
            {
                break;
            }

            if (start_collecting)
            {
                ranges.push_back(measurement.distance / 1000.);
                intensities.push_back(measurement.quality);
                angles.push_back(measurement.angle * (PI / 180.));
            }
        }

        // Check if we collected any data points
        if (start_collecting && !ranges.empty() && !intensities.empty() &&
            !angles.empty())
        {
            // Set dynamic scan parameters
            set_scan_parameters_and_publish(ranges, intensities, angles);
        }
    }

    delay(100);
}