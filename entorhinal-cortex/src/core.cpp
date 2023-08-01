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
}

void loop()
{
    if (lidar.isOpen())
    {
        // Collect all scanned points
        std::vector<float> ranges;
        std::vector<float> intensities;
        RPLidarMeasurement measurement;

        // Set scan parameters
        scan.angle_min = 0.0;
        scan.angle_max = 360.0;
        scan.angle_increment = 1.0;
        scan.range_min = 0.0;
        scan.range_max = 10.0;

        // Set frame_id
        scan.header.frame_id.data = (char*)"lidar";
        scan.header.frame_id.size = strlen(scan.header.frame_id.data);
        scan.header.frame_id.capacity = scan.header.frame_id.size + 1;

        // TODO: remove ranges.size() fix
        //? Why exactly does the error occur when not checking the range size?
        while (lidar.waitPoint() == RESULT_OK && ranges.size() < 360)
        {
            measurement = lidar.getCurrentPoint();
            Serial.println(measurement.angle);
            ranges.push_back(measurement.distance / 1000.);
            intensities.push_back(measurement.quality);
        }

        // Allocate memory for ranges and intensities
        scan.ranges.size = ranges.size();
        scan.ranges.capacity = ranges.size();
        scan.ranges.data = reinterpret_cast<float*>(
            allocator.allocate(sizeof(float) * ranges.size(), allocator.state));

        scan.intensities.size = intensities.size();
        scan.intensities.capacity = intensities.size();
        scan.intensities.data = reinterpret_cast<float*>(allocator.allocate(
            sizeof(float) * intensities.size(), allocator.state));

        // Copy data
        for (size_t i = 0; i < ranges.size(); i++)
        {
            scan.ranges.data[i] = ranges[i];
            scan.intensities.data[i] = intensities[i];
        }

        // Publish the scan data
        RCSOFTCHECK(rcl_publish(&publisher, &scan, NULL));

        // Free memory
        allocator.deallocate(scan.ranges.data, allocator.state);
        allocator.deallocate(scan.intensities.data, allocator.state);

        delay(100);
    }
    else
    {
        Serial.println("Lidar is not open!");
    }

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    delay(100);
}
