/**
 * @file core.cpp
 * @author Friedl Jakob (friedl.jak@gmail.com)
 * @brief //TODO: Add description
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
#include <std_msgs/msg/float32.h>

#include "conf_hardware.h"
#include "conf_network.h"

HardwareSerial RPLidarSerial(2);
RPLidar lidar;

rcl_publisher_t scan_publisher;
rcl_publisher_t battery_publisher;
sensor_msgs__msg__LaserScan scan;
std_msgs__msg__Float32 battery_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

unsigned long last_battery_publish_time = 0;
const unsigned long battery_publish_interval = 2000;

void setup()
{
    // Initialize serial and lidar
    Serial.begin(115200);
    RPLidarSerial.begin(115200, SERIAL_8N1, RPLIDAR_RX, RPLIDAR_TX);

    // Start the lidar's motor
    pinMode(RPLIDAR_MOTOR, OUTPUT);
    digitalWrite(RPLIDAR_MOTOR, HIGH);

    // Initialize battery voltage measurement pin and LED
    pinMode(PWR_IN, INPUT);
    pinMode(PWR_LED, OUTPUT);

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
               &scan_publisher, &node,
               ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
               "scan") != RCL_RET_OK)
    {
        delay(1000);
    }

    // Create ROS publisher for battery level
    while (rclc_publisher_init_default(
               &battery_publisher, &node,
               ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
               "battery_level") != RCL_RET_OK)
    {
        delay(1000);
    }

    while (rclc_executor_init(&executor, &support.context, 2, &allocator) !=
           RCL_RET_OK) // Increase the number of handles to 2
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

    // Read battery voltage level
    float battery_voltage = analogRead(PWR_IN) * (3.3 * PWR_FACTOR / 4095.0);

    // Check if the voltage is less than 80% of 12.4V
    if (battery_voltage < 0.8 * 12.4)
    {
        digitalWrite(PWR_LED, HIGH);
    }
    else
    {
        digitalWrite(PWR_LED, LOW);
    }

    // Publish battery level every 2 seconds
    unsigned long current_time = millis();
    if (current_time - last_battery_publish_time >= battery_publish_interval)
    {
        battery_msg.data = battery_voltage;
        RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
        last_battery_publish_time = current_time; // Update last publish time
    }

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
            scan.angle_min = *min_element(angles.begin(), angles.end());
            scan.angle_max = *max_element(angles.begin(), angles.end());
            scan.angle_increment =
                angles.size() > 1
                    ? (scan.angle_max - scan.angle_min) / (angles.size() - 1)
                    : 0;
            scan.range_min = *min_element(ranges.begin(), ranges.end());
            scan.range_max = *max_element(ranges.begin(), ranges.end());

            scan.ranges.size = scan.ranges.capacity = ranges.size();
            scan.ranges.data = reinterpret_cast<float*>(allocator.allocate(
                sizeof(float) * ranges.size(), allocator.state));

            scan.intensities.size = scan.intensities.capacity =
                intensities.size();
            scan.intensities.data = reinterpret_cast<float*>(allocator.allocate(
                sizeof(float) * intensities.size(), allocator.state));

            std::copy(ranges.begin(), ranges.end(), scan.ranges.data);
            std::copy(intensities.begin(), intensities.end(),
                      scan.intensities.data);

            RCSOFTCHECK(rcl_publish(&scan_publisher, &scan, NULL));

            allocator.deallocate(scan.ranges.data, allocator.state);
            allocator.deallocate(scan.intensities.data, allocator.state);
        }
    }

    delay(100);
}