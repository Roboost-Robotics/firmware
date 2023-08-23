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

#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/float32.h>

#include "conf_hardware.h"
#include "conf_network.h"

HardwareSerial RPLidarSerial(2);
RPLidar lidar;

rcl_publisher_t scan_publisher;
rcl_publisher_t battery_publisher;
rcl_publisher_t diagnostic_publisher;
sensor_msgs__msg__LaserScan scan_msg;
std_msgs__msg__Float32 battery_msg;
diagnostic_msgs__msg__DiagnosticStatus diagnostic_msg;

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

    while (rclc_publisher_init_default(
               &battery_publisher, &node,
               ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
               "battery_level") != RCL_RET_OK)
    {
        delay(1000);
    }

    while (
        rclc_publisher_init_default(
            &diagnostic_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus),
            "diagnostics") != RCL_RET_OK)
    {
        delay(1000);
    }

    while (rclc_executor_init(&executor, &support.context, 3, &allocator) !=
           RCL_RET_OK)
    {
        delay(1000);
    }

    delay(500);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // Set frame_id
    scan_msg.header.frame_id.data = (char*)"lidar";
    scan_msg.header.frame_id.size = strlen(scan_msg.header.frame_id.data);
    scan_msg.header.frame_id.capacity = scan_msg.header.frame_id.size + 1;
}

void loop()
{
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

    // Read battery voltage level
    float battery_voltage = analogRead(PWR_IN) * (3.3 * PWR_FACTOR / 4095.0);

    // Populate battery voltage diagnostic
    diagnostic_msg.level = diagnostic_msgs__msg__DiagnosticStatus__OK;
    diagnostic_msg.name.data = (char*)"Battery Voltage";
    diagnostic_msg.name.size = strlen(diagnostic_msg.name.data);
    diagnostic_msg.name.capacity = diagnostic_msg.name.size + 1;

    if (battery_voltage < 0.8 * 12.4)
    {
        digitalWrite(PWR_LED, HIGH);
        diagnostic_msg.level = diagnostic_msgs__msg__DiagnosticStatus__WARN;
        diagnostic_msg.message.data = (char*)"Low Battery Voltage";
        diagnostic_msg.message.size = strlen(diagnostic_msg.message.data);
        diagnostic_msg.message.capacity = diagnostic_msg.message.size + 1;
    }
    else
    {
        digitalWrite(PWR_LED, LOW);
        diagnostic_msg.message.data =
            (char*)"Battery voltage is within normal range";
        diagnostic_msg.message.size = strlen(diagnostic_msg.message.data);
        diagnostic_msg.message.capacity = diagnostic_msg.message.size + 1;
    }

    RCSOFTCHECK(rcl_publish(&diagnostic_publisher, &diagnostic_msg, NULL));

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
            scan_msg.angle_min = *min_element(angles.begin(), angles.end());
            scan_msg.angle_max = *max_element(angles.begin(), angles.end());
            scan_msg.angle_increment =
                angles.size() > 1 ? (scan_msg.angle_max - scan_msg.angle_min) /
                                        (angles.size() - 1)
                                  : 0;
            scan_msg.range_min = *min_element(ranges.begin(), ranges.end());
            scan_msg.range_max = *max_element(ranges.begin(), ranges.end());

            scan_msg.ranges.size = scan_msg.ranges.capacity = ranges.size();
            scan_msg.ranges.data = reinterpret_cast<float*>(allocator.allocate(
                sizeof(float) * ranges.size(), allocator.state));

            scan_msg.intensities.size = scan_msg.intensities.capacity =
                intensities.size();
            scan_msg.intensities.data =
                reinterpret_cast<float*>(allocator.allocate(
                    sizeof(float) * intensities.size(), allocator.state));

            std::copy(ranges.begin(), ranges.end(), scan_msg.ranges.data);
            std::copy(intensities.begin(), intensities.end(),
                      scan_msg.intensities.data);

            RCSOFTCHECK(rcl_publish(&scan_publisher, &scan_msg, NULL));

            allocator.deallocate(scan_msg.ranges.data, allocator.state);
            allocator.deallocate(scan_msg.intensities.data, allocator.state);
        }
        diagnostic_msg.level = diagnostic_msgs__msg__DiagnosticStatus__OK;
        diagnostic_msg.name.data = (char*)"LiDAR Status";
        diagnostic_msg.name.size = strlen(diagnostic_msg.name.data);
        diagnostic_msg.name.capacity = diagnostic_msg.name.size + 1;

        // Check LiDAR status and set diagnostic level and message accordingly
        if (!lidar.isOpen())
        {
            diagnostic_msg.level =
                diagnostic_msgs__msg__DiagnosticStatus__ERROR;
            diagnostic_msg.message.data = (char*)"LiDAR not open";
        }
        else if (ranges.empty() || intensities.empty() || angles.empty())
        {
            diagnostic_msg.level = diagnostic_msgs__msg__DiagnosticStatus__WARN;
            diagnostic_msg.message.data = (char*)"No LiDAR data collected";
        }
        else
        {
            diagnostic_msg.message.data = (char*)"LiDAR data collected";
        }

        RCSOFTCHECK(rcl_publish(&diagnostic_publisher, &diagnostic_msg, NULL));
    }

    delay(100);
}