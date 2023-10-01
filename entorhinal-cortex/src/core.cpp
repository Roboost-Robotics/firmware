/**
 * @file core.cpp
 * @author Friedl Jakob (friedl.jak@gmail.com)
 * @brief Core file handling ROS communication, initialization and distribution
 * of relevant objects. Currently it supports the Roboost-V2 sensorshield
 * hardware.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "rcl_checks.h"
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <std_msgs/msg/float32.h>

#include <RPLidar.h>

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

// Variables for calculating scan time
unsigned long scan_start_time = 0;

// Define global constants for maximum measurements and measurement buffers
const size_t max_measurements = 500;
float ranges_buffer[max_measurements];
float intensities_buffer[max_measurements];
size_t num_measurements = 0;

void setup()
{
    // Initialize serial and lidar
    Serial.begin(115200);
    RPLidarSerial.begin(115200, SERIAL_8N1, RPLIDAR_RX, RPLIDAR_TX);

    // Initialize the lidar's motor control pin
    pinMode(RPLIDAR_MOTOR, OUTPUT);

    // Initialize battery voltage measurement pin and LED
    pinMode(PWR_IN, INPUT);
    pinMode(PWR_LED, OUTPUT);

    // Initialize the status LED
    pinMode(LED_BUILTIN, OUTPUT);

    lidar.begin(RPLidarSerial);
    lidar.startScan();

    // Configure micro-ROS
    IPAddress agent_ip(AGENT_IP);
    uint16_t agent_port = AGENT_PORT;

    set_microros_wifi_transports((char*)SSID, (char*)SSID_PW, agent_ip,
                                 agent_port);
    delay(2000);

    allocator = rcl_get_default_allocator();

    while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK)
    {
        Serial.println("Error initializing rclc_support, trying again...");
        delay(100);
    }

    while (rclc_node_init_default(&node, "lidar_node", "", &support) !=
           RCL_RET_OK)
    {
        Serial.println("Error initializing rclc_node, trying again...");
        delay(100);
    }

    while (rclc_publisher_init_default(
               &scan_publisher, &node,
               ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
               "scan") != RCL_RET_OK)
    {
        Serial.println("Error initializing scan_publisher, trying again...");
        delay(100);
    }

    while (rclc_publisher_init_default(
               &battery_publisher, &node,
               ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
               "battery_level") != RCL_RET_OK)
    {
        Serial.println("Error initializing battery_publisher, trying again...");
        delay(100);
    }

    while (
        rclc_publisher_init_default(
            &diagnostic_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus),
            "diagnostics") != RCL_RET_OK)
    {
        Serial.println(
            "Error initializing diagnostic_publisher, trying again...");
        delay(100);
    }

    while (rclc_executor_init(&executor, &support.context, 1, &allocator) !=
           RCL_RET_OK)
    {
        Serial.println("Error initializing rclc_executor, trying again...");
        delay(100);
    }

    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(RPLIDAR_MOTOR, HIGH);

    // Set frame_id
    scan_msg.header.frame_id.data = (char*)"lidar";
    scan_msg.header.frame_id.size = strlen(scan_msg.header.frame_id.data);
    scan_msg.header.frame_id.capacity = scan_msg.header.frame_id.size + 1;

    scan_msg.range_min = 0.15;
    scan_msg.range_max = 16.0;

    // Allocate memory for scan message data
    scan_msg.ranges.data = ranges_buffer;
    scan_msg.ranges.size = 0;
    scan_msg.ranges.capacity = max_measurements;
    scan_msg.intensities.data = intensities_buffer;
    scan_msg.intensities.size = 0;
    scan_msg.intensities.capacity = max_measurements;
}

void loop()
{

    // Read battery voltage level
    float battery_voltage = analogRead(PWR_IN) * (3.3 * PWR_FACTOR / 4095.0);

    if (battery_voltage < 0.85 * 12.4) // 12.4 * 0.85 = 10.54 V
    {
        digitalWrite(PWR_LED, HIGH);
        diagnostic_msg.level = diagnostic_msgs__msg__DiagnosticStatus__WARN;
        diagnostic_msg.message.data = (char*)"Low Battery Voltage";
        diagnostic_msg.message.size = strlen(diagnostic_msg.message.data);
        diagnostic_msg.message.capacity = diagnostic_msg.message.size + 1;

        RCSOFTCHECK(rcl_publish(&diagnostic_publisher, &diagnostic_msg, NULL));
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
        std::vector<RPLidarMeasurement> measurements;

        // Read lidar measurements
        while (true)
        {
            lidar.waitPoint();
            if (lidar.getCurrentPoint().startBit)
            {
                scan_start_time = millis();
                break; // Start bit detected
            }
        }

        do
        {
            measurements.push_back(lidar.getCurrentPoint());
            lidar.waitPoint();
        } while (!lidar.getCurrentPoint().startBit);

        unsigned long scan_end_time = millis();

        // Order the measurements by angle.
        std::sort(measurements.begin(), measurements.end(),
                  [](const RPLidarMeasurement& a, const RPLidarMeasurement& b)
                  { return a.angle < b.angle; });

        // Update scan message data
        num_measurements = measurements.size();
        scan_msg.angle_min = measurements.front().angle * DEG_TO_RAD;
        scan_msg.angle_max = measurements.back().angle * DEG_TO_RAD;
        scan_msg.angle_increment =
            (scan_msg.angle_max - scan_msg.angle_min) / (num_measurements - 1);
        scan_msg.scan_time = (scan_end_time - scan_start_time) / 1000.0;
        scan_msg.time_increment = scan_msg.scan_time / (num_measurements - 1);

        // Populate scan message data
        for (size_t i = 0; i < num_measurements; i++)
        {
            scan_msg.ranges.data[i] = measurements[i].distance / 1000.0;
            scan_msg.intensities.data[i] = measurements[i].quality;
        }

        // Update scan message size
        scan_msg.ranges.size = num_measurements;
        scan_msg.intensities.size = num_measurements;

        // Publish the scan message
        RCSOFTCHECK(rcl_publish(&scan_publisher, &scan_msg, NULL));
    }
    else
    {
        // Publish diagnostics message
        diagnostic_msg.level = diagnostic_msgs__msg__DiagnosticStatus__WARN;
        diagnostic_msg.message.data = (char*)"Lidar not connected";
        diagnostic_msg.message.size = strlen(diagnostic_msg.message.data);
        diagnostic_msg.message.capacity = diagnostic_msg.message.size + 1;

        RCSOFTCHECK(rcl_publish(&diagnostic_publisher, &diagnostic_msg, NULL));
    }

    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
