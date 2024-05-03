/**
 * @file core.cpp
 * @author Friedl Jakob (friedl.jak@gmail.com)
 * @brief This file contains the main functionality for controlling the Roboost
 * robot connected over serial to the main computer.
 * @version 1.2
 * @date 2024-04-30
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <Arduino.h>
#include <ArduinoEigen.h>
#include <micro_ros_platformio.h>

#include "rcl_checks.h"
#include <rcl/rcl.h>
#include <rclc/executor.h>

#include <rclc/rclc.h>
#include <rosidl_runtime_c/string_functions.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

#include <rcl_checks.h>
#include <utils/logging.hpp>
#include <utils/timing.hpp>

rcl_subscription_t led_subscriber;
rcl_publisher_t count_publisher, missed_deadlines_publisher;

rcl_timer_t publish_timer, sync_timer;

std_msgs__msg__Bool led_msg;
std_msgs__msg__Int32 count_msg, missed_deadlines_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

const unsigned long time_sync_interval = 1000;
const unsigned long publish_interval = 100;
const int sync_timeout_ms = 500;

// TODO: Use better types
unsigned long last_time_sync_ms = 0;
unsigned long last_time_sync_ns = 0;
int64_t synced_time_ms = 0;
int64_t synced_time_ns = 0;

roboost::timing::TimingService& timing_service = roboost::timing::TimingService::get_instance(); // TODO: How to use this better for delta time?
roboost::timing::Task executor_task;

roboost::logging::SerialLogger& logger = roboost::logging::SerialLogger::getInstance(Serial);

IntervalTimer control_loop_timer;

volatile uint32_t control_loop_counter = 0;

// Function prototypes
void control_loop();
void led_subscription_callback(const void* msgin);
void count_publish();
void missed_deadlines_publish();
void pub_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time);
void init_count_msg();
void init_microros();
void time_sync();

/**
 * @brief Setup function for initializing micro-ROS, pin modes, etc.
 *
 */
void setup()
{
    // Configure serial transport
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    // Setup Timingservice
    // clang-format off
    executor_task = roboost::timing::Task([]() {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }, 100, 200, "microros_executor"); // execute every 100ms, 200ms timeout
    // clang-format on
    timing_service.addTask(executor_task);
    timing_service.reset();

    set_microros_serial_transports(Serial);

    init_microros();

    // TODO: How to share resources between the timer and the main loop?
    control_loop_timer.begin(control_loop, TIMING_MS_TO_NS(10));
}

/**
 * @brief Main loop
 *
 */
void loop() { timing_service.update(); }

void control_loop()
{
    // TODO: Add lock for control_loop_counter?
    control_loop_counter++;
}

void init_microros()
{
    allocator = rcl_get_default_allocator();

    INIT(rclc_support_init(&support, 0, NULL, &allocator), logger);
    INIT(rclc_node_init_default(&node, "teensy_microros", "", &support), logger);

    INIT(rclc_publisher_init_default(&count_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "count"), logger);
    INIT(rclc_publisher_init_default(&missed_deadlines_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "missed_deadlines"), logger);

    INIT(rclc_timer_init_default(&publish_timer, &support, RCL_MS_TO_NS(publish_interval), pub_timer_callback), logger);
    INIT(rclc_timer_init_default(&sync_timer, &support, RCL_MS_TO_NS(time_sync_interval), sync_timer_callback), logger);

    INIT(rclc_subscription_init_default(&led_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "led"), logger);

    INIT(rclc_executor_init(&executor, &support.context, 3, &allocator), logger);
    INIT(rclc_executor_add_subscription(&executor, &led_subscriber, &led_msg, &led_subscription_callback, ON_NEW_DATA), logger);
    INIT(rclc_executor_add_timer(&executor, &publish_timer), logger);
    INIT(rclc_executor_add_timer(&executor, &sync_timer), logger);

    init_count_msg();

    // Initial time sync
    time_sync();
}

void init_count_msg() { count_msg.data = 0; }

void led_subscription_callback(const void* msgin)
{
    const auto* msg = reinterpret_cast<const std_msgs__msg__Bool*>(msgin);

    digitalWrite(LED_BUILTIN, msg->data);
}

void count_publish()
{
    // Publish control loop counter
    // TODO: add lock for control_loop_counter?
    count_msg.data = control_loop_counter;
    RCSOFTCHECK(rcl_publish(&count_publisher, &count_msg, NULL), logger);
}

void missed_deadlines_publish()
{
    // Publish missed deadlines
    missed_deadlines_msg.data = executor_task.getMissedDeadlines();
    RCSOFTCHECK(rcl_publish(&missed_deadlines_publisher, &missed_deadlines_msg, NULL), logger);
}

void pub_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    if (timer == NULL)
    {
        Serial.println("Error in timer_callback: timer parameter is NULL\n");
        return;
    }

    count_publish();
    missed_deadlines_publish();
}

void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    if (timer == NULL)
    {
        Serial.println("Error in timer_callback: timer parameter is NULL\n");
        return;
    }

    time_sync();
}

void time_sync()
{
    // Synchronize time with the agent
    rmw_uros_sync_session(sync_timeout_ms);

    if (rmw_uros_epoch_synchronized())
    {
        // Get time in milliseconds or nanoseconds
        synced_time_ms = rmw_uros_epoch_millis();
        synced_time_ns = rmw_uros_epoch_nanos();
        last_time_sync_ms = millis();
        last_time_sync_ns = micros() * 1000;
    }
    else
    {
        Serial.println("Error in sync_timer_callback: time not synchronized\n");
    }
    Serial.println("Error in sync_timer_callback: time not synchronized\n");
}