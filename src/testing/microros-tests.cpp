#include <Arduino.h>
#include <SPIFFS.h>
#include <conf_network.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw/qos_profiles.h> // Include the QoS profiles header
#include <rmw_microros/rmw_microros.h>
#include <roboost/utils/logging.hpp>
#include <roboost/utils/rcl_checks.h>
#include <roboost/utils/rtos_task_manager.hpp>
#include <std_msgs/msg/int32.h>

using namespace roboost::logging;

WebLogger& webLogger = WebLogger::get_instance();

rcl_publisher_t simple_publisher;
std_msgs__msg__Int32 simple_msg;

rcl_timer_t simple_timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

const unsigned long simple_publish_interval = 500; // Publish every 500 ms

void simple_publish_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    if (timer != NULL)
    {
        simple_msg.data++;
        rcl_ret_t ret = rcl_publish(&simple_publisher, &simple_msg, NULL);
        if (ret != RCL_RET_OK)
        {
            webLogger.error("Failed to publish message");
        }
        else
        {
            // webLogger.debug("Published message");
        }
    }
}

void setup()
{
    Serial.begin(115200);

    webLogger.setup(SSID, SSID_PW);
    delay(1000); // Delay to allow for serial monitor to connect

    webLogger.info("Initializing Micro-ROS...");
    set_microros_serial_transports(Serial);

    webLogger.info("Initializing ROS 2 node...");
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "simple_publisher_node", "", &support);

    webLogger.info("Initializing ROS 2 publisher...");
    // Define a Best Effort QoS profile
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    custom_qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    custom_qos_profile.depth = 1; // Keep only the latest message

    // Initialize the publisher with the Best Effort QoS profile
    rclc_publisher_init(&simple_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "simple_publisher_topic", &custom_qos_profile);
    // rclc_publisher_init_default(&simple_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "simple_publisher_topic");

    webLogger.info("Initializing ROS 2 timer...");
    rclc_timer_init_default(&simple_timer, &support, RCL_MS_TO_NS(simple_publish_interval), simple_publish_callback);

    webLogger.info("Initializing ROS 2 executor...");
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &simple_timer);

    simple_msg.data = 0;
}

void loop()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1000));
    delay(10);
}
