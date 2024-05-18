#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <rmw_microros/rmw_microros.h>
#include <roboost/utils/logging.hpp>
#include <roboost/utils/rcl_checks.h>
#include <roboost/utils/rtos_task_manager.hpp>
#include <std_msgs/msg/int32.h>

using namespace roboost::logging;

SerialLogger& logger = SerialLogger::get_instance();

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
        RCSOFTCHECK(rcl_publish(&simple_publisher, &simple_msg, NULL), logger);
    }
}

void setup()
{
    Serial.begin(115200);

    logger.set_serial(Serial);
    set_microros_serial_transports(Serial);

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "simple_publisher_node", "", &support);

    rclc_publisher_init_default(&simple_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "simple_publisher_topic");

    rclc_timer_init_default(&simple_timer, &support, RCL_MS_TO_NS(simple_publish_interval), simple_publish_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &simple_timer);

    // Init RCL options and context
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_context_t context = rcl_get_zero_initialized_context();
    rcl_init_options_init(&init_options, rcl_get_default_allocator());

    // Take RMW options from RCL options
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    // TCP/UDP case: Set RMW IP parameters
    // rmw_uros_options_set_udp_address("127.0.0.1", "8888", rmw_options);

    // Serial case: Set RMW serial device parameters
    rmw_uros_options_set_serial_device("/dev/ttyAMA0", rmw_options);

    // Set RMW client key
    rmw_uros_options_set_client_key(0xBA5EBA11, rmw_options);

    // Init RCL
    rcl_init(0, NULL, &init_options, &context);
}

void loop()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    delay(10); // Small delay to prevent busy looping
}
