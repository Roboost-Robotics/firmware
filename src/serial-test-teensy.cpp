#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "rcl_checks.h"
#include <rcl/rcl.h>
#include <rclc/executor.h>

#include <rclc/rclc.h>
#include <rosidl_runtime_c/string_functions.h>

#include <roboost_msgs/msg/temp_humidity.h>

rcl_publisher_t temp_humidity_publisher;
roboost_msgs__msg__TempHumidity temp_humidity_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

IntervalTimer timer;

// Define a variable to keep track of the LED state
bool ledState = false;

bool performInitializationWithFeedback(std::function<rcl_ret_t()> initFunction)
{
    while (true)
    {
        if (initFunction() == RCL_RET_OK)
        {
            return true; // Initialization successful
        }
        else
        {
            // Flash LED to indicate failure
            // digitalWrite(LED_BUILTIN, HIGH);
            // delay(1000);
            // digitalWrite(LED_BUILTIN, LOW);
            // delay(100);
            while (true)
                ; // Infinite loop to indicate failure
        }
    }
}

#define INIT(initCall)                                                         \
    performInitializationWithFeedback([&]() { return (initCall); })

void led_timer_callback() // rcl_timer_t * timer, int64_t last_call_time)
{
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
}

void my_timer_callback(rcl_timer_t* timer, int64_t last_call_time)
{
    if (timer != NULL)
    {
        RCSOFTCHECK(
            rcl_publish(&temp_humidity_publisher, &temp_humidity_msg, NULL));
    }
    else
    {
        printf("Error in timer_callback: timer parameter is NULL\n");
    }
}

void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    pinMode(LED_BUILTIN, OUTPUT);

    delay(2000);

    allocator = rcl_get_default_allocator();

    // Use the IntervalTimer to blink the LED
    timer.begin(led_timer_callback, 500000);

    // Initialize support and node
    INIT(rclc_support_init(&support, 0, NULL, &allocator));
    INIT(
        rclc_node_init_default(&node, "micro_ros_tutorial_node", "", &support));

    // Initialize timers
    rcl_timer_t my_timer;
    // rcl_timer_t led_timer;
    const unsigned int timer_timeout = 1000; // in ms for sensor data
    // const unsigned int led_timer_timeout = 500; // in ms for LED blinking
    INIT(rclc_timer_init_default(
        &my_timer, &support, RCL_MS_TO_NS(timer_timeout), my_timer_callback));
    // INIT(rclc_timer_init_default(&led_timer, &support,
    // RCL_MS_TO_NS(led_timer_timeout), led_timer_callback));

    // Initialize publisher
    INIT(rclc_publisher_init_default(
        &temp_humidity_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(roboost_msgs, msg, TempHumidity),
        "TempHumidity"));

    // Setup executor
    executor = rclc_executor_get_zero_initialized_executor();

    // # handles = # timers + # subscriptions
    const uint8_t num_handles = 1; // 2;
    INIT(rclc_executor_init(&executor, &support.context, num_handles,
                            &allocator));
    INIT(rclc_executor_add_timer(&executor, &my_timer));
    // INIT(rclc_executor_add_timer(&executor, &led_timer));

    temp_humidity_msg.temperature = 25.0;
    temp_humidity_msg.humidity = 50.0;

    // Spin the executor in the setup (this will block any further code in setup
    // or loop)
    rclc_executor_spin(&executor);
}

void loop()
{
    // Loop is intentionally left empty
}
