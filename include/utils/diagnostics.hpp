/**
 * @file diagnostics.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Diagnostic utilities for publishing diagnostic messages.
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include <diagnostic_msgs/msg/diagnostic_status.h>
#include <utils/initialization.hpp>
#include <rcl_checks.h>
#include <rclc/rclc.h>
#include <Arduino.h>

rcl_publisher_t diagnostic_publisher;

diagnostic_msgs__msg__DiagnosticStatus diagnostic_msg;

/**
 * @brief Initializes the diagnostics publisher.
 * 
 * @param node The node to use for the diagnostics publisher.
 */
void initDiagnostics(rcl_node_t node)
{
    INIT(rclc_publisher_init_default(&diagnostic_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticStatus), "diagnostics"));
}

/**
 * @brief Publishes a diagnostic message with the given message.
 *
 * @param message The message to publish.
 */
void publishDiagnosticMessage(const char* message)
{
    diagnostic_msg.level = diagnostic_msgs__msg__DiagnosticStatus__STALE;
    diagnostic_msg.message.data = (char*)message;
    diagnostic_msg.message.size = strlen(diagnostic_msg.message.data);
    diagnostic_msg.message.capacity = diagnostic_msg.message.size + 1;

    RCSOFTCHECK(rcl_publish(&diagnostic_publisher, &diagnostic_msg, NULL));
}

// Template function to measure execution time of any callable function
template<typename Func, typename... Args>
unsigned long measureExecutionTime(Func func, Args&&... args) {
    unsigned long startTime = micros();
    func(std::forward<Args>(args)...);  // Execute the function with any provided arguments
    unsigned long endTime = micros();
    return endTime - startTime;  // Return the execution time in microseconds
}

#endif // DIAGNOSTICS_H