/**
 * @file initialization.hpp
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief Utility functions for initialization.
 * @version 0.1
 * @date 2023-10-08
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef INITIALIZATION_H
#define INITIALIZATION_H

#include <functional>
#include <rcl/rcl.h>
#include <Arduino.h>
#include "conf_hardware.h"

bool performInitializationWithFeedback(std::function<rcl_ret_t()> initFunction)
{
    while (true)
    {
        rcl_ret_t ret = initFunction();
        
        switch (ret)
        {
        case RCL_RET_OK:
            return true;
        case RCL_RET_ACTION_CLIENT_INVALID:
            Serial.println("RCL_RET_ACTION_CLIENT_INVALID");
            break;
        case RCL_RET_ACTION_SERVER_INVALID:
            Serial.println("RCL_RET_ACTION_SERVER_INVALID");
            break;
        case RCL_RET_BAD_ALLOC:
            Serial.println("RCL_RET_BAD_ALLOC");
            break;
        case RCL_RET_ERROR:
            Serial.println("RCL_RET_ERROR");
            break;
        case RCL_RET_INVALID_ARGUMENT:
            Serial.println("RCL_RET_INVALID_ARGUMENT");
            break;
        case RCL_RET_INVALID_ROS_ARGS:
            Serial.println("RCL_RET_INVALID_ROS_ARGS");
            break;
        case RCL_RET_NODE_INVALID:
            Serial.println("RCL_RET_NODE_INVALID");
            break;
        case RCL_RET_NODE_INVALID_NAME:
            Serial.println("RCL_RET_NODE_INVALID_NAME");
            break;
        case RCL_RET_NODE_INVALID_NAMESPACE:
            Serial.println("RCL_RET_NODE_INVALID_NAMESPACE");
            break;
        case RCL_RET_NOT_INIT:
            Serial.println("RCL_RET_NOT_INIT");
            break;
        case RCL_RET_PUBLISHER_INVALID:
            Serial.println("RCL_RET_PUBLISHER_INVALID");
            break;
        case RCL_RET_SERVICE_NAME_INVALID:
            Serial.println("RCL_RET_SERVICE_NAME_INVALID");
            break;
        case RCL_RET_SUBSCRIPTION_INVALID:
            Serial.println("RCL_RET_SUBSCRIPTION_INVALID");
            break;
        case RCL_RET_TOPIC_NAME_INVALID:
            Serial.println("RCL_RET_TOPIC_NAME_INVALID");
            break;
        case RCL_RET_UNKNOWN_SUBSTITUTION:
            Serial.println("RCL_RET_UNKNOWN_SUBSTITUTION");
            break;
        case RCL_RET_UNSUPPORTED:
            Serial.println("RCL_RET_UNSUPPORTED");
            break;
        default:
            Serial.println("Unknown error");
            break;
        }

        Serial.println("Going into error state. Press reset to restart.");

        // Infinite loop to indicate error by blinking the LED
        while (true)
        {
            // Serial.println(error_msg);
            digitalWrite(LED_BUILTIN, HIGH);
            delay(500);
            digitalWrite(LED_BUILTIN, LOW);
            delay(500);
        }
    }
}

#define INIT(initCall) performInitializationWithFeedback([&]() { return (initCall); })

#endif // INITIALIZATION_H