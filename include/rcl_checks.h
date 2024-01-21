/**
 * @file rcl_checks.h
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief This file contains the rcl checks for the micro-ROS communication.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef RCL_CHECKS_H
#define RCL_CHECKS_H

#include "conf_hardware.h"
#include <Arduino.h>

/**
 * @brief This function is used to blink the LED to indicate an error and retry.
 */
inline void error_indication()
{
    for (int i = 0; i < 5; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }
}

#define RCCHECK(fn)                                                            \
    {                                                                          \
        rcl_ret_t temp_rc = fn;                                                \
        while ((temp_rc != RCL_RET_OK))                                        \
        {                                                                      \
            error_indication();                                                \
            temp_rc = fn;                                                      \
        }                                                                      \
    }
#define RCSOFTCHECK(fn)                                                        \
    {                                                                          \
        rcl_ret_t temp_rc = fn;                                                \
        if ((temp_rc != RCL_RET_OK))                                           \
        {                                                                      \
        }                                                                      \
    }

#endif // RCL_CHECKS_H
