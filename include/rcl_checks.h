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

#include <Arduino.h>
#include <conf_hardware.h>

/**
 * @brief This function is used to check the return value of rcl functions.
 * If the return value is not RCL_RET_OK, the error_loop() function is called.
 *
 * @param fn The function to be checked.
 */
inline void error_loop()
{
    while (true)
    {
        Serial.println("Error in rcl function");
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    }
}

#define RCCHECK(fn)                                                                                                    \
    {                                                                                                                  \
        rcl_ret_t temp_rc = fn;                                                                                        \
        if ((temp_rc != RCL_RET_OK))                                                                                   \
        {                                                                                                              \
            error_loop();                                                                                              \
        }                                                                                                              \
    }
#define RCSOFTCHECK(fn)                                                                                                \
    {                                                                                                                  \
        rcl_ret_t temp_rc = fn;                                                                                        \
        if ((temp_rc != RCL_RET_OK))                                                                                   \
        {                                                                                                              \
        }                                                                                                              \
    }

#endif // RCL_CHECKS_H
