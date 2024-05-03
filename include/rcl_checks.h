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
#include <rcl/rcl.h>
#include <utils/logging.hpp>

namespace roboost
{
    namespace rcl_utils
    {
        using namespace roboost::logging;

        inline void error_loop(Logger& logger) // Pass logger as a parameter
        {
            while (true)
            {
                logger.error("Error in rcl function"); // Use logger instead of Serial
                digitalWrite(LED_BUILTIN, HIGH);
                delay(500);
                digitalWrite(LED_BUILTIN, LOW);
                delay(500);
            }
        }

        bool performInitializationWithFeedback(Logger& logger, std::function<rcl_ret_t()> initFunction)
        {
            while (true)
            {
                rcl_ret_t ret = initFunction();

                switch (ret)
                {
                    case RCL_RET_OK:
                        return true;
                    case RCL_RET_BAD_ALLOC:
                        logger.error("RCL_RET_BAD_ALLOC");
                        break;
                    case RCL_RET_ERROR:
                        logger.error("RCL_RET_ERROR");
                        break;
                    case RCL_RET_INVALID_ARGUMENT:
                        logger.error("RCL_RET_INVALID_ARGUMENT");
                        break;
                    case RCL_RET_INVALID_ROS_ARGS:
                        logger.error("RCL_RET_INVALID_ROS_ARGS");
                        break;
                    case RCL_RET_NODE_INVALID:
                        logger.error("RCL_RET_NODE_INVALID");
                        break;
                    case RCL_RET_NODE_INVALID_NAME:
                        logger.error("RCL_RET_NODE_INVALID_NAME");
                        break;
                    case RCL_RET_NODE_INVALID_NAMESPACE:
                        logger.error("RCL_RET_NODE_INVALID_NAMESPACE");
                        break;
                    case RCL_RET_NOT_INIT:
                        logger.error("RCL_RET_NOT_INIT");
                        break;
                    case RCL_RET_PUBLISHER_INVALID:
                        logger.error("RCL_RET_PUBLISHER_INVALID");
                        break;
                    case RCL_RET_SERVICE_NAME_INVALID:
                        logger.error("RCL_RET_SERVICE_NAME_INVALID");
                        break;
                    case RCL_RET_SUBSCRIPTION_INVALID:
                        logger.error("RCL_RET_SUBSCRIPTION_INVALID");
                        break;
                    case RCL_RET_TOPIC_NAME_INVALID:
                        logger.error("RCL_RET_TOPIC_NAME_INVALID");
                        break;
                    case RCL_RET_UNKNOWN_SUBSTITUTION:
                        logger.error("RCL_RET_UNKNOWN_SUBSTITUTION");
                        break;
                    case RCL_RET_UNSUPPORTED:
                        logger.error("RCL_RET_UNSUPPORTED");
                        break;
                    default:
                        logger.error("Unknown error");
                        break;
                }

                logger.error("Going into error state. Press reset to restart.");
                error_loop(logger); // Pass logger to error_loop
            }
        }

#define INIT(initCall, logger) roboost::rcl_utils::performInitializationWithFeedback(logger, [&]() { return (initCall); })

#define RCCHECK(fn, logger)                                                                                                                                                                            \
    {                                                                                                                                                                                                  \
        rcl_ret_t temp_rc = fn;                                                                                                                                                                        \
        if ((temp_rc != RCL_RET_OK))                                                                                                                                                                   \
        {                                                                                                                                                                                              \
            error_loop(logger);                                                                                                                                                                        \
        }                                                                                                                                                                                              \
    }

#define RCSOFTCHECK(fn, logger)                                                                                                                                                                        \
    {                                                                                                                                                                                                  \
        rcl_ret_t temp_rc = fn;                                                                                                                                                                        \
        if ((temp_rc != RCL_RET_OK))                                                                                                                                                                   \
        {                                                                                                                                                                                              \
            logger.warn("Soft error detected");                                                                                                                                                        \
        }                                                                                                                                                                                              \
    }

    } // namespace rcl_utils
} // namespace roboost

#endif // RCL_CHECKS_H
