/**
 * @file rcl_checks.h //TODO
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-07-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef RCL_CHECKS_H
#define RCL_CHECKS_H

#include <Arduino.h>

// Error handle loop
/**
 * @brief //TODO
 * 
 */
inline void error_loop() {
  while(1) {
    Serial.print(millis());
    Serial.println(" RC check failed. Press EN to reset.");
    delay(100);
  }
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#endif // RCL_CHECKS_H
