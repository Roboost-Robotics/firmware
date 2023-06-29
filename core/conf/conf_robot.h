/**
 * @file config.h
 * @author Friedl Jakob (friedl.jak@gmail.com)
 * @brief Configuration file for the Roboost firmware.
 * @version 0.1
 * @date 2023-03-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef CONF_ROBOT_H
#define CONF_ROBOT_H

/**
 * @brief Selection of robot kinematics.
 * MECANUM_4WHEEL: Kinematics of a four wheeled mecanum drive.
 * SWERVE_3WHEEL: Kinematics of a robot with three swerve drives.
 */
#define MECANUM_4WHEEL
//#define SWERVE_3WHEEL


#ifdef MECANUM_4WHEEL
    /**
    * @brief Definitions of hardware parameters. Leghts in meters, angles in degree.
    * 
    */
    #define WHEELRADIUS 0.075 // radius of wheels
    #define L_X 0.38 // distance between wheel contact point in x direction
    #define L_Y 0.32 // distance between wheel contact point in y direction
//--------------------------pinout definitions------------------------------------
    // motor back right
    #define M3_IN1 33
    #define M3_IN2 32
    #define M3_ENA 25

    // motor back left
    #define M2_IN1 26
    #define M2_IN2 27
    #define M2_ENA 13

    // motor front left
    #define M0_IN1 23
    #define M0_IN2 22
    #define M0_ENA 21

    // motor front right
    #define M1_IN1 18
    #define M1_IN2 14
    #define M1_ENA 19
    // PWM config
    #define M3_PWM_CNL 0
    #define M2_PWM_CNL 0
    #define M0_PWM_CNL 0
    #define M1_PWM_CNL 0

    #define M_PWM_FRQ 1000 // Hz
    #define M_PWM_RES 8 // 2^n Bits
#endif

// Uncomment if encoders should be used in the system
//#define ENCODERS
#ifdef ENCODERS

    // encoder back right
    #define EC_BR_A 39
    #define EC_BR_B 36

    // encoder back left
    #define EC_BL_A 35
    #define EC_BL_B 34

    // encoder front left
    #define EC_FL_A 5
    #define EC_FL_B 15

    // encoder front right
    #define EC_FR_A 17
    #define EC_FR_B 16

#endif

//--------------------------------------------------------------------------------

// TODO: refactor

#ifdef ENCODERS
// Encoder specific definitions and functions

  // B pin of encoder is not used. The direction of the motors will be deduced from H-Bridge. This is, however, a precision flaw
  volatile uint16_t count_BL = 0;
  volatile uint16_t count_BR = 0;
  volatile uint16_t count_FL = 0;
  volatile uint16_t count_FR = 0;

  // Interrup routines
  void IRAM_ATTR function_ISR_EC_BL() {
    // Encoder out A triggers interrupt
    // TODO: check last B state to determine direction
    count_BL++;
  }

  void IRAM_ATTR function_ISR_EC_BR() {
    count_BR++;
  }

  void IRAM_ATTR function_ISR_EC_FL() {
    count_FL++;
  }

  void IRAM_ATTR function_ISR_EC_FR() {
    count_FR++;
  }

#else

#endif

#endif // CONF_ROBOT_H