/**
 * @file conf_hardware.h
 * @author Friedl Jakob (friedl.jak@gmail.com)
 * @brief Configuration file for the Roboost firmware.
 * @version 0.1
 * @date 2023-03-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef CONF_HARDWARE_H
#define CONF_HARDWARE_H

/**
 * @brief ESP32 specific configs
 *
 */
#define LED_BUILTIN 2

/**
 * @brief Selection of robot kinematics.
 * MECANUM_4WHEEL: Kinematics of a four wheeled mecanum drive.
 * SWERVE_3WHEEL: Kinematics of a robot with three swerve drives.
 */
#define MECANUM_4WHEEL
// #define SWERVE_3WHEEL

#ifdef MECANUM_4WHEEL
/**
 * @brief Definitions of hardware parameters. Leghts in meters, angles in
 * degree.
 *
 */
#define WHEEL_RADIUS 0.075 // radius of wheels
#define WHEEL_BASE 0.38    // distance between wheel contact point in x direction
#define TRACK_WIDTH 0.32   // distance between wheel contact point in y direction

//--------------------------pinout
// definitions------------------------------------

// motor front left
#define M0_IN1 23
#define M0_IN2 22
#define M0_ENA 21

// motor front right
#define M1_IN1 18
#define M1_IN2 14
#define M1_ENA 19

// motor back left
#define M2_IN1 26
#define M2_IN2 27
#define M2_ENA 13

// motor back right
#define M3_IN1 33
#define M3_IN2 32
#define M3_ENA 25

// PWM config
#define M0_PWM_CNL 0
#define M1_PWM_CNL 1
#define M2_PWM_CNL 2
#define M3_PWM_CNL 3

#define M_PWM_FRQ 1000 // Hz
#define M_PWM_RES 8    // 2^n Bits
#endif

// Uncomment if encoders should be used in the system
#define ENCODERS
#ifdef ENCODERS

// encoder front left
#define M0_ENC_A 5
#define M0_ENC_B 15
#define M0_ENC_RESOLUTION 360

// encoder front right
#define M1_ENC_A 17
#define M1_ENC_B 16
#define M1_ENC_RESOLUTION 600

// encoder back left
#define M2_ENC_A 35
#define M2_ENC_B 34
#define M2_ENC_RESOLUTION 360

// encoder back right
#define M3_ENC_A 39
#define M3_ENC_B 36
#define M3_ENC_RESOLUTION 360

#endif

//--------------------------------------------------------------------------------

// todo refactor

#ifdef ENCODERS
// Encoder specific definitions and functions

// B pin of encoder is not used. The direction of the motors will be deduced
// from H-Bridge. This is, however, a precision flaw
volatile uint16_t count_BL = 0;
volatile uint16_t count_BR = 0;
volatile uint16_t count_FL = 0;
volatile uint16_t count_FR = 0;

// Interrup routines
void IRAM_ATTR function_ISR_EC_BL()
{
    // Encoder out A triggers interrupt
    // todo check last B state to determine direction
    count_BL++;
}

void IRAM_ATTR function_ISR_EC_BR() { count_BR++; }

void IRAM_ATTR function_ISR_EC_FL() { count_FL++; }

void IRAM_ATTR function_ISR_EC_FR() { count_FR++; }

#else

#endif

#endif // CONF_HARDWARE_H