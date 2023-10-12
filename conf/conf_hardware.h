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

#include <stdint.h>

/**
 * @brief ESP32 specific configs
 *
 */
const uint8_t LED_BUILTIN = 2;

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

// Bei 10 umdreheungen einen fehler von realeposition = gemesseneposition +
// PI/2i

const float CORRECTION_FACTOR = 1.0;                   // ~0.7;
const float WHEEL_RADIUS = 0.0625 * CORRECTION_FACTOR; // radius of wheels
const float WHEEL_BASE =
    0.315; // distance between wheel contact point in x direction
const float TRACK_WIDTH =
    0.39; // distance between wheel contact point in y direction

//--------------------------pinout
// definitions------------------------------------

// motor front left
const uint8_t M0_IN1 = 23;
const uint8_t M0_IN2 = 22;
const uint8_t M0_ENA = 21;

// motor front right
const uint8_t M1_IN1 = 14;
const uint8_t M1_IN2 = 18;
const uint8_t M1_ENA = 19;

// motor back left
const uint8_t M2_IN1 = 26;
const uint8_t M2_IN2 = 27;
const uint8_t M2_ENA = 13;

// motor back right
const uint8_t M3_IN1 = 32;
const uint8_t M3_IN2 = 33;
const uint8_t M3_ENA = 25;

// PWM config
const uint8_t M0_PWM_CNL = 0;
const uint8_t M1_PWM_CNL = 1;
const uint8_t M2_PWM_CNL = 2;
const uint8_t M3_PWM_CNL = 3;

const uint16_t M_PWM_FRQ = 1000; // Hz
const uint8_t M_PWM_RES = 8;     // 2^n Bits
#endif

// Uncomment if encoders should be used in the system
#define ENCODERS
#ifdef ENCODERS

// encoder front left
const uint8_t M0_ENC_A = 17;
const uint8_t M0_ENC_B = 16;
const uint16_t M0_ENC_RESOLUTION = 360;

// encoder front right
const uint8_t M1_ENC_A = 5;
const uint8_t M1_ENC_B = 15;
const uint16_t M1_ENC_RESOLUTION = 600;

// encoder back left
const uint8_t M2_ENC_A = 39;
const uint8_t M2_ENC_B = 36;
const uint16_t M2_ENC_RESOLUTION = 360;

// encoder back right
const uint8_t M3_ENC_A = 35;
const uint8_t M3_ENC_B = 34;
const uint16_t M3_ENC_RESOLUTION = 360;

#endif

#endif // CONF_HARDWARE_H