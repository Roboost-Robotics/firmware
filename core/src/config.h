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
    #define M_BR_CCW 33
    #define M_BR_CW 32
    #define M_BR_PWM 25

    // motor back left
    #define M_BL_CW 26
    #define M_BL_CCW 27
    #define M_BL_PWM 13

    // motor front left
    #define M_FL_CW 23
    #define M_FL_CCW 22
    #define M_FL_PWM 21

    // motor front right
    #define M_FR_CCW 18
    #define M_FR_CW 14
    #define M_FR_PWM 19
    // PWM config
    #define M_BR_PWM_CNL 0
    #define M_BL_PWM_CNL 0
    #define M_FR_PWM_CNL 0
    #define M_FL_PWM_CNL 0

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