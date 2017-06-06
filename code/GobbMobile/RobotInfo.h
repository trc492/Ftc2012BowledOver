#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="GobbInfo.h" />
///
/// <summary>
///     This module contains the GobbMobile information constants.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _GOBBINFO_H
#define _GOBBINFO_H

//#define _TRIBOT

//
// Drive info.
//
#ifdef _TRIBOT
    #define WHEEL_CIRCUMFERENCE 6.92    // in inches
    #define WHEELBASE_DISTANCE  8.75    // in inches
#else
    #define WHEEL_CIRCUMFERENCE 6.78    // in inches
    #define WHEELBASE_DISTANCE  13.15   // in inches
#endif
#define CLICKS_PER_REVOLUTION   360.0
#define DISTANCE_PER_CLICK      (WHEEL_CIRCUMFERENCE/CLICKS_PER_REVOLUTION)
#define DEGREES_PER_CLICK       (360.0/(PI*WHEELBASE_DISTANCE/ \
                                        DISTANCE_PER_CLICK))

#define ENCODER_DRIVE_KP        50.0
#define ENCODER_DRIVE_KI        0.0
#define ENCODER_DRIVE_KD        0.0
#define ENCODER_DRIVE_TOLERANCE 1.0
#define ENCODER_DRIVE_SETTLING  200

#define ENCODER_TURN_KP         5.0
#define ENCODER_TURN_KI         0.0
#define ENCODER_TURN_KD         0.0
#define ENCODER_TURN_TOLERANCE  1.0
#define ENCODER_TURN_SETTLING   200

#define GYRO_ERR_ADJ            0.95
#define GYRO_KP                 6.0
#define GYRO_KI                 0.0
#define GYRO_KD                 0.0
#define GYRO_TOLERANCE          1.0
#define GYRO_SETTLING           200

#define SONAR_KP                1.0
#define SONAR_KI                0.0
#define SONAR_KD                0.0
#define SONAR_TOLERANCE         0.5
#define SONAR_SETTLING          200

#define IRSEEK_KP               20.0
#define IRSEEK_KI               0.0
#define IRSEEK_KD               0.0
#define IRSEEK_TOLERANCE        0.1
#define IRSEEK_SETTLING         200

#define LNFOLLOW_LIGHT_THRESHOLD 460
#define LNFOLLOW_KP             0.5
#define LNFOLLOW_KI             0.0
#define LNFOLLOW_KD             0.0
#define LNFOLLOW_TOLERANCE      0.10
#define LNFOLLOW_SETTLING       200

#define RADAR_KP                5.0
#define RADAR_MOTOR_LIMIT       50
#define RADAR_FRONT_ANGLE       0.0
#define RADAR_SIDE_ANGLE        -90.0

#define WALLFOLLOW_KP           10.0
#define WALLFOLLOW_KI           0.0
#define WALLFOLLOW_KD           0.0
#define WALLFOLLOW_TOLERANCE    0.5
#define WALLFOLLOW_SETTLING     200

#endif  //ifndef _GOBBINFO_H
