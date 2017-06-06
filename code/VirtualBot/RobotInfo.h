#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="RobotInfo.h" />
///
/// <summary>
///     This module contains the Robot Info constants.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _ROBOTINFO_H
#define _ROBOTINFO_H

//
// Drive info.
//
#define GEAR_RATIO              (24.0/16.0)     //motor:wheel=24:16
#define WHEEL_CIRCUMFERENCE     13.00           //in inches
#define CLICKS_PER_REVOLUTION   1440.0          //in clicks
#define WHEELBASE_DISTANCE      60.00           //in inches

//
// Assuming the encoder is mounted on the motor shaft.
//
//#if 0
#define DISTANCE_PER_CLICK      (WHEEL_CIRCUMFERENCE*GEAR_RATIO/ \
                                 CLICKS_PER_REVOLUTION)
#define DEGREES_PER_CLICK       (360.0/(PI*WHEELBASE_DISTANCE/ \
                                        DISTANCE_PER_CLICK))
//#endif

//#define DISTANCE_PER_CLICK      1.0
//#define DEGREES_PER_CLICK       0.1

//
// PID Control constants.
//
#define ENCODER_DRIVE_KP        12.0
#define ENCODER_DRIVE_KI        0.0
#define ENCODER_DRIVE_KD        0.0
#define ENCODER_DRIVE_TOLERANCE 1.0
#define ENCODER_DRIVE_SETTLING  200

#define ENCODER_TURN_KP         10.0
#define ENCODER_TURN_KI         0.0
#define ENCODER_TURN_KD         0.0
#define ENCODER_TURN_TOLERANCE  2.0
#define ENCODER_TURN_SETTLING   200

#define GYRO_TURN_KP            12.0
#define GYRO_TURN_KI            0.0
#define GYRO_TURN_KD            0.0
#define GYRO_TURN_TOLERANCE     1.0
#define GYRO_TURN_SETTLING      200

#define IRSEEK_TURN_KP          10.0
#define IRSEEK_TURN_KI          0.0
#define IRSEEK_TURN_KD          0.0
#define IRSEEK_TURN_TOLERANCE   0.5
#define IRSEEK_TURN_SETTLING    200

#endif  //ifndef _ROBOTINFO_H
