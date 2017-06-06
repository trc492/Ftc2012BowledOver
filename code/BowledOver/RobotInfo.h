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
#define GEAR_RATIO              (16.0/24.0)     //motor:wheel=16:24
#define WHEEL_CIRCUMFERENCE     12.301           //in inches
#define WHEELBASE_DISTANCE      33.50           //in inches
#define CLICKS_PER_REVOLUTION   1440.0          //in clicks

//
// Assuming the encoder is mounted on the motor shaft.
//
#define DISTANCE_PER_CLICK      (WHEEL_CIRCUMFERENCE*GEAR_RATIO/ \
                                 CLICKS_PER_REVOLUTION)
#define DEGREES_PER_CLICK       (360.0/(PI*WHEELBASE_DISTANCE/ \
                                        DISTANCE_PER_CLICK))

//
// PID Control constants.
//
#define ENCODER_DRIVE_KP        15.0
#define ENCODER_DRIVE_KI        0.0
#define ENCODER_DRIVE_KD        1.0
#define ENCODER_DRIVE_TOLERANCE 2.0
#define ENCODER_DRIVE_SETTLING  200

#define ENCODER_TURN_KP         0.25
#define ENCODER_TURN_KI         0.0
#define ENCODER_TURN_KD         0.0
#define ENCODER_TURN_TOLERANCE  1.0
#define ENCODER_TURN_SETTLING   200

// Last good set: Kp=2.0, Ki=0.1, Kd=10.0
#define GYRO_ERR_ADJ            1.0
#define GYRO_KP                 2.5
#define GYRO_KI                 0.3
#define GYRO_KD                 20.0
#define GYRO_TOLERANCE          2.0
#define GYRO_SETTLING           200

#define ACCEL_TILT_THRESHOLD    1.5

#define SONAR_KP                6.2
#define SONAR_KI                0.01
#define SONAR_KD                0.5
#define SONAR_TOLERANCE         0.5
#define SONAR_SETTLING          200

#define IRSEEK_KP               15.0    //???
#define IRSEEK_KI               0.1
#define IRSEEK_KD               0.5
#define IRSEEK_TOLERANCE        0.5
#define IRSEEK_SETTLING         200

#define ELEVATOR_KP             15.0    //???
#define ELEVATOR_KI             0.0
#define ELEVATOR_KD             0.0
#define ELEVATOR_TOLERANCE      1.0
#define ELEVATOR_SETTLING       200
#define ELEVATOR_CAL_POWER      40
#define ELEVATOR_MAX_POWER      50
#define ELEVATOR_MIN_HEIGHT     0.0
#define ELEVATOR_MAX_HEIGHT     8.0
#define ELEVATOR_DOWN_POS       2.0
#define ELEVATOR_UP_POS         6.0
#define ELEVATOR_HEIGHT_PER_CLICK (2.0/1440)    //inches per click
#define ELEVATOR_HEIGHT_ABOVE_FLAP 6.0          //in Inches

#define CONVEYOR_POWER          50

#define ARM_LEFT_OPENED         120.0
#define ARM_RIGHT_OPENED        60.0
#define ARM_LEFT_CLOSED         65.0
#define ARM_RIGHT_CLOSED        115.0
#define ARM_LEFT_GRAB_BALL      45.0
#define ARM_RIGHT_GRAB_BALL     135.0
#define ARM_OPENCLOSE_TIME      100

#define LEFT_WRIST_UP           20.0
#define LEFT_WRIST_DOWN         200.0
#define RIGHT_WRIST_UP          180.0
#define RIGHT_WRIST_DOWN        0.0
//#define WRIST_HOLD_BALL         120.0

#define FLAP_OPENED             090.0
#define FLAP_CLOSED             190.0
#define FLAP_OPENCLOSE_TIME     200

#endif  //ifndef _ROBOTINFO_H
