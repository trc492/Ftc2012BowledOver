#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S1,     ,                    sensorI2CMuxController)
#pragma config(Sensor, S2,     topLimitSW,          sensorI2CCustom)
#pragma config(Sensor, S3,     botLimitSW,          sensorI2CCustom)
#pragma config(Sensor, S4,     gyroSensor,          sensorI2CCustom)
#pragma config(Motor,  mtr_S1_C1_1,     conveyorMotor, tmotorNormal, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C1_2,     leftMotor,     tmotorNormal, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     elevatorMotor, tmotorNormal, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C2_2,     rightMotor,    tmotorNormal, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C4_1,     marMotorRight,      tmotorNormal, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C4_2,     marMotorLeft,        tmotorNone, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    leftArm,              tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    rightArm,             tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    leftWrist,            tServoStandard)
#pragma config(Servo,  srvo_S1_C3_4,    flap,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C3_5,    rightWrist,           tServoStandard)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define leftIRSensor            msensor_S4_1
#ifdef _USE_ACCEL
    #define accelSensor         msensor_S4_2
#else
    #define sonarSensor         msensor_S4_2
#endif
#define rightIRSensor           msensor_S4_3
#ifdef _USE_MUX
    #define gyroSensor          msensor_S4_4
#endif
//
// Motor color codes:
//  leftMotor - Blue
//  rightMotor - Orange
//  elevator - Yellow
//  conveyor - Magenta
//

#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TeleOp.c" />
///
/// <summary>
///     This module contains the entry point for teleoperator mode.
///     This is template file and should not be modified. The main competition
///     code should live in TeleOpMain.h.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#define _COMPETITION_ENABLED
//#define _ARCADE_DRIVE

#include "..\RobotCDrivers\drivers\HTSMUX-driver.h"
#include "JoystickDriver.c"
#include "..\ftclib\trcdefs.h"
#include "..\ftclib\dbgtrace.h"
#include "RobotInfo.h"
#include "TeleOpMain.h"

/**
 *  This task is the program entry point.
 */
task main()
{
    long nextTime;
    long currTime;

    TraceInit(TRACE_MODULES, TRACE_LEVEL, MSG_LEVEL);

    //
    // The RobotInit function is provided by student code in TeleOpMain.h.
    //
    RobotInit();

    //
    // Wait for the beginning of teleop mode.
    //
#ifdef _COMPETITION_ENABLED
    waitForStart();
#endif

    nextTime = nPgmTime;
    while (true)
    {
        currTime = nPgmTime;
        HiFreqTasks();
        if (currTime >= nextTime)
        {
            nextTime = currTime + LOOP_TIME;
            getJoystickSettings(joystick);
            //
            // The following functions are provided by student code in
            // TeleMain.h.
            //
            InputTasks();
            MainTasks();
            OutputTasks();
        }
        EndTimeSlice();
    }
}   //main
