#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="servo.h" />
///
/// <summary>
///     This module contains the library functions for the servo subsystem.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _SERVO_H
#define _SERVO_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_SERVO

//
// Constants.
//
#define SERVOF_CONTINUOUS_ENABLED       0x0100
#define SERVOF_CONTINUOUS               0x0200

#define SERVO_MIN_VALUE                 0
#define SERVO_MAX_VALUE                 255

#define SERVO_CONTINUOUS_STOP           127
#define SERVO_CONTINUOUS_FWD_MAX        255
#define SERVO_CONTINUOUS_REV_MAX        0
#define SERVO_CONTINUOUS_LOW            -127
#define SERVO_CONTINUOUS_HIGH           128

#ifndef SERVO_ANGLE_RANGE
    #define SERVO_ANGLE_RANGE           200.0
    #define SERVO_POS_PER_DEGREE ((SERVO_MAX_VALUE - SERVO_MIN_VALUE)/ \
                                  SERVO_ANGLE_RANGE)
#endif

//
// Type definitions.
//
typedef struct
{
    TServoIndex servoMotor;
    int         servoFlags;
    tSensors    lowerTouchID;
    tSensors    upperTouchID;
    int         speed;
    float posPerDegree;
} SERVO;

/**
 *  This function initializes a standard servo motor.
 *
 *  @param serv Points to the SERVO structure to be initialized.
 *  @param servoMotor Specifies the servo motor ID.
 *  @param posPerDegree Specifies the position value per degree.
 *  @param initDegrees Specifies the initial position in degrees.
 *  @param servoFlags Specifies the servo flags.
 */
void
ServoInit(
    __out SERVO &serv,
    __in  TServoIndex servoMotor,
    __in  float initDegrees,
    __in  float posPerDegree = SERVO_POS_PER_DEGREE
    )
{
    TFuncName("ServoInit");
    TLevel(INIT);
    TEnter();

    serv.servoMotor = servoMotor;
    serv.servoFlags = 0;
    serv.lowerTouchID = (tSensors)-1;
    serv.upperTouchID = (tSensors)-1;
    serv.speed = SERVO_CONTINUOUS_STOP;
    serv.posPerDegree = posPerDegree;
    servo[servoMotor] = initDegrees*posPerDegree;

    TExit();
    return;
}   //ServoInit

/**
 *  This function initializes a continuous servo motor.
 *
 *  @param serv Points to the SERVO structure to be initialized.
 *  @param servoMotor Specifies the servo motor ID.
 *  @param lowerTouchID Specifies the sensor ID for the lower limit switch.
 *  @param upperTouchID Specifies the sensor ID for the upper limit switch.
 */
void
ServoInit(
    __out SERVO &serv,
    __in  TServoIndex servoMotor,
    __in  tSensors lowerTouchID = -1,
    __in  tSensors upperTouchID = -1
    )
{
    TFuncName("ServoInit");
    TLevel(INIT);
    TEnter();

    serv.servoMotor = servoMotor;
    serv.servoFlags = SERVOF_CONTINUOUS;
    serv.lowerTouchID = lowerTouchID;
    serv.upperTouchID = upperTouchID;
    serv.speed = SERVO_CONTINUOUS_STOP;
    serv.posPerDegree = 0;
    servo[servoMotor] = serv.speed;

    TExit();
    return;
}   //ServoInit

/**
 *  This function gets the current servo position in degrees.
 *
 *  @param serv Points to the SERVO structure.
 *
 *  @return Returns the current servo position in degrees.
 */
float
ServoGetAngle(
    __out SERVO &serv
    )
{
    float degree = 0.0;

    TFuncName("ServoGetPos");
    TLevel(API);
    TEnter();

    if (!(serv.servoFlags & SERVOF_CONTINUOUS))
    {
        degree = (float)ServoValue[serv.servoMotor]/serv.posPerDegree;
    }

    TExitMsg(("=%5.1f", degree));
    return degree;
}   //ServoGetAngle

/**
 *  This function sets servo position in degrees.
 *
 *  @param serv Points to the SERVO structure.
 *  @param degree Specifies the servo position in degrees.
 */
void
ServoSetAngle(
    __out SERVO &serv,
    __in  float degree
    )
{
    int servoPos;

    TFuncName("ServoSetPos");
    TLevel(API);
    TEnterMsg(("degree=%5.1f", degree));

    if (!(serv.servoFlags & SERVOF_CONTINUOUS))
    {
        servoPos = (int)(degree*serv.posPerDegree);
        if (servoPos > SERVO_MAX_VALUE)
        {
            servoPos = SERVO_MAX_VALUE;
        }
        else if (servoPos < SERVO_MIN_VALUE)
        {
            servoPos = SERVO_MIN_VALUE;
        }
        servo[serv.servoMotor] = servoPos;
    }

    TExit();
    return;
}   //ServoSetAngle

/**
 *  This function sets servo position in degrees.
 *
 *  @param serv Points to the SERVO structure.
 *  @param speed Specifies the continuous servo speed.
 */
void
ServoContinuousSetSpeed(
    __out SERVO &serv,
    __in  int speed
    )
{
    TFuncName("ServoContSetSpeed");
    TLevel(API);
    TEnterMsg(("speed=%5.1f", speed));

    if (serv.servoFlags & SERVOF_CONTINUOUS)
    {
        serv.speed = BOUND(speed,
                           SERVO_CONTINUOUS_LOW,
                           SERVO_CONTINUOUS_HIGH) + 127;

        servo[serv.servoMotor] = serv.speed;
        serv.servoFlags |= SERVOF_CONTINUOUS_ENABLED;
    }

    TExit();
    return;
}   //ServoContinuousSetSpeed

/**
 *  This function performs the servo task.
 *
 *  @param serv Points to the SERVO structure.
 */
void
ServoTask(
    __inout SERVO &serv
    )
{
    TFuncName("ServoTask");
    TLevel(TASK);
    TEnter();

    if (serv.servoFlags & SERVOF_CONTINUOUS_ENABLED)
    {
        if ((serv.lowerTouchID != -1) &&
            (SensorValue[serv.lowerTouchID] == 1) ||
            (serv.upperTouchID != -1) &&
            (SensorValue[serv.upperTouchID] == 1))
        {
            servo[serv.servoMotor] = SERVO_CONTINUOUS_STOP;
            serv.servoFlags &= ~SERVOF_CONTINUOUS_ENABLED;
        }
    }

    TExit();
    return;
}   //ServoTask

#endif  //ifndef _SERVO_H
