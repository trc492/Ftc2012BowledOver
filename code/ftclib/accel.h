#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="accel.h" />
///
/// <summary>
///     This module contains the library functions for the accelerometer
///     sensor.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _ACCEL_H
#define _ACCEL_H

#include "..\RobotCDrivers\drivers\HTAC-driver.h"

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_ACCEL

//
// Constants.
//
#define ACCELF_USER_MASK        0x00ff
#define ACCELF_HTSMUX           0x0001
#define ACCELF_ENABLED          0x0100
#define ACCEL_COUNT_PER_G       200
#define ACCEL_NUM_CAL_SAMPLES   50
#define ACCEL_CAL_INTERVAL      10

//
// Type definitions.
//
typedef struct
{
    tSensors    sensorID;
    int         accelFlags;
    int         xZeroOffset;
    int         yZeroOffset;
    int         zZeroOffset;
    int         xDeadBand;
    int         yDeadBand;
    int         zDeadBand;
    float       xAccel;
    float       yAccel;
    float       zAccel;
    float       xVel;
    float       yVel;
    float       zVel;
    float       xDist;
    float       yDist;
    float       zDist;
    long        prevTime;
} ACCEL;

/**
 *  This function calibrates the accelerometer for zero offsets and deadband
 *  on all axes.
 *
 *  @param accel Points to the ACCEL structure to be initialized.
 *  @param numSamples Specifies the number of calibration samples.
 *  @param calInterval Specifies the calibration interval in msec.
 */
void
AccelCal(
    __out ACCEL &accel,
    __in  int numSamples,
    __in  int calInterval
    )
{
    int i;
    int xRaw, yRaw, zRaw;
    int xMin, yMin, zMin;
    int xMax, yMax, zMax;
    bool fSMux;

    TFuncName("AccelCal");
    TLevel(API);

    accel.xZeroOffset = 0;
    accel.yZeroOffset = 0;
    accel.zZeroOffset = 0;
    accel.xDeadBand = 0;
    accel.yDeadBand = 0;
    accel.zDeadBand = 0;
    xMin = yMin = zMin = 1023;
    xMax = yMax = zMax = 0;
    fSMux = (accel.accelFlags & ACCELF_HTSMUX) != 0;
    for (i = 0; i < numSamples; i++)
    {
#ifdef __HTSMUX_SUPPORT__
        if (fSMux &&
            HTACreadAllAxes((tMUXSensor)accel.sensorID, xRaw, yRaw, zRaw) ||
            !fSMux &&
            HTACreadAllAxes(accel.sensorID, xRaw, yRaw, zRaw))
#else
        if (HTACreadAllAxes(accel.sensorID, xRaw, yRaw, zRaw))
#endif
        {
            accel.xZeroOffset += xRaw;
            accel.yZeroOffset += yRaw;
            accel.zZeroOffset += zRaw;

            if (xRaw < xMin)
            {
                xMin = xRaw;
            }
            else if (xRaw > xMax)
            {
                xMax = xRaw;
            }

            if (yRaw < yMin)
            {
                yMin = yRaw;
            }
            else if (yRaw > yMax)
            {
                yMax = yRaw;
            }

            if (zRaw < zMin)
            {
                zMin = zRaw;
            }
            else if (zRaw > zMax)
            {
                zMax = zRaw;
            }
        }
        wait1Msec(calInterval);
    }

    accel.xZeroOffset /= numSamples;
    accel.yZeroOffset /= numSamples;
    accel.zZeroOffset /= numSamples;

    accel.xDeadBand = xMax - xMin;
    accel.yDeadBand = yMax - yMin;
    accel.zDeadBand = zMax - zMin;

    TExit();
    return;
}   //AccelCal

/**
 *  This function resets the accelerometer values.
 *
 *  @param accel Points to the ACCEL structure.
 */
void
AccelReset(
    __out ACCEL &accel
    )
{
    TFuncName("AccelReset");
    TLevel(API);
    TEnter();

    accel.xAccel = 0.0;
    accel.yAccel = 0.0;
    accel.zAccel = 0.0;
    accel.xVel = 0.0;
    accel.yVel = 0.0;
    accel.zVel = 0.0;
    accel.xDist = 0.0;
    accel.yDist = 0.0;
    accel.zDist = 0.0;

    TExit();
    return;
}   //AccelReset

/**
 *  This function initializes the accelerometer sensor.
 *
 *  @param accel Points to the ACCEL structure to be initialized.
 *  @param sensorID Specifies the ID of the accelerometer sensor.
 *  @param accelFlags Specifies the accelerometer flags.
 */
void
AccelInit(
    __out ACCEL &accel,
    __in  tSensors sensorID,
    __in  int accelFlags = 0
    )
{
    TFuncName("AccelInit");
    TLevel(INIT);
    TEnter();

    accel.sensorID = sensorID;
    accel.accelFlags = accelFlags & ACCELF_USER_MASK;
    AccelCal(accel, ACCEL_NUM_CAL_SAMPLES, ACCEL_CAL_INTERVAL);
    AccelReset(accel);
    accel.prevTime = 0;

    TExit();
    return;
}   //AccelInit

/**
 *  This function gets the acceleration value of the X-axis in in/sec/sec.
 *
 *  @param accel Points to the ACCEL structure.
 *
 *  @return Returns the acceleration value of the X-axis.
 */
float
AccelGetXAccel(
    __in  ACCEL &accel
    )
{
    float value = accel.xAccel*INCHES_PER_METER;

    TFuncName("AccelGetXAccel");
    TLevel(API);
    TEnter();
    TExitMsg(("=%5.1f", value));
    return value;
}   //AccelGetXAccel

/**
 *  This function gets the acceleration value of the Y-axis in in/sec/sec.
 *
 *  @param accel Points to the ACCEL structure.
 *
 *  @return Returns the acceleration value of the Y-axis.
 */
float
AccelGetYAccel(
    __in  ACCEL &accel
    )
{
    float value = accel.yAccel*INCHES_PER_METER;

    TFuncName("AccelGetYAccel");
    TLevel(API);
    TEnter();
    TExitMsg(("=%5.1f", value));
    return value;
}   //AccelGetYAccel

/**
 *  This function gets the acceleration value of the Z-axis in in/sec/sec.
 *
 *  @param accel Points to the ACCEL structure.
 *
 *  @return Returns the acceleration value of the Z-axis.
 */
float
AccelGetZAccel(
    __in  ACCEL &accel
    )
{
    float value = accel.zAccel*INCHES_PER_METER;

    TFuncName("AccelGetZAccel");
    TLevel(API);
    TEnter();
    TExitMsg(("=%5.1f", value));
    return value;
}   //AccelGetZAccel

/**
 *  This function gets the velocity value of the X-axis in unit in/sec.
 *
 *  @param accel Points to the ACCEL structure.
 *
 *  @return Returns the velocity value of the X-axis.
 */
float
AccelGetXVel(
    __in  ACCEL &accel
    )
{
    float value = accel.xVel*INCHES_PER_METER;

    TFuncName("AccelGetXVel");
    TLevel(API);
    TEnter();
    TExitMsg(("=%5.1f", value));
    return value;
}   //AccelGetXVel

/**
 *  This function gets the velocity value of the Y-axis in in/sec.
 *
 *  @param accel Points to the ACCEL structure.
 *
 *  @return Returns the velocity value of the Y-axis.
 */
float
AccelGetYVel(
    __in  ACCEL &accel
    )
{
    float value = accel.yVel*INCHES_PER_METER;

    TFuncName("AccelGetYVel");
    TLevel(API);
    TEnter();
    TExitMsg(("=%5.1f", value));
    return value;
}   //AccelGetYVel

/**
 *  This function gets the velocity value of the Z-axis in in/sec.
 *
 *  @param accel Points to the ACCEL structure.
 *
 *  @return Returns the velocity value of the Z-axis.
 */
float
AccelGetZVel(
    __in  ACCEL &accel
    )
{
    float value = accel.zVel*INCHES_PER_METER;

    TFuncName("AccelGetZVel");
    TLevel(API);
    TEnter();
    TExitMsg(("=%5.1f", value));
    return value;
}   //AccelGetZVel

/**
 *  This function gets the distance value of the X-axis in inches.
 *
 *  @param accel Points to the ACCEL structure.
 *
 *  @return Returns the distance value of the X-axis.
 */
float
AccelGetXDist(
    __in  ACCEL &accel
    )
{
    float value = accel.xDist*INCHES_PER_METER;

    TFuncName("AccelGetXDist");
    TLevel(API);
    TEnter();
    TExitMsg(("=%5.1f", value));
    return value;
}   //AccelGetXDist

/**
 *  This function gets the distance value of the Y-axis in inches.
 *
 *  @param accel Points to the ACCEL structure.
 *
 *  @return Returns the distance value of the Y-axis.
 */
float
AccelGetYDist(
    __in  ACCEL &accel
    )
{
    float value = accel.yDist*INCHES_PER_METER;

    TFuncName("AccelGetYDist");
    TLevel(API);
    TEnter();
    TExitMsg(("=%5.1f", value));
    return value;
}   //AccelGetYDist

/**
 *  This function gets the distance value of the Z-axis in inches.
 *
 *  @param accel Points to the ACCEL structure.
 *
 *  @return Returns the distance value of the Z-axis.
 */
float
AccelGetZDist(
    __in  ACCEL &accel
    )
{
    float value = accel.zDist*INCHES_PER_METER;

    TFuncName("AccelGetZDist");
    TLevel(API);
    TEnter();
    TExitMsg(("=%5.1f", value));
    return value;
}   //AccelGetZDist

/**
 *  This function is called to enable or disable the accelerometer integrator.
 *
 *  @param accel Points to the ACCEL structure.
 */
void
AccelSetEnable(
    __out ACCEL &accel,
    __in  bool fEnable
    )
{
    TFuncName("AccelSetEnable");
    TLevel(API);
    TEnterMsg(("fEnable=%d", fEnable));

    if (fEnable)
    {
        accel.accelFlags |= ACCELF_ENABLED;
    }
    else
    {
        accel.accelFlags &= ~ACCELF_ENABLED;
    }

    TExit();
    return;
}   //AccelSetEnable

/**
 *  This function is called periodically to get the accelerometer axis values
 *  and integrate them to velocity and distance values.
 *
 *  @param accel Points to the ACCEL structure.
 */
void
AccelTask(
    __inout ACCEL &accel
    )
{
    TFuncName("AccelTask");
    TLevel(TASK);
    TEnter();

    if (accel.accelFlags & ACCELF_ENABLED)
    {
        long currTime;
        float period;
        bool rc;
        int xRaw, yRaw, zRaw;

        currTime = nPgmTime;
        period = (accel.prevTime == 0)? 0.0:
                                        (currTime - accel.prevTime)/1000.0;
#ifdef __HTSMUX_SUPPORT__
        rc = (accel.accelFlags & ACCELF_HTSMUX)?
             HTACreadAllAxes((tMUXSensor)accel.sensorID, xRaw, yRaw, zRaw):
             HTACreadAllAxes(accel.sensorID, xRaw, yRaw, zRaw);
#else
        rc = HTACreadAllAxes(accel.sensorID, xRaw, yRaw, zRaw);
#endif
        if (rc)
        {
            accel.xAccel = (float)(DEADBAND(xRaw - accel.xZeroOffset,
                                            accel.xDeadBand))/
                           ACCEL_COUNT_PER_G;
            accel.yAccel = (float)(DEADBAND(yRaw - accel.yZeroOffset,
                                            accel.yDeadBand))/
                           ACCEL_COUNT_PER_G;
            accel.zAccel = (float)(DEADBAND(zRaw - accel.zZeroOffset,
                                            accel.zDeadBand))/
                           ACCEL_COUNT_PER_G;
            accel.xVel += accel.xAccel*period;
            accel.yVel += accel.yAccel*period;
            accel.zVel += accel.zAccel*period;
            accel.xDist += accel.xVel*period;
            accel.yDist += accel.yVel*period;
            accel.zDist += accel.zVel*period;
        }
        accel.prevTime = currTime;
    }

    TExit();
    return;
}   //AccelTask

#endif  //ifndef _ACCEL_H
