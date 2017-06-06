#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="gyro.h" />
///
/// <summary>
///     This module contains the library functions for the gyro sensor.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _GYRO_H
#define _GYRO_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_GYRO

//
// Constants.
//
#define GYROF_USER_MASK         0x00ff
#define GYROF_HTSMUX            0x0001
#define GYROF_INVERSE           0x0002

#define GYRO_SAMPLING_INTERVAL  5

#define GYRO_NUM_CAL_SAMPLES    50
#define GYRO_CAL_INTERVAL       10

//
// Macros
//
#ifdef __HTSMUX_SUPPORT__
    #define GyroGetRawValue(p)  ((p.gyroFlags & GYROF_HTSMUX)? \
                                 HTSMUXreadAnalogue((tMUXSensor)p.sensorID): \
                                 SensorValue[p.sensorID])
#else
    #define GyroGetRawValue(p)  SensorValue[p.sensorID]
#endif
#define GyroGetTurnRate(p)      (p.turnRate)
#define GyroGetHeading(p)       (p.heading*p.errAdj)

//
// Type definitions.
//
typedef struct
{
    tSensors    sensorID;
    int         gyroFlags;
    float       errAdj;
    int         zeroOffset;
    int         deadBand;
    long        timestamp;
    long        nextTime;
    int         turnRate;
    float       heading;
} GYRO;

/**
 *  This function calibrates the gyro for zero offset and deadband.
 *
 *  @param gyro Points to the GYRO structure to be initialized.
 *  @param numSamples Specifies the number of calibration samples.
 *  @param calInterval Specifies the calibration interval in msec.
 */
void
GyroCal(
    __out GYRO &gyro,
    __in  int numSamples,
    __in  int calInterval
    )
{
    int i;
    int turnRate;
    int minValue, maxValue;

    TFuncName("GyroCal");
    TLevel(API);
    TEnter();

    gyro.zeroOffset = 0;
    gyro.deadBand = 0;
    minValue = 1023;
    maxValue = 0;

    for (i = 0; i < numSamples; i++)
    {
        turnRate = GyroGetRawValue(gyro);
        gyro.zeroOffset += turnRate;

        if (turnRate < minValue)
        {
            minValue = turnRate;
        }
        else if (turnRate > maxValue)
        {
            maxValue = turnRate;
        }

        wait1Msec(calInterval);
    }

    gyro.zeroOffset /= numSamples;
    gyro.deadBand = maxValue - minValue;

    TExit();
    return;
}   //GyroCal

/**
 *  This function performs the gyro task where it integrates the turn rate
 *  into a heading value.
 *
 *  @param gyro Points to the GYRO structure.
 */
void
GyroTask(
    __inout GYRO &gyro
    )
{
    long currTime;

    TFuncName("GyroTask");
    TLevel(TASK);
    TEnter();

    currTime = nPgmTime;
    if (currTime >= gyro.nextTime)
    {
        gyro.turnRate = GyroGetRawValue(gyro);
        gyro.turnRate -= gyro.zeroOffset;
        gyro.turnRate = DEADBAND(gyro.turnRate, gyro.deadBand);
        if (gyro.gyroFlags & GYROF_INVERSE)
        {
            gyro.turnRate = -gyro.turnRate;
        }
        gyro.heading += (float)gyro.turnRate*(currTime - gyro.timestamp)/1000;
        gyro.timestamp = currTime;
        gyro.nextTime = currTime + GYRO_SAMPLING_INTERVAL;
    }

    TExit();
    return;
}   //GyroTask

/**
 *  This function resets the gyro heading.
 *
 *  @param gyro Points to the GYRO structure to be reset.
 */
void
GyroReset(
    __out GYRO &gyro
    )
{
    TFuncName("GyroReset");
    TLevel(API);
    TEnter();

    GyroTask(gyro);
    gyro.heading = 0;

    TExit();
    return;
}   //GyroReset

/**
 *  This function initializes the gyro sensor.
 *
 *  @param gyro Points to the GYRO structure to be initialized.
 *  @param sensorID Specifies the ID of the gyro sensor.
 *  @param gyroFlags Specifies the gyro flags.
 *  @param errAdj Specifies the error adjustment factor.
 */
void
GyroInit(
    __out GYRO &gyro,
    __in  tSensors sensorID,
    __in  int gyroFlags = 0,
    __in  float errAdj = 1.0
    )
{
    TFuncName("GyroInit");
    TLevel(INIT);
    TEnter();

    gyro.sensorID = sensorID;
    gyro.gyroFlags = gyroFlags & GYROF_USER_MASK;
    gyro.errAdj = errAdj;

    if (!(gyroFlags & GYROF_HTSMUX) &&
        (SensorType[sensorID] != sensorAnalogInactive))
    {
        SetSensorType(sensorID, sensorAnalogInactive);
        wait1Msec(100);
    }

    GyroCal(gyro, GYRO_NUM_CAL_SAMPLES, GYRO_CAL_INTERVAL);
    gyro.timestamp = nPgmTime;
    gyro.nextTime = gyro.timestamp;
    GyroReset(gyro);

    TExit();
    return;
}   //GyroInit

#endif  //ifndef _GYRO_H
