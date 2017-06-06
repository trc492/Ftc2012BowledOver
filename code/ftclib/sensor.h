#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="sensor.h" />
///
/// <summary>
///     This module contains the library functions to handle various sensors.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _SENSOR_H
#define _SENSOR_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_SENSOR

//
// Constants.
//
#define SENSORZONE_LO           0
#define SENSORZONE_MID          1
#define SENSORZONE_HI           2
#define NUM_SENSOR_ZONES        3

#define SENSORF_USER_MASK       0x00ff
#define SENSORF_HTSMUX          0x0001
#define SENSORF_INVERSE         0x0002
#define SENSORF_ENABLE_EVENTS   0x0004
#define SENSORF_CALIBRATING     0x0100

#define SensorCalibrating(s)    (s.sensorFlags & SENSORF_CALIBRATING)

//
// Type definitions.
//
typedef struct
{
    tSensors    sensorID;
    int         lowThreshold;
    int         highThreshold;
    int         sensorFlags;
    int         sensorValue;
    int         sensorZone;
    int         rawMin;
    int         rawMax;
} SENSOR;

//
// Import function prototypes.
//
void
SensorEvent(
    __in SENSOR &sensor
    );

/**
 *  This function initializes the sensor system.
 *
 *  @param sensor Points to the SENSOR structure to be initialized.
 *  @param sensorID Specifies the sensor ID.
 *  @param lowThreshold Specifies the low threshold value.
 *  @param highThreshold Specifies the high threshold value.
 *  @param sensorFlags Specifies the sensor flags.
 */
void
SensorInit(
    __out SENSOR &sensor,
    __in  tSensors sensorID,
    __in  int lowThreshold,
    __in  int highThreshold,
    __in  int sensorFlags = 0
    )
{
    TFuncName("SensorInit");
    TLevel(INIT);
    TEnter();

    sensor.sensorID = sensorID;
    sensor.lowThreshold = lowThreshold;
    sensor.highThreshold = highThreshold;
    sensor.sensorFlags = sensorFlags & SENSORF_USER_MASK;
    sensor.sensorValue = 0;
    sensor.sensorZone = SENSORZONE_LO;

    TExit();
    return;
}   //SensorInit

/**
 *  This function starts or stops the sensor calibration process.
 *
 *  @param sensor Points to the SENSOR structure.
 *  @param fStart Specifies whether to start or stop the calibration.
 */
void
SensorCal(
    __inout SENSOR &sensor,
    __in    bool fStart
    )
{
    TFuncName("SensorCal");
    TLevel(API);
    TEnterMsg(("fStart=%d", (byte)fStart));

    if (fStart)
    {
        sensor.sensorFlags |= SENSORF_CALIBRATING;
        sensor.rawMin = 1023;
        sensor.rawMax = 0;
    }
    else
    {
        int zoneRange = (sensor.rawMax - sensor.rawMin)/3;

        sensor.sensorFlags &= ~SENSORF_CALIBRATING;
        sensor.lowThreshold = sensor.rawMin + zoneRange;
        sensor.highThreshold = sensor.rawMax - zoneRange;
        TInfo(("LoTh=%d,HiTh=%d",
               sensor.lowThreshold, sensor.highThreshold));
    }

    TExit();
    return;
}   //SensorCal

/**
 *  This function processes the sensor reading and sends a trigger event if
 *  necessary.
 *
 *  @param sensor Points to the SENSOR structure.
 */
void
SensorTask(
    __inout SENSOR &sensor
    )
{
    int flags = sensor.sensorFlags;

    TFuncName("SensorTask");
    TLevel(TASK);
    TEnter();

#ifdef __HTSMUX_SUPPORT__
    if (flags & SENSORF_HTSMUX)
    {
        sensor.sensorValue = 1023 -
                             HTSMUXreadAnalogue((tMUXSensor)sensor.sensorID);
    }
    else
    {
        sensor.sensorValue = SensorRaw[sensor.sensorID];
    }
#else
    sensor.sensorValue = SensorRaw[sensor.sensorID];
#endif
    if (flags & SENSORF_CALIBRATING)
    {
        //
        // We are in calibration mode.
        //
        if (sensor.sensorValue < sensor.rawMin)
        {
            sensor.rawMin = sensor.sensorValue;
        }
        else if (sensor.sensorValue > sensor.rawMax)
        {
            sensor.rawMax = sensor.sensorValue;
        }
    }
    else
    {
        int zone;

        if (sensor.sensorValue <= sensor.lowThreshold)
        {
            zone = (flags & SENSORF_INVERSE)?
                   SENSORZONE_HI: SENSORZONE_LO;
        }
        else if (sensor.sensorValue <= sensor.highThreshold)
        {
            zone = SENSORZONE_MID;
        }
        else
        {
            zone = (flags & SENSORF_INVERSE)?
                   SENSORZONE_LO: SENSORZONE_HI;
        }

        if (zone != sensor.sensorZone)
        {
            //
            // We have crossed to another zone, let's send a sensor event.
            //
            sensor.sensorZone = zone;
            if (flags & SENSORF_ENABLE_EVENTS)
            {
                SensorEvent(sensor);
            }
        }
    }

    TExit();
    return;
}   //SensorTask

#endif  //ifndef _SENSOR_H
