#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="touch.h" />
///
/// <summary>
///     This module contains the library functions for the touch sensor.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _TOUCH_H
#define _TOUCH_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_TOUCH

//
// Constants.
//
#define TOUCHF_USER_MASK        0x00ff
#define TOUCHF_HTSMUX           0x0001
#define TOUCHF_ENABLE_EVENTS    0x0002

//
// Macros.
//
#ifdef __HTSMUX_SUPPORT__
    #define TouchGetState(p)    ((p.touchFlags & TOUCHF_HTSMUX)? \
                                    HTSMUXreadAnalogue( \
                                        (tMUXSensor)p.sensorID) < 500: \
                                    SensorRaw[p.sensorID] < 500)
#else
    #define TouchGetState(p)    (SensorRaw[p.sensorID] < 500)
#endif

//
// Type definitions.
//
typedef struct
{
    tSensors    sensorID;
    int         touchFlags;
    bool        fActive;
} TOUCH;

//
// Import function prototypes.
//
void
TouchEvent(
    __in TOUCH &touch,
    __in bool fActive
    );

/**
 *  This function initializes the touch sensor.
 *
 *  @param touch Points to the TOUCH structure to be initialized.
 *  @param sensorID Specifies the ID of the gyro sensor.
 *  @param touchFlags Specifies the touch flags.
 */
void
TouchInit(
    __out TOUCH &touch,
    __in  tSensors sensorID,
    __in  int touchFlags = 0
    )
{
    TFuncName("TouchInit");
    TLevel(INIT);
    TEnter();

    touch.sensorID = sensorID;
    touch.touchFlags = touchFlags & TOUCHF_USER_MASK;
    touch.fActive = false;
    if (!(touchFlags & TOUCHF_HTSMUX) &&
        (SensorType(sensorID) != sensorTouch) &&
        (SensorMode(sensorID) != modeBoolean))
    {
        SetSensorType(sensorID, sensorTouch);
        SetSensorMode(sensorID, modeBoolean);
        wait1Msec(10);
    }

    TExit();
    return;
}   //TouchInit

/**
 *  This function performs the touch task where it monitors the touch sensor
 *  state and send a notification if necessary.
 *
 *  @param touch Points to the TOUCH structure.
 */
void
TouchTask(
    __inout TOUCH &touch
    )
{
    bool fActive;

    TFuncName("TouchTask");
    TLevel(TASK);
    TEnter();

    if (touch.touchFlags & TOUCHF_ENABLE_EVENTS)
    {
        fActive = TouchGetState(touch);
        if (fActive != touch.fActive)
        {
            //
            // Touch sensor has changed state.
            //
            TouchEvent(touch, fActive);
            touch.fActive = fActive;
        }
    }

    TExit();
    return;
}   //TouchTask

#endif  //ifndef _TOUCH_H
