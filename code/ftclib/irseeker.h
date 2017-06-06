#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="irseeker.h" />
///
/// <summary>
///     This module contains the library functions for the IR Seeker sensor.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _IRSEEKER_H
#define _IRSEEKER_H

#include "..\RobotCDrivers\drivers\HTIRS2-driver.h"

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_IRSEEKER

//
// Constants.
//
#define IRSEEKERF_USER_MASK     0x00ff
#define IRSEEKERF_HTSMUX        0x0001

#ifdef __HTSMUX_SUPPORT__
    #define IRSeekerGetRawACDir(p) ((p.irseekerFlags & IRSEEKERF_HTSMUX)? \
                                    HTIRS2readACDir((tMUXSensor)p.sensorID): \
                                    HTIRS2readACDir(p.sensorID))
    #define IRSeekerGetRawACData(p) ((p.irseekerFlags & IRSEEKERF_HTSMUX)? \
                                      HTIRS2readAllACStrength( \
                                        (tMUXSensor)p.sensorID, \
                                        p.acStrength[0], \
                                        p.acStrength[1], \
                                        p.acStrength[2], \
                                        p.acStrength[3], \
                                        p.acStrength[4]): \
                                      HTIRS2readAllACStrength( \
                                        p.sensorID, \
                                        p.acStrength[0], \
                                        p.acStrength[1], \
                                        p.acStrength[2], \
                                        p.acStrength[3], \
                                        p.acStrength[4]))

#else
    #define IRSeekerGetRawACDir(p)  HTIRS2readACDir(p.sensorID)
    #define IRSeekerGetRawACData(p) HTIRS2readAllACStrength( \
                                        p.sensorID, \
                                        p.acStrength[0], \
                                        p.acStrength[1], \
                                        p.acStrength[2], \
                                        p.acStrength[3], \
                                        p.acStrength[4])
#endif

//
// Type definitions.
//
typedef struct
{
    tSensors    sensorID;
    int         irseekerFlags;
    int         acStrength[5];
    float       prevACDir;
} IRSEEKER;

/**
 *  This function initializes the IR Seeker sensor.
 *
 *  @param irseeker Points to the IRSEEKER structure to be initialized.
 *  @param sensorID Specifies the ID of the IR Seeker sensor.
 *  @param irseekerFlags Specifies the IR Seeker flags.
 */
void
IRSeekerInit(
    __out IRSEEKER &irseeker,
    __in  tSensors sensorID,
    __in  int irseekerFlags = 0
    )
{
    TFuncName("IRSeekerInit");
    TLevel(INIT);
    TEnter();

    irseeker.sensorID = sensorID;
    irseeker.irseekerFlags = irseekerFlags & IRSEEKERF_USER_MASK;
    irseeker.prevACDir = 0.0;

    TExit();
    return;
}   //IRSeekerInit

/**
 *  This function gets the AC Direction of the IR Seeker sensor.
 *
 *  @param irseeker Points to the IRSEEKER structure.
 *
 *  @return Returns the AC Direction value.
 */
float
IRSeekerGetACDir(
    __in IRSEEKER &irseeker
    )
{
    float value;
    int dir;

    TFuncName("IRSeekerGetACDir");
    TLevel(API);
    TEnter();

    dir = IRSeekerGetRawACDir(irseeker);
    value = (float)dir;
    if (dir == 0)
    {
        value = irseeker.prevACDir;
    }
    else if (IRSeekerGetRawACData(irseeker))
    {
        dir = (dir - 1)/2;
        if ((dir < 4) &&
            (irseeker.acStrength[dir] != 0) &&
            (irseeker.acStrength[dir + 1] != 0))
        {
            value += (float)(irseeker.acStrength[dir + 1] -
                             irseeker.acStrength[dir])/
                     max(irseeker.acStrength[dir],
                         irseeker.acStrength[dir + 1]);
        }
    }
    irseeker.prevACDir = value;

    TEnter();
    TExitMsg(("=%5.1f", value));
    return value;
}   //IRSeekerGetACDir

#endif  //ifndef _IRSEEKER_H
