#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="batt.h" />
///
/// <summary>
///     This module contains the library functions for dealing with the NXT
///     battery and the external battery.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _BATT_H
#define _BATT_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_BATT

#define BATTF_ENABLED           0x0100
#define BATTF_USER_MASK         0x00ff
#define BATTF_SHOW_DETAILS      0x0001

//
// Type definitions.
//
typedef struct
{
    int summaryLineNum;
    int battFlags;
    int minIntBatt;
    int maxIntBatt;
    int minExtBatt;
    int maxExtBatt;
    int intBatt;
    int extBatt;
} BATT;

/**
 *  This function display the battery info on the NXT LCD screen.
 *
 *  @param batt Points to the BATT structure.
 */
void
BattShowInfo(
    __in BATT &batt
    )
{
    nxtDisplayTextLine(batt.summaryLineNum, "Int:%3.1f Ext:%4.1f",
                       (float)batt.intBatt/1000.0,
                       (float)batt.extBatt/1000.0);

    if (batt.battFlags & BATTF_SHOW_DETAILS)
    {
        nxtDisplayTextLine(batt.summaryLineNum + 1, "%4.1f<=Int<=%4.1f",
                           (float)batt.minIntBatt/1000.0,
                           (float)batt.maxIntBatt/1000.0);
        nxtDisplayTextLine(batt.summaryLineNum + 2, "%4.1f<=Ext<=%4.1f",
                           (float)batt.minExtBatt/1000.0,
                           (float)batt.maxExtBatt/1000.0);
    }

    return;
}   //BattShowInfo

/**
 *  This function initializes the BATT info.
 *
 *  @param batt Specifies the BATT structure to be initialized.
 *  @param summaryLineNum If positive, a battery summary line will be displayed
 *         at the given line number. If -1, no battery summary is displayed.
 *  @param battFlags Specifies the options.
 */
void
BattInit(
    __out BATT &batt,
    __in  int summaryLineNum,
    __in  int battFlags = 0
    )
{
    TFuncName("BattInit");
    TLevel(INIT);
    TEnter();

    batt.summaryLineNum = summaryLineNum;
    batt.battFlags = battFlags & BATTF_USER_MASK;
    batt.battFlags |= BATTF_ENABLED;
    batt.minIntBatt = nAvgBatteryLevel;
    batt.maxIntBatt = batt.minIntBatt;
    batt.minExtBatt = (externalBatteryAvg < 0)? 0: externalBatteryAvg;
    batt.maxExtBatt = batt.minExtBatt;
#if (_TARGET == "Robot")
    StopTask(displayDiagnostics);
#endif
    eraseDisplay();

    TExit();
    return;
}   //BattInit

/**
 *  This function enables or disables the display of battery info.
 *
 *  @param batt Points to the BATT structure.
 *  @param fEnable If true, enables the battery info display, false otherwise.
 */
void
BattSetState(
    __inout BATT &batt,
    __in    bool fEnable
    )
{
    TFuncName("BattSetState");
    TLevel(API);
    TEnter();

    if (fEnable)
    {
        batt.battFlags |= BATTF_ENABLED;
    }
    else
    {
        batt.battFlags &= ~BATTF_ENABLED;
    }

    TExit();
    return;
}   //BattSetState

/**
 *  This function checks the battery voltages against the min and max values
 *  for determining the operating voltage range. This is helpful to determine
 *  if the batteries are running low under load.
 *
 *  @param batt Points to the BATT structure.
 */
void
BattTask(
    __inout BATT &batt
    )
{
    int currIntBatt;
    int currExtBatt;

    TFuncName("BattTask");
    TLevel(TASK);
    TEnter();

    batt.intBatt = nAvgBatteryLevel;
    batt.extBatt = (externalBatteryAvg < 0)? 0: externalBatteryAvg;

    if (batt.intBatt < batt.minIntBatt)
    {
        batt.minIntBatt = batt.intBatt;
    }
    else if (batt.intBatt > batt.maxIntBatt)
    {
        batt.maxIntBatt = batt.intBatt;
    }

    if (batt.extBatt < batt.minExtBatt)
    {
        batt.minExtBatt = batt.extBatt;
    }
    else if (batt.extBatt > batt.maxExtBatt)
    {
        batt.maxExtBatt = batt.extBatt;
    }

    if (batt.battFlags & BATTF_ENABLED)
    {
        BattShowInfo(batt);
    }

    TExit();
    return;
}   //BattTask

#endif  //ifndef _BATT_H
