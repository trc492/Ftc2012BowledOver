#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="lnfollow.h" />
///
/// <summary>
///     This module contains the library functions for a line follower.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _LNFOLLOW_H
#define _LNFOLLOW_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif

#define MOD_ID                  MOD_LNFOLLOW

//
// Constants.
//
#define LNFOLLOWF_STARTED       0x0001
#define LnFollowStarted(l)      (l.lnFollowFlags & LNFOLLOWF_STARTED)

#define LNFOLLOW_DEF_MAX_DRIVE_POWER    100
#define LNFOLLOW_DEF_FIND_DRIVE_POWER   50
#define LNFOLLOW_DEF_FIND_TURN_POWER    50

#define LNFOLLOW_INVALID_VALUE  -1.0

//
// Type definitions.
//
typedef struct
{
    int  driveID;
    int  pidCtrlID;
    int  lnFollowFlags;
    int  maxDrivePower;
    int  findDrivePower;
    int  findTurnPower;
    long expiredTime;
} LNFOLLOW;

/**
 *  This function initializes the line follower object.
 *
 *  @param lnfollow Points to the LNFOLLOW structure to be initialized.
 *  @param driveID Specifies the drive ID.
 *  @param pidCtrlID Specifies the PID controller ID.
 */
void
LnFollowInit(
    __out LNFOLLOW &lnfollow,
    __in  int driveID,
    __in  int pidCtrlID
    )
{
    int i;

    TFuncName("LnFollowInit");
    TLevel(INIT);
    TEnter();

    lnfollow.driveID = driveID;
    lnfollow.pidCtrlID = pidCtrlID;
    lnfollow.lnFollowFlags = 0;
    lnfollow.maxDrivePower = LNFOLLOW_DEF_MAX_DRIVE_POWER;
    lnfollow.findDrivePower = LNFOLLOW_DEF_FIND_DRIVE_POWER;
    lnfollow.findTurnPower = LNFOLLOW_DEF_FIND_TURN_POWER;
    lnfollow.expiredTime = 0;

    TExit();
    return;
}   //LnFollowInit

/**
 *  This function starts the line follower.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 *  @param targetValue Specifies the center sensor value.
 *  @param maxDrivePower Specifies the maximum drive power.
 *  @param findDrivePower Specifies the maximum drive power when finding
 *         the target.
 *  @param findTurnPower Specifies the maximum turn power when finding
 *         the target.
 *  @param timeout Specifies the timeout value.
 */
void
LnFollowStart(
    __inout LNFOLLOW &lnfollow,
    __in    float targetValue,
    __in    int maxDrivePower,
    __in    int findDrivePower,
    __in    int findTurnPower,
    __in    int timeout
    )
{
    TFuncName("LnFollowStart");
    TLevel(API);
    TEnterMsg(("target=%5.1f", targetValue));

    lnfollow.maxDrivePower = maxDrivePower;
    lnfollow.findDrivePower = findDrivePower;
    lnfollow.findTurnPower = findTurnPower;
    lnfollow.expiredTime = (timeout != 0)? nPgmTime + timeout: 0;
    PIDCtrlSetTarget(g_pidCtrls[lnfollow.pidCtrlID], targetValue, 0.0);
    lnfollow.lnFollowFlags |= LNFOLLOWF_STARTED;

    TExit();
    return;
}   //LnFollowStart

/**
 *  This function stops the line follower.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 */
void
LnFollowStop(
    __inout LNFOLLOW &lnfollow,
    )
{
    TFuncName("LnFollowStop");
    TLevel(API);
    TEnter();

    DriveReset(g_drives[lnfollow.driveID]);
    PIDCtrlReset(g_pidCtrls[lnfollow.pidCtrlID]);
    lnfollow.expiredTime = 0;
    lnfollow.lnFollowFlags &= ~LNFOLLOWF_STARTED;

    TExit();
    return;
}   //LnFollowStop

/**
 *  This function sets the find target drive and turn powers.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 *  @param drivePower Specifies the drive power for finding the target.
 *  @param turnPower Specifies the turn power for finding the target.
 */
void
LnFollowSetFindPower(
    __inout LNFOLLOW &lnfollow,
    __in int drivePower,
    __in int turnPower
    )
{
    TFuncName("SetFindPower");
    TLevel(TASK);
    TEnter();

    lnfollow.findDrivePower = drivePower;
    lnfollow.findTurnPower = turnPower;

    TExit();
    return;
}   //LnFollowSetFindPower

/**
 *  This function processes the line follower task.
 *
 *  @param lnfollow Points to the LNFOLLOW structure.
 */
void
LnFollowTask(
    __inout LNFOLLOW &lnfollow
    )
{
    TFuncName("LnFollowTask");
    TLevel(TASK);
    TEnter();

    float currValue = PIDCtrlGetInput(lnfollow.pidCtrlID);
    //
    // PIDCtrlGetInput() may determine the stopping condition and
    // call LnFollowStop().
    //
    if (LnFollowStarted(lnfollow))
    {
        if ((lnfollow.expiredTime != 0) && (nPgmTime >= lnfollow.expiredTime))
        {
            //
            // We run out of time, so stop.
            //
            LnFollowStop(lnfollow);
            //
            // notify caller if necessary.
            //
            //????
        }
        else if (currValue != LNFOLLOW_INVALID_VALUE)
        {
            int turnPower = (int)PIDCtrlOutput(
                                    g_pidCtrls[lnfollow.pidCtrlID],
                                    currValue);
            int drivePower = lnfollow.maxDrivePower*
                             (MOTOR_MAX_VALUE - abs(turnPower))/
                             MOTOR_MAX_VALUE;
            DriveArcade(g_drives[lnfollow.driveID], drivePower, turnPower);
        }
        else
        {
            //
            // We lost the line. Find it again.
            //
            DriveArcade(g_drives[lnfollow.driveID],
                        lnfollow.findDrivePower,
                        lnfollow.findTurnPower);
        }
    }

    TExit();
    return;
}   //LnFollowTask

#endif  //ifndef _LNFOLLOW_H
