#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="wallfollow.h" />
///
/// <summary>
///     This module contains the library functions for a wall follower.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _WALLFOLLOW_H
#define _WALLFOLLOW_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif

#define MOD_ID                          MOD_WALLFOLLOW

//
// Constants.
//
#define WALLFOLLOWF_STARTED             0x0001
#define WALLFOLLOWF_LEFT                0x0002
#define WallFollowIsStarted(l)          (l.wallFollowFlags & \
                                         WALLFOLLOWF_STARTED)

#define WALLFOLLOW_DEF_DRIVE_POWER      50
#define WALLFOLLOW_DEF_FIND_DRIVE_POWER 50
#define WALLFOLLOW_DEF_FIND_TURN_POWER  30

#define WFMODE_FIND_WALL                0
#define WFMODE_AVOID_OBSTACLE           1
#define WFMODE_FOLLOW_WALL              2

//
// Type definitions.
//
typedef struct
{
    int     driveID;
    int     frontPidCtrlID;
    int     sidePidCtrlID;
    int     wallFollowFlags;
    int     mode;
    float   targetDist;
    int     drivePower;
    int     findDrivePower;
    int     findTurnPower;
} WALLFOLLOW;

/**
 *  This function initializes the wall follower object.
 *
 *  @param wallfollow Points to the WALLFOLLOW structure to be initialized.
 *  @param driveID Specifies the drive ID.
 *  @param frontPidCtrlID Specifies front sensor PID controller ID.
 *  @param leftPidCtrlID Specifies left sensor PID controller ID.
 *  @param rightPidCtrlID Specifies right sensor PID controller ID.
 */
void
WallFollowInit(
    __out WALLFOLLOW &wallfollow,
    __in  int driveID,
    __in  int frontPidCtrlID,
    __in  int sidePidCtrlID
    )
{
    int i;

    TFuncName("WallFollowInit");
    TLevel(INIT);
    TEnter();

    wallfollow.driveID = driveID;
    wallfollow.frontPidCtrlID = frontPidCtrlID;
    wallfollow.sidePidCtrlID = sidePidCtrlID;
    wallfollow.wallFollowFlags = 0;
    wallfollow.mode = WFMODE_FIND_WALL;
    wallfollow.targetDist = 0.0;
    wallfollow.drivePower = WALLFOLLOW_DEF_DRIVE_POWER;
    wallfollow.findDrivePower = WALLFOLLOW_DEF_FIND_DRIVE_POWER;
    wallfollow.findTurnPower = WALLFOLLOW_DEF_FIND_TURN_POWER;

    TExit();
    return;
}   //WallFollowInit

/**
 *  This function starts the wall follower.
 *
 *  @param wallfollow Points to the WALLFOLLOW structure.
 *  @param targetDist Specifies the target distance from the wall.
 *  @param drivePower Specifies the drive power for wall following.
 *  @param findDrivePower Specifies the maximum drive power when finding
 *         the wall.
 *  @param findTurnPower Specifies the maximum turn power when finding
 *         the wall.
 */
void
WallFollowStart(
    __inout WALLFOLLOW &wallfollow,
    __in    float targetDist,
    __in    int drivePower = WALLFOLLOW_DEF_DRIVE_POWER,
    __in    int findDrivePower = WALLFOLLOW_DEF_FIND_DRIVE_POWER,
    __in    int findTurnPower = WALLFOLLOW_DEF_FIND_TURN_POWER,
    )
{
    TFuncName("WallFollowStart");
    TLevel(API);
    TEnterMsg(("target=%5.1f", targetDist));

    wallfollow.targetDist = abs(targetDist);
    if (targetDist < 0.0)
    {
        wallfollow.wallFollowFlags |= WALLFOLLOWF_LEFT;
    }
    else
    {
        wallfollow.wallFollowFlags &= ~WALLFOLLOWF_LEFT;
    }
    PIDCtrlSetTarget(g_pidCtrls[wallfollow.sidePidCtrlID],
                     wallfollow.targetDist, 0.0);
    wallfollow.drivePower = drivePower;
    wallfollow.findDrivePower = findDrivePower;
    wallfollow.findTurnPower = findTurnPower;
    wallfollow.wallFollowFlags |= WALLFOLLOWF_STARTED;

    TExit();
    return;
}   //WallFollowStart

/**
 *  This function stops the wall follower.
 *
 *  @param wallfollow Points to the WALLFOLLOW structure.
 */
void
WallFollowStop(
    __inout WALLFOLLOW &wallfollow,
    )
{
    TFuncName("WallFollowStop");
    TLevel(API);
    TEnter();

    DriveReset(g_drives[wallfollow.driveID]);
    PIDCtrlReset(g_pidCtrls[wallfollow.frontPidCtrlID]);
    PIDCtrlReset(g_pidCtrls[wallfollow.sidePidCtrlID]);
    wallfollow.wallFollowFlags &= ~WALLFOLLOWF_STARTED;

    TExit();
    return;
}   //FollowStop

/**
 *  This function sets the find target drive and turn powers.
 *
 *  @param wallfollow Points to the WALLFOLLOW structure.
 *  @param drivePower Specifies the drive power for finding the target.
 *  @param turnPower Specifies the turn power for finding the target.
 */
void
WallFollowSetFindPower(
    __inout WALLFOLLOW &wallfollow,
    __in int drivePower,
    __in int turnPower
    )
{
    TFuncName("SetFindPower");
    TLevel(TASK);
    TEnter();

    wallfollow.findDrivePower = drivePower;
    wallfollow.findTurnPower = turnPower;

    TExit();
    return;
}   //WallFollowSetFindPower

/**
 *  This function processes the wall follow task.
 *
 *  @param wallfollow Points to the WALLFOLLOW structure.
 */
void
WallFollowTask(
    __inout WALLFOLLOW &wallfollow
    )
{
    TFuncName("WallFollowTask");
    TLevel(TASK);
    TEnter();

    if (WallFollowIsStarted(wallfollow))
    {
        float frontDist = PIDCtrlGetInput(wallfollow.frontPidCtrlID);
        float sideDist = PIDCtrlGetInput(wallfollow.sidePidCtrlID);

        switch (wallfollow.mode)
        {
            case WFMODE_FIND_WALL:
                if (frontDist <= wallfollow.targetDist)
                {
                    wallfollow.mode = WFMODE_AVOID_OBSTACLE;
                }
                else
                {
                    nxtDisplayTextLine(3, "FindWall");
                    DriveArcade(g_drives[wallfollow.driveID],
                                wallfollow.findDrivePower, 0);
                }
                break;

            case WFMODE_AVOID_OBSTACLE:
                if (frontDist > wallfollow.targetDist*8.0)
                {
                    wallfollow.mode = WFMODE_FOLLOW_WALL;
                }
                else
                {
                    nxtDisplayTextLine(3, "AvoidObstacle");
                    DriveArcade(
                        g_drives[wallfollow.driveID],
                        0,
                        (wallfollow.wallFollowFlags & WALLFOLLOWF_LEFT)?
                            wallfollow.findTurnPower:
                            -wallfollow.findTurnPower);
                }
                break;

            case WFMODE_FOLLOW_WALL:
                if (frontDist <= wallfollow.targetDist)
                {
                    wallfollow.mode = WFMODE_AVOID_OBSTACLE;
                }
                else
                {
                    nxtDisplayTextLine(3, "FollowWall");
                    int turnPower = (int)PIDCtrlOutput(
                                        g_pidCtrls[wallfollow.sidePidCtrlID],
                                        sideDist);
                    if (wallfollow.wallFollowFlags & WALLFOLLOWF_LEFT)
                    {
                        DriveTank(g_drives[wallfollow.driveID],
                                  wallfollow.drivePower,
                                  wallfollow.drivePower + turnPower);
                     }
                    else
                    {
                        DriveTank(g_drives[wallfollow.driveID],
                                  wallfollow.drivePower + turnPower,
                                  wallfollow.drivePower);
                    }
                }
                break;
        }
    }

    TExit();
    return;
}   //WallFollowTask

#endif  //ifndef _WALLFOLLOW_H
