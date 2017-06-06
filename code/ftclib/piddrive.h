#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="piddrive.h" />
///
/// <summary>
///     This module contains the library functions for the PID drive
///     subsystem.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _PIDDRIVE_H
#define _PIDDRIVE_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PIDDRIVE

//
// Constants.
//
#define PIDDRIVEF_PIDDRIVE_ON           0x0100
#define PIDDRIVEF_STOP_ONTARGET         0x0200
#define PIDDRIVEF_TURN_ONLY             0x0400
#define PIDDRIVEF_USER_MASK             0x00ff
#define PIDDRIVEF_ENABLE_EVENTS         0x0001
#define PIDDRIVEF_MECANUM_DRIVE         0x0002

//
// Type definitions.
//
typedef struct
{
    int  driveID;
    int  driveXInputID;
    int  driveYInputID;
    int  turnInputID;
    int  pidDriveFlags;
    long expiredTime;
} PIDDRIVE;

//
// Import function prototypes.
//
void
PIDDriveEvent(
    __in PIDDRIVE &pidDrive
    );

/**
 *  This function resets the PID Drive system.
 *
 *  @param pidDrive Points to the PIDDRIVE structure to be reset.
 */
void
PIDDriveReset(
    __out PIDDRIVE &pidDrive
    )
{
    TFuncName("PIDDriveReset");
    TLevel(API);
    TEnter();

    pidDrive.pidDriveFlags &= ~PIDDRIVEF_PIDDRIVE_ON;
    DriveReset(g_drives[pidDrive.driveID]);
    if (pidDrive.pidDriveFlags & PIDDRIVEF_MECANUM_DRIVE)
    {
        PIDCtrlReset(g_pidCtrls[pidDrive.driveXInputID]);
    }
    PIDCtrlReset(g_pidCtrls[pidDrive.driveYInputID]);
    PIDCtrlReset(g_pidCtrls[pidDrive.turnInputID]);
    DriveStallProtect(g_drives[pidDrive.driveID], false);
    pidDrive.expiredTime = 0;

    TExit();
    return;
}   //PIDDriveReset

/**
 *  This function initializes the drive system.
 *
 *  @param pidDrive Points to the PIDDRIVE structure to be initialized.
 *  @param driveID Specifies the drive ID.
 *  @param driveXInputID Specifies the drive input ID for X.
 *  @param driveYInputID Specifies the drive input ID for Y.
 *  @param turnInputID Specifies the turn input ID.
 *  @param pidDriveFlags Specifies the drive flags.
 */
void
PIDDriveInit(
    __out PIDDRIVE &pidDrive,
    __in  int driveID,
    __in  int driveXInputID,
    __in  int driveYInputID,
    __in  int turnInputID,
    __in  int pidDriveFlags = 0
    )
{
    TFuncName("PIDDriveInit");
    TLevel(INIT);
    TEnter();

    pidDrive.driveID = driveID;
    if (pidDriveFlags & PIDDRIVEF_MECANUM_DRIVE)
    {
        pidDrive.driveXInputID = driveXInputID;
    }
    pidDrive.driveYInputID = driveYInputID;
    pidDrive.turnInputID = turnInputID;
    pidDrive.pidDriveFlags = pidDriveFlags & PIDDRIVEF_USER_MASK;

    TExit();
    return;
}   //PIDDriveInit

/**
 *  This function sets PID drive target with the given drive distance and
 *  turn angle setpoints.
 *
 *  @param pidDrive Points to the PIDDRIVE structure.
 *  @param distXSetPoint Specifies the target X distance to travel.
 *  @param distYSetPoint Specifies the target Y distance to travel.
 *  @param angleSetPoint Specifies the target angle to turn.
 *  @param fStopOnTarget If true, stop PIDDrive when target is reached.
 *         Otherwise, continue to monitor the target and readjust if necessary.
 *  @param timeout Specifies the optional timeout period.
 */
void
PIDDriveSetTarget(
    __out PIDDRIVE &pidDrive,
    __in  float distXSetPoint,
    __in  float distYSetPoint,
    __in  float angleSetPoint,
    __in  bool fStopOnTarget,
    __in  long timeout = 0
    )
{
    TFuncName("PIDDriveSetTarget");
    TLevel(API);
    TEnterMsg(("DY=%5.1f,A=%5.1f", distYSetPoint, angleSetPoint));

    if (pidDrive.pidDriveFlags & PIDDRIVEF_MECANUM_DRIVE)
    {
        PIDCtrlSetTarget(g_pidCtrls[pidDrive.driveXInputID],
                         distXSetPoint,
                         PIDCtrlGetInput(pidDrive.driveXInputID));
    }
    PIDCtrlSetTarget(g_pidCtrls[pidDrive.driveYInputID],
                     distYSetPoint,
                     PIDCtrlGetInput(pidDrive.driveYInputID));
    PIDCtrlSetTarget(g_pidCtrls[pidDrive.turnInputID],
                     angleSetPoint,
                     PIDCtrlGetInput(pidDrive.turnInputID));
    pidDrive.expiredTime = (timeout != 0)? nPgmTime + timeout: 0;
    if (fStopOnTarget)
    {
        pidDrive.pidDriveFlags |= PIDDRIVEF_STOP_ONTARGET;
    }
    else
    {
        pidDrive.pidDriveFlags &= ~PIDDRIVEF_STOP_ONTARGET;
    }
    pidDrive.pidDriveFlags |= PIDDRIVEF_PIDDRIVE_ON;
    if ((distXSetPoint == 0.0) &&
        (distYSetPoint == 0.0) &&
        (angleSetPoint != 0.0))
    {
        pidDrive.pidDriveFlags |= PIDDRIVEF_TURN_ONLY;
    }
    else
    {
        pidDrive.pidDriveFlags &= ~PIDDRIVEF_TURN_ONLY;
    }
    DriveStallProtect(g_drives[pidDrive.driveID], true);

    TExit();
    return;
}   //PIDDriveSetTarget

/**
 *  This function sets PID drive target with the given drive distance and
 *  turn angle setpoints.
 *
 *  @param pidDrive Points to the PIDDRIVE structure.
 *  @param distSetPoint Specifies the target Y distance to travel.
 *  @param angleSetPoint Specifies the target angle to turn.
 *  @param fStopOnTarget If true, stop PIDDrive when target is reached.
 *         Otherwise, continue to monitor the target and readjust if necessary.
 *  @param timeout Specifies the optional timeout period.
 */
void
PIDDriveSetTarget(
    __out PIDDRIVE &pidDrive,
    __in  float distSetPoint,
    __in  float angleSetPoint,
    __in  bool fStopOnTarget,
    __in  long timeout = 0
    )
{
    TFuncName("PIDDriveSetTarget");
    TLevel(API);
    TEnterMsg(("D=%5.1f,A=%5.1f", distSetPoint, angleSetPoint));

    PIDDriveSetTarget(pidDrive,
                      0.0,
                      distSetPoint,
                      angleSetPoint,
                      fStopOnTarget,
                      timeout);

    TExit();
    return;
}   //PIDDriveSetTarget

/**
 *  This function performs the PID drive.
 *
 *  @param pidDrive Points to the PIDDRIVE structure.
 */
void
PIDDriveTask(
    __inout PIDDRIVE &pidDrive
    )
{
    TFuncName("PIDDriveTask");
    TLevel(TASK);
    TEnter();

    if (pidDrive.pidDriveFlags & PIDDRIVEF_PIDDRIVE_ON)
    {
        int driveXPower = (pidDrive.pidDriveFlags & PIDDRIVEF_MECANUM_DRIVE)?
                            (int)PIDCtrlOutput(
                                    g_pidCtrls[pidDrive.driveXInputID],
                                    PIDCtrlGetInput(pidDrive.driveXInputID)):
                            0;

        int driveYPower = (int)PIDCtrlOutput(
                                g_pidCtrls[pidDrive.driveYInputID],
                                PIDCtrlGetInput(pidDrive.driveYInputID));
        int turnPower = (int)PIDCtrlOutput(
                                g_pidCtrls[pidDrive.turnInputID],
                                PIDCtrlGetInput(pidDrive.turnInputID));
        if ((pidDrive.expiredTime != 0) &&
            (nPgmTime >= pidDrive.expiredTime) ||
            PIDCtrlIsOnTarget(g_pidCtrls[pidDrive.turnInputID]) &&
            ((pidDrive.pidDriveFlags & PIDDRIVEF_TURN_ONLY) ||
             PIDCtrlIsOnTarget(g_pidCtrls[pidDrive.driveYInputID]) &&
             (!(pidDrive.pidDriveFlags & PIDDRIVEF_MECANUM_DRIVE) ||
              PIDCtrlIsOnTarget(g_pidCtrls[pidDrive.driveXInputID]))))
        {
            if (pidDrive.pidDriveFlags & PIDDRIVEF_STOP_ONTARGET)
            {
                PIDDriveReset(pidDrive);
                if (pidDrive.pidDriveFlags & PIDDRIVEF_ENABLE_EVENTS)
                {
                    PIDDriveEvent(pidDrive);
                }
            }
            else
            {
                DriveStop(g_drives[pidDrive.driveID]);
            }
        }
        else if (pidDrive.pidDriveFlags & PIDDRIVEF_MECANUM_DRIVE)
        {
            DriveMecanumCartesian(g_drives[pidDrive.driveID],
                                  driveXPower, driveYPower, turnPower);
        }
        else
        {
            DriveArcade(g_drives[pidDrive.driveID], driveYPower, turnPower);
        }
    }

    TExit();
    return;
}   //PIDDriveTask

#endif  //ifndef _PIDDRIVE_H
