#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="drive.h" />
///
/// <summary>
///     This module contains the library functions for the drive subsystem.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _DRIVE_H
#define _DRIVE_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_DRIVE

//
// Constants.
//
#define DRIVEF_USER_MASK        0x00ff
#define DRIVEF_ENABLE_EVENTS    0x0001
#define DRIVEF_ON               0x0100
#define DRIVEF_STALL_PROTECT_ON 0x0200
#define DRIVEF_STALLED          0x0400
#define DRIVEF_FOUR_MOTORS      0x0800

#define MIN_STALL_POWER         20
#define STALL_TIME              2000    //2 seconds

#define MAX_NUM_MOTORS          4
#define IDX_FRONT_LEFT          0
#define IDX_FRONT_RIGHT         1
#define IDX_REAR_LEFT           2
#define IDX_REAR_RIGHT          3

#define DriveIsStalled(d)       (d.driveFlags & DRIVEF_STALLED)

//
// Type definitions.
//
typedef struct
{
    int     driveFlags;
    tMotor  motorIDs[MAX_NUM_MOTORS];
    int     motorPowers[MAX_NUM_MOTORS];
    int     motorEncoders[MAX_NUM_MOTORS];
    long    stallTimer;
} DRIVE;

//
// Import function prototypes.
//
void
DriveEvent(
    __in DRIVE &drive
    );

/**
 *  This function stops the motors in the drive system.
 *
 *  @param drive Points to the DRIVE structure.
 */
void
DriveStop(
    __out DRIVE &drive
    )
{
    TFuncName("DriveStop");
    TLevel(API);
    TEnter();

    int numMotors = (drive.driveFlags & DRIVEF_FOUR_MOTORS)? 4: 2;

    drive.driveFlags &= ~DRIVEF_ON;
    //
    // Stop the motors.
    //
    for (int i = 0; i < numMotors; i++)
    {
        drive.motorPowers[i] = 0;
        drive.motorEncoders[i] = 0;
        motor[drive.motorIDs[i]] = 0;
    }

    drive.stallTimer = 0;

    TExit();
    return;
}   //DriveStop

/**
 *  This function resets the drive system.
 *
 *  @param drive Points to the DRIVE structure to be reset.
 */
void
DriveReset(
    __out DRIVE &drive
    )
{
    TFuncName("DriveReset");
    TLevel(API);
    TEnter();

    DriveStop(drive);
    drive.driveFlags &= ~DRIVEF_STALLED;

    TExit();
    return;
}   //DriveReset

/**
 *  This function initializes the drive system for 2-motor drive.
 *
 *  @param drive Points to the DRIVE structure to be initialized.
 *  @param leftMotor Specifies the left motor.
 *  @param rightMotor Specifies the right motor.
 *  @param driveFlags Specifies the drive flags.
 */
void
DriveInit(
    __out DRIVE &drive,
    __in  tMotor leftMotor,
    __in  tMotor rightMotor,
    __in  int driveFlags = 0
    )
{
    TFuncName("DriveInit");
    TLevel(INIT);
    TEnter();

    drive.motorIDs[IDX_FRONT_LEFT] = leftMotor;
    drive.motorIDs[IDX_FRONT_RIGHT] = rightMotor;
    drive.motorIDs[IDX_REAR_LEFT] = (tMotor)0;
    drive.motorIDs[IDX_REAR_RIGHT] = (tMotor)0;
    drive.driveFlags = driveFlags & DRIVEF_USER_MASK;
    DriveReset(drive);

    TExit();
    return;
}   //DriveInit

/**
 *  This function initializes the drive system for 4-motor drive.
 *
 *  @param drive Points to the DRIVE structure to be initialized.
 *  @param frontLeftMotor Specifies the front left motor.
 *  @param frontRightMotor Specifies the front right motor.
 *  @param rearLeftMotor Specifies the rear left motor.
 *  @param rearRightMotor Specifies the rear right motor.
 */
void
DriveInit(
    __out DRIVE &drive,
    __in  tMotor frontLeftMotor,
    __in  tMotor frontRightMotor,
    __in  tMotor rearLeftMotor,
    __in  tMotor rearRightMotor
    )
{
    TFuncName("DriveInit");
    TLevel(INIT);
    TEnter();

    drive.motorIDs[IDX_FRONT_LEFT] = frontLeftMotor;
    drive.motorIDs[IDX_FRONT_RIGHT] = frontRightMotor;
    drive.motorIDs[IDX_REAR_LEFT] = rearLeftMotor;
    drive.motorIDs[IDX_REAR_RIGHT] = rearRightMotor;
    drive.driveFlags = DRIVEF_FOUR_MOTORS;
    DriveReset(drive);

    TExit();
    return;
}   //DriveInit

/**
 *  This function enables or disables stall protection.
 *
 *  @param drive Points to the DRIVE structure to be initialized.
 *  @param fOn If true, enables stall protection.
 */
void
DriveStallProtect(
    __inout DRIVE &drive,
    __in    bool fOn
    )
{
    TFuncName("DriveStallProtect");
    TLevel(API);
    TEnterMsg(("fOn=%d", (byte)fOn));

    if (fOn)
    {
        drive.driveFlags |= DRIVEF_STALL_PROTECT_ON;
    }
    else
    {
        drive.driveFlags &= ~DRIVEF_STALL_PROTECT_ON;
    }

    TExit();
    return;
}   //DriveStallProtect

/**
 *  This function sets power of the motors for tank drive.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param leftPower Specifies the left motor power.
 *  @param rightPower Specifies the right motor power.
 */
void
DriveTank(
    __out DRIVE &drive,
    __in  int leftPower,
    __in  int rightPower
    )
{
    TFuncName("DriveTank");
    TLevel(API);
    TEnterMsg(("Left=%d,Right=%d", leftPower, rightPower));

    drive.motorPowers[IDX_FRONT_LEFT] =
        BOUND(leftPower, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    drive.motorPowers[IDX_FRONT_RIGHT] =
        BOUND(rightPower, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    if (drive.driveFlags & DRIVEF_FOUR_MOTORS)
    {
        drive.motorPowers[IDX_REAR_LEFT] = drive.motorPowers[IDX_FRONT_LEFT];
        drive.motorPowers[IDX_REAR_RIGHT] = drive.motorPowers[IDX_FRONT_RIGHT];
    }
    drive.driveFlags |= DRIVEF_ON;

    TExit();
    return;
}   //DriveTank

/**
 *  This function sets power of the motors for arcade drive.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param drivePower Specifies the drive power.
 *  @param turnPower Specifies the turn power.
 */
void
DriveArcade(
    __out DRIVE &drive,
    __in  int drivePower,
    __in  int turnPower
    )
{
    TFuncName("DriveArcade");
    TLevel(API);
    TEnterMsg(("Drive=%d,Turn=%d", drivePower, turnPower));

    drivePower = BOUND(drivePower, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    turnPower = BOUND(turnPower, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    if (drivePower + turnPower > MOTOR_MAX_VALUE)
    {
        //
        // Forward right:
        //  left = drive + turn - (drive + turn - MOTOR_MAX_VALUE)
        //  right = drive - turn - (drive + turn - MOTOR_MAX_VALUE)
        //
        drive.motorPowers[IDX_FRONT_LEFT] = MOTOR_MAX_VALUE;
        drive.motorPowers[IDX_FRONT_RIGHT] = -2*turnPower + MOTOR_MAX_VALUE;
    }
    else if (drivePower - turnPower > MOTOR_MAX_VALUE)
    {
        //
        // Forward left:
        //  left = drive + turn - (drive - turn - MOTOR_MAX_VALUE)
        //  right = drive - turn - (drive - turn - MOTOR_MAX_VALUE)
        //
        drive.motorPowers[IDX_FRONT_LEFT] = 2*turnPower + MOTOR_MAX_VALUE;
        drive.motorPowers[IDX_FRONT_RIGHT] = MOTOR_MAX_VALUE;
    }
    else if (drivePower + turnPower < MOTOR_MIN_VALUE)
    {
        //
        // Backward left:
        //  left = drive + turn - (drive + turn - MOTOR_MIN_VALUE)
        //  right = drive - turn - (drive + turn - MOTOR_MIN_VALUE)
        //
        drive.motorPowers[IDX_FRONT_LEFT] = MOTOR_MIN_VALUE;
        drive.motorPowers[IDX_FRONT_RIGHT] = -2*turnPower + MOTOR_MIN_VALUE;
    }
    else if (drivePower - turnPower < MOTOR_MIN_VALUE)
    {
        //
        // Backward right:
        //  left = drive + turn - (drive - turn - MOTOR_MIN_VALUE)
        //  right = drive - turn - (drive - turn - MOTOR_MIN_VALUE)
        //
        drive.motorPowers[IDX_FRONT_LEFT] = 2*turnPower + MOTOR_MIN_VALUE;
        drive.motorPowers[IDX_FRONT_RIGHT] = MOTOR_MIN_VALUE;
    }
    else
    {
        drive.motorPowers[IDX_FRONT_LEFT] = drivePower + turnPower;
        drive.motorPowers[IDX_FRONT_RIGHT] = drivePower - turnPower;
    }

    if (drive.driveFlags & DRIVEF_FOUR_MOTORS)
    {
        drive.motorPowers[IDX_REAR_LEFT] = drive.motorPowers[IDX_FRONT_LEFT];
        drive.motorPowers[IDX_REAR_RIGHT] = drive.motorPowers[IDX_FRONT_RIGHT];
    }

    drive.driveFlags |= DRIVEF_ON;

    TExit();
    return;
}   //DriveArcade

/**
 *  This function sets power of the motors for mecanum drive in cartesian
 *  coordinate system.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param x Specifies the x speed.
 *  @param y Specifies the y speed.
 *  @param rot Specifies the rotaton speed.
 */
void
DriveMecanumCartesian(
    __out DRIVE &drive,
    __in  int x,
    __in  int y,
    __in  int rot
    )
{
    TFuncName("DriveCartesian");
    TLevel(API);
    TEnterMsg(("x=%d,y=%d,rot=%d", x, y, rot));

    if (drive.driveFlags & DRIVEF_FOUR_MOTORS)
    {
        int mag, maxMag, i;

        drive.motorPowers[IDX_FRONT_LEFT] = x + y + rot;
        drive.motorPowers[IDX_FRONT_RIGHT] = -x + y - rot;
        drive.motorPowers[IDX_REAR_LEFT] = -x + y + rot;
        drive.motorPowers[IDX_REAR_RIGHT] = x + y - rot;
        //
        // Normalize
        //
        maxMag = abs(drive.motorPowers[0]);
        for (i = 1; i < MAX_NUM_MOTORS; i++)
        {
            mag = abs(drive.motorPowers[i]);
            if (mag > maxMag)
            {
                maxMag = mag;
            }
        }

        if (maxMag > MOTOR_MAX_VALUE)
        {
            for (i = 0; i < MAX_NUM_MOTORS; i++)
            {
                drive.motorPowers[i] =
                    (drive.motorPowers[i]*MOTOR_MAX_VALUE)/maxMag;
            }
        }
    }
    else
    {
        DriveArcade(drive, y, rot);
    }

    drive.driveFlags |= DRIVEF_ON;

    TExit();
    return;
}   //DriveMecanumCartesian

/**
 *  This function sets power of the motors for mecanum drive in polar
 *  coordinate system.
 *
 *  @param drive Points to the DRIVE structure.
 *  @param mag Specifies the magnitude.
 *  @param dir Specifies the direction.
 *  @param rot Specifies the rotaton.
 */
void
DriveMecanumPolar(
    __out DRIVE &drive,
    __in  int mag,
    __in  int dir,
    __in  int rot
    )
{
    TFuncName("DrivePolar");
    TLevel(API);
    TEnterMsg(("m=%d,d=%d,r=%d", mag, dir, rot));

    if (drive.driveFlags & DRIVEF_FOUR_MOTORS)
    {
        float magnitude = (BOUND(mag, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE)*
                           sqrt(2.0))/MOTOR_MAX_VALUE;
        float rotation = (float)BOUND(rot, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE)/
                         MOTOR_MAX_VALUE;
        float dirInRad = (dir + 45.0)*PI/180.0;
        float sinD = sin(dirInRad);
        float cosD = cos(dirInRad);
        float powers[MAX_NUM_MOTORS];

        powers[IDX_FRONT_LEFT] = sinD*magnitude + rotation;
        powers[IDX_FRONT_RIGHT] = cosD*magnitude - rotation;
        powers[IDX_REAR_LEFT] = cosD*magnitude + rotation;
        powers[IDX_REAR_RIGHT] = sinD*magnitude - rotation;
        //
        // Normalize
        //
        float maxMag = abs(powers[0]);
        float mag;
        int i;

        for (i = 1; i < MAX_NUM_MOTORS; i++)
        {
            mag = abs(powers[i]);
            if (mag > maxMag)
            {
                maxMag = mag;
            }
        }

        for (i = 0; i < MAX_NUM_MOTORS; i++)
        {
            if (maxMag > 1.0)
            {
                drive.motorPowers[i] = (int)
                    (powers[i]*MOTOR_MAX_VALUE/maxMag);
            }
            else
            {
                drive.motorPowers[i] = (int)(powers[i]*MOTOR_MAX_VALUE);
            }
        }
    }
    else
    {
        DriveArcade(drive, mag, rot);
    }

    drive.driveFlags |= DRIVEF_ON;

    TExit();
    return;
}   //DriveMecanumPolar

/**
 *  This function performs the driving task according to the drive state.
 *
 *  @param drive Points to the DRIVE structure.
 */
void
DriveTask(
    __inout DRIVE &drive
    )
{
    TFuncName("DriveTask");
    TLevel(TASK);
    TEnter();

    if (drive.driveFlags & DRIVEF_ON)
    {
        if ((drive.driveFlags & DRIVEF_STALLED) == 0)
        {
            motor[drive.motorIDs[IDX_FRONT_LEFT]] =
                drive.motorPowers[IDX_FRONT_LEFT];
            motor[drive.motorIDs[IDX_FRONT_RIGHT]] =
                drive.motorPowers[IDX_FRONT_RIGHT];
            if (drive.driveFlags & DRIVEF_FOUR_MOTORS)
            {
                motor[drive.motorIDs[IDX_REAR_LEFT]] =
                    drive.motorPowers[IDX_REAR_LEFT];
                motor[drive.motorIDs[IDX_REAR_RIGHT]] =
                    drive.motorPowers[IDX_REAR_RIGHT];
            }

            if (drive.driveFlags & DRIVEF_STALL_PROTECT_ON)
            {
                long currTime = nPgmTime;
                if ((drive.stallTimer == 0) ||
                    (abs(drive.motorPowers[IDX_FRONT_LEFT]) <=
                     MIN_STALL_POWER) &&
                    (abs(drive.motorPowers[IDX_FRONT_RIGHT]) <=
                     MIN_STALL_POWER) &&
                    (!(drive.driveFlags & DRIVEF_FOUR_MOTORS) ||
                     (abs(drive.motorPowers[IDX_REAR_LEFT]) <=
                      MIN_STALL_POWER) &&
                     (abs(drive.motorPowers[IDX_REAR_RIGHT]) <=
                      MIN_STALL_POWER)) ||
                    (nMotorEncoder[drive.motorIDs[IDX_FRONT_LEFT]] !=
                     drive.motorEncoders[IDX_FRONT_LEFT]) ||
                    (nMotorEncoder[drive.motorIDs[IDX_FRONT_RIGHT]] !=
                     drive.motorEncoders[IDX_FRONT_RIGHT]) ||
                    (drive.driveFlags & DRIVEF_FOUR_MOTORS) &&
                     ((nMotorEncoder[drive.motorIDs[IDX_REAR_LEFT]] !=
                       drive.motorEncoders[IDX_REAR_LEFT]) ||
                      (nMotorEncoder[drive.motorIDs[IDX_REAR_RIGHT]] !=
                       drive.motorEncoders[IDX_REAR_RIGHT])))
                {
                    drive.motorEncoders[IDX_FRONT_LEFT] =
                        nMotorEncoder[drive.motorIDs[IDX_FRONT_LEFT]];
                    drive.motorEncoders[IDX_FRONT_RIGHT] =
                        nMotorEncoder[drive.motorIDs[IDX_FRONT_RIGHT]];
                    if (drive.driveFlags & DRIVEF_FOUR_MOTORS)
                    {
                        drive.motorEncoders[IDX_REAR_LEFT] =
                            nMotorEncoder[drive.motorIDs[IDX_REAR_LEFT]];
                        drive.motorEncoders[IDX_REAR_RIGHT] =
                            nMotorEncoder[drive.motorIDs[IDX_REAR_RIGHT]];
                    }
                    drive.stallTimer = currTime;
                }

                if (currTime - drive.stallTimer >= STALL_TIME)
                {
                    motor[drive.motorIDs[IDX_FRONT_LEFT]] = 0;
                    motor[drive.motorIDs[IDX_FRONT_RIGHT]] = 0;
                    if (drive.driveFlags & DRIVEF_FOUR_MOTORS)
                    {
                        motor[drive.motorIDs[IDX_REAR_LEFT]] = 0;
                        motor[drive.motorIDs[IDX_REAR_RIGHT]] = 0;
                    }
                    drive.driveFlags |= DRIVEF_STALLED;
                    PlayImmediateTone(1000, 100);
                    if (drive.driveFlags & DRIVEF_ENABLE_EVENTS)
                    {
                        DriveEvent(drive);
                    }
                }
            }
        }
    }
    else
    {
        motor[drive.motorIDs[IDX_FRONT_LEFT]] = 0;
        motor[drive.motorIDs[IDX_FRONT_RIGHT]] = 0;
        if (drive.driveFlags & DRIVEF_FOUR_MOTORS)
        {
            motor[drive.motorIDs[IDX_REAR_LEFT]] = 0;
            motor[drive.motorIDs[IDX_REAR_RIGHT]] = 0;
        }
    }

    TExit();
    return;
}   //DriveTask

#endif  //ifndef _DRIVE_H
