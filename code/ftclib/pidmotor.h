#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="pidmotor.h" />
///
/// <summary>
///     This module contains the library functions for the PID motor
///     subsystem.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _PIDMOTOR_H
#define _PIDMOTOR_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PIDMOTOR

//
// Constants.
//
#define PIDMOTORF_PIDMODE_ON    0x0100
#define PIDMOTORF_STOP_ONTARGET 0x0200
#define PIDMOTORF_USER_MASK     0x00ff
#define PIDMOTORF_ENABLE_EVENTS 0x0001

//
// Type definitions.
//
typedef struct
{
    tMotor  motorID;
    int     inputID;
    int     pidMotorFlags;
    int     motorPower;
    long    expiredTime;
} PIDMOTOR;

//
// Import function prototypes.
//
void
PIDMotorEvent(
    __in PIDMOTOR &pidMotor
    );

/**
 *  This function resets the PID motor system.
 *
 *  @param pidMotor Points to the PIDMOTOR structure to be reset.
 */
void
PIDMotorReset(
    __out PIDMOTOR &pidMotor
    )
{
    TFuncName("PIDMotorReset");
    TLevel(API);
    TEnter();

    pidMotor.pidMotorFlags &= ~PIDMOTORF_PIDMODE_ON;
    motor[pidMotor.motorID] = (tMotor)0;
    PIDCtrlReset(g_pidCtrls[pidMotor.inputID]);
    pidMotor.motorPower = 0;
    pidMotor.expiredTime = 0;

    TExit();
    return;
}   //PIDMotorReset

/**
 *  This function initializes the PID motor system.
 *
 *  @param pidMotor Points to the PIDMOTOR structure to be initialized.
 *  @param motorID Specifies the motor ID.
 *  @param inputID Specifies the input ID.
 *  @param pidMotorFlags Specifies the flags.
 */
void
PIDMotorInit(
    __out PIDMOTOR &pidMotor,
    __in  tMotor motorID,
    __in  int inputID,
    __in  int pidMotorFlags = 0
    )
{
    TFuncName("PIDMotorInit");
    TLevel(INIT);
    TEnter();

    pidMotor.motorID = motorID;
    pidMotor.inputID = inputID;
    pidMotor.pidMotorFlags = pidMotorFlags & PIDMOTORF_USER_MASK;
    pidMotor.expiredTime = 0;

    TExit();
    return;
}   //PIDMotorInit

/**
 *  This function sets PID motor power.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param power Specifies the power level to set the motor.
 */
void
PIDMotorSetPower(
    __out PIDMOTOR &pidMotor,
    __in  int power
    )
{
    TFuncName("PIDMotorSetPower");
    TLevel(API);
    TEnterMsg(("power=%d", power));

    power = BOUND(power, MOTOR_MIN_VALUE, MOTOR_MAX_VALUE);
    PIDMotorReset(pidMotor);
    motor[pidMotor.motorID] = power;

    TExit()
    return;
}   //PIDMotorSetPower

/**
 *  This function sets PID motor target.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 *  @param setPoint Specifies the set motor target.
 *  @param fStopOnTarget If true, stop PIDMotor when target is reached.
 *         Otherwise, continue to monitor the target and readjust if necessary.
 *  @param timeout Specifies the optional timeout period.
 */
void
PIDMotorSetTarget(
    __out PIDMOTOR &pidMotor,
    __in  float setPoint,
    __in  bool fStopOnTarget,
    __in  long timeout = 0
    )
{
    TFuncName("PIDMotorSetTarget");
    TLevel(API);
    TEnterMsg(("setPt=%5.1f", setPoint));

    PIDCtrlSetTarget(g_pidCtrls[pidMotor.inputID],
                     setPoint,
                     PIDCtrlGetInput(pidMotor.inputID));
    pidMotor.expiredTime = (timeout != 0)? nPgmTime + timeout: 0;
    if (fStopOnTarget)
    {
        pidMotor.pidMotorFlags |= PIDMOTORF_STOP_ONTARGET;
    }
    else
    {
        pidMotor.pidMotorFlags &= ~PIDMOTORF_STOP_ONTARGET;
    }
    pidMotor.pidMotorFlags |= PIDMOTORF_PIDMODE_ON;

    TExit();
    return;
}   //PIDMotorSetTarget

/**
 *  This function performs the PID motor task.
 *
 *  @param pidMotor Points to the PIDMOTOR structure.
 */
void
PIDMotorTask(
    __inout PIDMOTOR &pidMotor
    )
{
    int output;

    TFuncName("PIDMotorTask");
    TLevel(TASK);
    TEnter();

    if (pidMotor.pidMotorFlags & PIDMOTORF_PIDMODE_ON)
    {
        pidMotor.motorPower = (int)
                              PIDCtrlOutput(
                                g_pidCtrls[pidMotor.inputID],
                                PIDCtrlGetInput(pidMotor.inputID));
        if ((pidMotor.expiredTime != 0) &&
            (nPgmTime >= pidMotor.expiredTime) ||
            PIDCtrlIsOnTarget(g_pidCtrls[pidMotor.inputID]))
        {
            if (pidMotor.pidMotorFlags & PIDMOTORF_STOP_ONTARGET)
            {
                PIDMotorReset(pidMotor);
                if (pidMotor.pidMotorFlags & PIDMOTORF_ENABLE_EVENTS)
                {
                    PIDMotorEvent(pidMotor);
                }
            }
            else
            {
                pidMotor.motorPower = 0;
                motor[pidMotor.motorID] = 0;
            }
        }
        else
        {
            motor[pidMotor.motorID] = pidMotor.motorPower;
        }
    }

    TExit();
    return;
}   //PIDMotorTask

#endif  //ifndef _PIDMOTOR_H
