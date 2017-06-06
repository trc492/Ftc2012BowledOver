#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="pidctrl.h" />
///
/// <summary>
///     This module contains the library functions for PID Control.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _PIDCTRL_H
#define _PIDCTRL_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PIDCTRL

//
// Constants.
//
#define PIDCTRLF_USER_MASK      0x00ff
#define PIDCTRLF_ABS_SETPOINT   0x0001
#define PIDCTRLF_INVERSE        0x0002
#define PIDCTRLF_NO_OSCILLATE   0x0004

#define PIDCtrlGetTarget(p)     ((p).setPoint)

//
// Type definitions.
//
typedef struct
{
    int   inputID;
    float Kp;
    float Ki;
    float Kd;
    float tolerance;
    long  settlingTime;
    float minOutput;
    float maxOutput;
    int   pidCtrlFlags;
    float prevError;
    float totalError;
    long  startSettling;
    float setPoint;
} PIDCTRL;

float
PIDCtrlGetInput(
    __in int inputID
    );

/**
 *  This function resets the PID control.
 *
 *  @param pidCtrl Points to the PID Control structure to be reset.
 */
void
PIDCtrlReset(
    __out PIDCTRL &pidCtrl
    )
{
    TFuncName("PIDCtrlReset");
    TLevel(API);
    TEnter();

    pidCtrl.prevError = 0.0;
    pidCtrl.totalError = 0.0;

    TExit();
    return;
}   //PIDCtrlReset

/**
 *  This function initializes the PID control object.
 *
 *  @param pidCtrl Points to the PID Control structure to be initialized.
 *  @param inputID Specifies the input sensor ID.
 *  @param Kp Specifies the Kp constant.
 *  @param Ki Specifies the Ki constant.
 *  @param Kd Specifies the Kd constant.
 *  @param tolerance Specifies the on-target tolerance.
 *  @param settlingTime Specifies the on-target settling time in msec.
 *  @param pidCtrlFlags Specifies the PID controller flags.
 */
void
PIDCtrlInit(
    __out PIDCTRL &pidCtrl,
    __in  int inputID,
    __in  float Kp,
    __in  float Ki,
    __in  float Kd,
    __in  float tolerance,
    __in  long  settlingTime,
    __in  int pidCtrlFlags = 0
    )
{
    TFuncName("PIDCtrlInit");
    TLevel(INIT);
    TEnter();

    pidCtrl.inputID = inputID;
    pidCtrl.Kp = Kp;
    pidCtrl.Ki = Ki;
    pidCtrl.Kd = Kd;
    pidCtrl.tolerance = tolerance;
    pidCtrl.settlingTime = settlingTime;
    pidCtrl.minOutput = MOTOR_MIN_VALUE;
    pidCtrl.maxOutput = MOTOR_MAX_VALUE;
    pidCtrl.pidCtrlFlags = pidCtrlFlags & PIDCTRLF_USER_MASK;
    pidCtrl.startSettling = 0;
    pidCtrl.setPoint = 0.0;
    PIDCtrlReset(pidCtrl);

    TExit();
    return;
}   //PIDCtrlInit

/**
 *  This function gets the previous error value.
 *
 *  @param pidCtrl Points to the PID Control structure.
 *
 *  @return Returns the previous error value.
 */
float
PIDCtrlGetError(
    __out PIDCTRL &pidCtrl
    )
{
    TFuncName("PIDCtrlGetError");
    TLevel(API);
    TEnter();
    TExitMsg(("=%f", pidCtrl.prevError));
    return pidCtrl.prevError;
}   //PIDCtrlGetError

/**
 *  This function sets the output power limit.
 *
 *  @param pidCtrl Points to the PID Control structure.
 *  @param minOutput Specifies the minimum output level.
 *  @param maxOutput Specifies the maximum output level.
 */
void
PIDCtrlSetPowerLimit(
    __out PIDCTRL &pidCtrl,
    __in  float minOutput,
    __in  float maxOutput
    )
{
    TFuncName("PIDCtrlSetPwrLimit");
    TLevel(API);
    TEnterMsg(("Min=%5.1f,Max=%5.1f", minOutput, maxOutput));

    pidCtrl.minOutput = minOutput;
    pidCtrl.maxOutput = maxOutput;

    TExit();
    return;
}   //PIDCtrlSetPowerLimit

/**
 *  This function sets the SetPoint.
 *
 *  @param pidCtrl Points to the PID Control structure.
 *  @param setPoint Specifies the SetPoint target.
 *  @param currInput Specifies the current input value.
 */
void
PIDCtrlSetTarget(
    __inout PIDCTRL &pidCtrl,
    __in    float setPoint,
    __in    float currInput
    )
{
    TFuncName("PIDCtrlSetTarget");
    TLevel(API);
    TEnter();

    if (!(pidCtrl.pidCtrlFlags & PIDCTRLF_ABS_SETPOINT))
    {
        setPoint += currInput;
    }
    pidCtrl.setPoint = setPoint;
    pidCtrl.prevError = setPoint - currInput;
    pidCtrl.totalError = 0.0;
    pidCtrl.startSettling = nPgmTime;

    TExit();
    return;
}   //PIDCtrlSetTarget

/**
 *  This function determines if we are on target by checking if the previous
 *  error is within target tolerance and remain within tolerance for at least
 *  the settling period.
 *
 *  @param pidCtrl Points to the PID Control structure.
 *
 *  @returns Returns true if we are on target, false otherwise.
 */
bool
PIDCtrlIsOnTarget(
    __inout PIDCTRL &pidCtrl
    )
{
    bool fOnTarget = false;

    TFuncName("PIDCtrlIsOnTarget");
    TLevel(HIFREQ);
    TEnter();

    if (pidCtrl.pidCtrlFlags & PIDCTRLF_NO_OSCILLATE)
    {
        if (abs(pidCtrl.prevError) <= pidCtrl.tolerance)
        {
            fOnTarget = true;
        }
    }
    else if (abs(pidCtrl.prevError) > pidCtrl.tolerance)
    {
        pidCtrl.startSettling = nPgmTime;
    }
    else if (nPgmTime - pidCtrl.startSettling >= pidCtrl.settlingTime)
    {
        fOnTarget = true;
    }

    TExitMsg(("=%x", fOnTarget));
    return fOnTarget;
}   //PIDCtrlIsOnTarget

/**
 *  This function calculates the output based on the current input.
 *
 *  @param pidCtrl Points to the PID structure.
 *  @param currInput Specifies the current input value.
 *
 *  @return Returns the calculate output value.
 */
float
PIDCtrlOutput(
    __inout PIDCTRL &pidCtrl,
    __in    float currInput
    )
{
    float output;
    float error;
    float adjTotalError;

    TFuncName("PIDCtrlOutput");
    TLevel(API);
    TEnter();

    error = pidCtrl.setPoint - currInput;
    if (pidCtrl.pidCtrlFlags & PIDCTRLF_INVERSE)
    {
        error = -error;
    }
    adjTotalError = pidCtrl.Ki*(pidCtrl.totalError + error);
    if ((adjTotalError >= pidCtrl.minOutput) &&
        (adjTotalError <= pidCtrl.maxOutput))
    {
        pidCtrl.totalError += error;
    }

    output = pidCtrl.Kp*error +
             pidCtrl.Ki*pidCtrl.totalError +
             pidCtrl.Kd*(error - pidCtrl.prevError);

    pidCtrl.prevError = error;
    if (output < pidCtrl.minOutput)
    {
        output = pidCtrl.minOutput;
    }
    else if (output > pidCtrl.maxOutput)
    {
        output = pidCtrl.maxOutput;
    }

    TExitMsg(("=%f", output));
    return output;
}   //PIDCtrlOutput

#endif  //ifndef _PIDCTRL_H
