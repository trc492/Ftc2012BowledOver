#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="AutoMain.h" />
///
/// <summary>
///     This module contains the main autonomous tasks code.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#define _IRDRIVE_ENABLED

#include "..\ftclib\timer.h"
#include "..\ftclib\sm.h"
#include "..\ftclib\drive.h"
#include "..\ftclib\pidctrl.h"
DRIVE   g_drives[1];
PIDCTRL g_pidCtrls[2];
#include "..\ftclib\piddrive.h"

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_MAIN

//
// Trace info.
//
#define TRACE_MODULES           (MOD_MAIN)
#define TRACE_LEVEL             TASK
#define MSG_LEVEL               INFO

#define INPUTID_ENCODER_DRIVE   0
#define INPUTID_ENCODER_TURN    1

//
// State machine event types.
//
#define EVTTYPE_PIDDRIVE        (EVTTYPE_NONE + 1)
#define EVTTYPE_TIMER           (EVTTYPE_NONE + 2)

//
// Input and sensors.
//
TIMER   g_Timer;

//
// Drive subsystems.
//
PIDDRIVE g_EncoderDrive;

//
// State machines.
//
SM      g_AutoSM;

/**
 *  This function handles the PID drive notification events.
 *
 *  @param drive Points to the PIDDRIVE structure that generated the event.
 */
void
PIDDriveEvent(
    __in PIDDRIVE &pidDrive
    )
{
    TFuncName("PIDDriveEvent");
    TLevel(EVENT);
    TEnter();



    TExit();
    return;
}   //PIDDriveEvent

/**
 *  This function handles the timer notification events.
 *
 *  @param timer Points to the TIMER structure that generated the event.
 */
void
TimerEvent(
    __in TIMER &timer
    )
{
    TFuncName("TimerEvent");
    TLevel(EVENT);
    TEnter();



    TExit();
    return;
}   //TimerEvent

/**
 *  This function provides the input value for various PID controllers.
 *
 *  @param inputID Specifies the input sensor ID.
 *
 *  @return Returns the input value for the input sensor.
 */
float
PIDCtrlGetInput(
    __in int inputID
    )
{
    float inputValue = 0.0;

    TFuncName("PIDCtrlGetInput");
    TLevel(CALLBK);
    TEnterMsg(("ID=%d", inputID));

    switch (inputID)
    {
        case 0:
            //
            // Encoder drive.
            //
            inputValue = (float)(nMotorEncoder[motorLeft] +
                                 nMotorEncoder[motorRight])*
                         DISTANCE_PER_CLICK/2.0;
            break;

        case 1:
            //
            // Encoder turn.
            //
            inputValue = (float)(nMotorEncoder[motorLeft] -
                                 nMotorEncoder[motorRight])*
                         DEGREES_PER_CLICK;
            break;
    }

    TExitMsg(("=%5.1f", inputValue));
    return inputValue;
}   //PIDCtrlGetInput

/**
 *  This function implements the state machine for autonomous mode.
 *
 *  @param sm Points to the SM structure.
 */
void
AutonomousSM(
    __inout SM &sm
    )
{
    TFuncName("AutonomousSM");
    TLevel(TASK);
    TEnter();

    if (SMIsReady(sm))
    {
        switch (sm.currState)
        {
            case SMSTATE_STARTED:
                // Drive forward 2 ft.
                PIDDriveSetTarget(g_EncoderDrive,
                                  24.0,
                                  0.0,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1);
                break;

            case SMSTATE_STARTED + 1:
                // Turn right.
                PIDDriveSetTarget(g_EncoderDrive,
                                  0.0,
                                  90.0,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1);
                break;

            case SMSTATE_STARTED + 2:
                // Drive forward 6 ft.
                PIDDriveSetTarget(g_EncoderDrive,
                                  72.0,
                                  0.0,
                                  true);
                SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE, -1, -1);
                SMWaitEvents(sm, sm.currState + 1);
                break;

            default:
                SMStop(sm);
                break;
        }
    }

    TExit();
    return;
}   //AutonomousSM

/**
 *  This function initializes the robot and its subsystems.
 */
void
RobotInit()
{
    TFuncName("RobotInit");
    TLevel(INIT);
    TEnter();

    //
    // Initialize the input subsystems.
    //
    TimerInit(g_Timer, TIMERF_ENABLE_EVENTS);

    //
    // Intialize the Drive subsystem of the robot running base.
    //
    DriveInit(g_drives[0], motorLeft, motorRight);

    //
    // Encoder Drive.
    //
    PIDCtrlInit(g_pidCtrls[INPUTID_ENCODER_DRIVE],
                INPUTID_ENCODER_DRIVE,
                ENCODER_DRIVE_KP, ENCODER_DRIVE_KI, ENCODER_DRIVE_KD,
                ENCODER_DRIVE_TOLERANCE, ENCODER_DRIVE_SETTLING);
    PIDCtrlInit(g_pidCtrls[INPUTID_ENCODER_TURN],
                INPUTID_ENCODER_TURN,
                ENCODER_TURN_KP, ENCODER_TURN_KI, ENCODER_TURN_KD,
                ENCODER_TURN_TOLERANCE, ENCODER_TURN_SETTLING);
    PIDDriveInit(g_EncoderDrive,
                 0,     //Drive
                 -1,    //pidCtrlX
                 0,     //EncoderDrive
                 1,     //GyroTurn
                 PIDDRIVEF_ENABLE_EVENTS);
    //
    // Initialize subsystems.
    //

    //
    // Initialize the Autonomous state machine.
    //
    SMInit(g_AutoSM);
    SMStart(g_AutoSM);

    TExit();
    return;
}   //RobotInit

/**
 *  This function processes all the high frequency tasks that needs to run
 *  more often than other tasks such as sensor integration tasks.
 */
void
HiFreqTasks()
{
    TFuncName("HiFreqTasks");
    TLevel(TASK);
    TEnter();
    TExit();
    return;
}   //HiFreqTasks

/**
 *  This function processes all the input tasks.
 */
void
InputTasks()
{
    TFuncName("InputTasks");
    TLevel(TASK);
    TEnter();

    TimerTask(g_Timer);

    TExit();
    return;
}   //InputTasks

/**
 *  This function processes all the main tasks.
 */
void
MainTasks()
{
    TFuncName("MainTasks");
    TLevel(TASK);
    TEnter();

    if (SMIsEnabled(g_AutoSM))
    {
        //
        // Autonomous mode.
        //
        nxtDisplayTextLine(0, "Auto=%d,Flags=%x",
                           g_AutoSM.currState, g_AutoSM.smFlags);
        AutonomousSM(g_AutoSM);
    }

    TExit();
    return;
}   //MainTasks

/**
 *  This function processes all the output tasks. Output tasks are where all
 *  the actions are taking place. All other tasks are just changing states of
 *  various objects. There is no action taken until the output tasks.
 */
void
OutputTasks()
{
    TFuncName("OutputTasks");
    TLevel(TASK);
    TEnter();

    //
    // The Drive task programs the drive motors and set the robot into
    // action.
    //
    PIDDriveTask(g_EncoderDrive);
    DriveTask(g_drives[0]);
    nxtDisplayTextLine(1, "L=%d,R=%d",
                       g_drives[0].motorPowers[IDX_FRONT_LEFT],
                       g_drives[0].motorPowers[IDX_FRONT_RIGHT]);

    TExit();
    return;
}   //OutputTasks
