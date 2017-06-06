#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TeleOpMain.h" />
///
/// <summary>
///     This module contains the main TeleOp tasks code.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#include "..\ftclib\batt.h"
#include "..\ftclib\joybtn.h"
#include "..\ftclib\drive.h"

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

//
// Global data.
//
BATT    g_Batt;

//
// Input and sensors.
//
JOYBTN  g_JoyBtn1;
#if (_TARGET == "Robot")
JOYBTN  g_JoyBtn2;
#endif

//
// Actuators.
//
DRIVE   g_drives[1];

/**
 *  This function handles the joystick button notification events.
 *
 *  @param joystickID Specifies the joystick the event was from.
 *  @param eventType Specifies the event type.
 *  @param eventID Specifies the event ID.
 *  @param fPressed Specifies the event is a press or a release.
 */
void
JoyBtnEvent(
    __in int  joystickID,
    __in int  eventType,
    __in int  eventID,
    __in bool fPressed
    )
{
    TFuncName("JoyBtnEvent");
    TLevel(EVENT);
    TEnterMsg(("Type=%x,ID=%x,On=%x", eventType, eventID, fPressed));

    if (joystickID == 1)
    {
        if (eventType == JOYEVENT_BUTTON)
        {
            switch (eventID)
	        {
	            case Logitech_Btn1:
	                break;

	            default:
	                break;
	        }
        }
        else if (eventType == JOYEVENT_TOPHAT)
        {
            switch (eventID)
	        {
	            case 0:
	                break;

	            default:
	                break;
	        }
        }
    }
#if (_TARGET == "Robot")
    else if (joystickID == 2)
    {
        if (eventType == JOYEVENT_BUTTON)
        {
            switch (eventID)
	        {
	            case Logitech_Btn1:
	                break;

	            default:
	                break;
	        }
        }
        else if (eventType == JOYEVENT_TOPHAT)
        {
            switch (eventID)
	        {
	            case 0:
	                break;

	            default:
	                break;
	        }
        }
    }
#endif

	TExit();
    return;
}   //JoyBtnEvent

/**
 *  This function initializes the robot and its subsystems.
 */
void
RobotInit()
{
    TFuncName("RobotInit");
    TLevel(INIT);
    TEnter();

    BattInit(g_Batt, 5, true);
    //
    // Initialize the input subsystems.
    //
    JoyBtnInit(g_JoyBtn1, 1, JOYBTNF_ENABLE_EVENTS);
#if (_TARGET == "Robot")
    JoyBtnInit(g_JoyBtn2, 2, JOYBTNF_ENABLE_EVENTS);
#endif

    //
    // Intialize the Drive subsystem of the robot running base.
    //
    DriveInit(g_drives[0], motorLeft, motorRight);

    //
    // Initialize subsystems.
    //

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

    JoyBtnTask(g_JoyBtn1);
#if (_TARGET == "Robot")
    JoyBtnTask(g_JoyBtn2);
#endif

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

    //
    // TeleOp mode.
    //
    nxtDisplayTextLine(0, "Mode=TeleOp");
#ifdef _ARCADE_DRIVE
    int drivePower = NORMALIZE_DRIVE(DEADBAND_INPUT(joystick.joy1_y1),
                                     -128, 127);
    int turnPower = NORMALIZE_DRIVE(DEADBAND_INPUT(joystick.joy1_x1),
                                    -128, 127);
    nxtDisplayTextLine(1, "D=%4d,T=%4d", drivePower, turnPower);
    DriveArcade(g_drives[0], drivePower, turnPower);
#else
    int leftPower = NORMALIZE_DRIVE(DEADBAND_INPUT(joystick.joy1_y1),
                                    -128, 127);
    int rightPower = NORMALIZE_DRIVE(DEADBAND_INPUT(joystick.joy1_y2),
                                     -128, 127);
    nxtDisplayTextLine(1, "L=%4d,R=%4d", leftPower, rightPower);
    DriveTank(g_drives[0], leftPower, rightPower);
#endif

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
    DriveTask(g_drives[0]);
    BattTask(g_Batt);

    TExit();
    return;
}   //OutputTasks
