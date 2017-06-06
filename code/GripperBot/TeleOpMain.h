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

#include "..\ftclib\joybtn.h"
#include "..\ftclib\drive.h"
DRIVE       g_drives[1];

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

//
// Input and sensors.
//
JOYBTN      g_JoyBtn;

//
// Actuators.
//

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
    int  joystickID,
    int  eventType,
    int  eventID,
    bool fPressed
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
                case Xbox_A:
                    if (fPressed)
                    {
                        motor[armMotor] = 35;
                        nxtDisplayTextLine(3, "arm=up");
                    }
                    else
                    {
                        motor[armMotor] = 0;
                        nxtDisplayTextLine(3, "arm=stop");
                    }
                    break;

                case Xbox_B:
                    if (fPressed)
                    {
                        motor[armMotor] = -35;
                        nxtDisplayTextLine(3, "arm=down");
                    }
                    else
                    {
                        motor[armMotor] = 0;
                        nxtDisplayTextLine(3, "arm=stop");
                    }
                    break;

                case Xbox_X:
                    if (fPressed)
                    {
                        motor[wristMotor] = 75;
                    }
                    else
                    {
                        motor[wristMotor] = 0;
                    }
                    break;

                case Xbox_Y:
                    if (fPressed)
                    {
                        motor[wristMotor] = -75;
                    }
                    else
                    {
                        motor[wristMotor] = 0;
                    }
                    break;

                case Xbox_LB:
                    if (fPressed)
                    {
                        motor[gripperMotor] = 50;
                    }
                    else
                    {
                        motor[gripperMotor] = 0;
                    }
                    break;

                case Xbox_RB:
                    if (fPressed)
                    {
                        motor[gripperMotor] = -50;
                    }
                    else
                    {
                        motor[gripperMotor] = 0;
                    }
                    break;

                default:
                    break;
            }
        }
        else if (eventType == JOYEVENT_TOPHAT)
        {
            switch (eventID)
            {
                case TopHat_Up:
                    break;

                default:
                    break;
            }
        }
    }

    TExit();
    return;
}   //JoyBtnEvent

/**
 *  This function handles the PID motor notification events.
 *
 *  @param pidMotor Points to the PIDMOTOR structure that generated the event.
 */
void
DriveEvent(
    DRIVE &drive
    )
{
    TFuncName("DriveEvent");
    TLevel(EVENT);
    TEnter();
    TExit();
    return;
}   //DriveEvent

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
    JoyBtnInit(g_JoyBtn, 1, JOYBTNF_ENABLE_EVENTS);

    //
    // Intialize the Drive subsystem of the robot running base.
    //
    DriveInit(g_drives[0], leftMotor, rightMotor);

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

    JoyBtnTask(g_JoyBtn);

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

    //
    // Drive Task
    //
#ifdef _ARCADE_DRIVE
    int drivePower = JOYSTICK_POWER(joystick.joy1_y1);
    int turnPower = JOYSTICK_POWER(joystick.joy1_x1);
    nxtDisplayTextLine(1, "D=%4d,T=%4d", drivePower, turnPower);
    DriveArcade(g_drives[0], drivePower, turnPower);
#else
    int leftPower = JOYSTICK_POWER(joystick.joy1_y1);
    int rightPower = JOYSTICK_POWER(joystick.joy1_y2);
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

    TExit();
    return;
}   //OutputTasks
