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
#include "..\ftclib\touch.h"
#include "..\ftclib\timer.h"
#include "..\ftclib\sm.h"
#include "..\ftclib\servo.h"
#include "..\ftclib\drive.h"
#include "..\ftclib\pidctrl.h"
DRIVE       g_drives[1];
PIDCTRL     g_pidCtrls[1];
#include "..\ftclib\pidmotor.h"

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

//#define _CALIBRATE_ELEVATOR

#define INPUTID_ELEVATOR        0

//
// State machine event types.
//
#define EVTTYPE_TIMER           (EVTTYPE_NONE + 1)

//
// Global data.
//
BATT        g_Batt;

//
// Input and sensors.
//
JOYBTN      g_JoyBtn1;
#if (_TARGET == "Robot")
JOYBTN      g_JoyBtn2;
#endif
TOUCH       g_BotLimitSW;
TOUCH       g_TopLimitSW;
TIMER       g_Timer;

//
// Actuators.
//
PIDMOTOR    g_Elevator;
SERVO       g_LeftArm;
SERVO       g_RightArm;
SERVO       g_LeftWrist;
SERVO       g_RightWrist;
SERVO       g_Flap;
bool        g_fFlapOpened;

//
// State machines.
//
SM          g_ToggleFlapSM;

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
                case TopHat_Up:
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
            static bool fConveyorOn = false;
            static bool fWristDown = false;
            static bool fCalElevator = false;

            switch (eventID)
            {
                case Logitech_Btn1:
                    if (fPressed)
                    {
                        //
                        // Toggle Converyor ON and OFF.
                        //
                        fConveyorOn = !fConveyorOn;
                        motor[conveyorMotor] = fConveyorOn? CONVEYOR_POWER: 0;
                    }
                    break;

                case Logitech_Btn2:
                    if (fPressed)
                    {
                        //
                        // Toggle Flap open and close.
                        //
                        SMStart(g_ToggleFlapSM);
                    }
                    break;

                case Logitech_Btn3:
                    if (fPressed)
                    {
                        //
                        // Toggle Wrist up and down.
                        //
                        fWristDown = !fWristDown;
                        ServoSetAngle(g_LeftWrist,
                                      fWristDown? LEFT_WRIST_DOWN:
                                                  LEFT_WRIST_UP);
                        ServoSetAngle(g_RightWrist,
                                      fWristDown? RIGHT_WRIST_DOWN:
                                                  RIGHT_WRIST_UP);
                    }
                    break;

                case Logitech_Btn4:
#if 0   //No one is using this.
                    if (fPressed)
                    {
                        //
                        // Set Wrist to hold bowling ball.
                        //
                        ServoSetAngle(g_LeftWrist, WRIST_HOLD_BALL);
                    }
#endif
                    break;

#if 0
                case Logitech_Btn4:
                    if (fPressed)
                    {
                        //
                        // Toggle Elevator Calibration ON and OFF.
                        //
                        fCalElevator = !fCalElevator;
                        if (fCalElevator)
                        {
                            if (TouchGetState(g_BotLimitSW))
                            {
                                //
                                // Elevator is already at the bottom.
                                //
                                PIDMotorReset(g_Elevator);
                                nMotorEncoder[elevatorMotor] = 0;
                            }
                            else
                            {
                                //
                                // Make the Elevator to go down.
                                //
                                PIDMotorSetPower(g_Elevator,
                                                 ELEVATOR_CAL_POWER);
                            }
                        }
                        else
                        {
                            //
                            // Abort Elevator Calibration.
                            //
                            PIDMotorReset(g_Elevator);
                        }
                    }
                    break;
#endif

                case Logitech_LB5:
                    if (fPressed)
                    {
                        ServoSetAngle(g_LeftArm, ARM_LEFT_OPENED);
                        ServoSetAngle(g_RightArm, ARM_RIGHT_OPENED);
                    }
                    break;

                case Logitech_RB6:
                    if (fPressed)
                    {
                        ServoSetAngle(g_LeftArm, ARM_LEFT_CLOSED);
                        ServoSetAngle(g_RightArm, ARM_RIGHT_CLOSED);
                    }
                    break;

                case Logitech_LB7:
                    if (fPressed)
                    {
                        motor[marMotorLeft] = 50;
                    }
                    else
                    {
                        motor[marMotorLeft] = 0;
                    }
                    break;

                case Logitech_RB8:
                    if (fPressed)
                    {
                        motor[marMotorRight] = 50;
                    }
                    else
                    {
                        motor[marMotorRight] = 0;
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
#if 0   //No one is using this.
                    PIDMotorSetTarget(g_Elevator, ELEVATOR_UP_POS, true);
#endif
                    break;

                case TopHat_Down:
#if 0   //No one is using this.
                    PIDMotorSetTarget(g_Elevator, ELEVATOR_DOWN_POS, true);
#endif
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
 *  This function handles the touch sensor events.
 *
 *  @param touch Points to the TOUCH structure that generated the event.
 *  @param fActive If true, the touch sensor is active, false otherwise.
 */
void
TouchEvent(
    __in TOUCH &touch,
    __in bool fActive
    )
{
    TFuncName("TouchEvent");
    TLevel(EVENT);
    TEnter();

    if (fActive)
    {
        if (touch.sensorID == (int)botLimitSW)
        {
            PIDMotorReset(g_Elevator);
            nMotorEncoder[elevatorMotor] = 0;
            //
            // Play a descending set of notes.
            //
            PlayTone(800, 10);
            PlayTone(700, 10);
            PlayTone(600, 10);
        }
        else if (touch.sensorID == (int)topLimitSW)
        {
            PIDMotorReset(g_Elevator);
            //
            // Play a ascending set of notes.
            //
            PlayTone(800, 10);
            PlayTone(900, 10);
            PlayTone(1000, 10);
        }
    }
    else
    {
        PlayTone(440, 10);
    }


    TExit();
    return;
}   //TouchEvent

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

    if (SMIsEnabled(g_ToggleFlapSM))
    {
        SMSetEvent(g_ToggleFlapSM, EVTTYPE_TIMER, 0, 0, 0, 0);
    }

    TExit();
    return;
}   //TimerEvent

/**
 *  This function handles the PID motor notification events.
 *
 *  @param pidMotor Points to the PIDMOTOR structure that generated the event.
 */
void
PIDMotorEvent(
    __in PIDMOTOR &pidMotor
    )
{
    TFuncName("PIDMotorEvent");
    TLevel(EVENT);
    TEnter();
    TExit();
    return;
}   //PIDMotorEvent

/**
 *  This function handles the PID motor notification events.
 *
 *  @param pidMotor Points to the PIDMOTOR structure that generated the event.
 */
void
DriveEvent(
    __in DRIVE &drive
    )
{
    TFuncName("DriveEvent");
    TLevel(EVENT);
    TEnter();
    TExit();
    return;
}   //DriveEvent

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
        case INPUTID_ELEVATOR:
            inputValue = (float)nMotorEncoder[elevatorMotor]*
                         ELEVATOR_HEIGHT_PER_CLICK;
            break;
    }

    TExitMsg(("=%5.1f", inputValue));
    return inputValue;
}   //PIDCtrlGetInput

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
    TouchInit(g_BotLimitSW, botLimitSW, TOUCHF_ENABLE_EVENTS);
    TouchInit(g_TopLimitSW, topLimitSW, TOUCHF_ENABLE_EVENTS);
    TimerInit(g_Timer, TIMERF_ENABLE_EVENTS);

    //
    // Intialize the Drive subsystem of the robot running base.
    //
    DriveInit(g_drives[0], leftMotor, rightMotor);

    //
    // Initialize subsystems.
    //
    PIDCtrlInit(g_pidCtrls[INPUTID_ELEVATOR],
                INPUTID_ELEVATOR,
                ELEVATOR_KP, ELEVATOR_KI, ELEVATOR_KD,
                ELEVATOR_TOLERANCE, ELEVATOR_SETTLING,
                PIDCTRLF_ABS_SETPOINT);
    PIDMotorInit(g_Elevator, elevatorMotor, INPUTID_ELEVATOR);
    ServoInit(g_RightArm, rightArm, ARM_RIGHT_CLOSED);
    ServoInit(g_LeftArm, leftArm, ARM_LEFT_CLOSED);
    ServoInit(g_LeftWrist, leftWrist, LEFT_WRIST_UP);
    ServoInit(g_RightWrist, rightWrist, RIGHT_WRIST_UP);
    ServoInit(g_Flap, flap, FLAP_CLOSED);
    g_fFlapOpened = false;

    SMInit(g_ToggleFlapSM);

#ifdef _CALIBRATE_ELEVATOR
    //
    // Calibrate elevator.
    //
    if (TouchGetState(g_BotLimitSW))
    {
        //
        // Elevator is already at the bottom.
        //
        PIDMotorReset(g_Elevator);
        nMotorEncoder[elevatorMotor] = 0;
    }
    else
    {
        //
        // Make the Elevator to go down.
        //
        PIDMotorSetPower(g_Elevator, ELEVATOR_CAL_POWER);
        while(TouchGetState(g_BotLimitSW) == false) {}
        PIDMotorReset(g_Elevator);
        nMotorEncoder[elevatorMotor] = 0;
        PlayTone(440, 15);
    }
#endif

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
    TouchTask(g_BotLimitSW);
    TouchTask(g_TopLimitSW);
    TimerTask(g_Timer);

    TExit();
    return;
}   //InputTasks

/**
 *  This function executes the toggle flap state machine.
 *
 *  @param sm Points to the SM structure.
 */
void
ToggleFlapSMTask(
    __inout SM &sm
    )
{
    TFuncName("ToggleFlapSMTask");
    TLevel(TASK);
    TEnterMsg(("fOpened=%d", !g_fFlapOpened));

    switch (sm.currState)
    {
        case SMSTATE_STARTED:
            //
            // Check the elevator height to see if it clears the flap.
            //
#ifdef      FLAP_PROTECTION_ELEVATOR_OVERRIDE
            if (PIDCtrlGetInput(INPUTID_ELEVATOR) >= ELEVATOR_HEIGHT_ABOVE_FLAP)
            {
                //
                // Elevator clears the flap.
                //
                ServoSetAngle(g_Flap, g_fFlapOpened? FLAP_CLOSED: FLAP_OPENED);
                TimerSet(g_Timer, FLAP_OPENCLOSE_TIME);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, sm.currState + 1);
            }
            else
            {
                //
                // Elevator is interfering with the flap, must raise it before
                // we can open or close the flap.
                //
                PIDMotorSetPower(g_Elevator, ELEVATOR_MAX_POWER);
            }
#else
            ServoSetAngle(g_Flap, g_fFlapOpened? FLAP_CLOSED: FLAP_OPENED);
            TimerSet(g_Timer, FLAP_OPENCLOSE_TIME);
            SMAddWaitEvent(sm, EVTTYPE_TIMER);
            SMWaitEvents(sm, sm.currState + 1);
#endif
            break;

        default:
            SMStop(sm);
            g_fFlapOpened = !g_fFlapOpened;
            break;
    }

    TExit();
    return;
}   //ToggleFlapSMTask

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

    static bool fOverride = false;
    static long backoffTime = 0;
    static bool fClosingFlap = false;
    int elevatorPower = NORMALIZE(DEADBAND_INPUT(joystick.joy2_y1),
                                  -128, 127, -50, 50);
    if (elevatorPower != 0)
    {
        fOverride = true;
        //
        // Allow it to go up only if the top limit switch is not active.
        // Allow it to go down only if the bottom limit switch is not active.
        //
        if ((elevatorPower > 0) && !TouchGetState(g_TopLimitSW) ||
            (elevatorPower < 0) && !TouchGetState(g_BotLimitSW))
        {
            //
            // Backoff the conveyor for 100 msec when the elevator is going
            // down. This will prevent the racquet balls from pushing the
            // flap out that will interfere the elevator going down.
            //
            if ((elevatorPower < 0) && (backoffTime == 0))
            {
                backoffTime = nPgmTime + 100;
                motor[conveyorMotor] = -CONVEYOR_POWER;
            }
            else if (nPgmTime > backoffTime)
            {
                motor[conveyorMotor] = 0;
            }

            //
            // If the elvator is going down, check to make sure the flap is
            // closed. If not, we will pause and close the flap first before
            // allowing the elevator to go down.
            //
            if (elevatorPower < 0 && g_fFlapOpened &&
                PIDCtrlGetInput(INPUTID_ELEVATOR) <= ELEVATOR_HEIGHT_ABOVE_FLAP)
            {
                //
                // Pause the elevator.
                //
                elevatorPower = 0;
                if (!fClosingFlap)
                {
                    fClosingFlap = true;
                    SMStart(g_ToggleFlapSM);
                }
            }
            else
            {
                fClosingFlap = false;
            }

            PIDMotorSetPower(g_Elevator, elevatorPower);
        }
    }
    else if (fOverride)
    {
        backoffTime = 0;
        fOverride = false;
        PIDMotorSetPower(g_Elevator, 0);
    }
    nxtDisplayTextLine(2, "ElevHt=%5.1f,P=%d",
                       PIDCtrlGetInput(INPUTID_ELEVATOR), elevatorPower);

    if (SMIsEnabled(g_ToggleFlapSM) && SMIsReady(g_ToggleFlapSM))
    {
        ToggleFlapSMTask(g_ToggleFlapSM);
    }

    //show timer
    nxtDisplayTextLine(4, "time=%d", nPgmTime / 1000);

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
    PIDMotorTask(g_Elevator);
    BattTask(g_Batt);

    TExit();
    return;
}   //OutputTasks
