#pragma config(Sensor, S1,     gyroSensor,          sensorI2CCustom)
#pragma config(Sensor, S2,     lightSensor,         sensorLightInactive)
#pragma config(Sensor, S3,     sonarSensor,         sensorSONAR)
#pragma config(Sensor, S4,     sonarLeft,           sensorSONAR)
#pragma config(Motor,  motorA,          rightMotor,    tmotorNormal, openLoop, encoder)
#pragma config(Motor,  motorB,          radarMotor,    tmotorNormal, openLoop, encoder)
#pragma config(Motor,  motorC,          leftMotor,     tmotorNormal, openLoop, encoder)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#define irSeekerSensor 4

#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="GobbMobile.c" />
///
/// <summary>
///     This module contains the main code.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

//#define _DEBUG_TRACE

#include "JoystickDriver.c"
#include "..\ftclib\trcdefs.h"
#include "..\ftclib\dbgtrace.h"
#include "..\ftclib\batt.h"
#include "..\ftclib\menu.h"
#include "..\ftclib\joybtn.h"
#include "..\ftclib\nxtbtn.h"
#include "..\ftclib\analog.h"
#include "..\ftclib\gyro.h"
#include "..\ftclib\irseeker.h"
#include "..\ftclib\radar.h"
#include "..\ftclib\drive.h"
#include "..\ftclib\pidctrl.h"

#define INPUTID_ENCODER_DRIVE           0
#define INPUTID_ENCODER_TURN            1
#define INPUTID_GYRO                    2
#define INPUTID_SONAR                   3
#define INPUTID_IRSEEKER                4
#define INPUTID_LNFOLLOWER              5
#define INPUTID_FRONT_SONAR             6
#define INPUTID_SIDE_SONAR              7
DRIVE   g_drives[1];
PIDCTRL g_pidCtrls[9];

#include "..\ftclib\piddrive.h"
#include "..\ftclib\lnfollow.h"
#include "..\ftclib\wallfollow.h"
#include "RobotInfo.h"

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                          MOD_MAIN

//
// Trace info.
//
#define TRACE_MODULES                   (MOD_MAIN)
#define TRACE_LEVEL                     TASK
#define MSG_LEVEL                       INFO

#define ROBOTMODE_TELEOP                0
#define ROBOTMODE_AUTO                  1
#define ROBOTMODE_MENU                  2

#define TASK_ARCADE_DRIVE               0
#define TASK_TANK_DRIVE                 1
#define TASK_ENCODER_DRIVE              2
#define TASK_ENCODER_TURN               3
#define TASK_GYRO_TURN                  4
#define TASK_SONARIR_DRIVE              5
#define TASK_FOLLOW_LINE                6
#define TASK_FOLLOW_WALL                7
#define DEF_TASK                        TASK_ARCADE_DRIVE

//
// Global data.
//
BATT        g_Batt;
MENU        g_Menu;
//
// Inputs and sensors.
//
JOYBTN      g_JoyBtn;
NXTBTN      g_NxtBtn;
ANALOG      g_Light;
GYRO        g_Gyro;
IRSEEKER    g_IRSeeker;
RADAR       g_Radar;
//
// Drive subsystems.
//
PIDDRIVE    g_EncoderDrive;
PIDDRIVE    g_GyroDrive;
PIDDRIVE    g_SonarIRDrive;
LNFOLLOW    g_LnFollower;
WALLFOLLOW  g_WallFollower;
//
// Global states.
//
int         g_TaskChoice = DEF_TASK;
int         g_RobotMode = ROBOTMODE_MENU;
int         g_LightMin = 1024;
int         g_LightMax = 0;
bool        g_fCalibrating = false;

/**
 *  This function turns the light on the light sensor on or off.
 *
 *  @param sensorID Specifies the light sensor ID.
 *  @param fOn If true, turn light on, off otherwise.
 *  @param fUseMUX If true, the light sensor is on a senor MUX.
 */
void
SetLightState(
    __in int sensorID,
    __in bool fOn,
    __in bool fUseMUX = false
    )
{
    TFuncName("SetLightState");
    TLevel(FUNC);
    TEnterMsg(("ID=%d,fOn=%d", sensorID, fOne));

    if (fOn)
    {
        if (fUseMUX)
        {
#ifdef __HTSMUX_SUPPORT__
            HTSMUXsetAnalogueActive((tMUXSensor)sensorID);
#else
            PlayTone(1000, 200);
#endif
        }
        else
        {
            SensorType[(tSensors)sensorID] = sensorLightActive;
            SensorMode[(tSensors)sensorID] = modeRaw;
            wait1Msec(5);
        }
    }
    else
    {
        if (fUseMUX)
        {
#ifdef __HTSMUX_SUPPORT__
            HTSMUXsetAnalogueInactive((tMUXSensor)sensorID);
#else
            PlayTone(1000, 200);
#endif
        }
        else
        {
            SensorType[(tSensors)sensorID] = sensorLightInactive;
            SensorMode[(tSensors)sensorID] = modeRaw;
            wait1Msec(5);
        }
    }

    TExit();
    return;
}   //SetLightState

/**
 *  This function enables or disables light sensor calibration.
 *
 *  @param fEnable If true enables light sensor calibration.
 */
void
CalibrateLightSensor(
    __in bool fEnable
    )
{
    TFuncName("CalibrateLS");
    TLevel(FUNC);
    TEnterMsg(("fEnable=%x", fEnable));

    if (fEnable)
    {
        g_LightMin = 1024;
        g_LightMax = 0;
        g_fCalibrating = true;
        SetLightState(lightSensor, true);
    }
    else
    {
        SetLightState(lightSensor, false);
        AnalogSetCalibration(g_Light, (g_LightMin + g_LightMax)/2, 0);
        g_fCalibrating = false;
    }

    TExit();
    return;
}   //CalibrateLightSensor

/**
 *  This function enables or disables menu mode.
 *
 *  @param fEnable If true enables menu mode.
 */
void
SetMenuMode(
    __in bool fEnable
    )
{
    TFuncName("SetMenuMode");
    TLevel(FUNC);
    TEnterMsg(("fEnable=%x", fEnable));

    if (fEnable)
    {
        BattSetState(g_Batt, false);
        MenuSetState(g_Menu, true);
        g_RobotMode = ROBOTMODE_MENU;
    }
    else
    {
        MenuSetState(g_Menu, false);
        BattSetState(g_Batt, true);
        g_RobotMode = ROBOTMODE_TELEOP;
    }

    TExit();
    return;
}   //SetMenuMode

/**
 *  This function stops the autonomous operation.
 */
void
StopAuto(
    void
    )
{
    TFuncName("StopAuto");
    TLevel(FUNC);
    TEnter();

    switch (g_TaskChoice)
    {
        case TASK_ENCODER_DRIVE:
        case TASK_ENCODER_TURN:
            PIDDriveReset(g_EncoderDrive);
            break;

        case TASK_GYRO_TURN:
            PIDDriveReset(g_GyroDrive);
            break;

        case TASK_SONARIR_DRIVE:
            PIDDriveReset(g_SonarIRDrive);
            break;

        case TASK_FOLLOW_LINE:
            LnFollowStop(g_LnFollower);
            SetLightState(lightSensor, false);
            break;

        case TASK_FOLLOW_WALL:
            WallFollowStop(g_WallFollower);
            RadarSetState(g_Radar, false);
            break;

        default:
            break;
    }
    SetMenuMode(true);

    TExit();
    return;
}   //StopAuto

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
                case Xbox_A:
                    if (fPressed)
                    {
                        switch (g_RobotMode)
                        {
                            case ROBOTMODE_TELEOP:
                                if (g_fCalibrating)
                                {
                                    CalibrateLightSensor(false);
                                }
                                //
                                // Disable battery info and show choice menu.
                                //
                                SetMenuMode(true);
                                break;

                            case ROBOTMODE_AUTO:
                                StopAuto();
                                break;

                            case ROBOTMODE_MENU:
                                SetMenuMode(false);
                                g_TaskChoice = MenuGetUserChoice(g_Menu);
                                break;
                        }
                    }
                    break;

                case Xbox_X:
                    if (fPressed && (g_RobotMode == ROBOTMODE_TELEOP))
                    {
                        CalibrateLightSensor(!g_fCalibrating);
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
                    if (g_RobotMode == ROBOTMODE_MENU)
                    {
                        MenuIncChoice(g_Menu, -1);
                    }
                    break;

                case TopHat_Down:
                    if (g_RobotMode == ROBOTMODE_MENU)
                    {
                        MenuIncChoice(g_Menu, 1);
                    }
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
 *  This function handles the NXT button notification events.
 *
 *  @param nxtButton Specifies the button ID.
 *  @param fPressed Specifies the event is a press or a release.
 */
void
NxtBtnEvent(
    __in TButtons nxtButton,
    __in bool     fPressed
    )
{
    TFuncName("NxtBtnEvent");
    TLevel(EVENT);
    TEnterMsg(("Btn=%x,On=%x", nxtButton, fPressed));

    if (fPressed)
    {
        switch (g_RobotMode)
        {
            case ROBOTMODE_TELEOP:
                if (nxtButton == kEnterButton)
                {
                    if (g_fCalibrating)
                    {
                        CalibrateLightSensor(false);
                    }
                    //
                    // Disable battery info and show choice menu.
                    //
                    SetMenuMode(true);
                }
                break;

            case ROBOTMODE_AUTO:
                if (nxtButton == kEnterButton)
                {
                    StopAuto();
                }
                break;

            case ROBOTMODE_MENU:
                if (nxtButton == kEnterButton)
                {
                    SetMenuMode(false);
                    g_TaskChoice = MenuGetUserChoice(g_Menu);
                }
                else if (nxtButton == kLeftButton)
                {
                    MenuIncChoice(g_Menu, -1);
                }
                else if (nxtButton == kRightButton)
                {
                    MenuIncChoice(g_Menu, 1);
                }
                break;
        }
    }

    TExit();
    return;
}   //NxtBtnEvent

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
 *  This function handles the drive notification events.
 *
 *  @param drive Points to the DRIVE structure that generated the event.
 */
void
PIDDriveEvent(
    __in PIDDRIVE &pidDrive
    )
{
    TFuncName("PIDDriveEvent");
    TLevel(EVENT);
    TEnter();

    if (g_TaskChoice == TASK_FOLLOW_LINE)
    {
        SetLightState(lightSensor, false);
    }
    SetMenuMode(true);

    TExit();
    return;
}   //PIDDriveEvent

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
    TEnter();

    switch (inputID)
    {
        case INPUTID_ENCODER_DRIVE:
            inputValue = (float)(nMotorEncoder[leftMotor] +
                                 nMotorEncoder[rightMotor])*
                         DISTANCE_PER_CLICK/2.0;
            break;

        case INPUTID_ENCODER_TURN:
            inputValue = (float)(nMotorEncoder[leftMotor] -
                                 nMotorEncoder[rightMotor])*
                         DEGREES_PER_CLICK;
            break;

        case INPUTID_GYRO:
            inputValue = GyroGetHeading(g_Gyro);
            break;

        case INPUTID_SONAR:
            inputValue = (float)SensorValue[sonarSensor]*INCHES_PER_CM;
            break;

        case INPUTID_IRSEEKER:
            inputValue = IRSeekerGetACDir(g_IRSeeker);
            break;

        case INPUTID_LNFOLLOWER:
            inputValue = (float)(AnalogReadValue(g_Light));
            break;

        case INPUTID_FRONT_SONAR:
//            inputValue = RadarGetData(g_Radar, RADAR_FRONT_ANGLE);
            inputValue = (float)SensorValue[sonarSensor]*INCHES_PER_CM;
            break;

        case INPUTID_SIDE_SONAR:
//            inputValue = RadarGetData(g_Radar, RADAR_SIDE_ANGLE);
            inputValue = (float)SensorValue[sonarLeft]*INCHES_PER_CM;
            break;
    }

    TExitMsg(("=%5.1f", inputValue));
    return inputValue;
}   //PIDCtrlGetInput

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

    GyroTask(g_Gyro);
    NxtBtnTask(g_NxtBtn);

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

    getJoystickSettings(joystick);
    JoyBtnTask(g_JoyBtn);
    if (g_fCalibrating)
    {
        int data = SensorRaw[lightSensor];

        if (data < g_LightMin)
        {
            g_LightMin = data;
        }

        if (data > g_LightMax)
        {
            g_LightMax = data;
        }
    }
    RadarTask(g_Radar);
//    if (RadarIsEnabled(g_Radar))
    if ((g_RobotMode == ROBOTMODE_AUTO) && (g_TaskChoice == TASK_FOLLOW_WALL))
    {
        nxtDisplayTextLine(1, "F:%5.1f,S:%5.1f",
                           (float)SensorValue[sonarSensor]*INCHES_PER_CM,
                           (float)SensorValue[sonarLeft]*INCHES_PER_CM);
#if 0
        nxtDisplayTextLine(1, "F:%5.1f,S:%5.1f",
                           RadarGetData(g_Radar, RADAR_FRONT_ANGLE),
                           RadarGetData(g_Radar, RADAR_SIDE_ANGLE));
#endif
    }

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

    if (g_RobotMode == ROBOTMODE_TELEOP)
    {
        switch (g_TaskChoice)
        {
            case TASK_ARCADE_DRIVE:
                int drivePower  = NORMALIZE_DRIVE(
                                    DEADBAND_INPUT(joystick.joy1_y1),
                                    -128, 127);
                int turnPower = NORMALIZE_DRIVE(
                                    DEADBAND_INPUT(joystick.joy1_x1),
                                    -128, 127);
                nxtDisplayTextLine(0, "Arcade Drive:");
                nxtDisplayTextLine(1, "D=%04d, T=%04d", drivePower, turnPower);
                nxtDisplayTextLine(2, "Min/Max=%d/%d", g_LightMin, g_LightMax);
                nxtDisplayTextLine(3, "LS=%d(%s)",
                                   (g_LightMin + g_LightMax)/2,
                                   g_fCalibrating? "On": "Off");
                DriveArcade(g_drives[0], drivePower, turnPower);
                break;

            case TASK_TANK_DRIVE:
                int leftPower  = NORMALIZE_DRIVE(
                                    DEADBAND_INPUT(joystick.joy1_y1),
                                    -128, 127);
                int rightPower = NORMALIZE_DRIVE(
                                    DEADBAND_INPUT(joystick.joy1_y2),
                                    -128, 127);
                nxtDisplayTextLine(0, "Tank Drive:");
                nxtDisplayTextLine(1, "L=%04d, R=%04d", leftPower, rightPower);
                DriveTank(g_drives[0], leftPower, rightPower);
                break;

            case TASK_ENCODER_DRIVE:
                nxtDisplayTextLine(0, "Encoder Drive:96");
                PIDDriveSetTarget(g_EncoderDrive, 96.0, 0.0, true);
                g_RobotMode = ROBOTMODE_AUTO;
                break;

            case TASK_ENCODER_TURN:
                nxtDisplayTextLine(0, "Encoder Turn:360");
                PIDDriveSetTarget(g_EncoderDrive, 0.0, 360.0, true);
                g_RobotMode = ROBOTMODE_AUTO;
                break;

            case TASK_GYRO_TURN:
                nxtDisplayTextLine(0, "Gyro Turn:360");
                PIDDriveSetTarget(g_GyroDrive, 0.0, 360.0, true);
                g_RobotMode = ROBOTMODE_AUTO;
                break;

            case TASK_SONARIR_DRIVE:
                nxtDisplayTextLine(0, "SonarIR Drive:");
                PIDDriveSetTarget(g_SonarIRDrive, 0.0, 12.0, 8.0, false);
                g_RobotMode = ROBOTMODE_AUTO;
                break;

            case TASK_FOLLOW_LINE:
                nxtDisplayTextLine(0, "Follow Line:");
                SetLightState(lightSensor, true);
                LnFollowStart(g_LnFollower, 0, 50, 50, 0, 0);
                g_RobotMode = ROBOTMODE_AUTO;
                break;

            case TASK_FOLLOW_WALL:
                nxtDisplayTextLine(0, "Follow Wall:");
//                RadarSetState(g_Radar, true);
                WallFollowStart(g_WallFollower, -12.0);
                g_RobotMode = ROBOTMODE_AUTO;
                break;

            default:
                break;
        }
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

    PIDDriveTask(g_EncoderDrive);
    PIDDriveTask(g_GyroDrive);
    PIDDriveTask(g_SonarIRDrive);
    LnFollowTask(g_LnFollower);
    WallFollowTask(g_WallFollower);
    DriveTask(g_drives[0]);
    BattTask(g_Batt);

    TExit();
    return;
}   //OutputTasks

/**
 *  This function initializes the robot and its subsystems.
 */
void
RobotInit()
{
    TFuncName("RobotInit");
    TLevel(INIT);
    TEnter();

    BattInit(g_Batt, 7, false);
    //
    // Init choice menu.
    //
    MenuInit(g_Menu, "Task choices:");
    MenuAddChoice(g_Menu, "Arcade Drive", TASK_ARCADE_DRIVE);
    MenuAddChoice(g_Menu, "Tank Drive", TASK_TANK_DRIVE);
    MenuAddChoice(g_Menu, "Encoder Drive", TASK_ENCODER_DRIVE);
    MenuAddChoice(g_Menu, "Encoder Turn", TASK_ENCODER_TURN);
    MenuAddChoice(g_Menu, "Gyro Turn", TASK_GYRO_TURN);
    MenuAddChoice(g_Menu, "SonarIR Drive", TASK_SONARIR_DRIVE);
    MenuAddChoice(g_Menu, "Follow Line", TASK_FOLLOW_LINE);
    MenuAddChoice(g_Menu, "Follow Wall", TASK_FOLLOW_WALL);
    //
    // Initialize inputs and sensors.
    //
    JoyBtnInit(g_JoyBtn, 1, JOYBTNF_ENABLE_EVENTS);
    NxtBtnInit(g_NxtBtn, NXTBTNF_ENABLE_EVENTS);
    AnalogInit(g_Light, lightSensor, ANALOGF_INVERSE);
    GyroInit(g_Gyro, gyroSensor, 0, GYRO_ERR_ADJ);
    IRSeekerInit(g_IRSeeker, (tSensors)irSeekerSensor);
    RadarInit(g_Radar,
              sonarSensor, radarMotor,
              RADAR_CLICKS_PER_DEGREE,
              RADAR_KP,
              RADAR_MOTOR_LIMIT);
    RadarAddSample(g_Radar, RADAR_SIDE_ANGLE);
    RadarAddSample(g_Radar, RADAR_FRONT_ANGLE);
    //
    // Initialize the PID Drive subsystem.
    //
    DriveInit(g_drives[0], leftMotor, rightMotor);
    //
    // Encode Drive.
    //
    PIDCtrlInit(g_pidCtrls[INPUTID_ENCODER_DRIVE],
                INPUTID_ENCODER_DRIVE,
                ENCODER_DRIVE_KP, ENCODER_DRIVE_KI, ENCODER_DRIVE_KD,
                ENCODER_DRIVE_TOLERANCE, ENCODER_DRIVE_SETTLING);
    PIDCtrlInit(g_pidCtrls[INPUTID_ENCODER_TURN],
                INPUTID_ENCODER_TURN,
                ENCODER_TURN_KP, ENCODER_TURN_KI, ENCODER_TURN_KD,
                ENCODER_TURN_TOLERANCE, ENCODER_TURN_SETTLING);
    PIDCtrlSetPowerLimit(g_pidCtrls[INPUTID_ENCODER_TURN], -70, 70);
    PIDDriveInit(g_EncoderDrive,
                 0,     //Drive,
                 -1,    //pidCtrlX
                 INPUTID_ENCODER_DRIVE,
                 INPUTID_ENCODER_TURN,
                 PIDDRIVEF_ENABLE_EVENTS);
    //
    // Gyro Drive.
    //
    PIDCtrlInit(g_pidCtrls[INPUTID_GYRO],
                INPUTID_GYRO,
                GYRO_KP, GYRO_KI, GYRO_KD,
                GYRO_TOLERANCE, GYRO_SETTLING);
    PIDCtrlSetPowerLimit(g_pidCtrls[INPUTID_GYRO], -70, 70);
    PIDDriveInit(g_GyroDrive,
                 0,     //Drive,
                 -1,    //pidCtrlX
                 INPUTID_ENCODER_DRIVE,
                 INPUTID_GYRO,
                 PIDDRIVEF_ENABLE_EVENTS);
    //
    // SonarIR Drive.
    //
    PIDCtrlInit(g_pidCtrls[INPUTID_SONAR],
                INPUTID_SONAR,
                SONAR_KP, SONAR_KI, SONAR_KD,
                SONAR_TOLERANCE, SONAR_SETTLING,
                PIDCTRLF_ABS_SETPOINT | PIDCTRLF_INVERSE);
    PIDCtrlInit(g_pidCtrls[INPUTID_IRSEEKER],
                INPUTID_IRSEEKER,
                IRSEEK_KP, IRSEEK_KI, IRSEEK_KD,
                IRSEEK_TOLERANCE, IRSEEK_SETTLING,
                PIDCTRLF_ABS_SETPOINT | PIDCTRLF_INVERSE);
    PIDDriveInit(g_SonarIRDrive,
                 0,     //Drive,
                 -1,    //pidCtrlX
                 INPUTID_SONAR,
                 INPUTID_IRSEEKER,
                 PIDDRIVEF_ENABLE_EVENTS);
    //
    // Initialize the Line Following subsystem.
    //
    PIDCtrlInit(g_pidCtrls[INPUTID_LNFOLLOWER],
                INPUTID_LNFOLLOWER,
                LNFOLLOW_KP, LNFOLLOW_KI, LNFOLLOW_KD,
                LNFOLLOW_TOLERANCE, LNFOLLOW_SETTLING,
                PIDCTRLF_ABS_SETPOINT);
    PIDCtrlSetPowerLimit(g_pidCtrls[INPUTID_LNFOLLOWER], -50, 50);
    LnFollowInit(g_LnFollower, 0, INPUTID_LNFOLLOWER);
    //
    // Initialize Wall Following subsystem.
    //
    PIDCtrlInit(g_pidCtrls[INPUTID_FRONT_SONAR],
                INPUTID_FRONT_SONAR,
                WALLFOLLOW_KP, WALLFOLLOW_KI, WALLFOLLOW_KD,
                WALLFOLLOW_TOLERANCE, WALLFOLLOW_SETTLING,
                PIDCTRLF_ABS_SETPOINT | PIDCTRLF_INVERSE);
    PIDCtrlInit(g_pidCtrls[INPUTID_SIDE_SONAR],
                INPUTID_SIDE_SONAR,
                WALLFOLLOW_KP, WALLFOLLOW_KI, WALLFOLLOW_KD,
                WALLFOLLOW_TOLERANCE, WALLFOLLOW_SETTLING,
                PIDCTRLF_ABS_SETPOINT | PIDCTRLF_INVERSE);
    WallFollowInit(g_WallFollower,
                   0,
                   INPUTID_FRONT_SONAR,
                   INPUTID_SIDE_SONAR);
    //
    // Initialize global states.
    //
    g_TaskChoice = DEF_TASK;
    g_fCalibrating = false;
    SetMenuMode(true);

    TExit();
    return;
}   //RobotInit

/**
 *  This task is the program entry point.
 */
task main()
{
    long nextTime;
    long currTime;

    TraceInit(TRACE_MODULES, TRACE_LEVEL, MSG_LEVEL);

    RobotInit();

    nextTime = nPgmTime;
    while (true)
    {
        currTime = nPgmTime;
        HiFreqTasks();
        if (currTime >= nextTime)
        {
            nextTime = currTime + LOOP_TIME;
            InputTasks();
            MainTasks();
            OutputTasks();
        }
        EndTimeSlice();
    }
}   //main
