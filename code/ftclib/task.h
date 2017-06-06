#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="task.h" />
///
/// <summary>
///     This module contains the library functions for handling various robot
///     tasks.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _TASK_H
#define _TASK_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_TASK

#ifndef MAX_NUM_TASKS
    #define MAX_NUM_TASKS       8
#endif

//
// Macros.
//
#define TaskRegisterHiFreq(m)   (g_HiFreqTasks |= (m))
#define TaskRegisterInput(m)    (g_InputTasks |= (m))
#define TaskRegisterOutput(m)   (g_OutputTask |= (m))

//
// Type definitions.
//
typedef struct
{
    long  module;
    void &context;
} ROBOTTASK;

typedef struct
{
    ROBOTTASK hifreqTasks[MAX_NUM_TASKS];
    ROBOTTASK inputTasks[MAX_NUM_TASKS];
    ROBOTTASK outputTask[MAX_NUM_TASKS];
    int numHiFreqTasks;
    int numInputTasks;
    int numOutputTasks;
} TASKS;

//
// Global data..
//
TASKS g_RobotTasks;

/**
 *  This function initializes the robot task tables.
 */
void
TaskInit()
{
    int i;

    TFuncName("TaskInit");
    TLevel(INIT);
    TEnter();

    for (i = 0; i < MAX_NUM_TASKS; i++)
    {
        g_RobotTasks.hifreqTasks[i].module = 0;
        g_RobotTasks.hifreqTasks[i].context = 0;
        g_RobotTasks.inputTasks[i].module = 0;
        g_RobotTasks.inputTasks[i].context = 0;
        g_RobotTasks.outputTasks[i].module = 0;
        g_RobotTasks.outputTasks[i].context = 0;
    }
    g_RobotTasks.numHiFreqTasks = 0;
    g_RobotTasks.numInputTasks = 0;
    g_RobotTasks.numOutputTasks = 0;

    TExit();
    return;
}   //TaskInit

/**
 *  This function registers a high frequency robot task.
 *
 *  @param module Specifies the module that has a high frequency task.
 *  @param context Spcifies the context of the task.
 *
 *  @return Returns true if successful, false if the task table is full.
 */
bool
TaskRegHiFreq(
    __in long module,
    __in void &context
    )
{
    bool fSuccess = false;

    TFuncName("TaskRegHiFreq");
    TEnter();

    if (g_RobotTasks.numHiFreqTasks < MAX_NUM_TASKS)
    {
        g_RobotTasks.hifreqTasks[g_RobotTasks.numHiFreqTasks].module = module;
        g_RobotTasks.hifreqTasks[g_RobotTasks.numHiFreqTasks].context = context;
        g_RobotTasks.numHiFreqTasks++;
        fSuccess = true;
    }

    TExit();
    return fSuccess;
}   //TaskRegHiFreq

/**
 *  This function registers a robot input task.
 *
 *  @param module Specifies the module that has an input task.
 *  @param context Spcifies the context of the task.
 *
 *  @return Returns true if successful, false if the task table is full.
 */
bool
TaskRegInput(
    __in long module,
    __in void &context
    )
{
    bool fSuccess = false;

    TFuncName("TaskRegInput");
    TEnter();

    if (g_RobotTasks.numInputTasks < MAX_NUM_TASKS)
    {
        g_RobotTasks.inputTasks[g_RobotTasks.numInputTasks].module = module;
        g_RobotTasks.inputTasks[g_RobotTasks.numInputTasks].context = context;
        g_RobotTasks.numInputTasks++;
        fSuccess = true;
    }

    TExit();
    return fSuccess;
}   //TaskRegInput

/**
 *  This function registers a robot output task.
 *
 *  @param module Specifies the module that has an output task.
 *  @param context Spcifies the context of the task.
 *
 *  @return Returns true if successful, false if the task table is full.
 */
bool
TaskRegOutput(
    __in long module,
    __in void &context
    )
{
    bool fSuccess = false;

    TFuncName("TaskRegOutput");
    TEnter();

    if (g_RobotTasks.numOutputTasks < MAX_NUM_TASKS)
    {
        g_RobotTasks.outputTasks[g_RobotTasks.numOutputTasks].module = module;
        g_RobotTasks.outputTasks[g_RobotTasks.numOutputTasks].context = context;
        g_RobotTasks.numOutputTasks++;
        fSuccess = true;
    }

    TExit();
    return fSuccess;
}   //TaskRegOutput

/**
 *  This function dispatches all the registered high frequency tasks.
 */

/**
 *  This function dispatches all the registered high frequency tasks.
 */
void
TaskProcessHiFreq()
{
    TFuncName("TaskProcessHiFreq");
    TLevel(TASK);
    TEnter();

#ifdef _GYRO_H
    if (g_HiFreqTasks & MOD_GYRO)
    {
        
    }
#endif

    TExit();
    return;
}   //TaskProcessHiFreq

/**
 *  This function processes the changed buttons and sends button event
 *  notifications.
 *
 *  @param nxtbtn Points to the NXTBTN structure.
 */
void
NxtBtnTask(
    __inout NXTBTN &nxtbtn
    )
{
    int currBtn = nNxtButtonPressed;
    bool fPressed;

    TFuncName("NxtBtnTask");
    TLevel(TASK);
    TEnterMsg(("Prev=%x,Curr=%x", nxtbtn.prevBtn, currBtn));

    if (currBtn != nxtbtn.prevBtn)
    {
        if (currBtn == kNoButton)
        {
            currBtn = nxtbtn.prevBtn;
            fPressed = false;
        }
        else
        {
            fPressed = true;
        }

        if (nxtbtn.nxtBtnFlags & NXTBTNF_ENABLE_EVENTS)
        {
            NxtBtnEvent(nxtbtn, currBtn, fPressed);
        }

        nxtbtn.prevBtn = currBtn;
    }

    TExit();
    return;
}   //NxtBtnTask

#endif  //ifndef _TASK_H
