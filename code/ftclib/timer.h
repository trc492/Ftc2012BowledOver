#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="timer.h" />
///
/// <summary>
///     This module contains the library functions for the event timer.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _TIMER_H
#define _TIMER_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_TIMER

//
// Constants.
//
#define TIMERF_USER_MASK        0x00ff
#define TIMERF_ENABLED          0x0100
#define TIMERF_ENABLE_EVENTS    0x0001

//
// Type definitions.
//
typedef struct
{
    long expiredTime;
    int  timerFlags;
} TIMER;

//
// Import function prototypes.
//
void
TimerEvent(
    __in TIMER &timer
    );

/**
 *  This function initializes the event timer.
 *
 *  @param timer Points to the TIMER structure to be initialized.
 *  @param timerFlags Specifies the timer flags.
 */
void
TimerInit(
    __out TIMER &timer,
    __in  int timerFlags = 0
    )
{
    TFuncName("TimerInit");
    TLevel(INIT);
    TEnter();

    timer.expiredTime = 0;
    timer.timerFlags = timerFlags & TIMERF_USER_MASK;

    TExit();
    return;
}   //TimerInit

/**
 *  This function sets the event timer.
 *
 *  @param timer Points to the TIMER structure.
 *  @param time Specifies the expire time relative to current time in msec.
 */
void
TimerSet(
    __out TIMER &timer,
    __in  long time
    )
{
    TFuncName("TimerSet");
    TLevel(API);

    timer.expiredTime = (long)nPgmTime + time;
    timer.timerFlags |= TIMERF_ENABLED;

    TExit();
    return;
}   //TimerSet

/**
 *  This function resets the event timer.
 *
 *  @param timer Points to the TIMER structure.
 */
void
TimerReset(
    __out TIMER &timer
    )
{
    TFuncName("TimerReset");
    TLevel(API);

    timer.expiredTime = 0;
    timer.timerFlags &= TIMERF_USER_MASK;

    TExit();
    return;
}   //TimerReset

/**
 *  This function performs the timer task.
 *
 *  @param timer Points to the TIMER structure.
 */
void
TimerTask(
    __inout TIMER &timer
    )
{
    TFuncName("TimerTask");
    TLevel(TASK);
    TEnter();

    if ((timer.timerFlags & TIMERF_ENABLED) &&
        ((long)nPgmTime >= timer.expiredTime) &&
        (timer.timerFlags & TIMERF_ENABLE_EVENTS))
    {
        timer.timerFlags &= ~TIMERF_ENABLED;
        TimerEvent(timer);
    }

    TExit();
    return;
}   //TimerTask

#endif  //ifndef _TIMER_H
