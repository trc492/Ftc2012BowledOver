#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="trcdefs.h" />
///
/// <summary>
///     This module contains common definitions that can be used anywhere.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _TRCDEFS_H
#define _TRCDEFS_H

//
// Macros.
//
#define __in
#define __out
#define __inout

#define ARRAYSIZE(a)            (sizeof(a)/sizeof(a[0]))
#define max(a, b)               (((a) > (b))? (a): (b))
#define min(a, b)               (((a) < (b))? (a): (b))

/**
 *  The BOUND macro limits the value (n) within the bounds between the given
 *  low (l) and high (h).
 */
#define BOUND(n,l,h)            (((n) < (l))? (l): ((n) > (h))? (h): (n))

//
// Joystick input macros.
//
#ifndef DEADBAND_INPUT_THRESHOLD
    #define DEADBAND_INPUT_THRESHOLD 20
#endif

/**
 *  These macros ignore input value (n) that is within the DEADBAND_THRESHOLD.
 *  This is necessary because analog joysticks do not always centered at zero.
 *  So if the joystick is at the rest position, we will consider it zero even
 *  though the value is non-zero but within DEADBAND_THRESHOLD.
 */
#define DEADBAND(n,t)           ((abs(n) > (t))? (n): 0)
#define DEADBAND_INPUT(n)       DEADBAND(n, DEADBAND_INPUT_THRESHOLD)

/**
 *  This macro limits the input value (n) to the range between -128 and 127.
 *  This is useful when calculations on the input value may bring the result
 *  outside of the valid range. This macro will make sure the result is within
 *  bounds.
 */
#define BOUND_INPUT(n)          BOUND(n, -128, 127)

/**
 *  The NORMALIZE macro transforms a value (n) in the range between (sl) and
 *  (sh) to the range between (tl) and (th).
 */
#define MOTOR_MIN_VALUE         -100
#define MOTOR_MAX_VALUE         100

#define NORMALIZE(n,sl,sh,tl,th) (int)(((long)(n) - (sl))*((th) - (tl))/((sh) - (sl)) + (tl))
#define NORMALIZE_DRIVE(x,m,n)  NORMALIZE(x, m, n, \
                                          MOTOR_MIN_VALUE, MOTOR_MAX_VALUE)
#define JOYSTICK_POWER(n)       NORMALIZE_DRIVE(DEADBAND_INPUT(n), -128, 127)

#ifndef LOOP_TIME
    #define LOOP_TIME           20      //20-msec
#endif

#define INCHES_PER_METER        39.370079
#define INCHES_PER_CM           (1.0/2.54)

#define NXTBTN_DEBOUNCE_TIME    20

#endif  //ifndef _TRCDEFS_H
