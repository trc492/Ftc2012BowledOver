#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="nxtbtn.h" />
///
/// <summary>
///     This module contains the library functions for handling the NXT button
///     events.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _NXTBTN_H
#define _NXTBTN_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_NXTBTN

//
// Constants.
//
#define NXTBTNF_USER_MASK       0x00ff
#define NXTBTNF_ENABLE_EVENTS   0x0001

//
// Type definitions.
//
typedef struct
{
    int      nxtBtnFlags;
    TButtons prevBtn;
    long     debounceTimeout;
} NXTBTN;

//
// Import function prototypes.
//
void
NxtBtnEvent(
    __in TButtons nxtButton,
    __in bool     fPressed
    );

/**
 *  This function initializes the NXT button system.
 *
 *  @param nxtbtn Points to the NXTBTN structure to be initialized.
 *  @param nxtBtnFlags Specifies the button flags.
 */
void
NxtBtnInit(
    __out NXTBTN &nxtbtn,
    __in  int nxtBtnFlags = 0
    )
{
    TFuncName("NxtBtnInit");
    TLevel(INIT);
    TEnter();

    nxtbtn.nxtBtnFlags = nxtBtnFlags & NXTBTNF_USER_MASK;
    nxtbtn.prevBtn = nNxtButtonPressed;
    nxtbtn.debounceTimeout = 0;

    TExit();
    return;
}   //NxtBtnInit

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
    long currTime = nPgmTime;

    TFuncName("NxtBtnTask");
    TLevel(TASK);
    TEnterMsg(("Prev=%d,Curr=%d", nxtbtn.prevBtn, currBtn));

    if ((nxtbtn.debounceTimeout == 0) || (currTime >= nxtbtn.debounceTimeout))
    {
        TButtons currBtn = nNxtButtonPressed;
        TButtons btnID;
        bool fPressed;

        nxtbtn.debounceTimeout = 0;
        if (currBtn != nxtbtn.prevBtn)
        {
            if (currBtn == kNoButton)
            {
                btnID = nxtbtn.prevBtn;
                fPressed = false;
            }
            else
            {
                btnID = currBtn;
                fPressed = true;
            }

            if (nxtbtn.nxtBtnFlags & NXTBTNF_ENABLE_EVENTS)
            {
                NxtBtnEvent(btnID, fPressed);
            }

            nxtbtn.prevBtn = currBtn;
            nxtbtn.debounceTimeout = currTime + NXTBTN_DEBOUNCE_TIME;
        }
    }

    TExit();
    return;
}   //NxtBtnTask

#endif  //ifndef _NXTBTN_H
