#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="ParkBack1.h" />
///
/// <summary>
///     This module contains the autonomous routine for one of the strategies.
///
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_MAIN

/**
 *  This function implements the "Parking at the back" strategy.
 *
 *  The robot is positioned at the outside corner of the alliance platform,
 *  going straight down the ramp and park at the back parking zone.
 *  This will earn 5 points.
 *
 *  @param sm Points to the SM structure.
 *  @param direction Specifies the direction of turns (1.0 for blue alliance
 *         or -1.0 for red alliance). This parameter is not used in this
 *         strategy.
 *  @param delay Specifies the delay in msec before starting the strategy.
 *         This will allow time for our partner to move out of the way.
 *  @param time Specifies time to delay before moving (in msec).
 */
void
ParkBack1(
  __inout SM &sm,
  __in    float direction,
  __in    long delay = 0
  )
{
    TFuncName("ParkBack1");
    TLevel(TASK);
    TEnter();

    switch (sm.currState)
    {
        case SMSTATE_STARTED:
            if (delay != 0)
            {
                TimerSet(g_Timer, delay);
                SMAddWaitEvent(sm, EVTTYPE_TIMER);
                SMWaitEvents(sm, sm.currState + 1);
                break;
            }
            else
            {
                sm.currState++;
                //
                // Let it fall through to the next state immediately.
                //
            }

        case SMSTATE_STARTED + 1:
            //
            // Drive forward to the back parking zone.
            //
            PIDDriveSetTarget(g_EncoderDrive, 112.0, 0.0, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1);
            break;

        default:
            SMStop(sm);
            break;
    }

    TExit();
    return;
}   //ParkBack1
