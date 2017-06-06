#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="ParkFront1.h" />
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
 *  This function implements the "Parking at the front" strategy.
 *
 *  The robot is positioned at the inside of the alliance platform, going down
 *  the ramp then turn towards the bowling ball, capture it and turn towards
 *  the front parking zone, push the bowling and park there. This will earn
 *  10 points for parking the robot in the front and addition 10 points for
 *  parking the bowling ball in the front for a total of 20 points.
 *
 *  @param sm Points to the SM structure.
 *  @param direction Specifies the direction of turns (1.0 for blue alliance
 *         or -1.0 for red alliance).
 *  @param delay Specifies the delay in msec before starting the strategy.
 *         This will allow time for our partner to move out of the way.
 */
void
ParkFront1(
  __inout SM &sm,
  __in    float direction,
  __in    long delay = 0
  )
{
    TFuncName("ParkFront1");
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
            // Drive forward to push the crate stacks out of the way.
            //
            PIDDriveSetTarget(g_EncoderDrive, 66.0, 0.0, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1);
            break;

        case SMSTATE_STARTED + 2:
            //
            // Backup before turning to avoid hitting the crates.
            //
            PIDDriveSetTarget(g_EncoderDrive, -12.0, 0.0, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1);
            break;

        case SMSTATE_STARTED + 3:
            //
            // Turn robot so that it is facing the ball.
            //
            PIDDriveSetTarget(g_EncoderDrive, 0.0, 90.0*direction, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1);
            break;

        case SMSTATE_STARTED + 4:
            //
            // Open the arms, drive towards the bowling ball.
            // Curve a little to better align to the ball.
            //
            ServoSetAngle(g_LeftArm, ARM_LEFT_OPENED);
            ServoSetAngle(g_RightArm, ARM_RIGHT_OPENED);
            PIDDriveSetTarget(g_EncoderDrive, 20.0, 20.0*direction, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1);
            break;

        case SMSTATE_STARTED + 5:
            PIDDriveSetTarget(g_EncoderDrive, 60.0, 5.0*direction, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1);
            break;

        default:
            SMStop(sm);
            break;
    }

    TExit();
    return;
}   //ParkFront1
