#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="CrateFront.h" />
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
 *  This function implements the "CrateFront" strategy.
 *
 *  1. Start on the outside corner of the alliance platform.
 *  2. Go down the ramp.
 *  3. Turn, go forward and grab your crate stack (the one with balls).
 *  4. Turn and bulldoze forward.
 *  5. Turn to face the bowling ball.
 *  6. Bulldoze forward and push the bowling ball to the front parking zone.
 *  7. Turn to the safe zone.
 *  8. Push the crate stack into the safe zone.
 *  9. Backup and turn to the front parking zone.
 * 10. Go and park in the front parking zone.
 *
 *  Alternatively, we will attempt to push the second crate stack to the front
 *  parking zone as well. This extended strategy will deviate from the above
 *  starting at step 9.
 *  9. Backup and turn so that the back of the robot is facing the isle
 *     between the crate stacks.
 * 10. Continue to back up until we pass the 2nd crate stack a little.
 * 11. Turn to face the crate stack.
 * 12. Go forward to grab the crate stack.
 * 13. Turn to face the front parking zone.
 * 14. Push the crate stack to the safe zone.
 * 15. Backup and turn to the front parking zone.
 * 16. Go and park in the front parking zone.
 *
 * Yet another alternative path is instead of backing up between the opponent's
 * ramp and the crate stack which has vey narrow path way, we back up through
 * the center isle. This gives us a lot more room to work with.
 *
 *  @param sm Points to the SM structure.
 *  @param direction Specifies the direction of turns (1.0 for blue alliance
 *         or -1.0 for red alliance).
 *  @param delay Specifies the delay in msec before starting the strategy.
 *         This will allow time for our partner to move out of the way.
 *  @param strategy Specifies which crate front strategy the user chose.
 *
 *  @note If our alliance has an autonomous to push the bowling ball to the
 *        front parking zone, we need to wait a little so we don't interfere
 *        with our alliance. Then we would do the same thing except that we
 *        no longer have the bowling ball to deal with but that's fine. We
 *        will just push the crate stack into our safe zone that will score
 *        us 25 points in teleop. In the extended strategy, we potentially
 *        can earn another 25 points in teleop if we push the 2nd crate
 *        stack to the front parking zone.
 */
void
CrateFront(
    __inout SM &sm,
    __in    float direction,
    __in    long delay = 0,
    __in    int strategy = STRATEGY_CRATE_FRONT
    )
{
    TFuncName("CrateFront");
    TLevel(TASK);
    TEnter();

    const long delayTime = 000;
    int nextState = 9999;
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
            // Get off platform.
            //
            PIDDriveSetTarget(g_EncoderDrive, 45.0, 0.0, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 2:
            //
            // Turn to face crates, open arms.
            //
            ServoSetAngle(g_LeftArm, ARM_LEFT_OPENED);
            ServoSetAngle(g_RightArm, ARM_RIGHT_OPENED);
            PIDDriveSetTarget(g_EncoderDrive, 0.0, 28.0*direction, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 3:
            //
            // Go forward to the crates.
            //
            PIDDriveSetTarget(g_EncoderDrive, 16.0, 0.0, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 4:
            //
            // Close arms to grab crates and wait.
            //
            ServoSetAngle(g_LeftArm, ARM_LEFT_CLOSED);
            ServoSetAngle(g_RightArm, ARM_RIGHT_CLOSED);
            TimerSet(g_Timer, ARM_OPENCLOSE_TIME);
            SMAddWaitEvent(sm, EVTTYPE_TIMER);
            SMWaitEvents(sm, sm.currState + 1);
            break;

        case SMSTATE_STARTED + 5:
            //
            // Begin your adventure circumnavigating the platform.
            // Turn to face the bowling ball.
            //
            PIDDriveSetTarget(g_EncoderDrive, 0.0, 62.0*direction, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 6:
            //
            // After we turned, we might have lose grip of the crate stack.
            // Open up the arms, move forward to allow the crate stack to
            // realign.
            // Move forward to clear the platform and turn slightly left.
            //
            ServoSetAngle(g_LeftArm, ARM_LEFT_OPENED);
            ServoSetAngle(g_RightArm, ARM_RIGHT_OPENED);
            TimerSet(g_Timer, ARM_OPENCLOSE_TIME);
            SMAddWaitEvent(sm, EVTTYPE_TIMER);
            SMWaitEvents(sm, sm.currState + 1);
            break;

        case SMSTATE_STARTED + 7:
            ServoSetAngle(g_LeftArm, ARM_LEFT_CLOSED);
            ServoSetAngle(g_RightArm, ARM_RIGHT_CLOSED);
            PIDDriveSetTarget(g_EncoderDrive, 29.5, 5.0*direction, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

#if 0
        case SMSTATE_STARTED + 7:
            ServoSetAngle(g_LeftArm, ARM_LEFT_CLOSED);
            ServoSetAngle(g_RightArm, ARM_RIGHT_CLOSED);
            TimerSet(g_Timer, ARM_OPENCLOSE_TIME);
            SMAddWaitEvent(sm, EVTTYPE_TIMER);
            SMWaitEvents(sm, sm.currState + 1);
            break;
#endif

        case SMSTATE_STARTED + 8:
            //
            // Turn to align the bowling ball to the front parking zone.
            //
            PIDDriveSetTarget(g_EncoderDrive, 0.0, 25.0*direction, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 9:
            //
            // Push the bowling ball towards the front parking zone.
            //
            PIDDriveSetTarget(g_EncoderDrive, 45.0, 0.0, true, 3000);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 10:
            //
            // Turn to the safe zone.
            //
            PIDDriveSetTarget(g_EncoderDrive, 0.0, 90.0*direction, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 11:
            //
            // Push the crates into the safe zone.
            //
            PIDDriveSetTarget(g_EncoderDrive, 5.0, 0.0, true, 1200);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 12:
            //
            // Let go of the crates.
            //
            ServoSetAngle(g_LeftArm, ARM_LEFT_OPENED);
            ServoSetAngle(g_RightArm, ARM_RIGHT_OPENED);
            TimerSet(g_Timer, ARM_OPENCLOSE_TIME);
            SMAddWaitEvent(sm, EVTTYPE_TIMER);
            SMWaitEvents(sm, sm.currState + 1);
            break;

        case SMSTATE_STARTED + 13:
            //
            // Backup.
            //
            PIDDriveSetTarget(g_EncoderDrive,
                              (strategy == STRATEGY_CRATE_FRONT)? -5.0: -16.0,
                              0.0, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            switch (strategy)
            {
                case STRATEGY_CRATE_FRONT:
                    nextState = SMSTATE_STARTED + 100;
                    break;

                case STRATEGY_BOTH_CRATES_FRONT1:
                    nextState = SMSTATE_STARTED + 200;
                    break;

                case STRATEGY_BOTH_CRATES_FRONT2:
                    nextState = SMSTATE_STARTED + 300;
                    break;
            }
            SMWaitEvents(sm, nextState, delayTime);
            break;

        //
        // STRATEGY_CRATE_FRONT:
        //
        case SMSTATE_STARTED + 100:
            //
            // Turn towards the front parking zone.
            //
            PIDDriveSetTarget(g_EncoderDrive, 0.0, -90.0*direction, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1);
            break;

        case SMSTATE_STARTED + 101:
            //
            // Go park in the front parking zone.
            //
            PIDDriveSetTarget(g_EncoderDrive, 30.0, 00.0, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1);
            break;

        //
        // STRATEGY_BOTH_CRATES_FRONT1:
        // Altrnate path that goes through untouched crates rather than enemy
        // ramp.
        //
        case SMSTATE_STARTED + 200:
            //
            // Turn to line yourself up with untouched crate path (you'll
            // back into it).
            //
            PIDDriveSetTarget(g_EncoderDrive, 0.0, -75.0*direction, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 201:
            //
            // Back into untouched crate area.
            //
            PIDDriveSetTarget(g_EncoderDrive, -76.0, 0.0, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, 7000, SMF_SET_TIMEOUT);
            break;

        case SMSTATE_STARTED + 202:
            //
            // Turn to face crate.
            //
//            if (DriveIsStalled(g_drives[0]))
            if (SMIsTimedOut(sm))
            {
                //
                // Go to recovery.
                //
                sm.currState = SMSTATE_STARTED + 400;
                break;
            }
            ServoSetAngle(g_LeftArm, ARM_LEFT_OPENED);
            ServoSetAngle(g_RightArm, ARM_RIGHT_OPENED);
            PIDDriveSetTarget(g_EncoderDrive, 0.0, -30.0*direction, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 203:
            //
            // Go forward to grab crates.
            //
            PIDDriveSetTarget(g_EncoderDrive, 24.0, 0.0, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 204:
            //
            // Close arms and grab the crates.
            //
            ServoSetAngle(g_LeftArm, ARM_LEFT_CLOSED);
            ServoSetAngle(g_RightArm, ARM_RIGHT_CLOSED);
            TimerSet(g_Timer, ARM_OPENCLOSE_TIME);
            SMAddWaitEvent(sm, EVTTYPE_TIMER);
            SMWaitEvents(sm, sm.currState + 1);
            break;

        case SMSTATE_STARTED + 205:
            //
            // Turn back towards home parking zone.
            //
            PIDDriveSetTarget(g_EncoderDrive, 0.0, 40.0*direction, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 206:
            //
            // Drive into home parking zone.
            //
            PIDDriveSetTarget(g_EncoderDrive, 80.0, 0.0, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        //
        // STRATEGY_BOTH_CRATES_FRONT2:
        // Alternate path that hits enemy ramp.
        //
        case SMSTATE_STARTED + 300:
            //
            // Turn the back towards your other crate stack.
            //
            PIDDriveSetTarget(g_EncoderDrive, 0.0, -35.0*direction, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 301:
            //
            // Go backward and get ready to grab your other crate stack.
            //
            PIDDriveSetTarget(g_EncoderDrive, -80.0, 0.0, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 302:
            //
            // Turn to grab the other crate stack
            //
            ServoSetAngle(g_LeftArm, ARM_LEFT_OPENED);
            ServoSetAngle(g_RightArm, ARM_RIGHT_OPENED);
            PIDDriveSetTarget(g_EncoderDrive, 0.0, 45.0*direction, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 303:
            //
            // Go forward to grab the crate stack.
            //
            PIDDriveSetTarget(g_EncoderDrive, 21.0, 0.0, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 304:
            //
            // Close arms to grab the crate stack.
            //
            ServoSetAngle(g_LeftArm, ARM_LEFT_CLOSED);
            ServoSetAngle(g_RightArm, ARM_RIGHT_CLOSED);
            TimerSet(g_Timer, ARM_OPENCLOSE_TIME);
            SMAddWaitEvent(sm, EVTTYPE_TIMER);
            SMWaitEvents(sm, sm.currState + 1);
            break;

        case SMSTATE_STARTED + 305:
            //
            // Turn towards the safe zone.
            //
            PIDDriveSetTarget(g_EncoderDrive, 0.0, -25.0*direction, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 306:
            //
            // Push the crate stack to the safe zone.
            //
            PIDDriveSetTarget(g_EncoderDrive, 76.0, 0.0, true, 4500);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 307:
            //
            // Let go of the crate stack and back out.
            //
            ServoSetAngle(g_LeftArm, ARM_LEFT_OPENED);
            ServoSetAngle(g_RightArm, ARM_RIGHT_OPENED);
            PIDDriveSetTarget(g_EncoderDrive, -24.0, 0.0, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 308:
            //
            // Turn towards the front parking zone.
            //
            PIDDriveSetTarget(g_EncoderDrive, 0.0, -45.0*direction, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 309:
            //
            // Go forward to the front parking zone.
            //
            PIDDriveSetTarget(g_EncoderDrive, 40.0, 0.0, true);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        case SMSTATE_STARTED + 400:
            //
            // Recovery: too many obstacles, just go home.
            //
//            PIDDriveSetTarget(g_SonarIRDrive, 8.0 * 12.0, 0.0, true);
            PIDDriveSetTarget(g_EncoderDrive, 70.0, 0.0, true, 7000);
            SMAddWaitEvent(sm, EVTTYPE_PIDDRIVE);
            SMWaitEvents(sm, sm.currState + 1, delayTime);
            break;

        default:
            SMStop(sm);
            break;
    }

    TExit();
    return;
}   //CrateFront
