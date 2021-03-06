#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="radar.h" />
///
/// <summary>
///     This module contains the library functions for the sonar sensor
///     on a rotating platform.
/// </summary>
///
/// <remarks>
///     Environment: RobotC for Lego Mindstorms NXT.
/// </remarks>
#endif

#ifndef _RADAR_H
#define _RADAR_H

#pragma systemFile

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_RADAR

//
// Constants.
//
#define RADARF_USER_MASK        0x00ff
#define RADARF_SERVO_MOTOR      0x0001
#define RADARF_INVERSE          0x0002  //for encoder only
#define RADARF_ENABLED          0x0100
#define RADARF_DIR_LOW          0x0200

#ifndef RADAR_CLICKS_PER_DEGREE
    #define RADAR_CLICKS_PER_DEGREE     1.0
#endif
#ifndef RADAR_SAMPLING_DELAY
    #define RADAR_SAMPLING_DELAY        100
#endif
#ifndef MAX_SAMPLE_POINTS
    #define MAX_SAMPLE_POINTS           4
#endif

//
// Macros
//
#define RadarIsEnabled(p)               (p.radarFlags & RADARF_ENABLED)

//
// Type definitions.
//
typedef struct
{
    tSensors    sonarID;
    tMotor      motorID;
    float       posPerDegree;
    float       Kp;             //for encoder only
    int         motorPower;     //motor power limit
    long        samplingDelay;
    int         radarFlags;
    int         numSamples;
    float       sampleAngles[MAX_SAMPLE_POINTS];
    int         samplePoints[MAX_SAMPLE_POINTS];    //for encoder only
    int         sampleData[MAX_SAMPLE_POINTS];
    int         nextSample;
    long        samplingTime;
} RADAR;

/**
 *  This function initializes the radar subsystem.
 *
 *  @param radar Points to the RADAR structure to be initialized.
 *  @param sonarID Specifies the ID of the sonar sensor.
 *  @param motorID Specifies the ID of the motor.
 *  @param posPerDegree Specifes the encoder clicks per degree or
 *         position per degree for servo motor.
 *  @param Kp Specifies the proportional PID constant for the rotation.
 *  @param motorPower Specifies the motor power limit (must be positive).
 *  @param samplingDelay Specifies the sampling delay time in msec.
 *  @param radarFlags Specifies the radar flags.
 */
void
RadarInit(
    __out RADAR &radar,
    __in  tSensors sonarID,
    __in  tMotor motorID,
    __in  float posPerDegree,
    __in  float Kp,
    __in  int motorPower,
    __in  long samplingDelay = RADAR_SAMPLING_DELAY,
    __in  int radarFlags = 0
    )
{
    TFuncName("RadarInit");
    TLevel(INIT);
    TEnter();

    radar.sonarID = sonarID;
    radar.motorID = motorID;
    radar.radarFlags = radarFlags & RADARF_USER_MASK;
    radar.posPerDegree = posPerDegree;
    if (!(radarFlags & RADARF_SERVO_MOTOR))
    {
        nMotorEncoder[motorID] = 0;
        radar.Kp = Kp;
    }
    else
    {
        //
        // INVERSE flag is not applicable for servo motor.
        //
        radar.radarFlags &= ~RADARF_INVERSE;
    }
    radar.motorPower = abs(motorPower);
    radar.samplingDelay = samplingDelay;
    radar.numSamples = 0;
    radar.nextSample = 0;
    radar.samplingTime = 0;

    TExit();
    return;
}   //RadarInit

/**
 *  This function adds a sampling angle.
 *
 *  @param radar Points to the RADAR structure to be initialized.
 *  @param angle Specifies the sample angling in degrees.
 *
 *  @return Returns true if successful, false otherwise.
 */
bool
RadarAddSample(
    __inout RADAR &radar,
    __in    float angle
    )
{
    bool fSuccess = false;
    int i, j;

    TFuncName("RadarAddSample");
    TLevel(API);
    TEnter();

    if (radar.numSamples < MAX_SAMPLE_POINTS)
    {
        if (radar.radarFlags & RADARF_SERVO_MOTOR)
        {
            //
            // The sample angles must be sorted, so let's do an insertion
            // sort here.
            //
            for (i = 0; i < radar.numSamples; i++)
            {
                if (angle < radar.sampleAngles[i])
                {
                    for (j = radar.numSamples - 1; j >= i; j--)
                    {
                        radar.sampleAngles[j + 1] = radar.sampleAngles[j];
                    }
                    fSuccess = true;
                    break;
                }
                else if (angle == radar.sampleAngles[i])
                {
                    //
                    // If sample angle already exists, don't add another one.
                    //
                    break;
                }
            }

            if (fSuccess || (i == radar.numSamples))
            {
                radar.sampleAngles[i] = angle;
                radar.numSamples++;
                fSuccess = true;
            }
        }
        else
        {
            int point = (radar.radarFlags & RADARF_INVERSE)?
                            (int)(-angle*radar.posPerDegree):
                            (int)(angle*radar.posPerDegree);
            //
            // The sample points must be sorted, so let's do an insertion sort
            // here.
            //
            for (i = 0; i < radar.numSamples; i++)
            {
                if (point < radar.samplePoints[i])
                {
                    for (j = radar.numSamples - 1; j >= i; j--)
                    {
                        radar.sampleAngles[j + 1] = radar.sampleAngles[j];
                        radar.samplePoints[j + 1] = radar.samplePoints[j];
                    }
                    fSuccess = true;
                    break;
                }
                else if (point == radar.samplePoints[i])
                {
                    //
                    // If sample point already exists, don't add another one.
                    //
                    break;
                }
            }

            if (fSuccess || (i == radar.numSamples))
            {
                radar.sampleAngles[i] = angle;
                radar.samplePoints[i] = point;
                radar.numSamples++;
                fSuccess = true;
            }
        }
    }

    TExitMsg(("=%d", fSuccess));
    return fSuccess;
}   //RadarAddSample

/**
 *  This function gets the sample data for the angle sampling point.
 *
 *  @param radar Points to the RADAR structure.
 *  @param angle Specifies the sampling angle to get its data.
 *
 *  @return If success, it returns the sonar reading of the given angle.
 *          If failure, it returns a negative value.
 */
float
RadarGetData(
    __inout RADAR &radar,
    __in    float angle
    )
{
    float data = -1.0;

    TFuncName("RadarGetData");
    TLevel(API);
    TEnterMsg(("Angle=%5.1f", angle));

    for (int i = 0; i < radar.numSamples; i++)
    {
        if (angle == radar.sampleAngles[i])
        {
            data = radar.sampleData[i]*INCHES_PER_CM;
            break;
        }
    }

    TExitMsg(("=%5.1f", data));
    return data;
}   //RadarGetData

/**
 *  This function enables or disables the radar subsystem.
 *
 *  @param radar Points to the RADAR structure.
 *  @param fEnable If true, enable the radar subsystem, disable otherwise.
 */
void
RadarSetState(
    __inout RADAR &radar,
    __in    bool fEnable
    )
{
    TFuncName("RadarSetState");
    TLevel(API);
    TEnterMsg(("fEnable=%d", fEnable));

    if (fEnable)
    {
        if (radar.radarFlags & RADARF_SERVO_MOTOR)
        {
            radar.samplingTime = nPgmTime + radar.samplingDelay;
        }
        else
        {
            int currPos = nMotorEncoder[radar.motorID];

            radar.nextSample = radar.numSamples - 1;
            for (int i = 0; i < radar.numSamples; i++)
            {
                if (currPos < radar.samplePoints[i])
                {
                    radar.nextSample = i;
                    break;
                }
            }

            if (currPos < radar.samplePoints[radar.nextSample])
            {
                radar.radarFlags &= ~RADARF_DIR_LOW;
            }
            else
            {
                radar.radarFlags |= RADARF_DIR_LOW;
            }

            radar.samplingTime = 0;
        }

        radar.radarFlags |= RADARF_ENABLED;
    }
    else
    {
        if (!(radar.radarFlags & RADARF_SERVO_MOTOR))
        {
            motor[radar.motorID] = 0;
        }
        radar.radarFlags &= ~RADARF_ENABLED;
    }

    TExit();
    return;
}   //RadarSetEnable

/**
 *  This function records the sonar reading and calculate the next sample
 *  point.
 *
 *  @param radar Points to the RADAR structure.
 */
void
RadarSampleData(
    __inout RADAR &radar
    )
{
    TFuncName("RadarSampleData");
    TLevel(FUNC);
    TEnter();

    radar.sampleData[radar.nextSample] = SensorValue[radar.sonarID];
    if (radar.radarFlags & RADARF_DIR_LOW)
    {
        if (radar.nextSample == 0)
        {
            //
            // Reached the low limit, reverse.
            //
            radar.nextSample = 1;
            radar.radarFlags &= ~RADARF_DIR_LOW;
        }
        else
        {
            radar.nextSample--;
        }
    }
    else
    {
        if (radar.nextSample == radar.numSamples - 1)
        {
            //
            // Reached the high limit, reverse.
            //
            radar.nextSample = radar.numSamples - 2;
            radar.radarFlags |= RADARF_DIR_LOW;
        }
        else
        {
            radar.nextSample++;
        }
    }

    TExit();
    return;
}   //RadarSampleData

/**
 *  This function performs the radar task where it will turn the sonar sensor
 *  left and right.
 *
 *  @param radar Points to the RADAR structure.
 */
void
RadarTask(
    __inout RADAR &radar
    )
{
    TFuncName("RadarTask");
    TLevel(TASK);
    TEnter();

    if (radar.radarFlags & RADARF_ENABLED)
    {
        long currTime = nPgmTime;

        if (radar.radarFlags & RADARF_SERVO_MOTOR)
        {
            if (currTime >= radar.samplingTime)
            {
                //
                // The servo has reached the next sample point, take a sample,
                // calculate the next sample point and program the servo to
                // go to the next sample point.
                //
                RadarSampleData(radar);
                servo[(TServoIndex)radar.motorID] = radar.sampleAngles[radar.nextSample]*
                                       radar.posPerDegree;
                radar.samplingTime = currTime + radar.samplingDelay;
            }
        }
        else
        {
            int currPos = nMotorEncoder[radar.motorID];


            if (radar.samplingTime == 0)
            {
                //
                // The motor still moving, check for target and apply PID.
                //
                if ((radar.radarFlags & RADARF_DIR_LOW) &&
                    (currPos <= radar.samplePoints[radar.nextSample]) ||
                    !(radar.radarFlags & RADARF_DIR_LOW) &&
                    (currPos >= radar.samplePoints[radar.nextSample]))
                {
                    //
                    // We have reached the next sample point. Stop the motor,
                    // wait for it to settle before we take the sample.
                    //
                    motor[radar.motorID] = 0;
                    radar.samplingTime = currTime + radar.samplingDelay;
                }
            }
            else if (currTime >= radar.samplingTime)
            {
                //
                // The motor has reached the next sample point, take a sample
                // and calculate the next sample point.
                //
                RadarSampleData(radar);
                radar.samplingTime = 0;
            }

            if (radar.samplingTime == 0)
            {
                //
                // The motor should be moving, do PID control on the motor.
                //
                int error = radar.samplePoints[radar.nextSample] - currPos;
                motor[radar.motorID] = BOUND(error*radar.Kp,
                                             -radar.motorPower,
                                             radar.motorPower);
            }
        }
    }

    TExit();
    return;
}   //RadarTask

#endif  //ifndef _RADAR_H
