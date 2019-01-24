/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <cmath>
#include <iostream>

// cisst
#include <cisstNumerical/nmrLSMinNorm.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitRevision.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitConfig.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitMTM.h>
#include "robGravityCompensationMTM.h"

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitMTM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsIntuitiveResearchKitMTM::mtsIntuitiveResearchKitMTM(const std::string & componentName, const double periodInSeconds):
    mtsIntuitiveResearchKitArm(componentName, periodInSeconds)
{
    Init();
}

mtsIntuitiveResearchKitMTM::mtsIntuitiveResearchKitMTM(const mtsTaskPeriodicConstructorArg & arg):
    mtsIntuitiveResearchKitArm(arg)
{
    Init();
}

mtsIntuitiveResearchKitMTM::~mtsIntuitiveResearchKitMTM()
{
    delete GravityCompensationMTM;
}

void mtsIntuitiveResearchKitMTM::Configure(const std::string & filename)
{
    try {
        std::ifstream jsonStream;
        Json::Value jsonConfig;
        Json::Reader jsonReader;

        jsonStream.open(filename.c_str());
        if (!jsonReader.parse(jsonStream, jsonConfig)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": failed to parse configuration\n"
                                     << jsonReader.getFormattedErrorMessages();
            exit(EXIT_FAILURE);
        }

        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << this->GetName()
                                   << " using file \"" << filename << "\"" << std::endl
                                   << "----> content of configuration file: " << std::endl
                                   << jsonConfig << std::endl
                                   << "<----" << std::endl;

        // detect is we're using 1.7 and up with two fields, kinematic and gravity-compensation
        const auto jsonKinematic = jsonConfig["kinematic"];
        if (!jsonKinematic.isNull()) {
            // extract path of main json config file to search other files relative to it
            cmnPath configPath(cmnPath::GetWorkingDirectory());
            std::string fullname = configPath.Find(filename);
            std::string configDir = fullname.substr(0, fullname.find_last_of('/'));
            configPath.Add(configDir, cmnPath::TAIL); // for arm file
            configPath.Add(std::string(sawIntuitiveResearchKit_SOURCE_DIR) + "/../share", cmnPath::TAIL); // for kinematic file

            // kinematic
            const auto fileKinematic = configPath.Find(jsonKinematic.asString());
            if (fileKinematic == "") {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: " << this->GetName()
                                         << " using file \"" << filename << "\" can't find kinematic file \""
                                         << jsonKinematic.asString() << "\"" << std::endl;
                exit(EXIT_FAILURE);
            } else {
                ConfigureDH(fileKinematic);
            }

            // gravity compensation
            const auto jsonGC = jsonConfig["gravity-compensation"];
            if (!jsonGC.isNull()) {
                const auto fileGC = configPath.Find(jsonGC.asString());
                if (fileGC == "") {
                    CMN_LOG_CLASS_INIT_ERROR << "Configure: " << this->GetName()
                                             << " using file \"" << filename << "\" can't find gravity-compensation file \""
                                             << jsonGC.asString() << "\"" << std::endl;
                    exit(EXIT_FAILURE);
                } else {
                    ConfigureGC(fileGC);
                }
            }
        } else {
            std::stringstream message;
            message << "Configure " << this->GetName() << ":" << std::endl
                    << "----------------------------------------------------" << std::endl
                    << " Warning:" << std::endl
                    << "   To take advantage of the MTM CUHK gravity compensation" << std::endl
                    << "   you should have a \"arm\" file for each MTM in the console" << std::endl
                    << "   file.  The arm file should contain the fields" << std::endl
                    << "   \"kinematic\" and \"gravity-compensation\"." << std::endl
                    << "----------------------------------------------------";
            std::cerr << "mtsIntuitiveResearchKitConsole::" << message.str() << std::endl;
            CMN_LOG_CLASS_INIT_VERBOSE << message.str() << std::endl;
            ConfigureDH(jsonConfig);
        }

    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() << ": make sure the file \""
                                 << filename << "\" is in JSON format" << std::endl;
        exit(EXIT_FAILURE);
    }
}

vct3 SO3toRPY(const vctMatRot3 & R)
{
    vct3 rpy;
    if (fabs(R[2][2]) < 1e-12 && fabs(R[1][2]) < 1e-12) {
        rpy[0] = 0.0;
        rpy[1] = atan2(R[0][2], R[2][2]);
        rpy[2] = atan2(R[1][0], R[1][1]);
    }
    else {
        rpy[0] = atan2(-R[1][2], R[2][2]);
        double sr = sin(rpy[0]);
        double cr = cos(rpy[0]);
        rpy[1] = atan2(R[0][2], cr*R[2][2] - sr*R[1][2]);
        rpy[2] = atan2(-R[0][1], R[0][0] );
    }
    return rpy;
}

void mtsIntuitiveResearchKitMTM::ConfigureGC(const std::string & filename)
{
    try {
        std::ifstream jsonStream;
        Json::Value jsonConfig;
        Json::Reader jsonReader;

        jsonStream.open(filename.c_str());
        if (!jsonReader.parse(jsonStream, jsonConfig)) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureGC " << this->GetName()
                                     << ": failed to parse gravity compensation (GC) configuration\n"
                                     << jsonReader.getFormattedErrorMessages();
            return;
        }

        CMN_LOG_CLASS_INIT_VERBOSE << "ConfigureGC: " << this->GetName()
                                   << " using file \"" << filename << "\"" << std::endl
                                   << "----> content of gravity compensation (GC) configuration file: " << std::endl
                                   << jsonConfig << std::endl
                                   << "<----" << std::endl;

        if (!jsonConfig.isNull()) {
            auto result = robGravityCompensationMTM::Create(jsonConfig);
            if (!result.Pointer) {
                CMN_LOG_CLASS_INIT_ERROR << "ConfigureGC " << this->GetName()
                                         << ": failed to create an instance of robGravityCompensationMTM with \""
                                         << filename << "\" because " << result.ErrorMessage << std::endl;
                exit(EXIT_FAILURE);
            }
            GravityCompensationMTM = result.Pointer;

        }

    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureGC " << this->GetName() << ": make sure the file \""
                                 << filename << "\" is in JSON format" << std::endl;
    }
}

robManipulator::Errno mtsIntuitiveResearchKitMTM::InverseKinematics(vctDoubleVec & jointSet,
                                                                    const vctFrm4x4 & cartesianGoal)
{
    // current orientations
    vctMatRot3 R04(Manipulator->ForwardKinematics(jointSet, 4).Rotation());
    vctMatRot3 R07(Manipulator->ForwardKinematics(jointSet, 7 ).Rotation());

    vctMatRot3 R47s(R04.Transpose() * cartesianGoal.Rotation());
    vctMatRot3 R47(R04.Transpose() * R07);

    vct3 rpys = SO3toRPY(R47s);
    vct3 rpy = SO3toRPY(R47);

    vct3 e = rpys - rpy;
    jointSet[4] += e[1];
    jointSet[5] -= e[0];
    jointSet[6] += e[2];

    // pre-feed inverse kinematics with preferred values for joint 3
    jointSet[3] = 0.0; // \todo there is room for improvement here, this should be based on RPY above

    // clamp the initial joint values just to be safe
    // \todo these values should probably be copied at beginning instead of finding them everytime
    vctDoubleVec lowerLimit(NumberOfJointsKinematics());
    vctDoubleVec upperLimit(NumberOfJointsKinematics());
    Manipulator->GetJointLimits(lowerLimit, upperLimit);
    for (size_t index = 0; index < NumberOfJointsKinematics(); ++index) {
        jointSet[index] = std::max(jointSet[index], lowerLimit[index]);
        jointSet[index] = std::min(jointSet[index], upperLimit[index]);
    }

    if (Manipulator->InverseKinematics(jointSet, cartesianGoal) == robManipulator::ESUCCESS) {
        // find closest solution mod 2 pi
        const double difference = JointsKinematics.Position()[JNT_WRIST_ROLL] - jointSet[JNT_WRIST_ROLL];
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        jointSet[JNT_WRIST_ROLL] = jointSet[JNT_WRIST_ROLL] + differenceInTurns * 2.0 * cmnPI;
        return robManipulator::ESUCCESS;
    }
    return robManipulator::EFAILURE;
}

void mtsIntuitiveResearchKitMTM::Init(void)
{
    mtsIntuitiveResearchKitArm::Init();

    mHomedOnce = false;

    // state machine specific to MTM, see base class for other states
    mArmState.AddState("CALIBRATING_ROLL");
    mArmState.AddState("ROLL_CALIBRATED");
    mArmState.AddState("HOMING_ROLL");
    mArmState.AddState("RESETTING_ROLL_ENCODER");
    mArmState.AddState("ROLL_ENCODER_RESET");

    // after arm homed
    mArmState.SetTransitionCallback("ARM_HOMED",
                                    &mtsIntuitiveResearchKitMTM::TransitionArmHomed,
                                    this);
    mArmState.SetEnterCallback("CALIBRATING_ROLL",
                               &mtsIntuitiveResearchKitMTM::EnterCalibratingRoll,
                               this);
    mArmState.SetRunCallback("CALIBRATING_ROLL",
                             &mtsIntuitiveResearchKitMTM::RunCalibratingRoll,
                             this);
    mArmState.SetTransitionCallback("ROLL_CALIBRATED",
                                    &mtsIntuitiveResearchKitMTM::TransitionRollCalibrated,
                                    this);
    mArmState.SetEnterCallback("HOMING_ROLL",
                               &mtsIntuitiveResearchKitMTM::EnterHomingRoll,
                               this);
    mArmState.SetRunCallback("HOMING_ROLL",
                             &mtsIntuitiveResearchKitMTM::RunHomingRoll,
                             this);
    mArmState.SetEnterCallback("RESETTING_ROLL_ENCODER",
                               &mtsIntuitiveResearchKitMTM::EnterResettingRollEncoder,
                               this);
    mArmState.SetRunCallback("RESETTING_ROLL_ENCODER",
                             &mtsIntuitiveResearchKitMTM::RunResettingRollEncoder,
                             this);
    mArmState.SetTransitionCallback("ROLL_ENCODER_RESET",
                                    &mtsIntuitiveResearchKitMTM::TransitionRollEncoderReset,
                                    this);

    RobotType = MTM_NULL;
    SetMTMType();

    // joint values when orientation is locked
    mEffortOrientationJoint.SetSize(NumberOfJoints());

    // initialize gripper state
    Gripper.Name().SetSize(1);
    Gripper.Name().at(0) = "gripper";
    Gripper.Type().SetSize(1);
    Gripper.Type().at(0) = PRM_JOINT_REVOLUTE;
    Gripper.Position().SetSize(1);
    GripperClosed = false;

    // initialize trajectory data
    mJointTrajectory.VelocityMaximum.SetAll(90.0 * cmnPI_180); // degrees per second
    mJointTrajectory.VelocityMaximum.Element(JNT_WRIST_ROLL) = 360.0 * cmnPI_180; // roll can go fast
    SetJointVelocityRatio(1.0);
    mJointTrajectory.AccelerationMaximum.SetAll(90.0 * cmnPI_180);
    mJointTrajectory.AccelerationMaximum.Element(JNT_WRIST_ROLL) = 360.0 * cmnPI_180;
    SetJointAccelerationRatio(1.0);
    mJointTrajectory.GoalTolerance.SetAll(3.0 * cmnPI_180); // hard coded to 3 degrees
    mJointTrajectory.GoalTolerance.Element(JNT_WRIST_ROLL) = 6.0 * cmnPI_180; // roll has low encoder resolution

    // default PID tracking errors, defaults are used for homing
    PID.DefaultTrackingErrorTolerance.SetSize(NumberOfJoints());
    PID.DefaultTrackingErrorTolerance.SetAll(10.0 * cmnPI_180);
    // last 3 joints tend to be weaker
    PID.DefaultTrackingErrorTolerance.Ref(3, 4) = 30.0 * cmnPI_180;

    this->StateTable.AddData(Gripper, "StateGripper");

    // Main interface should have been created by base class init
    CMN_ASSERT(RobotInterface);
    RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitMTM::LockOrientation, this, "LockOrientation");
    RobotInterface->AddCommandVoid(&mtsIntuitiveResearchKitMTM::UnlockOrientation, this, "UnlockOrientation");

    // Gripper
    RobotInterface->AddCommandReadState(this->StateTable, Gripper, "GetStateGripper");
    RobotInterface->AddEventVoid(GripperEvents.GripperPinch, "GripperPinchEvent");
    RobotInterface->AddEventWrite(GripperEvents.GripperClosed, "GripperClosedEvent", true);
}

void mtsIntuitiveResearchKitMTM::SetMTMType(const bool autodetect, const MTM_TYPE type)
{
    if (autodetect) {
        if (GetName() == "MTML") {
            RobotType = MTM_LEFT;
        } else if (GetName() == "MTMR") {
            RobotType = MTM_RIGHT;
        } else {
            CMN_LOG_CLASS_INIT_WARNING << "SetMTMType: auto set type failed, please set type manually" << std::endl;
        }
    } else {
        RobotType = type;
    }
}

void mtsIntuitiveResearchKitMTM::GetRobotData(void)
{
    mtsIntuitiveResearchKitArm::GetRobotData();

    if (mIsSimulated) {
        return;
    }

    // get gripper based on analog inputs
    mtsExecutionResult executionResult = RobotIO.GetAnalogInputPosSI(AnalogInputPosSI);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to GetAnalogInputPosSI failed \""
                                << executionResult << "\"" << std::endl;
        return;
    }
    // for timestamp, we assume the value ws collected at the same time as other joints
    const double position = AnalogInputPosSI.Element(JNT_GRIPPER);
    Gripper.Position()[0] = position;
    Gripper.Timestamp() = JointsPID.Timestamp();
    Gripper.Valid() = JointsPID.Valid();

    // events associated to gripper
    if (GripperClosed) {
        if (position > 0.0) {
            GripperClosed = false;
            GripperEvents.GripperClosed(false);
        }
    } else {
        if (position < 0.0) {
            GripperClosed = true;
            GripperEvents.GripperClosed(true);
            GripperEvents.GripperPinch.Execute();
        }
    }
}

void mtsIntuitiveResearchKitMTM::SetGoalHomingArm(void)
{
    // compute joint goal position
    mJointTrajectory.Goal.SetAll(0.0);
    // last joint is calibrated later
    if (!mHomedOnce) {
        mJointTrajectory.Goal.Element(JNT_WRIST_ROLL) = JointsDesiredPID.Position().Element(JNT_WRIST_ROLL);
    }
}

void mtsIntuitiveResearchKitMTM::TransitionArmHomed(void)
{
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState("CALIBRATING_ROLL");
    }
}

void mtsIntuitiveResearchKitMTM::EnterCalibratingRoll(void)
{
    if (mIsSimulated || this->mHomedOnce) {
        return;
    }

    static const double maxTrackingError = 1.0 * cmnPI; // 1/2 turn
    static const double maxRollRange = 6.0 * cmnPI + maxTrackingError; // that actual device is limited to ~2.6 turns

    // set different PID tracking error for roll
    PID.DefaultTrackingErrorTolerance.Element(JNT_WRIST_ROLL) = 1.5 * maxRollRange;  // a bit more than maxRollRange
    PID.SetTrackingErrorTolerance(PID.DefaultTrackingErrorTolerance);

    // disable joint limits on PID
    PID.SetCheckPositionLimit(false);

    // compute joint goal position, we assume PID is on from previous state
    mJointTrajectory.Goal.SetAll(0.0);
    const double currentRoll = JointsDesiredPID.Position().Element(JNT_WRIST_ROLL);
    mJointTrajectory.Goal.Element(JNT_WRIST_ROLL) = currentRoll - maxRollRange;
    mJointTrajectory.GoalVelocity.SetAll(0.0);
    mJointTrajectory.EndTime = 0.0;
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);
    PID.EnableTrackingError(true);
    RobotInterface->SendStatus(this->GetName() + ": looking for roll lower limit");
}

void mtsIntuitiveResearchKitMTM::RunCalibratingRoll(void)
{
    if (mIsSimulated || this->mHomedOnce) {
        mArmState.SetCurrentState("ROLL_CALIBRATED");
        return;
    }

    static const double maxTrackingError = 1.0 * cmnPI; // 1/2 turn
    double trackingError;
    static const double extraTime = 2.0 * cmn_s;
    const double currentTime = this->StateTable.GetTic();

    mJointTrajectory.Reflexxes.Evaluate(JointSet,
                                        JointVelocitySet,
                                        mJointTrajectory.Goal,
                                        mJointTrajectory.GoalVelocity);
    SetPositionJointLocal(JointSet);

    const robReflexxes::ResultType trajectoryResult = mJointTrajectory.Reflexxes.ResultValue();

    switch (trajectoryResult) {

    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (mJointTrajectory.EndTime == 0.0) {
            mJointTrajectory.EndTime = currentTime + mJointTrajectory.Reflexxes.Duration();
            mHomingTimer = mJointTrajectory.EndTime;
        }

        // detect tracking error and set lower limit
        trackingError = std::abs(JointsPID.Position().Element(JNT_WRIST_ROLL) - JointSet.Element(JNT_WRIST_ROLL));
        if (trackingError > maxTrackingError) {
            mHomingCalibrateRollLower = JointsPID.Position().Element(JNT_WRIST_ROLL);
            // reset PID to go to current position to avoid applying too much torque
            JointSet.Element(JNT_WRIST_ROLL) = JointsPID.Position().Element(JNT_WRIST_ROLL);
            SetPositionJointLocal(JointSet);
            // reset PID tracking errors to something reasonable
            PID.DefaultTrackingErrorTolerance.SetAll(20.0 * cmnPI_180);
            PID.SetTrackingErrorTolerance(PID.DefaultTrackingErrorTolerance);
            PID.EnableTrackingError(true);

            RobotInterface->SendStatus(this->GetName() + ": found roll lower limit");
            mArmState.SetCurrentState("ROLL_CALIBRATED");
        } else {
            // time out
            if (currentTime > mHomingTimer + extraTime) {
                RobotInterface->SendError(this->GetName() + ": unable to hit roll lower limit in time");
                this->SetDesiredState(mFallbackState);
            }
        }
        break;

    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        // we shouldn't be able to reach this goal
        RobotInterface->SendError(this->GetName() + ": went past roll lower limit");
        this->SetDesiredState(mFallbackState);
        break;

    default:
        RobotInterface->SendError(this->GetName() + ": error while evaluating trajectory");
        break;
    }
    return;
}

void mtsIntuitiveResearchKitMTM::TransitionRollCalibrated(void)
{
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState("HOMING_ROLL");
    }
}

void mtsIntuitiveResearchKitMTM::EnterHomingRoll(void)
{
    if (mIsSimulated || this->mHomedOnce) {
        return;
    }
    // compute joint goal position, we assume PID is on from previous state
    mJointTrajectory.Goal.SetAll(0.0);
    mJointTrajectory.Goal.Element(JNT_WRIST_ROLL) = mHomingCalibrateRollLower + 480.0 * cmnPI_180;
    mJointTrajectory.GoalVelocity.SetAll(0.0);
    mJointTrajectory.EndTime = 0.0;

    // we want to start from zero velocity since we hit the joint limit
    JointVelocitySet.SetAll(0.0);
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);
    PID.EnableTrackingError(true);
    RobotInterface->SendStatus(this->GetName() + ": moving roll to center");
}

void mtsIntuitiveResearchKitMTM::RunHomingRoll(void)
{
    if (mIsSimulated || this->mHomedOnce) {
        mHomedOnce = true;
        mArmState.SetCurrentState("ROLL_ENCODER_RESET");
        return;
    }

    static const double extraTime = 2.0 * cmn_s;
    const double currentTime = this->StateTable.GetTic();

    // going to center position and check we have arrived
    mJointTrajectory.Reflexxes.Evaluate(JointSet,
                                        JointVelocitySet,
                                        mJointTrajectory.Goal,
                                        mJointTrajectory.GoalVelocity);
    SetPositionJointLocal(JointSet);
    const robReflexxes::ResultType trajectoryResult = mJointTrajectory.Reflexxes.ResultValue();
    bool isHomed;

    switch (trajectoryResult) {

    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (mJointTrajectory.EndTime == 0.0) {
            mJointTrajectory.EndTime = currentTime + mJointTrajectory.Reflexxes.Duration();
            mHomingTimer = mJointTrajectory.EndTime;
        }
        break;

    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        // check position
        mJointTrajectory.GoalError.DifferenceOf(mJointTrajectory.Goal, JointsPID.Position());
        mJointTrajectory.GoalError.AbsSelf();
        isHomed = !mJointTrajectory.GoalError.ElementwiseGreaterOrEqual(mJointTrajectory.GoalTolerance).Any();
        if (isHomed) {
            mArmState.SetCurrentState("RESETTING_ROLL_ENCODER");
        } else {
            // time out
            if (currentTime > mHomingTimer + extraTime) {
                CMN_LOG_CLASS_INIT_WARNING << GetName() << ": RunHomingRoll: unable to reach home position, error in degrees is "
                                           << mJointTrajectory.GoalError * (180.0 / cmnPI) << std::endl;
                RobotInterface->SendError(this->GetName() + ": unable to reach home position");
                this->SetDesiredState(mFallbackState);
            }
        }
        break;
    default:
        RobotInterface->SendError(this->GetName() + ": error while evaluating trajectory");
        break;
    }
}

void mtsIntuitiveResearchKitMTM::EnterResettingRollEncoder(void)
{
    mHomingRollEncoderReset = false;

    // disable PID on roll joint
    vctBoolVec enableJoints(NumberOfJoints());
    enableJoints.SetAll(true);
    enableJoints.at(6) = false;
    PID.EnableJoints(enableJoints);

    // start timer
    const double currentTime = this->StateTable.GetTic();
    mHomingTimer = currentTime;
}

void mtsIntuitiveResearchKitMTM::RunResettingRollEncoder(void)
{
    // wait for some time, no easy way to check if encoder has been reset
    const double timeToWait = 10.0 * cmn_ms;
    const double currentTime = this->StateTable.GetTic();

    // first step, reset encoder
    if (!mHomingRollEncoderReset) {
        // wait a bit to make sure PID roll is off
        if ((currentTime - mHomingTimer) < timeToWait) {
            return;
        }

        // reset encoder on last joint as well as PID target position to reflect new roll position = 0
        RobotIO.ResetSingleEncoder(static_cast<int>(JNT_WRIST_ROLL));
        mHomingRollEncoderReset = true;
        return;
    }

    // wait a bit to make sure encoder has been reset
    if ((currentTime - mHomingTimer) < timeToWait) {
        return;
    }

    // re-enable all joints
    JointSet.SetAll(0.0);
    SetPositionJointLocal(JointSet);
    PID.SetCheckPositionLimit(true);
    vctBoolVec enableJoints(NumberOfJoints());
    enableJoints.SetAll(true);
    PID.EnableJoints(enableJoints);
    // pre-load JointsDesiredPID since EnterReady will use them and
    // we're not sure the arm is already mJointReady
    JointsDesiredPID.Position().SetAll(0.0);

    mHomedOnce = true;
    mArmState.SetCurrentState("ROLL_ENCODER_RESET");
}

void mtsIntuitiveResearchKitMTM::TransitionRollEncoderReset(void)
{
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState("READY");
    }
}

void mtsIntuitiveResearchKitMTM::ControlEffortOrientationLocked(void)
{
    // don't get current joint values!
    // always initialize IK from position when locked
    vctDoubleVec jointSet(mEffortOrientationJoint);
    // compute desired position from current position and locked orientation
    CartesianPositionFrm.Translation().Assign(CartesianGetLocal.Translation());
    CartesianPositionFrm.Rotation().From(mEffortOrientation);
    if (InverseKinematics(jointSet, CartesianPositionFrm) == robManipulator::ESUCCESS) {
        // assign to joints used for kinematics
        JointSet.Ref(NumberOfJointsKinematics()).Assign(jointSet);
        // finally send new joint values
        SetPositionJointLocal(JointSet);
    } else {
        RobotInterface->SendWarning(this->GetName() + ": unable to solve inverse kinematics in ControlEffortOrientationLocked");
    }
}

void mtsIntuitiveResearchKitMTM::SetControlEffortActiveJoints(void)
{
    vctBoolVec torqueMode(NumberOfJoints());
    // if orientation is locked
    if (mEffortOrientationLocked) {
        // first 3 joints in torque mode
        torqueMode.Ref(3, 0).SetAll(true);
        // last 4 in PID mode
        torqueMode.Ref(4, 3).SetAll(false);
    } else {
        // all joints in effort mode
        torqueMode.SetAll(true);
    }
    PID.EnableTorqueMode(torqueMode);
}

void mtsIntuitiveResearchKitMTM::ControlEffortCartesianPreload(vctDoubleVec & effortPreload)
{
    // most efforts will be 0
    effortPreload.Zeros();
    // find ideal position for platform using IK
    vctDoubleVec jointGoal(JointsKinematics.Position());

    if (InverseKinematics(jointGoal, CartesianGetLocal) == robManipulator::ESUCCESS) {
        // apply a linear force on joint 3 to move toward the "ideal" position
        effortPreload[3] = -0.5 * (JointsKinematics.Position()[3] - jointGoal[3]);
        // cap effort
        effortPreload[3] = std::max(effortPreload[3], -0.2);
        effortPreload[3] = std::min(effortPreload[3],  0.2);
    } else {
        RobotInterface->SendWarning(this->GetName() + ": unable to solve inverse kinematics in ControlEffortCartesianPreload");
    }
}

void mtsIntuitiveResearchKitMTM::LockOrientation(const vctMatRot3 & orientation)
{
    // if we just started lock
    if (!mEffortOrientationLocked) {
        mEffortOrientationLocked = true;
        SetControlEffortActiveJoints();
    }
    // in any case, update desired orientation in local coordinate system
    // mEffortOrientation.Assign(BaseFrame.Rotation().Inverse() * orientation);
    BaseFrame.Rotation().ApplyInverseTo(orientation, mEffortOrientation);
    mEffortOrientationJoint.Assign(JointsPID.Position());
}

void mtsIntuitiveResearchKitMTM::UnlockOrientation(void)
{
    // only unlock if needed
    if (mEffortOrientationLocked) {
        mEffortOrientationLocked = false;
        SetControlEffortActiveJoints();
    }
}


void mtsIntuitiveResearchKitMTM::AddGravityCompensationEfforts(vctDoubleVec & efforts)
{
    if (GravityCompensationMTM) {
        GravityCompensationMTM->AddGravityCompensationEfforts(JointsKinematics.Position(),
                                                              JointsKinematics.Velocity(),
                                                              efforts);
    }
}
