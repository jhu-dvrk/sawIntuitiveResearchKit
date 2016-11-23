/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2016 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitMTM.h>

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
            return;
        }

        ConfigureDH(jsonConfig);

    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() << ": make sure the file \""
                                 << filename << "\" is in JSON format" << std::endl;
    }
}

robManipulator::Errno mtsIntuitiveResearchKitMTM::InverseKinematics(vctDoubleVec & jointSet,
                                                                    const vctFrm4x4 & cartesianGoal)
{
    // pre-feed inverse kinematics with preferred values for joint 6
    jointSet[5] = 0.0;
    if (Manipulator.InverseKinematics(jointSet, cartesianGoal) == robManipulator::ESUCCESS) {
        // find closest solution mod 2 pi
        const double difference = JointGet[6] - jointSet[6];
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        jointSet[6] = jointSet[6] + differenceInTurns * 2.0 * cmnPI;
        return robManipulator::ESUCCESS;
    }
    return robManipulator::EFAILURE;
}

void mtsIntuitiveResearchKitMTM::Init(void)
{
    mtsIntuitiveResearchKitArm::Init();

    RobotType = MTM_NULL;
    SetMTMType();

    // Impedance Controller
    mImpedanceController = new osaImpedanceController();

    // joint values when orientation is locked
    EffortOrientationJoint.SetSize(NumberOfJoints());

    // initialize gripper state
    GripperClosed = false;

    JointTrajectory.Velocity.SetAll(180.0 * cmnPI_180); // degrees per second
    JointTrajectory.Acceleration.SetAll(180.0 * cmnPI_180);
    JointTrajectory.Velocity.Element(6) = 360.0 * cmnPI_180; // roll can go fast
    JointTrajectory.Acceleration.Element(6) = 360.0 * cmnPI_180;
    JointTrajectory.GoalTolerance.SetAll(3.0 * cmnPI_180); // hard coded to 3 degrees
    JointTrajectory.GoalTolerance.Element(6) = 6.0 * cmnPI_180; // roll has low encoder resolution
     // IO level treats the gripper as joint :-)
    PotsToEncodersTolerance.SetAll(15.0 * cmnPI_180); // 15 degrees for rotations
    // pots on gripper rotation are not directly mapped to encoders
    PotsToEncodersTolerance.Element(6) = cmnTypeTraits<double>::PlusInfinityOrMax();
    // last joint is gripper, encoders can be anything
    PotsToEncodersTolerance.Element(7) = cmnTypeTraits<double>::PlusInfinityOrMax();

    this->StateTable.AddData(GripperPosition, "GripperAngle");

    // Main interface should have been created by base class init
    CMN_ASSERT(RobotInterface);
    RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitMTM::LockOrientation, this, "LockOrientation");
    RobotInterface->AddCommandVoid(&mtsIntuitiveResearchKitMTM::UnlockOrientation, this, "UnlockOrientation");
    RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitMTM::SetImpedanceGains, this, "SetImpedanceGains");

    // Gripper
    RobotInterface->AddCommandReadState(this->StateTable, GripperPosition, "GetGripperPosition");
    RobotInterface->AddEventVoid(GripperEvents.GripperPinch, "GripperPinchEvent");
    RobotInterface->AddEventWrite(GripperEvents.GripperClosed, "GripperClosedEvent", true);
}

void mtsIntuitiveResearchKitMTM::RunArmSpecific(void)
{
    switch (RobotState) {
    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_CALIBRATING_ROLL:
        RunHomingCalibrateRoll();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_EFFORT_CARTESIAN_IMPEDANCE:
        RunEffortCartesianImpedance();
        break;
    default:
        break;
    }
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
    }
    else {
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
    GripperPosition = AnalogInputPosSI.Element(7);
    if (GripperClosed) {
        if (GripperPosition > 0.0) {
            GripperClosed = false;
            GripperEvents.GripperClosed(false);
        }
    } else {
        if (GripperPosition < 0.0) {
            GripperClosed = true;
            GripperEvents.GripperClosed(true);
            GripperEvents.GripperPinch.Execute();
        }
    }
}

void mtsIntuitiveResearchKitMTM::SetState(const mtsIntuitiveResearchKitArmTypes::RobotStateType & newState)
{
    CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": SetState: new state "
                            << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(newState) << std::endl;

    vctBoolVec torqueMode(8);

    // first cleanup from previous state
    switch (RobotState) {

    case mtsIntuitiveResearchKitArmTypes::DVRK_EFFORT_CARTESIAN:
        // Disable torque mode for all joints
        torqueMode.SetAll(false);
        PID.EnableTorqueMode(torqueMode);
        PID.SetTorqueOffset(vctDoubleVec(8, 0.0));
        SetPositionJointLocal(JointGetDesired);
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_JOINT:
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN:
        StartTrajectory(false);
        break;

    default:
        break;
    }

    // setup transition
    switch (newState) {
    case mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED:
        RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfAxes(), 0.0));
        RobotIO.DisablePower();
        PID.Enable(false);
        PID.SetCheckJointLimit(true);
        StartTrajectory(false);
        RobotState = newState;
        MessageEvents.Status(this->GetName() + " not initialized");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_BIAS_ENCODER:
        HomingBiasEncoderRequested = false;
        RobotState = newState;
        MessageEvents.Status(this->GetName() + " updating encoders based on potentiometers");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_POWERING:
        HomingTimer = 0.0;
        HomingPowerRequested = false;
        RobotState = newState;
        MessageEvents.Status(this->GetName() + " powering");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_CALIBRATING_ARM:
        HomingCalibrateArmStarted = false;
        RobotState = newState;
        this->MessageEvents.Status(this->GetName() + " calibrating arm");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_CALIBRATING_ROLL:
        HomingCalibrateRollSeekLower = false;
        HomingCalibrateRollSeekCenter = false;
        HomingCalibrateRollLower = cmnTypeTraits<double>::MaxPositiveValue();
        RobotState = newState;
        this->MessageEvents.Status(this->GetName() + " calibrating roll");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_READY:
        RobotState = newState;
        MessageEvents.Status(this->GetName() + " ready");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_JOINT:
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_JOINT:
        if (this->RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_READY) {
            MessageEvents.Error(this->GetName() + " is not ready");
            return;
        }
        RobotState = newState;
        JointSet.Assign(JointGetDesired, this->NumberOfJoints());
        if (newState == mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_JOINT) {
            IsGoalSet = false;
            MessageEvents.Status(this->GetName() + " position joint");
        } else {
            StartTrajectory(true);
            MessageEvents.Status(this->GetName() + " position goal joint");
        }
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN:
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN:
    {
        if (this->RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_ARM_CALIBRATED) {
            MessageEvents.Error(this->GetName() + " is not calibrated");
            return;
        }
        RobotState = newState;
        //set jnt to current pos, otherwise the robot will jump to previous setpoint
        JointSet.ForceAssign(JointGet);
        SetPositionJointLocal(JointSet);

        if (newState == mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN)
        {
            IsGoalSet = false;
            MessageEvents.Status(this->GetName() + " position cartesian");
        } else {
            StartTrajectory(true);
            MessageEvents.Status(this->GetName() + " position goal cartesian");
        }
        break;
    }

    case mtsIntuitiveResearchKitArmTypes::DVRK_EFFORT_CARTESIAN:
        if (RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_READY) {
            MessageEvents.Error(this->GetName() + " is not ready");
            return;
        }
        torqueMode.SetAll(true);
        PID.EnableTorqueMode(torqueMode);
        PID.EnableTrackingError(false);
        PID.SetTorqueOffset(vctDoubleVec(8, 0.0));
        RobotState = newState;
        mWrenchSet.Force().Zeros();
        mWrenchType = WRENCH_UNDEFINED;
        EffortOrientationLocked = false;
        MessageEvents.Status(this->GetName() + " effort cartesian");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_EFFORT_CARTESIAN_IMPEDANCE:
        if (RobotState < mtsIntuitiveResearchKitArmTypes::DVRK_READY) {
            MessageEvents.Error(this->GetName() + " is not ready");
            return;
        }
        torqueMode.SetAll(true);
        PID.EnableTorqueMode(torqueMode);
        PID.EnableTrackingError(false);
        PID.SetTorqueOffset(vctDoubleVec(8, 0.0));
        RobotState = newState;
        mImpedanceController->ResetGains();
        mWrenchSet.Force().Zeros();
        mWrenchType = WRENCH_BODY;
        mWrenchBodyOrientationAbsolute = true;
        EffortOrientationLocked = false;
        MessageEvents.Status(this->GetName() + " effort cartesian impedance");
        break;

    default:
        break;
    }

    // Emit event with current state
    MessageEvents.RobotState(mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(this->RobotState));
}

void mtsIntuitiveResearchKitMTM::RunHomingCalibrateArm(void)
{
    if (mIsSimulated) {
        this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_CALIBRATING_ROLL);
        return;
    }

    static const double extraTime = 2.0 * cmn_s;
    const double currentTime = this->StateTable.GetTic();

    // trigger motion
    if (!HomingCalibrateArmStarted) {
        // disable joint limits
        PID.SetCheckJointLimit(false);
        // enable PID and start from current position
        JointSet.ForceAssign(JointGet);
        SetPositionJointLocal(JointSet);
        PID.Enable(true);

        // compute joint goal position
        JointTrajectory.Goal.SetAll(0.0);
        JointTrajectory.GoalVelocity.SetAll(0.0);
        JointTrajectory.EndTime = 0.0;

        // last joint is calibrated later
        if (!HomedOnce) {
            JointTrajectory.Goal.Element(JNT_WRIST_ROLL) = JointGet.Element(JNT_WRIST_ROLL);
        }
        StartTrajectory(true);
        HomingCalibrateArmStarted = true;
    }

    JointTrajectory.Reflexxes.Evaluate(JointSet,
                                       JointVelocitySet,
                                       JointTrajectory.Goal,
                                       JointTrajectory.GoalVelocity);
    SetPositionJointLocal(JointSet);

    const robReflexxes::ResultType trajectoryResult = JointTrajectory.Reflexxes.ResultValue();

    switch (trajectoryResult) {

    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (JointTrajectory.EndTime == 0.0) {
            JointTrajectory.EndTime = currentTime + JointTrajectory.Reflexxes.Duration();
            HomingTimer = JointTrajectory.EndTime;
        }
        break;

    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        {
            // check position
            JointTrajectory.GoalError.DifferenceOf(JointTrajectory.Goal, JointGet);
            JointTrajectory.GoalError.AbsSelf();
            bool isHomed = !JointTrajectory.GoalError.ElementwiseGreaterOrEqual(JointTrajectory.GoalTolerance).Any();
            if (isHomed) {
                PID.SetCheckJointLimit(true);
                MessageEvents.Status(this->GetName() + " arm calibrated");
                this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_CALIBRATING_ROLL);
            } else {
                // time out
                if (currentTime > HomingTimer + extraTime) {
                    CMN_LOG_CLASS_INIT_WARNING << GetName() << ": RunHomingCalibrateArm: unable to reach home position, error in degrees is "
                                               << JointTrajectory.GoalError * (180.0 / cmnPI) << std::endl;
                    MessageEvents.Error(this->GetName() + " unable to reach home position during calibration on pots.");
                    this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
                }
            }
        }
        break;

    default:
        MessageEvents.Error(this->GetName() + " error while evaluating trajectory.");
        break;
    }
}

void mtsIntuitiveResearchKitMTM::RunHomingCalibrateRoll(void)
{
    if (mIsSimulated || this->HomedOnce) {
        this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_READY);
        return;
    }

    static const double maxTrackingError = 1.0 * cmnPI; // 1/2 turn
    static const double maxRollRange = 6.0 * cmnPI + maxTrackingError; // that actual device is limited to ~2.6 turns
    static const double extraTime = 2.0 * cmn_s;

    const double currentTime = this->StateTable.GetTic();
    double trackingError;
    robReflexxes::ResultType trajectoryResult;

    // trigger search of lower limit
    if (!HomingCalibrateRollSeekLower) {
        // disable joint limits on PID
        PID.SetCheckJointLimit(false);
        // compute joint goal position, we assume PID is on from previous state
        JointTrajectory.Goal.SetAll(0.0);
        const double currentRoll = JointGetDesired.Element(JNT_WRIST_ROLL);
        JointTrajectory.Goal.Element(JNT_WRIST_ROLL) = currentRoll - maxRollRange;
        JointTrajectory.GoalVelocity.SetAll(0.0);
        JointTrajectory.EndTime = 0.0;
        // set flag to indicate that homing has started
        HomingCalibrateRollSeekLower = true;
        MessageEvents.Status(this->GetName() + " looking for roll lower limit");
        return;
    }

    // looking for lower limit has start but not found yet
    if (HomingCalibrateRollSeekLower
        && (HomingCalibrateRollLower == cmnTypeTraits<double>::MaxPositiveValue())) {
        JointTrajectory.Reflexxes.Evaluate(JointSet,
                                           JointVelocitySet,
                                           JointTrajectory.Goal,
                                           JointTrajectory.GoalVelocity);
        SetPositionJointLocal(JointSet);
        trajectoryResult = JointTrajectory.Reflexxes.ResultValue();
        switch (trajectoryResult) {
        case robReflexxes::Reflexxes_WORKING:
            // if this is the first evaluation, we can't calculate expected completion time
            if (JointTrajectory.EndTime == 0.0) {
                JointTrajectory.EndTime = currentTime + JointTrajectory.Reflexxes.Duration();
                HomingTimer = JointTrajectory.EndTime;
            }
            // detect tracking error and set lower limit
            trackingError = std::abs(JointGet.Element(JNT_WRIST_ROLL) - JointSet.Element(JNT_WRIST_ROLL));
            if (trackingError > maxTrackingError) {
                HomingCalibrateRollLower = JointGet.Element(JNT_WRIST_ROLL);
                MessageEvents.Status(this->GetName() + " found roll lower limit");
            } else {
                // time out
                if (currentTime > HomingTimer + extraTime) {
                    MessageEvents.Error(this->GetName() + " unable to hit roll lower limit in time");
                    this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
                }
            }
            break;
        case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
            // we shouldn't be able to reach this goal
            MessageEvents.Error(this->GetName() + " went past roll lower limit");
            this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
            break;
        default:
            MessageEvents.Error(this->GetName() + " error while evaluating trajectory.");
            break;
        }
        return;
    }

    // compute trajectory to go to center point
    if (!HomingCalibrateRollSeekCenter) {
        // compute joint goal position, we assume PID is on from previous state
        JointTrajectory.Goal.SetAll(0.0);
        JointTrajectory.Goal.Element(JNT_WRIST_ROLL) = HomingCalibrateRollLower + 480.0 * cmnPI_180;
        JointTrajectory.GoalVelocity.SetAll(0.0);
        JointTrajectory.EndTime = 0.0;

        // we want to start from zero velocity since we hit the joint limit
        JointVelocitySet.SetAll(0.0);
        // set flag to indicate that homing has started
        HomingCalibrateRollSeekCenter = true;
        MessageEvents.Status(this->GetName() + " moving roll to center");
        return;
    }

    // going to center position and check we have arrived
    JointTrajectory.Reflexxes.Evaluate(JointSet,
                                       JointVelocitySet,
                                       JointTrajectory.Goal,
                                       JointTrajectory.GoalVelocity);
    SetPositionJointLocal(JointSet);
    trajectoryResult = JointTrajectory.Reflexxes.ResultValue();
    bool isHomed;
    switch (trajectoryResult) {
    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (JointTrajectory.EndTime == 0.0) {
            JointTrajectory.EndTime = currentTime + JointTrajectory.Reflexxes.Duration();
            HomingTimer = JointTrajectory.EndTime;
        }
        break;
    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        // check position
        JointTrajectory.GoalError.DifferenceOf(JointTrajectory.Goal, JointGet);
        JointTrajectory.GoalError.AbsSelf();
        isHomed = !JointTrajectory.GoalError.ElementwiseGreaterOrEqual(JointTrajectory.GoalTolerance).Any();
        if (isHomed) {
            // reset encoder on last joint as well as PID target position to reflect new roll position = 0
            RobotIO.ResetSingleEncoder(static_cast<int>(JNT_WRIST_ROLL));
            JointSet.SetAll(0.0);
            SetPositionJointLocal(JointSet);
            PID.SetCheckJointLimit(true);
            MessageEvents.Status(this->GetName() + " roll calibrated");
            HomedOnce = true;
            this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_READY);
        } else {
            // time out
            if (currentTime > HomingTimer + extraTime) {
                CMN_LOG_CLASS_INIT_WARNING << GetName() << ": RunHomingCalibrateRoll: unable to reach home position, error in degrees is "
                                           << JointTrajectory.GoalError * (180.0 / cmnPI) << std::endl;
                MessageEvents.Error(this->GetName() + " unable to reach home position");
                this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
            }
        }
        break;
    default:
        MessageEvents.Error(this->GetName() + " error while evaluating trajectory.");
        break;
    }
}

void mtsIntuitiveResearchKitMTM::RunEffortOrientationLocked(void)
{
    // don't get current joint values!
    // always initialize IK from position when locked
    vctDoubleVec jointSet(EffortOrientationJoint.Ref(NumberOfJointsKinematics()));
    // compute desired position from current position and locked orientation
    CartesianPositionFrm.Translation().Assign(CartesianGetLocal.Translation());
    CartesianPositionFrm.Rotation().From(EffortOrientation);
    if (Manipulator.InverseKinematics(jointSet, CartesianPositionFrm) == robManipulator::ESUCCESS) {
        // find closest solution mod 2 pi
        const double difference = JointGet[6] - jointSet[6];
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        jointSet[6] = jointSet[6] + differenceInTurns * 2.0 * cmnPI;

        // assign to joints used for kinematics
        JointSet.Ref(NumberOfJointsKinematics()).Assign(jointSet);
        // finally send new joint values
        SetPositionJointLocal(JointSet);
    } else {
        MessageEvents.Warning(this->GetName() + " unable to solve inverse kinematics.");
    }
}

void mtsIntuitiveResearchKitMTM::RunEffortCartesianImpedance(void)
{
    prmForceCartesianSet wrench;
    mImpedanceController->Update(CartesianGetParam, CartesianVelocityGetParam, wrench);
    SetWrenchBody(wrench);
    RunEffortCartesian();
}

void mtsIntuitiveResearchKitMTM::LockOrientation(const vctMatRot3 & orientation)
{
    // if we just started lock
    if (!EffortOrientationLocked) {
        vctBoolVec torqueMode(8);
        // first 3 joints in torque mode
        torqueMode.Ref(3, 0).SetAll(true);
        // last 4 in PID mode
        torqueMode.Ref(4, 3).SetAll(false);
        PID.EnableTorqueMode(torqueMode);
        EffortOrientationLocked = true;
    }
    // in any case, update desired orientation
    EffortOrientation.Assign(orientation);
    EffortOrientationJoint.Assign(JointGet);
}

void mtsIntuitiveResearchKitMTM::UnlockOrientation(void)
{
    // only unlock if needed
    if (EffortOrientationLocked) {
        vctBoolVec torqueMode(8);
        torqueMode.SetAll(true);
        PID.EnableTorqueMode(torqueMode);
        EffortOrientationLocked = false;
    }
}

void mtsIntuitiveResearchKitMTM::SetRobotControlState(const std::string & state)
{
    if (state == "Home") {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_BIAS_ENCODER);
    } else if (state == "Cartesian position") {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN);
    } else if (state == "Teleop") {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN);
    } else {
        mtsIntuitiveResearchKitArmTypes::RobotStateType stateEnum;
        try {
            stateEnum = mtsIntuitiveResearchKitArmTypes::RobotStateTypeFromString(state);
        } catch (std::exception e) {
            MessageEvents.Error(this->GetName() + ": MTM unsupported state " + state + ": " + e.what());
            return;
        }
        SetState(stateEnum);
    }
}

void mtsIntuitiveResearchKitMTM::SetImpedanceGains(const prmFixtureGainCartesianSet &newGains)
{
    if(CurrentStateIs(mtsIntuitiveResearchKitArmTypes::DVRK_EFFORT_CARTESIAN_IMPEDANCE)) {
        mImpedanceController->SetGains(newGains);
    }
}
