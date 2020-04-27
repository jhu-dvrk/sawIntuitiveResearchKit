/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen, Rishibrata Biswas, Adnan Munawar
  Created on: 2013-05-15

  (C) Copyright 2013-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <cmath>
#include <iostream>
#include <algorithm>
#include <numeric>

// cisst
#include <cisstCommon/cmnPath.h>
#include <cisstNumerical/nmrLSMinNorm.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmMaskedVector.h>

#include <sawIntuitiveResearchKit/robManipulatorMTM.h>
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


void mtsIntuitiveResearchKitMTM::Init(void)
{
    mtsIntuitiveResearchKitArm::Init();

    // state machine specific to MTM, see base class for other states
    mArmState.AddState("CALIBRATING_ROLL");
    mArmState.AddState("ROLL_CALIBRATED");
    mArmState.AddState("HOMING_ROLL");
    mArmState.AddState("RESETTING_ROLL_ENCODER");
    mArmState.AddState("ROLL_ENCODER_RESET");

    // after arm homed
    mArmState.SetTransitionCallback("ENCODERS_BIASED",
                                    &mtsIntuitiveResearchKitMTM::TransitionEncodersBiased,
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
    mArmState.SetEnterCallback("RESETTING_ROLL_ENCODER",
                               &mtsIntuitiveResearchKitMTM::EnterResettingRollEncoder,
                               this);
    mArmState.SetRunCallback("RESETTING_ROLL_ENCODER",
                             &mtsIntuitiveResearchKitMTM::RunResettingRollEncoder,
                             this);
    mArmState.SetTransitionCallback("ROLL_ENCODER_RESET",
                                    &mtsIntuitiveResearchKitMTM::TransitionRollEncoderReset,
                                    this);

    // joint values when orientation is locked
    mEffortOrientationJoint.SetSize(NumberOfJoints());

    // initialize gripper state
    StateGripper.Name().SetSize(1);
    StateGripper.Name().at(0) = "gripper";
    StateGripper.Position().SetSize(1);

    ConfigurationGripper.Name().SetSize(1);
    ConfigurationGripper.Name().at(0) = "gripper";
    ConfigurationGripper.Type().SetSize(1);
    ConfigurationGripper.Type().at(0) = PRM_JOINT_REVOLUTE;
    ConfigurationGripper.PositionMin().SetSize(1);
    ConfigurationGripper.PositionMin().at(0) = 0.0 * cmnPI_180;
    ConfigurationGripper.PositionMax().SetSize(1);
    ConfigurationGripper.PositionMax().at(0) = 60.0 * cmnPI_180; // based on dVRK MTM gripper calibration procedure

    GripperClosed = false;

    // initialize trajectory data
    mJointTrajectory.VelocityMaximum.SetAll(90.0 * cmnPI_180); // degrees per second
    mJointTrajectory.VelocityMaximum.at(JNT_WRIST_ROLL) = 360.0 * cmnPI_180; // roll can go fast
    SetJointVelocityRatio(1.0);
    mJointTrajectory.AccelerationMaximum.SetAll(90.0 * cmnPI_180);
    mJointTrajectory.AccelerationMaximum.at(JNT_WRIST_ROLL) = 360.0 * cmnPI_180;
    SetJointAccelerationRatio(1.0);
    mJointTrajectory.GoalTolerance.SetAll(3.0 * cmnPI_180); // hard coded to 3 degrees
    mJointTrajectory.GoalTolerance.at(JNT_WRIST_ROLL) = 6.0 * cmnPI_180; // roll has low encoder resolution

    // default PID tracking errors, defaults are used for homing
    PID.DefaultTrackingErrorTolerance.SetSize(NumberOfJoints());
    PID.DefaultTrackingErrorTolerance.SetAll(10.0 * cmnPI_180);
    // last 3 joints tend to be weaker
    PID.DefaultTrackingErrorTolerance.Ref(3, 4) = 30.0 * cmnPI_180;

    this->StateTable.AddData(StateGripper, "StateGripper");

    // Main interface should have been created by base class init
    CMN_ASSERT(RobotInterface);
    RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitMTM::LockOrientation, this, "LockOrientation");
    RobotInterface->AddCommandVoid(&mtsIntuitiveResearchKitMTM::UnlockOrientation, this, "UnlockOrientation");

    // Gripper
    RobotInterface->AddCommandReadState(this->StateTable, StateGripper, "GetStateGripper");
    RobotInterface->AddEventVoid(GripperEvents.GripperPinch, "GripperPinchEvent");
    RobotInterface->AddEventWrite(GripperEvents.GripperClosed, "GripperClosedEvent", true);
}

void mtsIntuitiveResearchKitMTM::PreConfigure(const Json::Value & jsonConfig,
                                              const cmnPath & configPath,
                                              const std::string & filename)
{
    // gravity compensation
    const auto jsonGC = jsonConfig["gravity-compensation"];
    if (!jsonGC.isNull()) {
        const auto fileGC = configPath.Find(jsonGC.asString());
        if (fileGC == "") {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: " << this->GetName()
                                     << " can't find gravity-compensation file \""
                                     << jsonGC.asString() << "\" defined in \""
                                     << filename << "\"" << std::endl;
            exit(EXIT_FAILURE);
        } else {
            ConfigureGC(fileGC);
        }
    }

    // which IK to use
    const auto jsonKinematic = jsonConfig["kinematic-type"];
    if (!jsonKinematic.isNull()) {
        const auto kinematicType = jsonKinematic.asString();
        const std::list<std::string> options {"ITERATIVE", "CLOSED"};
        if (std::find(options.begin(), options.end(), kinematicType) != options.end()) {
            if (kinematicType == "ITERATIVE") {
                mKinematicType = MTM_ITERATIVE;
            } else if (kinematicType == "CLOSED") {
                mKinematicType = MTM_CLOSED;
            }
            CreateManipulator();
        } else {
            const std::string allOptions = std::accumulate(options.begin(),
                                                           options.end(),
                                                           std::string{},
                                                           [](const std::string & a, const std::string & b) {
                                                               return a.empty() ? b : a + ", " + b; });
            CMN_LOG_CLASS_INIT_ERROR << "Configure: " << this->GetName()
                                     << " kinematic-type \"" << kinematicType << "\" is not valid.  Valid options are: "
                                     << allOptions << std::endl;
            exit(EXIT_FAILURE);
        }
    }
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
                                     << ": failed to parse gravity compensation (GC) configuration file \""
                                     << filename << "\"\n"
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
            if (!result.ErrorMessage.empty()) {
                CMN_LOG_CLASS_INIT_WARNING << "ConfigureGC " << this->GetName()
                                           << ": robGravityCompensationMTM created from file \""
                                           << filename << "\" warns " << result.ErrorMessage << std::endl;
            }
        }
    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureGC " << this->GetName() << ": make sure the file \""
                                 << filename << "\" is in JSON format" << std::endl;
    }
}

robManipulator::Errno mtsIntuitiveResearchKitMTM::InverseKinematics(vctDoubleVec & jointSet,
                                                                    const vctFrm4x4 & cartesianGoal)
{
    if (mKinematicType == MTM_ITERATIVE) {
        // projection of roll axis on platform tells us how the platform
        // should move.  the projection angle is +/- q5 based on q4.  we
        // also scale the increment based on cos(q[4]) so increment is
        // null if roll axis is perpendicular to platform
        jointSet[3] += jointSet[5] * cos(jointSet[4]);

        // make sure we respect joint limits
        const double q3Max = Manipulator->links[3].GetKinematics()->PositionMax();
        const double q3Min = Manipulator->links[3].GetKinematics()->PositionMin();
        if (jointSet[3] > q3Max) {
            jointSet[3] = q3Max;
        } else if (jointSet[3] < q3Min) {
            jointSet[3] = q3Min;
        }
    }

    if (Manipulator->InverseKinematics(jointSet, cartesianGoal) == robManipulator::ESUCCESS) {
        // find closest solution mod 2 pi
        const double difference = m_measured_js_kin.Position()[JNT_WRIST_ROLL] - jointSet[JNT_WRIST_ROLL];
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        jointSet[JNT_WRIST_ROLL] = jointSet[JNT_WRIST_ROLL] + differenceInTurns * 2.0 * cmnPI;
        return robManipulator::ESUCCESS;
    }
    return robManipulator::EFAILURE;
}

void mtsIntuitiveResearchKitMTM::CreateManipulator(void)
{
    if (Manipulator) {
        delete Manipulator;
    }

    if (mKinematicType == MTM_ITERATIVE) {
        Manipulator = new robManipulator();
    } else {
        Manipulator = new robManipulatorMTM();
    }
}

bool mtsIntuitiveResearchKitMTM::IsHomed(void) const
{
    return m_encoders_biased;
}

void mtsIntuitiveResearchKitMTM::UnHome(void)
{
    // to force re-bias on pots
    m_re_home = true;
    m_encoders_biased_from_pots = false;
    // to force roll bias
    m_encoders_biased = false;
}

void mtsIntuitiveResearchKitMTM::SetGoalHomingArm(void)
{
    // compute joint goal position
    mJointTrajectory.Goal.SetAll(0.0);
}

void mtsIntuitiveResearchKitMTM::TransitionEncodersBiased(void)
{
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState("CALIBRATING_ROLL");
    }
}

void mtsIntuitiveResearchKitMTM::EnterCalibratingRoll(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, true);

    if (m_simulated || IsHomed()) {
        return;
    }

    static const double maxTrackingError = 0.5 * cmnPI; // 1/4 turn
    static const double maxRollRange = 6.0 * cmnPI + maxTrackingError; // that actual device is limited to ~2.6 turns

    // compute joint goal position, we assume PID is on from previous state
    CMN_ASSERT(m_joint_ready);
    GetRobotData();
    mJointTrajectory.Goal.Assign(m_setpoint_js_pid.Position());
    const double currentRoll = m_setpoint_js_pid.Position().at(JNT_WRIST_ROLL);
    mJointTrajectory.Goal.at(JNT_WRIST_ROLL) = currentRoll - maxRollRange;
    mJointTrajectory.GoalVelocity.SetAll(0.0);
    mJointTrajectory.EndTime = 0.0;
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);

    // disable safety features so we can look for physical joint limit
    PID.SetCheckPositionLimit(false);
    PID.EnableTrackingError(false);
    // enable PID for roll only
    vctBoolVec enableJoints(NumberOfJoints());
    enableJoints.SetAll(false);
    enableJoints.at(JNT_WRIST_ROLL) = true;
    PID.EnableJoints(enableJoints);
    PID.Enable(true);

    RobotInterface->SendStatus(this->GetName() + ": looking for roll lower limit");
}

void mtsIntuitiveResearchKitMTM::RunCalibratingRoll(void)
{
    if (m_simulated || IsHomed()) {
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
        CMN_ASSERT(m_joint_ready);
        GetRobotData();
        trackingError = std::abs(m_measured_js_pid.Position().at(JNT_WRIST_ROLL) - JointSet.at(JNT_WRIST_ROLL));
        if (trackingError > maxTrackingError) {
            // disable PID
            PID.Enable(false);
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
        mArmState.SetCurrentState("RESETTING_ROLL_ENCODER");
    }
}

void mtsIntuitiveResearchKitMTM::EnterResettingRollEncoder(void)
{
    if (m_simulated || IsHomed()) {
        mArmState.SetCurrentState("HOMING");
        return;
    }

    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, true);

    // reset encoder on last joint as well as PID target position to reflect new roll position = 0
    prmMaskedDoubleVec values(NumberOfJoints());
    values.Mask().SetAll(false);
    values.Data().SetAll(0.0); // this shouldn't be used but safe to zero
    values.Mask().at(JNT_WRIST_ROLL) = true;
    values.Data().at(JNT_WRIST_ROLL) = -480.0 * cmnPI_180;
    RobotIO.SetSomeEncoderPosition(values);

    // start timer
    const double currentTime = this->StateTable.GetTic();
    mHomingTimer = currentTime;
}

void mtsIntuitiveResearchKitMTM::RunResettingRollEncoder(void)
{
    // wait for some time, no easy way to check if encoder has been reset
    const double timeToWait = 10.0 * cmn_ms;
    const double currentTime = this->StateTable.GetTic();
    if ((currentTime - mHomingTimer) < timeToWait) {
        return;
    }

    // check current roll position, it should be -480 degrees
    double positionError = std::abs(m_measured_js_pid.Position().at(JNT_WRIST_ROLL) - -480.0 * cmnPI_180);
    if (positionError > 5.0 * cmn180_PI) {
        RobotInterface->SendError(this->GetName() + ": roll encoder not properly reset to -480 degrees");
        this->SetDesiredState(mFallbackState);
    } else {
        RobotInterface->SendStatus(this->GetName() + ": roll encoder properly reset to -480 degrees");
    }

    // now all encoders are biased (IsHomed) for MTM
    m_encoders_biased = true;
    mArmState.SetCurrentState("ROLL_ENCODER_RESET");
}

void mtsIntuitiveResearchKitMTM::TransitionRollEncoderReset(void)
{
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState("HOMING");
    }
}

void mtsIntuitiveResearchKitMTM::GetRobotData(void)
{
    mtsIntuitiveResearchKitArm::GetRobotData();

    if (m_simulated) {
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
    StateGripper.Position()[0] = position;
    StateGripper.Timestamp() = m_measured_js_pid.Timestamp();
    StateGripper.Valid() = m_measured_js_pid.Valid();

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

void mtsIntuitiveResearchKitMTM::ControlEffortOrientationLocked(void)
{
    // don't get current joint values!
    // always initialize IK from position when locked
    vctDoubleVec jointSet(mEffortOrientationJoint);
    // compute desired position from current position and locked orientation
    CartesianPositionFrm.Translation().Assign(m_measured_cp_local_frame.Translation());
    CartesianPositionFrm.Rotation().From(mEffortOrientation);
    // important note, lock uses numerical IK as it finds a solution close to current position
    if (Manipulator->InverseKinematics(jointSet, CartesianPositionFrm) == robManipulator::ESUCCESS) {
        // find closest solution mod 2 pi
        const double difference = m_measured_js_pid.Position()[JNT_WRIST_ROLL] - jointSet[JNT_WRIST_ROLL];
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        jointSet[JNT_WRIST_ROLL] = jointSet[JNT_WRIST_ROLL] + differenceInTurns * 2.0 * cmnPI;
        // initialize trajectory
        mJointTrajectory.Goal.Ref(NumberOfJointsKinematics()).Assign(jointSet);
        mJointTrajectory.Reflexxes.Evaluate(JointSet,
                                            JointVelocitySet,
                                            mJointTrajectory.Goal,
                                            mJointTrajectory.GoalVelocity);
        mtsIntuitiveResearchKitArm::SetPositionJointLocal(JointSet);
    } else {
        RobotInterface->SendWarning(this->GetName() + ": unable to solve inverse kinematics in ControlEffortOrientationLocked");
    }
}

void mtsIntuitiveResearchKitMTM::SetControlEffortActiveJoints(void)
{
    vctBoolVec torqueMode(NumberOfJoints());
    // if orientation is locked
    if (m_effort_orientation_locked) {
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

void mtsIntuitiveResearchKitMTM::ControlEffortCartesianPreload(vctDoubleVec & effortPreload,
                                                               vctDoubleVec & wrenchPreload)
{
    if (mWrenchType == WRENCH_SPATIAL) {
        effortPreload.SetAll(0.0);
        wrenchPreload.SetAll(0.0);
        return;
    }
    // most efforts will be 0
    effortPreload.Zeros();

    // create a vector reference make code more readable
    vctDynamicConstVectorRef<double> q(m_measured_js_kin.Position());

    // projection of roll axis on platform tells us how the platform
    // should move.  the projection angle is +/- q5 based on q4.  we
    // also scale the increment based on cos(q[4]) so increment is
    // null if roll axis is perpendicular to platform
    double q3Increment = q[5] * cos(q[4]);

    // now make sure incremental is now too large, i.e. cap the rate
    const double q3MaxIncrement = cmnPI * 0.05; // this is a velocity
    if (q3Increment > q3MaxIncrement) {
        q3Increment = q3MaxIncrement;
    } else if (q3Increment < -q3MaxIncrement) {
        q3Increment = -q3MaxIncrement;
    }

    // set goal
    double q3Goal = q[3] + q3Increment;

    // make sure we respect joint limits
    const double q3Max = Manipulator->links[3].GetKinematics()->PositionMax();
    const double q3Min = Manipulator->links[3].GetKinematics()->PositionMin();
    if (q3Goal > q3Max) {
        q3Goal = q3Max;
    } else if (q3Goal < q3Min) {
        q3Goal = q3Min;
    }

    // apply a linear force on joint 3 to move toward the goal position
    effortPreload[3] = -0.4 * (m_measured_js_kin.Position()[3] - q3Goal)
        - 0.05 * m_measured_js_kin.Velocity()[3];

    // cap effort to be totally safe - this has to be the most non-linear behavior around
    effortPreload[3] = std::max(effortPreload[3], -0.1);
    effortPreload[3] = std::min(effortPreload[3],  0.1);

    // find equivalent wrench but don't apply all (too much torque on roll)
    // wrenchPreload.ProductOf(mJacobianPInverseData.PInverse(), effortPreload);
    // wrenchPreload.Multiply(0.2);
    wrenchPreload.SetAll(0.0);
}

void mtsIntuitiveResearchKitMTM::LockOrientation(const vctMatRot3 & orientation)
{
    // if we just started lock
    if (!m_effort_orientation_locked) {
        m_effort_orientation_locked = true;
        SetControlEffortActiveJoints();
        // initialize trajectory
        JointSet.Assign(m_measured_js_pid.Position(), NumberOfJoints());
        JointVelocitySet.Assign(m_measured_js_pid.Velocity(), NumberOfJoints());
        mJointTrajectory.Reflexxes.Set(mJointTrajectory.Velocity,
                                       mJointTrajectory.Acceleration,
                                       StateTable.PeriodStats.PeriodAvg(),
                                       robReflexxes::Reflexxes_TIME);
    }
    // in any case, update desired orientation in local coordinate system
    // mEffortOrientation.Assign(m_base_frame.Rotation().Inverse() * orientation);
    m_base_frame.Rotation().ApplyInverseTo(orientation, mEffortOrientation);
    mEffortOrientationJoint.Assign(m_measured_js_pid.Position());
}

void mtsIntuitiveResearchKitMTM::UnlockOrientation(void)
{
    // only unlock if needed
    if (m_effort_orientation_locked) {
        m_effort_orientation_locked = false;
        SetControlEffortActiveJoints();
    }
}


void mtsIntuitiveResearchKitMTM::AddGravityCompensationEfforts(vctDoubleVec & efforts)
{
    if (GravityCompensationMTM) {
        GravityCompensationMTM->AddGravityCompensationEfforts(m_measured_js_kin.Position(),
                                                              m_measured_js_kin.Velocity(),
                                                              efforts);
    }
}
