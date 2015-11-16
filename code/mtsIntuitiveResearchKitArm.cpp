/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>
#include <time.h>

// cisst
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArm.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitArm, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsIntuitiveResearchKitArm::mtsIntuitiveResearchKitArm(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
}

mtsIntuitiveResearchKitArm::mtsIntuitiveResearchKitArm(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg)
{
}

void mtsIntuitiveResearchKitArm::Init(void)
{
    mCounter = 0;

    IsGoalSet = false;
    SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);

    // initialize trajectory data
    JointGet.SetSize(NumberOfJoints());
    JointVelocityGet.SetSize(NumberOfJoints());
    JointSet.SetSize(NumberOfJoints());
    JointSetParam.Goal().SetSize(NumberOfAxes());
    JointTrajectory.Velocity.SetSize(NumberOfJoints());
    JointTrajectory.Acceleration.SetSize(NumberOfJoints());
    JointTrajectory.Start.SetSize(NumberOfJoints());
    JointTrajectory.Goal.SetSize(NumberOfJoints());
    JointTrajectory.GoalError.SetSize(NumberOfJoints());
    JointTrajectory.GoalTolerance.SetSize(NumberOfJoints());
    JointTrajectory.EndTime = 0.0;
    PotsToEncodersTolerance.SetSize(NumberOfAxes());

    // initialize velocity
    CartesianVelocityGetParam.SetVelocityLinear(vct3(0.0));
    CartesianVelocityGetParam.SetVelocityAngular(vct3(0.0));
    CartesianVelocityGetParam.SetTimestamp(0.0);
    CartesianVelocityGetParam.SetValid(false);

    // base frame, mostly for cases where no base frame is set by user
    BaseFrame = vctFrm4x4::Identity();
    BaseFrameValid = true;

    // cartesian position are timestamped using timestamps provided by PID
    CartesianGetParam.SetAutomaticTimestamp(false);
    CartesianGetDesiredParam.SetAutomaticTimestamp(false);
    this->StateTable.AddData(CartesianGetLocalParam, "CartesianPositionLocal");
    this->StateTable.AddData(CartesianGetLocalDesiredParam, "CartesianPositionLocalDesired");
    this->StateTable.AddData(CartesianGetParam, "CartesianPosition");
    this->StateTable.AddData(CartesianGetDesiredParam, "CartesianPositionDesired");
    this->StateTable.AddData(BaseFrame, "BaseFrame");
    this->StateTable.AddData(JointGetParam, "JointPosition");
    this->StateTable.AddData(JointGetDesired, "JointPositionDesired");
    this->StateTable.AddData(CartesianVelocityGetParam, "CartesianVelocityGetParam");
    this->StateTable.AddData(StateJointParam, "StateJoint");
    this->StateTable.AddData(StateJointDesiredParam, "StateJointDesired");

    // PID
    PIDInterface = AddInterfaceRequired("PID");
    if (PIDInterface) {
        PIDInterface->AddFunction("Enable", PID.Enable);
        PIDInterface->AddFunction("EnableJoints", PID.EnableJoints);
        PIDInterface->AddFunction("GetPositionJoint", PID.GetPositionJoint);
        PIDInterface->AddFunction("GetPositionJointDesired", PID.GetPositionJointDesired);
        PIDInterface->AddFunction("GetStateJoint", PID.GetStateJoint);
        PIDInterface->AddFunction("GetStateJointDesired", PID.GetStateJointDesired);
        PIDInterface->AddFunction("SetPositionJoint", PID.SetPositionJoint);
        PIDInterface->AddFunction("SetCheckJointLimit", PID.SetCheckJointLimit);
        PIDInterface->AddFunction("SetJointLowerLimit", PID.SetJointLowerLimit);
        PIDInterface->AddFunction("SetJointUpperLimit", PID.SetJointUpperLimit);
        PIDInterface->AddFunction("GetVelocityJoint", PID.GetVelocityJoint);
        PIDInterface->AddFunction("EnableTorqueMode", PID.EnableTorqueMode);
        PIDInterface->AddFunction("SetTorqueJoint", PID.SetTorqueJoint);
        PIDInterface->AddFunction("SetTorqueOffset", PID.SetTorqueOffset);
        PIDInterface->AddFunction("EnableTrackingError", PID.EnableTrackingError);
        PIDInterface->AddFunction("SetTrackingErrorTolerances", PID.SetTrackingErrorTolerance);
        PIDInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::JointLimitEventHandler, this, "JointLimit");
        PIDInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::ErrorEventHandler, this, "Error");
    }

    // Robot IO
    IOInterface = AddInterfaceRequired("RobotIO");
    if (IOInterface) {
        IOInterface->AddFunction("GetSerialNumber", RobotIO.GetSerialNumber);
        IOInterface->AddFunction("SetCoupling", RobotIO.SetCoupling);
        IOInterface->AddFunction("EnablePower", RobotIO.EnablePower);
        IOInterface->AddFunction("DisablePower", RobotIO.DisablePower);
        IOInterface->AddFunction("GetActuatorAmpStatus", RobotIO.GetActuatorAmpStatus);
        IOInterface->AddFunction("GetBrakeAmpStatus", RobotIO.GetBrakeAmpStatus);
        IOInterface->AddFunction("BiasEncoder", RobotIO.BiasEncoder);
        IOInterface->AddFunction("ResetSingleEncoder", RobotIO.ResetSingleEncoder);
        IOInterface->AddFunction("GetAnalogInputPosSI", RobotIO.GetAnalogInputPosSI);
        IOInterface->AddFunction("SetActuatorCurrent", RobotIO.SetActuatorCurrent);
        IOInterface->AddFunction("UsePotsForSafetyCheck", RobotIO.UsePotsForSafetyCheck);
        IOInterface->AddFunction("SetPotsToEncodersTolerance", RobotIO.SetPotsToEncodersTolerance);
        IOInterface->AddFunction("BrakeRelease", RobotIO.BrakeRelease);
        IOInterface->AddFunction("BrakeEngage", RobotIO.BrakeEngage);
    }

    // Setup Joints
    SUJInterface = AddInterfaceRequired("BaseFrame", MTS_OPTIONAL);
    if (SUJInterface) {
        SUJInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::SetBaseFrame, this, "BaseFrameDesired");
        SUJInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::ErrorEventHandler, this, "Error");
    }

    // Arm
    RobotInterface = AddInterfaceProvided("Robot");
    if (RobotInterface) {
        // Get
        RobotInterface->AddCommandReadState(this->StateTable, JointGetParam, "GetPositionJoint");
        RobotInterface->AddCommandReadState(this->StateTable, JointGetDesired, "GetPositionJointDesired");
        RobotInterface->AddCommandReadState(this->StateTable, StateJointParam, "GetStateJoint");
        RobotInterface->AddCommandReadState(this->StateTable, StateJointDesiredParam, "GetStateJointDesired");
        RobotInterface->AddCommandReadState(this->StateTable, CartesianGetLocalParam, "GetPositionCartesianLocal");
        RobotInterface->AddCommandReadState(this->StateTable, CartesianGetLocalDesiredParam, "GetPositionCartesianLocalDesired");
        RobotInterface->AddCommandReadState(this->StateTable, CartesianGetParam, "GetPositionCartesian");
        RobotInterface->AddCommandReadState(this->StateTable, CartesianGetDesiredParam, "GetPositionCartesianDesired");
        RobotInterface->AddCommandReadState(this->StateTable, BaseFrame, "GetBaseFrame");
        RobotInterface->AddCommandReadState(this->StateTable, CartesianVelocityGetParam, "GetVelocityCartesian");
        // Set
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionJoint, this, "SetPositionJoint");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionGoalJoint, this, "SetPositionGoalJoint");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionCartesian, this, "SetPositionCartesian");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionGoalCartesian, this, "SetPositionGoalCartesian");
        // Trajectory events
        RobotInterface->AddEventWrite(JointTrajectory.GoalReachedEvent, "GoalReached", bool());
        // Robot State
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetRobotControlState,
                                        this, "SetRobotControlState", std::string(""));
        RobotInterface->AddCommandRead(&mtsIntuitiveResearchKitArm::GetRobotControlState,
                                       this, "GetRobotControlState", std::string(""));
        // Human readable messages
        RobotInterface->AddEventWrite(MessageEvents.Status, "Status", std::string(""));
        RobotInterface->AddEventWrite(MessageEvents.Warning, "Warning", std::string(""));
        RobotInterface->AddEventWrite(MessageEvents.Error, "Error", std::string(""));
        RobotInterface->AddEventWrite(MessageEvents.RobotState, "RobotState", std::string(""));

        // Stats
        RobotInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                            "GetPeriodStatistics");
    }
}

void mtsIntuitiveResearchKitArm::Configure(const std::string & filename)
{
    robManipulator::Errno result;
    result = this->Manipulator.LoadRobot(filename);
    if (result == robManipulator::EFAILURE) {
        CMN_LOG_CLASS_INIT_ERROR << GetName() << ": Configure: failed to load manipulator configuration file \""
                                 << filename << "\"" << std::endl;
    }
}

void mtsIntuitiveResearchKitArm::Startup(void)
{
    this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
}

void mtsIntuitiveResearchKitArm::Run(void)
{
    ProcessQueuedEvents();
    GetRobotData();

    switch (RobotState) {
    case mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED:
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_POWERING:
        RunHomingPower();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_CALIBRATING_ARM:
        RunHomingCalibrateArm();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_ARM_CALIBRATED:
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_READY:
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_JOINT:
        RunPositionJoint();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_JOINT:
        RunPositionGoalJoint();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN:
        RunPositionCartesian();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN:
        RunPositionGoalCartesian();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_MANUAL:
        break;
    default:
        RunArmSpecific();
        break;
    }

    RunEvent();
    ProcessQueuedCommands();

    mCounter++;
    CartesianGetPrevious = CartesianGet;
}

void mtsIntuitiveResearchKitArm::Cleanup(void)
{
    if (NumberOfBrakes() > 0) {
        RobotIO.BrakeEngage();
    }
    CMN_LOG_CLASS_INIT_VERBOSE << GetName() << ": Cleanup" << std::endl;
}

void mtsIntuitiveResearchKitArm::GetRobotControlState(std::string & state) const
{
    state = mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(this->RobotState);
}

bool mtsIntuitiveResearchKitArm::CurrentStateIs(const mtsIntuitiveResearchKitArmTypes::RobotStateType & state)
{
    if (RobotState == state) {
        return true;
    }
    CMN_LOG_CLASS_RUN_WARNING << GetName() << ": Checking state: arm not in "
                              << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN)
                              << ", current state is " << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(RobotState) << std::endl;
    return false;
}

void mtsIntuitiveResearchKitArm::GetRobotData(void)
{
    // we can start reporting some joint values after the robot is powered
    if (this->RobotState > mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_POWERING) {
        mtsExecutionResult executionResult;
        executionResult = PID.GetPositionJoint(JointGetParam);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to GetJointPosition failed \""
                                    << executionResult << "\"" << std::endl;
        }
        // assign to a more convenient vctDoubleVec
        JointGet.Assign(JointGetParam.Position(), NumberOfJoints());

        // desired joints
        executionResult = PID.GetPositionJointDesired(JointGetDesired);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to GetJointPositionDesired failed \""
                                    << executionResult << "\"" << std::endl;
        }

        // joint velocity
        executionResult = PID.GetVelocityJoint(JointVelocityGetParam);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to GetVelocityJoint failed \""
                                    << executionResult << "\"" << std::endl;
        }
        JointVelocityGet.Assign(JointVelocityGetParam.Velocity(), NumberOfJoints());

        // joint state, not used internally but available to users
        executionResult = PID.GetStateJoint(StateJointParam);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to GetJointState failed \""
                                    << executionResult << "\"" << std::endl;
        }

        // desired joint state
        executionResult = PID.GetStateJointDesired(StateJointDesiredParam);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to GetJointStateDesired failed \""
                                    << executionResult << "\"" << std::endl;
        }

        // when the robot is ready, we can compute cartesian position
        if (this->RobotState >= mtsIntuitiveResearchKitArmTypes::DVRK_READY) {
            // update cartesian position
            CartesianGetLocal = Manipulator.ForwardKinematics(JointGet, NumberOfJointsKinematics());
            CartesianGet = BaseFrame * CartesianGetLocal;
            // normalize
            CartesianGetLocal.Rotation().NormalizedSelf();
            CartesianGet.Rotation().NormalizedSelf();
            // flags
            CartesianGetLocalParam.SetTimestamp(JointGetParam.Timestamp());
            CartesianGetLocalParam.SetValid(true);
            CartesianGetParam.SetTimestamp(JointGetParam.Timestamp());
            CartesianGetParam.SetValid(BaseFrameValid);

#if 0
            // FIXME: bug
            // update cartesian velocity (caveat, velocities are not updated)
            const double dt = CartesianGetParam.Timestamp() - CartesianGetPreviousParam.Timestamp();
            if (dt > 0.0) {
                vct3 linearVelocity;
                linearVelocity.DifferenceOf(CartesianGetParam.Position().Translation(),
                                            CartesianGetPreviousParam.Position().Translation());
                linearVelocity.Divide(dt);
                CartesianVelocityGetParam.SetVelocityLinear(linearVelocity);
                CartesianVelocityGetParam.SetVelocityAngular(vct3(0.0));
                CartesianVelocityGetParam.SetTimestamp(CartesianGetParam.Timestamp());
                CartesianVelocityGetParam.SetValid(true);
            } else {
                CartesianVelocityGetParam.SetValid(false);
            }
#else
            vct3 linearVelocity;
            linearVelocity = (CartesianGet.Translation() - CartesianGetPrevious.Translation()).Divide(StateTable.Period);
            CartesianVelocityGetParam.SetVelocityLinear(linearVelocity);
            CartesianVelocityGetParam.SetVelocityAngular(vct3(0.0));
            CartesianVelocityGetParam.SetTimestamp(CartesianGetParam.Timestamp());
            CartesianVelocityGetParam.SetValid(true);
#endif

            // update cartesian position desired based on joint desired
            CartesianGetLocalDesired = Manipulator.ForwardKinematics(JointGetDesired);
            CartesianGetDesired = BaseFrame * CartesianGetLocalDesired;
            // normalize
            CartesianGetLocalDesired.Rotation().NormalizedSelf();
            CartesianGetDesired.Rotation().NormalizedSelf();
            // flags
            CartesianGetLocalDesiredParam.SetTimestamp(JointGetParam.Timestamp());
            CartesianGetLocalDesiredParam.SetValid(true);
            CartesianGetDesiredParam.SetTimestamp(JointGetParam.Timestamp());
            CartesianGetDesiredParam.SetValid(BaseFrameValid);
        } else {
            // update cartesian position
            CartesianGetLocal.Assign(vctFrm4x4::Identity());
            CartesianGet.Assign(vctFrm4x4::Identity());
            CartesianGetLocalParam.SetValid(false);
            CartesianGetParam.SetValid(false);
            // update cartesian position desired
            CartesianGetLocalDesired.Assign(vctFrm4x4::Identity());
            CartesianGetDesired.Assign(vctFrm4x4::Identity());
            CartesianGetLocalDesiredParam.SetValid(false);
            CartesianGetDesiredParam.SetValid(false);
        }
        CartesianGetLocalParam.Position().From(CartesianGetLocal);
        CartesianGetParam.Position().From(CartesianGet);
        CartesianGetLocalDesiredParam.Position().From(CartesianGetLocalDesired);
        CartesianGetDesiredParam.Position().From(CartesianGetDesired);
    } else {
        // set joint to zeros
        JointGet.Zeros();
        PID.GetPositionJoint(JointGetParam); // get size right
        JointGetParam.Position().Zeros();
        JointGetParam.SetValid(false);
        JointVelocityGet.Zeros();
        JointVelocityGetParam.Velocity().Zeros();
        JointVelocityGetParam.SetValid(false);
    }
}

void mtsIntuitiveResearchKitArm::RunHomingPower(void)
{
    const double timeToPower = 3.0 * cmn_s;

    const double currentTime = this->StateTable.GetTic();
    // first, request power to be turned on
    if (!HomingPowerRequested) {
        // in case we still have power but brakes are not engaged
        if (NumberOfBrakes() > 0) {
            RobotIO.BrakeEngage();
        }
        // bias encoders based on pots
        RobotIO.BiasEncoder(1000); // 1000 samples
        // use pots for redundancy
        if (UsePotsForSafetyCheck()) {
            RobotIO.SetPotsToEncodersTolerance(PotsToEncodersTolerance);
            RobotIO.UsePotsForSafetyCheck(true);
        }
        HomingTimer = currentTime;
        // make sure the PID is not sending currents
        PID.Enable(false);
        // pre-load the boards with zero current
        RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfAxes(), 0.0));
        // enable power and set a flags to move to next step
        RobotIO.EnablePower();
        HomingPowerRequested = true;
        MessageEvents.Status(this->GetName() + " power requested");
        return;
    }

    // second, check status
    if (HomingPowerRequested
        && ((currentTime - HomingTimer) > timeToPower)) {

        // pre-load PID to make sure desired position has some reasonable values
        PID.GetPositionJoint(JointGetParam);
        // assign to a more convenient vctDoubleVec
        JointGet.Assign(JointGetParam.Position(), NumberOfJoints());
        SetPositionJointLocal(JointGet);

        // check power status
        vctBoolVec actuatorAmplifiersStatus(NumberOfJoints());
        RobotIO.GetActuatorAmpStatus(actuatorAmplifiersStatus);
        vctBoolVec brakeAmplifiersStatus(NumberOfBrakes());
        if (NumberOfBrakes() > 0) {
            RobotIO.GetBrakeAmpStatus(brakeAmplifiersStatus);
        }
        if (actuatorAmplifiersStatus.All() && brakeAmplifiersStatus.All()) {
            MessageEvents.Status(this->GetName() + " power on");
            if (NumberOfBrakes() > 0) {
                RobotIO.BrakeRelease();
            }
            this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_CALIBRATING_ARM);
        } else {
            // make sure the PID is not sending currents
            PID.Enable(false);
            RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfAxes(), 0.0));
            RobotIO.DisablePower();
            MessageEvents.Error(this->GetName() + " failed to enable power.");
            this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
        }
    }
}

void mtsIntuitiveResearchKitArm::RunPositionJoint(void)
{
    if (IsGoalSet) {
        SetPositionJointLocal(JointSet);
    }
}

void mtsIntuitiveResearchKitArm::RunPositionGoalJoint(void)
{
    const double currentTime = this->StateTable.GetTic();
    if (currentTime <= JointTrajectory.EndTime) {
        JointTrajectory.LSPB.Evaluate(currentTime, JointSet);
        SetPositionJointLocal(JointSet);
    } else {
        if (JointTrajectory.EndTime != 0.0) {
            JointTrajectory.GoalReachedEvent(true);
            JointTrajectory.EndTime = 0.0;
        }
    }
}

void mtsIntuitiveResearchKitArm::RunPositionCartesian(void)
{
    if (IsGoalSet) {
        // copy current position
        vctDoubleVec jointSet(JointGet.Ref(NumberOfJointsKinematics()));

        // compute desired slave position
        CartesianPositionFrm.From(CartesianSetParam.Goal());
        if (this->InverseKinematics(jointSet, BaseFrame.Inverse() * CartesianPositionFrm) == robManipulator::ESUCCESS) {
            // assign to joints used for kinematics
            JointSet.Ref(NumberOfJointsKinematics()).Assign(jointSet);
            // finally send new joint values
            SetPositionJointLocal(JointSet);
        } else {
            MessageEvents.Error(this->GetName() + " unable to solve inverse kinematics.");
        }
        // reset flag
        IsGoalSet = false;
    }
}

void mtsIntuitiveResearchKitArm::RunPositionGoalCartesian(void)
{
    // trajectory are computed in joint space for now
    RunPositionGoalJoint();
}

void mtsIntuitiveResearchKitArm::SetPositionJointLocal(const vctDoubleVec & newPosition)
{
    JointSetParam.Goal().Zeros();
    JointSetParam.Goal().Assign(newPosition, NumberOfJoints());
    PID.SetPositionJoint(JointSetParam);
}

void mtsIntuitiveResearchKitArm::SetPositionJoint(const prmPositionJointSet & newPosition)
{
    if (CurrentStateIs(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_JOINT)) {
        JointSet.Assign(newPosition.Goal(), NumberOfJoints());
        IsGoalSet = true;
    }
}

void mtsIntuitiveResearchKitArm::SetPositionGoalJoint(const prmPositionJointSet & newPosition)
{
    if (CurrentStateIs(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_JOINT)) {
        const double currentTime = this->StateTable.GetTic();
        // starting point is last requested to PID component
        JointTrajectory.Start.Assign(JointGetDesired, NumberOfJoints());
        JointTrajectory.Goal.Assign(newPosition.Goal(), NumberOfJoints());
        JointTrajectory.LSPB.Set(JointTrajectory.Start, JointTrajectory.Goal,
                                 JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                 currentTime, robLSPB::LSPB_DURATION);
        JointTrajectory.EndTime = currentTime + JointTrajectory.LSPB.Duration();
    }
}

void mtsIntuitiveResearchKitArm::SetPositionCartesian(const prmPositionCartesianSet & newPosition)
{
    if ((RobotState == mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN)
        || (RobotState == mtsIntuitiveResearchKitArmTypes::DVRK_CONSTRAINT_CONTROLLER_CARTESIAN)) {
        CartesianSetParam = newPosition;
        IsGoalSet = true;
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": SetPositionCartesian: Arm not ready" << std::endl;
    }
}

void mtsIntuitiveResearchKitArm::SetPositionGoalCartesian(const prmPositionCartesianSet & newPosition)
{
    if (CurrentStateIs(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN)) {

        const size_t nbJoints = this->NumberOfJoints();
        const size_t nbJointsKin = this->NumberOfJointsKinematics();
        const double currentTime = this->StateTable.GetTic();

        // copy current position
        vctDoubleVec jointSet(JointGet.Ref(nbJointsKin));

        // compute desired slave position
        CartesianPositionFrm.From(newPosition.Goal());

        if (this->InverseKinematics(jointSet, BaseFrame.Inverse() * CartesianPositionFrm) == robManipulator::ESUCCESS) {
            // resize to proper number of joints
            jointSet.resize(nbJoints);

            // keep values of joints not handled by kinematics (PSM jaw)
            if (nbJoints > nbJointsKin) {
                // check if there is a trajectory active
                if (currentTime >= JointTrajectory.EndTime) {
                    // past any trajectory, use last desired joint position
                    jointSet.Ref(nbJoints - nbJointsKin,
                                 nbJointsKin).Assign(JointGetDesired.Ref(nbJoints - nbJointsKin,
                                                                         nbJointsKin));
                } else {
                    // there is an ongoing trajectory, use the trajectory goal
                    jointSet.Ref(nbJoints - nbJointsKin,
                                 nbJointsKin).Assign(JointTrajectory.Goal.Ref(nbJoints - nbJointsKin,
                                                                              nbJointsKin));
                }
            }
            const double currentTime = this->StateTable.GetTic();
            // starting point is last requested to PID component
            JointTrajectory.Start.Assign(JointGetDesired, NumberOfJoints());
            JointTrajectory.Goal.Assign(jointSet);
            JointTrajectory.LSPB.Set(JointTrajectory.Start, JointTrajectory.Goal,
                                     JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                     currentTime, robLSPB::LSPB_DURATION);
            JointTrajectory.EndTime = currentTime + JointTrajectory.LSPB.Duration();
        } else {
            MessageEvents.Error(this->GetName() + " unable to solve inverse kinematics.");
            JointTrajectory.GoalReachedEvent(false);
        }
    }
}

void mtsIntuitiveResearchKitArm::SetBaseFrame(const prmPositionCartesianGet & newBaseFrame)
{
    if (newBaseFrame.Valid()) {
        this->BaseFrame.FromNormalized(newBaseFrame.Position());
        this->BaseFrameValid = true;
    } else {
        this->BaseFrameValid = false;
    }
}

void mtsIntuitiveResearchKitArm::ErrorEventHandler(const std::string & message)
{
    RobotIO.DisablePower();
    // in case there was a trajectory going on
    if (JointTrajectory.EndTime != 0.0) {
        JointTrajectory.GoalReachedEvent(false);
        JointTrajectory.EndTime = 0.0;
    }
    MessageEvents.Error(this->GetName() + ": received [" + message + "]");
    SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
}

void mtsIntuitiveResearchKitArm::JointLimitEventHandler(const vctBoolVec & flags)
{
    // in case there was a trajectory going on
    if (JointTrajectory.EndTime != 0.0) {
        JointTrajectory.GoalReachedEvent(false);
        JointTrajectory.EndTime = 0.0;
    }
    MessageEvents.Warning(this->GetName() + ": PID joint limit");
}
