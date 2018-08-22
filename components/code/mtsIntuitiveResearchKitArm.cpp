/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen, Zerui Wang
  Created on: 2016-02-24

  (C) Copyright 2013-2018 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstNumerical/nmrIsOrthonormal.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArm.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitArm, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsIntuitiveResearchKitArm::mtsIntuitiveResearchKitArm(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    mArmState(componentName, "UNINITIALIZED"),
    mStateTableState(100, "State"),
    mControlCallback(0)
{
}

mtsIntuitiveResearchKitArm::mtsIntuitiveResearchKitArm(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    mArmState(arg.Name, "UNINITIALIZED"),
    mStateTableState(100, "State"),
    mControlCallback(0)
{
}

void mtsIntuitiveResearchKitArm::Init(void)
{
    // configure state machine common to all arms (ECM/MTM/PSM)
    // possible states
    mArmState.AddState("CALIBRATING_ENCODERS_FROM_POTS");
    mArmState.AddState("ENCODERS_BIASED");
    mArmState.AddState("POWERING");
    mArmState.AddState("POWERED");
    mArmState.AddState("HOMING_ARM");
    mArmState.AddState("ARM_HOMED");
    mArmState.AddState("READY");

    // possible desired states
    mArmState.AddAllowedDesiredState("UNINITIALIZED");
    mArmState.AddAllowedDesiredState("ENCODERS_BIASED");
    mArmState.AddAllowedDesiredState("POWERED");
    mArmState.AddAllowedDesiredState("READY");

    mFallbackState = "UNINITIALIZED";

    // state change, to convert to string events for users (Qt, ROS)
    mArmState.SetStateChangedCallback(&mtsIntuitiveResearchKitArm::StateChanged,
                                      this);

    // run for all states
    mArmState.SetRunCallback(&mtsIntuitiveResearchKitArm::RunAllStates,
                             this);

    // unitialized
    mArmState.SetEnterCallback("UNINITIALIZED",
                               &mtsIntuitiveResearchKitArm::EnterUninitialized,
                               this);

    mArmState.SetTransitionCallback("UNINITIALIZED",
                                    &mtsIntuitiveResearchKitArm::TransitionUninitialized,
                                    this);

    // bias encoders
    mArmState.SetEnterCallback("CALIBRATING_ENCODERS_FROM_POTS",
                               &mtsIntuitiveResearchKitArm::EnterCalibratingEncodersFromPots,
                               this);

    mArmState.SetTransitionCallback("CALIBRATING_ENCODERS_FROM_POTS",
                                    &mtsIntuitiveResearchKitArm::TransitionCalibratingEncodersFromPots,
                                    this);

    mArmState.SetTransitionCallback("ENCODERS_BIASED",
                                    &mtsIntuitiveResearchKitArm::TransitionEncodersBiased,
                                    this);

    // power
    mArmState.SetEnterCallback("POWERING",
                               &mtsIntuitiveResearchKitArm::EnterPowering,
                               this);

    mArmState.SetTransitionCallback("POWERING",
                                    &mtsIntuitiveResearchKitArm::TransitionPowering,
                                    this);

    mArmState.SetEnterCallback("POWERED",
                               &mtsIntuitiveResearchKitArm::EnterPowered,
                               this);

    mArmState.SetTransitionCallback("POWERED",
                                    &mtsIntuitiveResearchKitArm::TransitionPowered,
                                    this);

    // arm homing
    mArmState.SetEnterCallback("HOMING_ARM",
                               &mtsIntuitiveResearchKitArm::EnterHomingArm,
                               this);

    mArmState.SetRunCallback("HOMING_ARM",
                             &mtsIntuitiveResearchKitArm::RunHomingArm,
                             this);

    // state between ARM_HOMED and READY depends on the arm type, see
    // derived classes
    mArmState.SetEnterCallback("READY",
                               &mtsIntuitiveResearchKitArm::EnterReady,
                               this);

    mArmState.SetRunCallback("READY",
                             &mtsIntuitiveResearchKitArm::RunReady,
                             this);

    mArmState.SetLeaveCallback("READY",
                               &mtsIntuitiveResearchKitArm::LeaveReady,
                               this);

    // state table to maintain state :-)
    mStateTableState.AddData(mStateTableStateCurrent, "Current");
    mStateTableState.AddData(mStateTableStateDesired, "Desired");
    AddStateTable(&mStateTableState);
    mStateTableState.SetAutomaticAdvance(false);

    mCounter = 0;
    mControlSpace = mtsIntuitiveResearchKitArmTypes::UNDEFINED_SPACE;
    mControlMode = mtsIntuitiveResearchKitArmTypes::UNDEFINED_MODE;

    mJointControlReady = false;
    mCartesianControlReady = false;
    mIsSimulated = false;
    mEncoderBiased = false;
    mHomingGoesToZero = false; // MTM ignores this
    mHomingBiasEncoderRequested = false;

    mWrenchBodyOrientationAbsolute = false;
    mGravityCompensation = false;
    mCartesianImpedance = false;

    mHasNewPIDGoal = false;
    mJointRelative.SetSize(NumberOfJoints());
    mJointRelative.Zeros();

    mEffortOrientationLocked = false;

    mSafeForCartesianControlCounter = 0;
    mArmNotReadyCounter = 0;
    mArmNotReadyTimeLastMessage = 0.0;

    // initialize trajectory data
    JointSet.SetSize(NumberOfJoints());
    JointVelocitySet.SetSize(NumberOfJoints());
    JointSetParam.Goal().SetSize(NumberOfJoints());
    mJointTrajectory.Velocity.SetSize(NumberOfJoints());
    mJointTrajectory.Acceleration.SetSize(NumberOfJoints());
    mJointTrajectory.Goal.SetSize(NumberOfJoints());
    mJointTrajectory.GoalVelocity.SetSize(NumberOfJoints());
    mJointTrajectory.GoalError.SetSize(NumberOfJoints());
    mJointTrajectory.GoalTolerance.SetSize(NumberOfJoints());
    mJointTrajectory.IsWorking = false;

    // initialize velocity
    CartesianVelocityGetParam.SetVelocityLinear(vct3(0.0));
    CartesianVelocityGetParam.SetVelocityAngular(vct3(0.0));
    CartesianVelocityGetParam.SetValid(false);

    // manipulator
    Manipulator = new robManipulatorRMRC();

    // jacobian
    ResizeKinematicsData();
    this->StateTable.AddData(mJacobianBody, "JacobianBody");
    this->StateTable.AddData(mJacobianSpatial, "JacobianSpatial");

    // efforts for kinematics
    mEffortJointSet.SetSize(NumberOfJointsKinematics());
    mEffortJointSet.ForceTorque().SetAll(0.0);
    mWrenchGet.SetValid(false);

    // base frame, mostly for cases where no base frame is set by user
    BaseFrame = vctFrm4x4::Identity();
    BaseFrameValid = true;

    CartesianGetParam.SetAutomaticTimestamp(false); // based on PID timestamp
    CartesianGetParam.SetReferenceFrame(GetName() + "_base");
    CartesianGetParam.SetMovingFrame(GetName());
    this->StateTable.AddData(CartesianGetParam, "CartesianPosition");

    CartesianGetDesiredParam.SetAutomaticTimestamp(false); // based on PID timestamp
    CartesianGetDesiredParam.SetReferenceFrame(GetName() + "_base");
    CartesianGetDesiredParam.SetMovingFrame(GetName() + "_d");
    this->StateTable.AddData(CartesianGetDesiredParam, "CartesianPositionDesired");

    CartesianGetLocalParam.SetAutomaticTimestamp(false); // based on PID timestamp
    CartesianGetLocalParam.SetReferenceFrame(GetName() + "_base");
    CartesianGetLocalParam.SetMovingFrame(GetName());
    this->StateTable.AddData(CartesianGetLocalParam, "CartesianPositionLocal");

    CartesianGetLocalDesiredParam.SetAutomaticTimestamp(false); // based on PID timestamp
    CartesianGetLocalDesiredParam.SetReferenceFrame(GetName() + "_base");
    CartesianGetLocalDesiredParam.SetMovingFrame(GetName() + "_d");
    this->StateTable.AddData(CartesianGetLocalDesiredParam, "CartesianPositionLocalDesired");

    this->StateTable.AddData(BaseFrame, "BaseFrame");

    CartesianVelocityGetParam.SetAutomaticTimestamp(false); // keep PID timestamp
    CartesianVelocityGetParam.SetMovingFrame(GetName());
    CartesianVelocityGetParam.SetReferenceFrame(GetName() + "_base");
    this->StateTable.AddData(CartesianVelocityGetParam, "CartesianVelocityGetParam");

    mWrenchGet.SetAutomaticTimestamp(false); // keep PID timestamp
    this->StateTable.AddData(mWrenchGet, "WrenchGet");

    JointsKinematics.SetAutomaticTimestamp(false); // keep PID timestamp
    this->StateTable.AddData(JointsKinematics, "JointsKinematics");

    JointsDesiredKinematics.SetAutomaticTimestamp(false); // keep PID timestamp
    this->StateTable.AddData(JointsDesiredKinematics, "JointsDesiredKinematics");

    // PID
    PIDInterface = AddInterfaceRequired("PID");
    if (PIDInterface) {
        PIDInterface->AddFunction("SetCoupling", PID.SetCoupling);
        PIDInterface->AddFunction("Enable", PID.Enable);
        PIDInterface->AddFunction("EnableJoints", PID.EnableJoints);
        PIDInterface->AddFunction("Enabled", PID.Enabled);
        PIDInterface->AddFunction("GetStateJoint", PID.GetStateJoint);
        PIDInterface->AddFunction("GetStateJointDesired", PID.GetStateJointDesired);
        PIDInterface->AddFunction("SetPositionJoint", PID.SetPositionJoint);
        PIDInterface->AddFunction("SetCheckPositionLimit", PID.SetCheckPositionLimit);
        PIDInterface->AddFunction("SetPositionLowerLimit", PID.SetPositionLowerLimit);
        PIDInterface->AddFunction("SetPositionUpperLimit", PID.SetPositionUpperLimit);
        PIDInterface->AddFunction("SetTorqueLowerLimit", PID.SetTorqueLowerLimit);
        PIDInterface->AddFunction("SetTorqueUpperLimit", PID.SetTorqueUpperLimit);
        PIDInterface->AddFunction("EnableTorqueMode", PID.EnableTorqueMode);
        PIDInterface->AddFunction("SetTorqueJoint", PID.SetTorqueJoint);
        PIDInterface->AddFunction("SetTorqueOffset", PID.SetTorqueOffset);
        PIDInterface->AddFunction("EnableTrackingError", PID.EnableTrackingError);
        PIDInterface->AddFunction("SetTrackingErrorTolerances", PID.SetTrackingErrorTolerance);
        PIDInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::PositionLimitEventHandler, this, "PositionLimit");
        PIDInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::ErrorEventHandler, this, "Error");
    }

    // Robot IO
    IOInterface = AddInterfaceRequired("RobotIO");
    if (IOInterface) {
        IOInterface->AddFunction("GetSerialNumber", RobotIO.GetSerialNumber);
        IOInterface->AddFunction("EnablePower", RobotIO.EnablePower);
        IOInterface->AddFunction("DisablePower", RobotIO.DisablePower);
        IOInterface->AddFunction("GetActuatorAmpStatus", RobotIO.GetActuatorAmpStatus);
        IOInterface->AddFunction("GetBrakeAmpStatus", RobotIO.GetBrakeAmpStatus);
        IOInterface->AddFunction("BiasEncoder", RobotIO.BiasEncoder);
        IOInterface->AddFunction("ResetSingleEncoder", RobotIO.ResetSingleEncoder);
        IOInterface->AddFunction("GetAnalogInputPosSI", RobotIO.GetAnalogInputPosSI);
        IOInterface->AddFunction("SetActuatorCurrent", RobotIO.SetActuatorCurrent);
        IOInterface->AddFunction("UsePotsForSafetyCheck", RobotIO.UsePotsForSafetyCheck);
        IOInterface->AddFunction("BrakeRelease", RobotIO.BrakeRelease);
        IOInterface->AddFunction("BrakeEngage", RobotIO.BrakeEngage);
        IOInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::BiasEncoderEventHandler, this, "BiasEncoder");
    }

    // Arm
    RobotInterface = AddInterfaceProvided("Robot");
    if (RobotInterface) {
        RobotInterface->AddMessageEvents();

        // Get
        RobotInterface->AddCommandReadState(this->StateTable, JointsKinematics, "GetStateJoint");
        RobotInterface->AddCommandReadState(this->StateTable, JointsDesiredKinematics, "GetStateJointDesired");
        RobotInterface->AddCommandReadState(this->StateTable, CartesianGetLocalParam, "GetPositionCartesianLocal");
        RobotInterface->AddCommandReadState(this->StateTable, CartesianGetLocalDesiredParam, "GetPositionCartesianLocalDesired");
        RobotInterface->AddCommandReadState(this->StateTable, CartesianGetParam, "GetPositionCartesian");
        RobotInterface->AddCommandReadState(this->StateTable, CartesianGetDesiredParam, "GetPositionCartesianDesired");
        RobotInterface->AddCommandReadState(this->StateTable, BaseFrame, "GetBaseFrame");
        RobotInterface->AddCommandReadState(this->StateTable, CartesianVelocityGetParam, "GetVelocityCartesian");
        RobotInterface->AddCommandReadState(this->StateTable, mWrenchGet, "GetWrenchBody");
        RobotInterface->AddCommandReadState(this->StateTable, mJacobianBody, "GetJacobianBody");
        RobotInterface->AddCommandReadState(this->StateTable, mJacobianSpatial, "GetJacobianSpatial");
        RobotInterface->AddCommandReadState(this->mStateTableState,
                                            mStateTableStateCurrent, "GetCurrentState");
        RobotInterface->AddCommandReadState(this->mStateTableState,
                                            mStateTableStateDesired, "GetDesiredState");
        // Set
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetBaseFrame,
                                        this, "SetBaseFrame");
        RobotInterface->AddCommandVoid(&mtsIntuitiveResearchKitArm::Freeze,
                                       this, "Freeze");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionJoint,
                                        this, "SetPositionJoint");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionRelativeJoint,
                                        this, "SetPositionRelativeJoint");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionGoalJoint,
                                        this, "SetPositionGoalJoint");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionCartesian,
                                        this, "SetPositionCartesian");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionRelativeCartesian,
                                        this, "SetPositionRelativeCartesian");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionGoalCartesian,
                                        this, "SetPositionGoalCartesian");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetEffortJoint,
                                        this, "SetEffortJoint");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetWrenchBody,
                                        this, "SetWrenchBody");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetWrenchBodyOrientationAbsolute,
                                        this, "SetWrenchBodyOrientationAbsolute");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetWrenchSpatial,
                                        this, "SetWrenchSpatial");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetGravityCompensation,
                                        this, "SetGravityCompensation");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetCartesianImpedanceGains,
                                        this, "SetCartesianImpedanceGains");

        // Trajectory events
        RobotInterface->AddEventWrite(mJointTrajectory.GoalReachedEvent, "GoalReached", bool());
        // Robot State
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetDesiredState,
                                        this, "SetDesiredState", std::string(""));
        // Human readable messages
        RobotInterface->AddEventWrite(MessageEvents.DesiredState, "DesiredState", std::string(""));
        RobotInterface->AddEventWrite(MessageEvents.CurrentState, "CurrentState", std::string(""));

        // Stats
        RobotInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                            "GetPeriodStatistics");
    }

    // SetState will send log events, it needs to happen after the
    // provided interface has been created
    SetDesiredState("UNINITIALIZED");
}

void mtsIntuitiveResearchKitArm::SetDesiredState(const std::string & state)
{
    // try to find the state in state machine
    if (!mArmState.StateExists(state)) {
        RobotInterface->SendError(this->GetName() + ": unsupported state " + state);
        return;
    }
    // setting desired state triggers a new event so user nows which state is current
    MessageEvents.CurrentState(mArmState.CurrentState());
    // try to set the desired state
    try {
        mArmState.SetDesiredState(state);
    } catch (...) {
        RobotInterface->SendError(this->GetName() + ": " + state + " is not an allowed desired state");
        return;
    }
    // update state table
    mStateTableState.Start();
    mStateTableStateDesired = state;
    mStateTableState.Advance();

    MessageEvents.DesiredState(state);
    RobotInterface->SendStatus(this->GetName() + ": desired state " + state);
}

void mtsIntuitiveResearchKitArm::ResizeKinematicsData(void)
{
    mJacobianBody.SetSize(6, NumberOfJointsKinematics());
    mJacobianSpatial.SetSize(6, NumberOfJointsKinematics());
    mJacobianBodyTranspose.ForceAssign(mJacobianBody.Transpose());
    mJacobianPInverseData.Allocate(mJacobianBodyTranspose);
    mEffortJointSet.SetSize(NumberOfJointsKinematics());
    mEffortJointSet.ForceTorque().SetAll(0.0);
    mEffortJoint.SetSize(NumberOfJointsKinematics());
    mEffortJoint.SetAll(0.0);
}

void mtsIntuitiveResearchKitArm::Configure(const std::string & filename)
{
    robManipulator::Errno result;
    result = this->Manipulator->LoadRobot(filename);
    if (result == robManipulator::EFAILURE) {
        CMN_LOG_CLASS_INIT_ERROR << GetName() << ": Configure: failed to load manipulator configuration file \""
                                 << filename << "\"" << std::endl;
    }
    ResizeKinematicsData();
}

void mtsIntuitiveResearchKitArm::ConfigureDH(const Json::Value & jsonConfig)
{
    // load base offset transform if any (without warning)
    const Json::Value jsonBase = jsonConfig["base-offset"];
    if (!jsonBase.isNull()) {
        // save the transform as Manipulator Rtw0
        cmnDataJSON<vctFrm4x4>::DeSerializeText(Manipulator->Rtw0, jsonBase);
        if (!nmrIsOrthonormal(Manipulator->Rtw0.Rotation())) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": the base offset rotation doesn't seem to be orthonormal"
                                     << std::endl;
        }
    }

    // load DH parameters
    const Json::Value jsonDH = jsonConfig["DH"];
    if (jsonDH.isNull()) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": can find \"DH\" data in configuration file" << std::endl;
    }
    this->Manipulator->LoadRobot(jsonDH);
    std::stringstream dhResult;
    this->Manipulator->PrintKinematics(dhResult);
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure " << this->GetName()
                               << ": loaded kinematics" << std::endl << dhResult.str() << std::endl;
    ResizeKinematicsData();
}

void mtsIntuitiveResearchKitArm::Startup(void)
{
    this->SetDesiredState("UNINITIALIZED");
    MessageEvents.DesiredState(std::string("UNINITIALIZED"));
}

void mtsIntuitiveResearchKitArm::Run(void)
{
    // collect data from required interfaces
    ProcessQueuedEvents();
    try {
        mArmState.Run();
    } catch (std::exception & e) {
        RobotInterface->SendError(this->GetName() + ": in state " + mArmState.CurrentState()
                                  + ", caught exception \"" + e.what() + "\"");
        this->SetDesiredState("UNINITIALIZED");
    }
    // trigger ExecOut event
    RunEvent();
    ProcessQueuedCommands();

    CartesianGetPrevious = CartesianGet;
}

void mtsIntuitiveResearchKitArm::Cleanup(void)
{
    if (NumberOfBrakes() > 0) {
        RobotIO.BrakeEngage();
    }
    CMN_LOG_CLASS_INIT_VERBOSE << GetName() << ": Cleanup" << std::endl;
}

void mtsIntuitiveResearchKitArm::SetSimulated(void)
{
    mIsSimulated = true;
    // in simulation mode, we don't need IO
    RemoveInterfaceRequired("RobotIO");
}

void mtsIntuitiveResearchKitArm::GetRobotData(void)
{
    // check that the robot still has power
    if (mPowered) {
        vctBoolVec actuatorAmplifiersStatus(NumberOfJoints());
        RobotIO.GetActuatorAmpStatus(actuatorAmplifiersStatus);
        vctBoolVec brakeAmplifiersStatus(NumberOfBrakes());
        if (NumberOfBrakes() > 0) {
            RobotIO.GetBrakeAmpStatus(brakeAmplifiersStatus);
        }
        if (!(actuatorAmplifiersStatus.All() && brakeAmplifiersStatus.All())) {
            mPowered = false;
            RobotInterface->SendError(this->GetName() + ": detected power loss");
            mArmState.SetDesiredState("UNINITIALIZED");
            return;
        }
    }

    // we can start reporting some joint values after the robot is powered
    if (mJointReady) {
        mtsExecutionResult executionResult;
        // joint state
        executionResult = PID.GetStateJoint(JointsPID);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to GetJointState failed \""
                                    << executionResult << "\"" << std::endl;
        }

        // desired joint state
        executionResult = PID.GetStateJointDesired(JointsDesiredPID);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to GetJointStateDesired failed \""
                                    << executionResult << "\"" << std::endl;
        }

        // update joint states used for kinematics
        UpdateJointsKinematics();

        // when the robot is ready, we can compute cartesian position
        if (mCartesianReady) {
            // update cartesian position
            CartesianGetLocal = Manipulator->ForwardKinematics(JointsKinematics.Position());
            CartesianGet = BaseFrame * CartesianGetLocal;
            // normalize
            CartesianGetLocal.Rotation().NormalizedSelf();
            CartesianGet.Rotation().NormalizedSelf();
            // flags
            CartesianGetLocalParam.SetTimestamp(JointsKinematics.Timestamp());
            CartesianGetLocalParam.SetValid(true);
            CartesianGetParam.SetTimestamp(JointsKinematics.Timestamp());
            CartesianGetParam.SetValid(BaseFrameValid);
            // update jacobians
            Manipulator->JacobianSpatial(JointsKinematics.Position(), mJacobianSpatial);
            Manipulator->JacobianBody(JointsKinematics.Position(), mJacobianBody);

            // update cartesian velocity using the jacobian and joint
            // velocities.
            vctDoubleVec cartesianVelocity(6);
            cartesianVelocity.ProductOf(mJacobianBody, JointsKinematics.Velocity());
            vct3 relative, absolute;
            // linear
            relative.Assign(cartesianVelocity.Ref(3, 0));
            CartesianGet.Rotation().ApplyTo(relative, absolute);
            CartesianVelocityGetParam.SetVelocityLinear(absolute);
            // angular
            relative.Assign(cartesianVelocity.Ref(3, 3));
            CartesianGet.Rotation().ApplyTo(relative, absolute);
            CartesianVelocityGetParam.SetVelocityAngular(absolute);
            // valid/timestamp
            CartesianVelocityGetParam.SetValid(true);
            CartesianVelocityGetParam.SetTimestamp(JointsKinematics.Timestamp());


            // update wrench based on measured joint current efforts
            mJacobianBodyTranspose.Assign(mJacobianBody.Transpose());
            nmrPInverse(mJacobianBodyTranspose, mJacobianPInverseData);
            vctDoubleVec wrench(6);
            wrench.ProductOf(mJacobianPInverseData.PInverse(), JointsKinematics.Effort());
            if (mWrenchBodyOrientationAbsolute) {
                // forces
                relative.Assign(wrench.Ref(3, 0));
                CartesianGet.Rotation().ApplyTo(relative, absolute);
                mWrenchGet.Force().Ref<3>(0).Assign(absolute);
                // torques
                relative.Assign(wrench.Ref(3, 3));
                CartesianGet.Rotation().ApplyTo(relative, absolute);
                mWrenchGet.Force().Ref<3>(3).Assign(absolute);
            } else {
                mWrenchGet.Force().Assign(wrench);
            }
            // valid/timestamp
            mWrenchGet.SetValid(true);
            mWrenchGet.SetTimestamp(JointsKinematics.Timestamp());

            // update cartesian position desired based on joint desired
            CartesianGetLocalDesired = Manipulator->ForwardKinematics(JointsDesiredKinematics.Position());
            CartesianGetDesired = BaseFrame * CartesianGetLocalDesired;
            // normalize
            CartesianGetLocalDesired.Rotation().NormalizedSelf();
            CartesianGetDesired.Rotation().NormalizedSelf();
            // flags
            CartesianGetLocalDesiredParam.SetTimestamp(JointsDesiredKinematics.Timestamp());
            CartesianGetLocalDesiredParam.SetValid(true);
            CartesianGetDesiredParam.SetTimestamp(JointsDesiredKinematics.Timestamp());
            CartesianGetDesiredParam.SetValid(BaseFrameValid);
        } else {
            // update cartesian position
            CartesianGetLocal.Assign(vctFrm4x4::Identity());
            CartesianGet.Assign(vctFrm4x4::Identity());
            CartesianGetLocalParam.SetValid(false);
            CartesianGetParam.SetValid(false);
            // velocities and wrench
            CartesianVelocityGetParam.SetValid(false);
            mWrenchGet.SetValid(false);
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
        JointsPID.Position().Zeros();
        JointsPID.Velocity().Zeros();
        JointsPID.Effort().Zeros();
        JointsPID.SetValid(false);

        JointsKinematics.Position().Zeros();
        JointsKinematics.Velocity().Zeros();
        JointsKinematics.Effort().Zeros();
        JointsKinematics.SetValid(false);
    }
}

void mtsIntuitiveResearchKitArm::UpdateJointsKinematics(void)
{
    // for most cases, joints used for the kinematics are the first n joints
    // from PID

    // copy names and types only if first time
    if (JointsKinematics.Name().size() != NumberOfJointsKinematics()) {
        JointsKinematics.Name().ForceAssign(JointsPID.Name().Ref(NumberOfJointsKinematics()));
        JointsKinematics.Type().ForceAssign(JointsPID.Type().Ref(NumberOfJointsKinematics()));
    }
    JointsKinematics.Position().ForceAssign(JointsPID.Position().Ref(NumberOfJointsKinematics()));
    JointsKinematics.Velocity().ForceAssign(JointsPID.Velocity().Ref(NumberOfJointsKinematics()));
    JointsKinematics.Effort().ForceAssign(JointsPID.Effort().Ref(NumberOfJointsKinematics()));
    JointsKinematics.Timestamp() = JointsPID.Timestamp();
    // commanded
    if (JointsDesiredKinematics.Name().size() != NumberOfJointsKinematics()) {
        JointsDesiredKinematics.Name().ForceAssign(JointsDesiredPID.Name().Ref(NumberOfJointsKinematics()));
        JointsDesiredKinematics.Type().ForceAssign(JointsDesiredPID.Type().Ref(NumberOfJointsKinematics()));
    }
    JointsDesiredKinematics.Position().ForceAssign(JointsDesiredPID.Position().Ref(NumberOfJointsKinematics()));
    // JointsDesiredKinematics.Velocity().ForceAssign(JointsDesiredPID.Velocity().Ref(NumberOfJointsKinematics()));
    JointsDesiredKinematics.Effort().ForceAssign(JointsDesiredPID.Effort().Ref(NumberOfJointsKinematics()));
    JointsDesiredKinematics.Timestamp() = JointsDesiredPID.Timestamp();
}

void mtsIntuitiveResearchKitArm::ToJointsPID(const vctDoubleVec & jointsKinematics, vctDoubleVec & jointsPID)
{
    jointsPID.Assign(jointsKinematics);
}

void mtsIntuitiveResearchKitArm::StateChanged(void)
{
    const std::string newState = mArmState.CurrentState();
    // update state table
    mStateTableState.Start();
    mStateTableStateCurrent = newState;
    mStateTableState.Advance();
    // eventNumberOfJoints()
    MessageEvents.CurrentState(newState);
    RobotInterface->SendStatus(this->GetName() + ": current state " + newState);
}

void mtsIntuitiveResearchKitArm::RunAllStates(void)
{
    GetRobotData();

    // always allow to go to unitialized
    if (mArmState.DesiredStateIsNotCurrent()) {
        if (mArmState.DesiredState() == "UNINITIALIZED") {
            mArmState.SetCurrentState("UNINITIALIZED");
        } else {
            // error handling will require to swith to fallback state
            if (mArmState.DesiredState() == mFallbackState) {
                mArmState.SetCurrentState(mFallbackState);
            }
        }
    }
}

void mtsIntuitiveResearchKitArm::EnterUninitialized(void)
{
    mFallbackState = "UNINITIALIZED";
    if (NumberOfBrakes() > 0) {
        RobotIO.BrakeEngage();
    }

    RobotIO.UsePotsForSafetyCheck(false);
    RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfAxes(), 0.0));
    RobotIO.DisablePower();
    PID.Enable(false);
    PID.SetCheckPositionLimit(true);
    mPowered = false;
    mJointReady = false;
    mCartesianReady = false;
    mJointControlReady = false;
    mCartesianControlReady = false;
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::UNDEFINED_SPACE,
                           mtsIntuitiveResearchKitArmTypes::UNDEFINED_MODE);
}

void mtsIntuitiveResearchKitArm::TransitionUninitialized(void)
{
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState("CALIBRATING_ENCODERS_FROM_POTS");
    }
}

void mtsIntuitiveResearchKitArm::EnterCalibratingEncodersFromPots(void)
{
    // if simulated, no need to bias encoders
    if (mIsSimulated) {
        RobotInterface->SendStatus(this->GetName() + ": simulated mode, no need to calibrate encoders");
        return;
    }
    if (mEncoderBiased) {
        RobotInterface->SendStatus(this->GetName() + ": encoders have already been calibrated, skipping");
        return;
    }

    // request bias encoder
    const double currentTime = this->StateTable.GetTic();
    RobotIO.BiasEncoder(1970); // birth date, state table only contain 1999 elements anyway
    mHomingBiasEncoderRequested = true;
    mHomingTimer = currentTime;
}

void mtsIntuitiveResearchKitArm::TransitionCalibratingEncodersFromPots(void)
{
    if (mIsSimulated || mEncoderBiased) {
        mJointReady = true;
        mArmState.SetCurrentState("ENCODERS_BIASED");
        return;
    }

    const double currentTime = this->StateTable.GetTic();
    const double timeToBias = 30.0 * cmn_s; // large timeout
    if ((currentTime - mHomingTimer) > timeToBias) {
        mHomingBiasEncoderRequested = false;
        RobotInterface->SendError(this->GetName() + ": failed to bias encoders (timeout)");
        this->SetDesiredState(mFallbackState);
    }
}

void mtsIntuitiveResearchKitArm::TransitionEncodersBiased(void)
{
    // move to next stage if desired state is anything past post pot
    // calibration
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState("POWERING");
    }
}

void mtsIntuitiveResearchKitArm::EnterPowering(void)
{
    mPowered = false;

    if (mIsSimulated) {
        PID.EnableTrackingError(false);
        PID.Enable(true);
        vctDoubleVec goal(NumberOfJoints());
        goal.SetAll(0.0);
        mtsIntuitiveResearchKitArm::SetPositionJointLocal(goal);
        return;
    }

    const double currentTime = this->StateTable.GetTic();

    // in case we still have power but brakes are not engaged
    if (NumberOfBrakes() > 0) {
        RobotIO.BrakeEngage();
    }

    mHomingTimer = currentTime;
    // make sure the PID is not sending currents
    PID.Enable(false);
    // pre-load the boards with zero current
    RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfAxes(), 0.0));
    // enable power and set a flags to move to next step
    RobotIO.EnablePower();
    RobotInterface->SendStatus(this->GetName() + ": power requested");
}

void mtsIntuitiveResearchKitArm::TransitionPowering(void)
{
    if (mIsSimulated) {
        mArmState.SetCurrentState("POWERED");
        return;
    }

    const double currentTime = this->StateTable.GetTic();

    // check power status
    vctBoolVec actuatorAmplifiersStatus(NumberOfJoints());
    RobotIO.GetActuatorAmpStatus(actuatorAmplifiersStatus);
    vctBoolVec brakeAmplifiersStatus(NumberOfBrakes());
    if (NumberOfBrakes() > 0) {
        RobotIO.GetBrakeAmpStatus(brakeAmplifiersStatus);
    }
    if (actuatorAmplifiersStatus.All() && brakeAmplifiersStatus.All()) {
        RobotInterface->SendStatus(this->GetName() + ": power on");
        mArmState.SetCurrentState("POWERED");
    } else {
        if ((currentTime - mHomingTimer) > mtsIntuitiveResearchKit::TimeToPower) {
            RobotInterface->SendError(this->GetName() + ": failed to enable power");
            this->SetDesiredState(mFallbackState);
        }
    }
}

void mtsIntuitiveResearchKitArm::EnterPowered(void)
{
    if (mIsSimulated) {
        return;
    }

    // use pots for redundancy
    RobotIO.UsePotsForSafetyCheck(true);

    mPowered = true;
    mFallbackState = "POWERED";

    // disable PID for fallback
    RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfAxes(), 0.0));
    PID.Enable(false);
    mCartesianReady = false;
    mJointControlReady = false;
    mCartesianControlReady = false;
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::UNDEFINED_SPACE,
                           mtsIntuitiveResearchKitArmTypes::UNDEFINED_MODE);

    // engage brakes for fallback
    if (NumberOfBrakes() > 0) {
        RobotIO.BrakeEngage();
    }
}

void mtsIntuitiveResearchKitArm::TransitionPowered(void)
{
    // move to next stage if desired state is anything past power
    // unless user request new pots calibration
    if (mArmState.DesiredStateIsNotCurrent()) {
        if (mArmState.DesiredState() == "ENCODERS_BIASED") {
            mArmState.SetCurrentState("CALIBRATING_ENCODERS_FROM_POTS");
        } else {
            mArmState.SetCurrentState("HOMING_ARM");
        }
    }
}

void mtsIntuitiveResearchKitArm::EnterHomingArm(void)
{
    // disable joint limits
    PID.SetCheckPositionLimit(false);
    // enable tracking errors
    PID.SetTrackingErrorTolerance(PID.DefaultTrackingErrorTolerance);
    // enable PID
    PID.Enable(true);

    // release brakes if any
    if ((NumberOfBrakes() > 0) && !mIsSimulated) {
        RobotIO.BrakeRelease();
    }

    // compute joint goal position
    this->SetGoalHomingArm();
    mJointTrajectory.GoalVelocity.SetAll(0.0);
    mJointTrajectory.EndTime = 0.0;
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);
}

void mtsIntuitiveResearchKitArm::RunHomingArm(void)
{
    static const double extraTime = 2.0 * cmn_s;
    const double currentTime = this->StateTable.GetTic();

    mJointTrajectory.Reflexxes.Evaluate(JointSet,
                                        JointVelocitySet,
                                        mJointTrajectory.Goal,
                                        mJointTrajectory.GoalVelocity);
    mtsIntuitiveResearchKitArm::SetPositionJointLocal(JointSet);

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
            PID.SetCheckPositionLimit(true);
            mArmState.SetCurrentState("ARM_HOMED");
        } else {
            // time out
            if (currentTime > mHomingTimer + extraTime) {
                CMN_LOG_CLASS_INIT_WARNING << GetName() << ": RunHomingArm: unable to reach home position, error in degrees is "
                                           << mJointTrajectory.GoalError * (180.0 / cmnPI) << std::endl;
                RobotInterface->SendError(this->GetName() + ": unable to reach home position during calibration on pots");
                this->SetDesiredState(mFallbackState);
            }
        }
        break;

    default:
        RobotInterface->SendError(this->GetName() + ": error while evaluating trajectory");
        this->SetDesiredState(mFallbackState);
        break;
    }
}

void mtsIntuitiveResearchKitArm::EnterReady(void)
{
    // set ready flags, some might have been set earlier by derived
    // classes (e.g. PSM allows joint commands without tool
    mCartesianReady = true;
    mJointControlReady = true;
    mCartesianControlReady = true;
    // no control mode defined
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::UNDEFINED_SPACE,
                           mtsIntuitiveResearchKitArmTypes::UNDEFINED_MODE);

    if (mIsSimulated) {
        return;
    }

    // enable PID and start from current position
    mtsIntuitiveResearchKitArm::SetPositionJointLocal(JointsDesiredPID.Position());
    PID.EnableTrackingError(UsePIDTrackingError());
    PID.EnableJoints(vctBoolVec(NumberOfJoints(), true));
    PID.SetCheckPositionLimit(true);
    PID.Enable(true);
}

void mtsIntuitiveResearchKitArm::LeaveReady(void)
{
    // set ready flag
    mJointControlReady = false;
    mCartesianControlReady = false;
    // no control mode defined
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::UNDEFINED_SPACE,
                           mtsIntuitiveResearchKitArmTypes::UNDEFINED_MODE);
}

void mtsIntuitiveResearchKitArm::RunReady(void)
{
    if (mControlCallback) {
        mControlCallback->Execute();
    }
}

void mtsIntuitiveResearchKitArm::ControlPositionJoint(void)
{
    if (mHasNewPIDGoal) {
        SetPositionJointLocal(JointSet);
        // reset flag
        mHasNewPIDGoal = false;
    }
}

void mtsIntuitiveResearchKitArm::ControlPositionRelativeJoint(void)
{
    if (mHasNewPIDGoal) {

        std::cerr << CMN_LOG_DETAILS << " --------- tbd" << std::endl;

        // reset flag
        mJointRelative.Zeros();
        mHasNewPIDGoal = false;
    }
}

void mtsIntuitiveResearchKitArm::ControlPositionGoalJoint(void)
{
    // check if there's anything to do
    if (!mJointTrajectory.IsWorking) {
        return;
    }

    mJointTrajectory.Reflexxes.Evaluate(JointSet,
                                        JointVelocitySet,
                                        mJointTrajectory.Goal,
                                        mJointTrajectory.GoalVelocity);
    mtsIntuitiveResearchKitArm::SetPositionJointLocal(JointSet);

    const robReflexxes::ResultType trajectoryResult = mJointTrajectory.Reflexxes.ResultValue();
    const double currentTime = this->StateTable.GetTic();

    switch (trajectoryResult) {
    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (mJointTrajectory.EndTime == 0.0) {
            mJointTrajectory.EndTime = currentTime + mJointTrajectory.Reflexxes.Duration();
        }
        break;
    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        mJointTrajectory.GoalReachedEvent(true);
        mJointTrajectory.IsWorking = false;
        break;
    default:
        RobotInterface->SendError(this->GetName() + ": error while evaluating trajectory");
        mJointTrajectory.IsWorking = false;
        break;
    }
}

void mtsIntuitiveResearchKitArm::ControlPositionCartesian(void)
{
    if (mHasNewPIDGoal) {
        // copy current position
        vctDoubleVec jointSet(JointsKinematics.Position());

        // compute desired arm position
        CartesianPositionFrm.From(CartesianSetParam.Goal());
        if (this->InverseKinematics(jointSet, BaseFrame.Inverse() * CartesianPositionFrm) == robManipulator::ESUCCESS) {
            // finally send new joint values
            SetPositionJointLocal(jointSet);
        } else {
            RobotInterface->SendError(this->GetName() + ": unable to solve inverse kinematics");
        }
        // reset flag
        mHasNewPIDGoal = false;
    }
}

void mtsIntuitiveResearchKitArm::ControlPositionRelativeCartesian(void)
{
    if (mHasNewPIDGoal) {

        std::cerr << CMN_LOG_DETAILS << " --------- tbd" << std::endl;

        // reset flag
        mCartesianRelative = vctFrm3::Identity();
        mHasNewPIDGoal = false;
    }
}

void mtsIntuitiveResearchKitArm::ControlPositionGoalCartesian(void)
{
    // trajectories are computed in joint space for now
    ControlPositionGoalJoint();
}

bool mtsIntuitiveResearchKitArm::ArmIsReady(const std::string & methodName,
                                            const mtsIntuitiveResearchKitArmTypes::ControlSpace space)
{
    // reset counter if ready
    if (((space == mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)
         && mJointControlReady)
        || ((space == mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)
            && mCartesianControlReady)) {
        mArmNotReadyCounter = 0;
        mArmNotReadyTimeLastMessage = 0.0;
        return true;
    }
    // throttle messages in time
    if ((StateTable.GetTic() - mArmNotReadyTimeLastMessage) > 2.0 * cmn_s) {
        std::stringstream message;
        message << this->GetName() << ": " << methodName << ", arm not ready";
        if (mArmNotReadyCounter > 1) {
            message << " (" << mArmNotReadyCounter << " errors)";
        }
        RobotInterface->SendWarning(message.str());
        mArmNotReadyTimeLastMessage = StateTable.GetTic();
    }
    mArmNotReadyCounter++;
    return false;
}

void mtsIntuitiveResearchKitArm::SetControlSpaceAndMode(const mtsIntuitiveResearchKitArmTypes::ControlSpace space,
                                                        const mtsIntuitiveResearchKitArmTypes::ControlMode mode,
                                                        mtsCallableVoidBase * callback)
{
    // ignore if already in the same space
    if ((space == mControlSpace) && (mode == mControlMode)) {
        if (callback) {
            delete callback;
        }
        return;
    }

    // transitions
    if (space != mControlSpace) {
        // check if the arm is ready to use in cartesian space
        if (space == mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE) {
            if (this->IsSafeForCartesianControl()) {
                // set flag
                mControlSpace = space;
                mSafeForCartesianControlCounter = 0;
            } else {
                if (mSafeForCartesianControlCounter == 0) {
                    // message if needed
                    RobotInterface->SendWarning(this->GetName() + ": tool/endoscope needs to be inserted past the cannula to switch to cartesian space");
                }
                mSafeForCartesianControlCounter = (mSafeForCartesianControlCounter + 1) % 1000;
                return;
            }
        } else {
            // all other spaces, just set the current space
            mEffortOrientationLocked = false;
            mControlSpace = space;
        }
    }

    if (mode != mControlMode) {

        switch (mode) {
        case mtsIntuitiveResearchKitArmTypes::POSITION_MODE:
        case mtsIntuitiveResearchKitArmTypes::POSITION_INCREMENT_MODE:
            // configure PID
            PID.EnableTrackingError(UsePIDTrackingError());
            PID.EnableTorqueMode(vctBoolVec(NumberOfJoints(), false));
            mHasNewPIDGoal = false;
            mCartesianRelative = vctFrm3::Identity();
            mJointRelative.Zeros();
            mEffortOrientationLocked = false;
            break;
        case mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE:
            // configure PID
            PID.EnableTrackingError(UsePIDTrackingError());
            PID.EnableTorqueMode(vctBoolVec(NumberOfJoints(), false));
            mEffortOrientationLocked = false;
            // initialize trajectory
            JointSet.Assign(JointsDesiredPID.Position(), NumberOfJoints());
            if ((mControlMode == mtsIntuitiveResearchKitArmTypes::POSITION_MODE)
                || (mControlMode == mtsIntuitiveResearchKitArmTypes::POSITION_INCREMENT_MODE)) {
                JointVelocitySet.Assign(JointsPID.Velocity(), NumberOfJoints());
            } else {
                // we're switching from effort or no mode
                JointVelocitySet.SetSize(NumberOfJoints());
                JointVelocitySet.SetAll(0.0);
            }
            mJointTrajectory.Reflexxes.Set(mJointTrajectory.Velocity,
                                           mJointTrajectory.Acceleration,
                                           StateTable.PeriodStats.PeriodAvg(),
                                           robReflexxes::Reflexxes_TIME);
            break;
        case mtsIntuitiveResearchKitArmTypes::EFFORT_MODE:
            // configure PID
            PID.EnableTrackingError(false);
            mEffortJoint.Assign(vctDoubleVec(NumberOfJointsKinematics(), 0.0));
            SetControlEffortActiveJoints();
            break;
        default:
            break;
        }

        // set flag
        mControlMode = mode;
    }

    // set control callback for RunReady
    switch (mControlMode) {
    case mtsIntuitiveResearchKitArmTypes::POSITION_MODE:
        switch (mControlSpace) {
        case mtsIntuitiveResearchKitArmTypes::JOINT_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::ControlPositionJoint, this);
            break;
        case mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::ControlPositionCartesian, this);
            break;
        default:
            break;
        }
        break;
    case mtsIntuitiveResearchKitArmTypes::POSITION_INCREMENT_MODE:
        switch (mControlSpace) {
        case mtsIntuitiveResearchKitArmTypes::JOINT_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::ControlPositionRelativeJoint, this);
            break;
        case mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::ControlPositionRelativeCartesian, this);
            break;
        default:
            break;
        }
        break;
    case mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE:
        switch (mControlSpace) {
        case mtsIntuitiveResearchKitArmTypes::JOINT_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::ControlPositionGoalJoint, this);
            break;
        case mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::ControlPositionGoalCartesian, this);
            break;
        default:
            break;
        }
        break;
    case mtsIntuitiveResearchKitArmTypes::EFFORT_MODE:
        switch (mControlSpace) {
        case mtsIntuitiveResearchKitArmTypes::JOINT_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::ControlEffortJoint, this);
            break;
        case mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::ControlEffortCartesian, this);
            break;
        default:
            break;
        }
        break;
    default:
        SetControlCallback(0);
        break;
    }

    // use provided callback if the space or mode is user defined
    if ((mControlMode == mtsIntuitiveResearchKitArmTypes::USER_MODE)
        || (mControlSpace == mtsIntuitiveResearchKitArmTypes::USER_SPACE)) {
        mControlCallback = callback;
    }

    // messages
    RobotInterface->SendStatus(this->GetName() + ": control "
                               + cmnData<mtsIntuitiveResearchKitArmTypes::ControlSpace>::HumanReadable(mControlSpace)
                               + '/'
                               + cmnData<mtsIntuitiveResearchKitArmTypes::ControlMode>::HumanReadable(mControlMode));
}

void mtsIntuitiveResearchKitArm::SetControlEffortActiveJoints(void)
{
    PID.EnableTorqueMode(vctBoolVec(NumberOfJoints(), true));
}

void mtsIntuitiveResearchKitArm::ControlEffortJoint(void)
{
    // effort required
    mEffortJoint.Assign(mEffortJointSet.ForceTorque());

    // add gravity compensation if needed
    if (mGravityCompensation) {
        AddGravityCompensationEfforts(mEffortJoint);
    }

    // add custom efforts
    AddCustomEfforts(mEffortJoint);

    // convert to cisstParameterTypes
    SetEffortJointLocal(mEffortJoint);
}

void mtsIntuitiveResearchKitArm::ControlEffortCartesian(void)
{
    // update torques based on wrench
    vctDoubleVec wrench(6);

    // get force preload from derived classes, in most cases 0, platform control for MTM
    vctDoubleVec effortPreload(NumberOfJointsKinematics());
    
    ControlEffortCartesianPreload(effortPreload);
    vctDoubleVec wrenchPreload(6);
    //  \todo this looks seriously wrong, needs to be fixed using
    if (mWrenchType == WRENCH_SPATIAL) {
        std::cerr << CMN_LOG_DETAILS << " this needs to be fixed, mJacobianPInverseData is for body frame, not spatial" << std::endl;
    } else {
        wrenchPreload.ProductOf(mJacobianPInverseData.PInverse(), effortPreload);
    }
    
    // body wrench
    if (mWrenchType == WRENCH_BODY) {
        // either using wrench provided by user or cartesian impedance
        if (mCartesianImpedance) {
            mCartesianImpedanceController.Update(CartesianGetLocalParam,
                                                 CartesianVelocityGetParam,
                                                 mWrenchSet,
                                                 mWrenchBodyOrientationAbsolute);
            wrench.Assign(mWrenchSet.Force());
        } else {
            // user provided wrench
            if (mWrenchBodyOrientationAbsolute) {
                // use forward kinematics orientation to have constant wrench orientation
                vct3 relative, absolute;
                // wrench
                relative.Assign(mWrenchSet.Force().Ref<3>(0));
                CartesianGet.Rotation().ApplyInverseTo(relative, absolute);
                wrench.Ref(3, 0).Assign(absolute);
                // torque
                relative.Assign(mWrenchSet.Force().Ref<3>(3));
                CartesianGet.Rotation().ApplyInverseTo(relative, absolute);
                wrench.Ref(3, 3).Assign(absolute);
            } else {
                wrench.Assign(mWrenchSet.Force());
            }
        }
        mEffortJoint.ProductOf(mJacobianBody.Transpose(), wrench - wrenchPreload);
        mEffortJoint.Add(effortPreload);
    }
    // spatial wrench
    else if (mWrenchType == WRENCH_SPATIAL) {
        wrench.Assign(mWrenchSet.Force());
        mEffortJoint.ProductOf(mJacobianSpatial.Transpose(), wrench - wrenchPreload);
        mEffortJoint.Add(effortPreload);
    }

    // add gravity compensation if needed
    if (mGravityCompensation) {
        AddGravityCompensationEfforts(mEffortJoint);
    }

    // add custom efforts
    AddCustomEfforts(mEffortJoint);

    // send to PID
    SetEffortJointLocal(mEffortJoint);

    // lock orientation if needed
    if (mEffortOrientationLocked) {
        ControlEffortOrientationLocked();
    }
}

void mtsIntuitiveResearchKitArm::ControlEffortOrientationLocked(void)
{
    CMN_LOG_CLASS_RUN_ERROR << GetName()
                            << ": ControlEffortOrientationLocked, this method should never be called, MTMs are the only arms able to lock orientation and the derived implementation of this method should be called"
                            << std::endl;
}

void mtsIntuitiveResearchKitArm::SetEffortJointLocal(const vctDoubleVec & newEffort)
{
    // convert to cisstParameterTypes
    mTorqueSetParam.SetForceTorque(newEffort);
    PID.SetTorqueJoint(mTorqueSetParam);
}

void mtsIntuitiveResearchKitArm::SetPositionJointLocal(const vctDoubleVec & newPosition)
{
    JointSetParam.Goal().Zeros();
    JointSetParam.Goal().Assign(newPosition, NumberOfJoints());
    PID.SetPositionJoint(JointSetParam);
}

void mtsIntuitiveResearchKitArm::Freeze(void)
{
    if (!ArmIsReady("Freeze", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitArmTypes::POSITION_MODE);
    // set goal
    JointSet.Assign(JointsDesiredKinematics.Position(), NumberOfJointsKinematics());
    mHasNewPIDGoal = true;
}

void mtsIntuitiveResearchKitArm::SetPositionJoint(const prmPositionJointSet & newPosition)
{
    if (!ArmIsReady("SetPositionJoint", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitArmTypes::POSITION_MODE);
    // set goal
    JointSet.Assign(newPosition.Goal(), NumberOfJointsKinematics());
    mHasNewPIDGoal = true;
}

void mtsIntuitiveResearchKitArm::SetPositionRelativeJoint(const prmPositionJointSet & difference)
{
    if (!ArmIsReady("SetPositionCartesian", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitArmTypes::POSITION_INCREMENT_MODE);
    // set goal
    mJointRelative.Add(difference.Goal());
    mHasNewPIDGoal = true;
}

void mtsIntuitiveResearchKitArm::SetPositionGoalJoint(const prmPositionJointSet & newPosition)
{
    if (!ArmIsReady("SetPositionGoalJoint", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);
    // make sure trajectory is reset
    mJointTrajectory.IsWorking = true;
    mJointTrajectory.EndTime = 0.0;
    // new goal
    ToJointsPID(newPosition.Goal(), mJointTrajectory.Goal);
    mJointTrajectory.GoalVelocity.SetAll(0.0);
}

void mtsIntuitiveResearchKitArm::SetPositionCartesian(const prmPositionCartesianSet & newPosition)
{
    if (!ArmIsReady("SetPositionCartesian", mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitArmTypes::POSITION_MODE);
    // set goal
    CartesianSetParam = newPosition;
    mHasNewPIDGoal = true;
}

void mtsIntuitiveResearchKitArm::SetPositionRelativeCartesian(const prmPositionCartesianSet & difference)
{
    if (!ArmIsReady("SetPositionCartesian", mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitArmTypes::POSITION_INCREMENT_MODE);
    // set goal
    mCartesianRelative = mCartesianRelative * difference.Goal();
    mHasNewPIDGoal = true;
}

void mtsIntuitiveResearchKitArm::SetPositionGoalCartesian(const prmPositionCartesianSet & newPosition)
{
    if (!ArmIsReady("SetPositionGoalCartesian", mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);

    // copy current position
    vctDoubleVec jointSet(JointsKinematics.Position());

    // compute desired slave position
    CartesianPositionFrm.From(newPosition.Goal());

    if (this->InverseKinematics(jointSet, BaseFrame.Inverse() * CartesianPositionFrm) == robManipulator::ESUCCESS) {
        // make sure trajectory is reset
        mJointTrajectory.IsWorking = true;
        mJointTrajectory.EndTime = 0.0;
        // new goal
        ToJointsPID(jointSet, mJointTrajectory.Goal);
        mJointTrajectory.GoalVelocity.SetAll(0.0);
    } else {
        RobotInterface->SendError(this->GetName() + ": SetPositionGoalCartesian, unable to solve inverse kinematics");
        mJointTrajectory.GoalReachedEvent(false);
    }
}

void mtsIntuitiveResearchKitArm::SetBaseFrame(const prmPositionCartesianSet & newBaseFrame)
{
    if (newBaseFrame.Valid()) {
        this->BaseFrame.FromNormalized(newBaseFrame.Goal());
        this->BaseFrameValid = true;
        this->CartesianGetParam.SetReferenceFrame(newBaseFrame.ReferenceFrame());
        this->CartesianGetDesiredParam.SetReferenceFrame(newBaseFrame.ReferenceFrame());
    } else {
        this->BaseFrameValid = false;
    }
}

void mtsIntuitiveResearchKitArm::ErrorEventHandler(const mtsMessage & message)
{
    RobotInterface->SendError(this->GetName() + ": received [" + message.Message + "]");
    this->SetDesiredState(mFallbackState);
}

void mtsIntuitiveResearchKitArm::PositionLimitEventHandler(const vctBoolVec & CMN_UNUSED(flags))
{
    RobotInterface->SendWarning(this->GetName() + ": PID position limit");
}

void mtsIntuitiveResearchKitArm::BiasEncoderEventHandler(const int & nbSamples)
{
    std::stringstream nbSamplesString;
    nbSamplesString << nbSamples;
    RobotInterface->SendStatus(this->GetName() + ": encoders biased using " + nbSamplesString.str() + " potentiometer values");
    if (mHomingBiasEncoderRequested) {
        mHomingBiasEncoderRequested = false;
        mEncoderBiased = true;
        mJointReady = true;
        mArmState.SetCurrentState("ENCODERS_BIASED");
    } else {
        RobotInterface->SendWarning(this->GetName() + ": encoders have been biased by another process");
    }
}

void mtsIntuitiveResearchKitArm::SetEffortJoint(const prmForceTorqueJointSet & effort)
{
    if (!ArmIsReady("SetEffortJoint", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitArmTypes::EFFORT_MODE);

    // set new effort
    mEffortJointSet.ForceTorque().Assign(effort.ForceTorque());
}

void mtsIntuitiveResearchKitArm::SetWrenchBody(const prmForceCartesianSet & wrench)
{
    if (!ArmIsReady("SetWrenchBody", mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitArmTypes::EFFORT_MODE);

    // set new wrench
    mCartesianImpedance = false;
    mWrenchSet = wrench;
    if (mWrenchType != WRENCH_BODY) {
        mWrenchType = WRENCH_BODY;
        RobotInterface->SendStatus(this->GetName() + ": effort cartesian WRENCH_BODY");
    }
}

void mtsIntuitiveResearchKitArm::SetWrenchSpatial(const prmForceCartesianSet & wrench)
{
    if (!ArmIsReady("SetWrenchSpatial", mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitArmTypes::EFFORT_MODE);

    // set new wrench
    mCartesianImpedance = false;
    mWrenchSet = wrench;
    if (mWrenchType != WRENCH_SPATIAL) {
        mWrenchType = WRENCH_SPATIAL;
        RobotInterface->SendStatus(this->GetName() + ": effort cartesian WRENCH_SPATIAL");
    }
}

void mtsIntuitiveResearchKitArm::SetWrenchBodyOrientationAbsolute(const bool & absolute)
{
    mWrenchBodyOrientationAbsolute = absolute;
}

void mtsIntuitiveResearchKitArm::SetGravityCompensation(const bool & gravityCompensation)
{
    mGravityCompensation = gravityCompensation;
}

void mtsIntuitiveResearchKitArm::AddGravityCompensationEfforts(vctDoubleVec & efforts)
{
    vctDoubleVec qd(this->NumberOfJointsKinematics(), 0.0);
    vctDoubleVec gravityEfforts;
    gravityEfforts.ForceAssign(Manipulator->CCG(JointsKinematics.Position(), qd));  // should this take joint velocities?
    efforts.Add(gravityEfforts);
}

void mtsIntuitiveResearchKitArm::SetCartesianImpedanceGains(const prmCartesianImpedanceGains & gains)
{
    if (!ArmIsReady("SetWrenchBody", mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitArmTypes::EFFORT_MODE);

    // set new wrench
    mCartesianImpedance = true;
    mWrenchBodyOrientationAbsolute = true;
    mCartesianImpedanceController.SetGains(gains);
    if (mWrenchType != WRENCH_BODY) {
        mWrenchType = WRENCH_BODY;
        RobotInterface->SendStatus(this->GetName() + ": effort cartesian WRENCH_BODY");
    }
}
