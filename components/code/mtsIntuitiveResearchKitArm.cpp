/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen, Zerui Wang
  Created on: 2016-02-24

  (C) Copyright 2013-2016 Johns Hopkins University (JHU), All Rights Reserved.

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
    mArmState(componentName, "UNINITIALIZED")
{
}

mtsIntuitiveResearchKitArm::mtsIntuitiveResearchKitArm(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    mArmState(arg.Name, "UNINITIALIZED")
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
    mArmState.AddState("POSITION_JOINT");
    mArmState.AddState("POSITION_CARTESIAN");
    mArmState.AddState("POSITION_GOAL_JOINT");
    mArmState.AddState("POSITION_GOAL_CARTESIAN");
    mArmState.AddState("EFFORT_JOINT");
    mArmState.AddState("EFFORT_CARTESIAN");

    // possible desired states
    mArmState.AddAllowedDesiredState("UNINITIALIZED");
    mArmState.AddAllowedDesiredState("ENCODERS_BIASED");
    mArmState.AddAllowedDesiredState("POWERED");
    mArmState.AddAllowedDesiredState("READY");
    mArmState.AddAllowedDesiredState("POSITION_JOINT");
    mArmState.AddAllowedDesiredState("POSITION_CARTESIAN");
    mArmState.AddAllowedDesiredState("POSITION_GOAL_JOINT");
    mArmState.AddAllowedDesiredState("POSITION_GOAL_CARTESIAN");
    mArmState.AddAllowedDesiredState("EFFORT_JOINT");
    mArmState.AddAllowedDesiredState("EFFORT_CARTESIAN");

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

    mArmState.SetTransitionCallback("POWERED",
                                    &mtsIntuitiveResearchKitArm::TransitionPowered,
                                    this);

    #if 0
    // homing arm, i.e. go to zero position if needed and leave PID on
    // handles all DOF on ECM, all but roll on MTM and only first 3 on
    // PSM
    mArmState.SetEnterCallback("HOMING_ARM",
                               &mtsIntuitiveResearchKitArm::EnterHomingArm,
                               this);

    mArmState.SetTransitionCallback("HOMING_ARM",
                                    &mtsIntuitiveResearchKitArm::TransitionHomingArm,
                                    this);

    mArmState.SetTransitionCallback("ARM_HOMED",
                                    &mtsIntuitiveResearchKitArm::TransitionArmHomed,
                                    this);
#endif
    std::cerr << CMN_LOG_DETAILS << " need to finish state machine" << std::endl;

    mCounter = 0;
    mIsSimulated = false;
    mHomedOnce = false;
    mHomingGoesToZero = false; // MTM ignores this
    mHomingBiasEncoderRequested = false;

    mWrenchBodyOrientationAbsolute = false;
    mGravityCompensation = false;

    mHasNewPIDGoal = false;
    mEffortOrientationLocked = false;

    // initialize trajectory data
    JointGet.SetSize(NumberOfJoints());
    JointSet.SetSize(NumberOfJoints());
    JointVelocityGet.SetSize(NumberOfJoints());
    JointVelocitySet.SetSize(NumberOfJoints());
    JointSetParam.Goal().SetSize(NumberOfAxes());
    mJointTrajectory.Velocity.SetSize(NumberOfJoints());
    mJointTrajectory.Acceleration.SetSize(NumberOfJoints());
    mJointTrajectory.Goal.SetSize(NumberOfJoints());
    mJointTrajectory.GoalVelocity.SetSize(NumberOfJoints());
    mJointTrajectory.GoalError.SetSize(NumberOfJoints());
    mJointTrajectory.GoalTolerance.SetSize(NumberOfJoints());
    mJointTrajectory.IsWorking = false;
    PotsToEncodersTolerance.SetSize(NumberOfAxes());

    // initialize velocity
    CartesianVelocityGetParam.SetVelocityLinear(vct3(0.0));
    CartesianVelocityGetParam.SetVelocityAngular(vct3(0.0));
    CartesianVelocityGetParam.SetValid(false);

    // jacobian
    JacobianBody.SetSize(6, NumberOfJointsKinematics());
    this->StateTable.AddData(JacobianBody, "JacobianBody");
    JacobianSpatial.SetSize(6, NumberOfJointsKinematics());
    this->StateTable.AddData(JacobianSpatial, "JacobianSpatial");
    JacobianBodyTranspose.ForceAssign(JacobianBody.Transpose());
    mJacobianPInverseData.Allocate(JacobianBodyTranspose);
    JointExternalEffort.SetSize(NumberOfJointsKinematics());
    mEffortJointSet.SetSize(NumberOfJoints());
    mEffortJointSet.ForceTorque().SetAll(0.0);
    mWrenchGet.SetValid(false);

    // base frame, mostly for cases where no base frame is set by user
    BaseFrame = vctFrm4x4::Identity();
    BaseFrameValid = true;

    CartesianGetParam.SetAutomaticTimestamp(false); // based on PID timestamp
    this->StateTable.AddData(CartesianGetParam, "CartesianPosition");

    CartesianGetDesiredParam.SetAutomaticTimestamp(false); // based on PID timestamp
    this->StateTable.AddData(CartesianGetDesiredParam, "CartesianPositionDesired");

    this->StateTable.AddData(CartesianGetLocalParam, "CartesianPositionLocal");
    this->StateTable.AddData(CartesianGetLocalDesiredParam, "CartesianPositionLocalDesired");
    this->StateTable.AddData(BaseFrame, "BaseFrame");

    JointGetParam.SetAutomaticTimestamp(false); // keep PID timestamp
    this->StateTable.AddData(JointGetParam, "JointPosition");

    JointGetParam.SetAutomaticTimestamp(false); // keep PID timestamp
    this->StateTable.AddData(JointGetDesired, "JointPositionDesired");

    CartesianVelocityGetParam.SetAutomaticTimestamp(false); // keep PID timestamp
    this->StateTable.AddData(CartesianVelocityGetParam, "CartesianVelocityGetParam");

    mWrenchGet.SetAutomaticTimestamp(false); // keep PID timestamp
    this->StateTable.AddData(mWrenchGet, "WrenchGet");

    StateJointParam.SetAutomaticTimestamp(false); // keep PID timestamp
    this->StateTable.AddData(StateJointParam, "StateJoint");

    StateJointDesiredParam.SetAutomaticTimestamp(false); // keep PID timestamp
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
        IOInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::BiasEncoderEventHandler, this, "BiasEncoder");
    }

    // Setup Joints
    SUJInterface = AddInterfaceRequired("BaseFrame", MTS_OPTIONAL);
    if (SUJInterface) {
        SUJInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::SetBaseFrameEventHandler,
                                           this, "PositionCartesianDesired");
        SUJInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::ErrorEventHandler,
                                           this, "Error");
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
        RobotInterface->AddCommandReadState(this->StateTable, mWrenchGet, "GetWrenchBody");
        RobotInterface->AddCommandReadState(this->StateTable, JacobianBody, "GetJacobianBody");
        RobotInterface->AddCommandReadState(this->StateTable, JacobianSpatial, "GetJacobianSpatial");
        // Set
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetBaseFrame,
                                        this, "SetBaseFrame");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionJoint,
                                        this, "SetPositionJoint");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionGoalJoint,
                                        this, "SetPositionGoalJoint");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionCartesian,
                                        this, "SetPositionCartesian");
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

        // Trajectory events
        RobotInterface->AddEventWrite(mJointTrajectory.GoalReachedEvent, "GoalReached", bool());
        // Robot State
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetDesiredState,
                                        this, "SetDesiredState", std::string(""));
        // Human readable messages
        RobotInterface->AddEventWrite(MessageEvents.Status, "Status", std::string(""));
        RobotInterface->AddEventWrite(MessageEvents.Warning, "Warning", std::string(""));
        RobotInterface->AddEventWrite(MessageEvents.Error, "Error", std::string(""));
        RobotInterface->AddEventWrite(MessageEvents.DesiredState, "DesiredState", std::string(""));
        RobotInterface->AddEventWrite(MessageEvents.CurrentState, "CurrentState", std::string(""));

        // Stats
        RobotInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                            "GetPeriodStatistics");
    }
}

void mtsIntuitiveResearchKitArm::SetDesiredState(const std::string & state)
{
    // try to find the state in state machine
    if (!mArmState.StateExists(state)) {
        MessageEvents.Error(this->GetName() + ": unsupported state " + state);
        return;
    }
    // if state is same as current, return
    if (mArmState.CurrentState() == state) {
        return;
    }
    // try to set the desired state
    if (!mArmState.SetDesiredState(state)) {
        MessageEvents.Error(this->GetName() + ": " + state + " is not an allowed desired state");
        return;
    }
    MessageEvents.DesiredState(state);
    MessageEvents.Status(this->GetName() + ": set desired state to " + state);
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

void mtsIntuitiveResearchKitArm::ConfigureDH(const Json::Value & jsonConfig)
{
    // load base offset transform if any (without warning)
    const Json::Value jsonBase = jsonConfig["base-offset"];
    if (!jsonBase.isNull()) {
        // save the transform as Manipulator Rtw0
        cmnDataJSON<vctFrm4x4>::DeSerializeText(Manipulator.Rtw0, jsonBase);
        if (!nmrIsOrthonormal(Manipulator.Rtw0.Rotation())) {
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
    this->Manipulator.LoadRobot(jsonDH);
    std::stringstream dhResult;
    this->Manipulator.PrintKinematics(dhResult);
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure " << this->GetName()
                               << ": loaded kinematics" << std::endl << dhResult.str() << std::endl;
}

void mtsIntuitiveResearchKitArm::Startup(void)
{
    this->SetDesiredState("UNINITIALIZED");
}

void mtsIntuitiveResearchKitArm::Run(void)
{
    // collect data from required interfaces
    ProcessQueuedEvents();
    mArmState.Run();
    // trigger ExecOut event
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

void mtsIntuitiveResearchKitArm::SetSimulated(void)
{
    mIsSimulated = true;
    // in simulation mode, we don't need IO
    RemoveInterfaceRequired("RobotIO");
}

void mtsIntuitiveResearchKitArm::GetRobotData(void)
{
    // we can start reporting some joint values after the robot is powered
    if (this->mJointReady) {
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
        if (this->mCartesianReady) {
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
            // update jacobians
            Manipulator.JacobianSpatial(JointGet, JacobianSpatial);
            Manipulator.JacobianBody(JointGet, JacobianBody);

            // update cartesian velocity using the jacobian and joint
            // velocities.
            vctDoubleVec cartesianVelocity(6);
            cartesianVelocity.ProductOf(JacobianBody,
                                        StateJointParam.Velocity().Ref(NumberOfJointsKinematics()));
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
            CartesianVelocityGetParam.SetTimestamp(StateJointParam.Timestamp());


            // update wrench based on measured joint current efforts
            JacobianBodyTranspose.Assign(JacobianBody.Transpose());
            nmrPInverse(JacobianBodyTranspose, mJacobianPInverseData);
            vctDoubleVec wrench(6);
            wrench.ProductOf(mJacobianPInverseData.PInverse(),
                             StateJointParam.Effort().Ref(this->NumberOfJointsKinematics()));
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
            mWrenchGet.SetTimestamp(StateJointParam.Timestamp());

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
        JointGet.Zeros();
        PID.GetPositionJoint(JointGetParam); // get size right
        JointGetParam.Position().Zeros();
        JointGetParam.SetValid(false);
        JointVelocityGet.Zeros();
        JointVelocityGetParam.Velocity().Zeros();
        JointVelocityGetParam.SetValid(false);
    }
}

void mtsIntuitiveResearchKitArm::StateChanged(void)
{
    const std::string newState = mArmState.CurrentState();
    MessageEvents.CurrentState(newState);
    MessageEvents.Status(this->GetName() + ", current state " + newState);
}

void mtsIntuitiveResearchKitArm::RunAllStates(void)
{
    GetRobotData();

    // always allow to go to unitialized
    if (mArmState.DesiredStateIsNotCurrent()) {
        if (mArmState.DesiredState() == "UNINITIALIZED") {
            mArmState.SetCurrentState("UNINITIALIZED");
        }
    }
}

void mtsIntuitiveResearchKitArm::EnterUninitialized(void)
{
    RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfAxes(), 0.0));
    RobotIO.DisablePower();
    PID.Enable(false);
    PID.SetCheckJointLimit(true);
    // in case there was a trajectory going on
    if (mJointTrajectory.IsWorking) {
        mJointTrajectory.GoalReachedEvent(false);
    }
    SetControlSpace(UNDEFINED_SPACE);
    SetControlMode(UNDEFINED_MODE);
}

void mtsIntuitiveResearchKitArm::TransitionUninitialized(void)
{
    if (mArmState.DesiredStateIsNotCurrent()) {
        if (mHomedOnce) {
            mArmState.SetCurrentState("CALIBRATING_ENCODERS_FROM_POTS");
        } else {
            mArmState.SetCurrentState("POWERING");
        }
    }
}

void mtsIntuitiveResearchKitArm::EnterCalibratingEncodersFromPots(void)
{
    // if simulated, no need to bias encoders
    if (mIsSimulated) {
        mArmState.SetCurrentState("POWERING");
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
    const double currentTime = this->StateTable.GetTic();
    const double timeToBias = 30.0 * cmn_s; // large timeout
    if ((currentTime - mHomingTimer) > timeToBias) {
        mHomingBiasEncoderRequested = false;
        MessageEvents.Error(this->GetName() + " failed to bias encoders (timeout).");
        mArmState.SetCurrentState("UNINITIALIZED");
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
    if (mIsSimulated) {
        PID.EnableTrackingError(false);
        PID.Enable(true);
        vctDoubleVec goal(NumberOfJoints());
        goal.SetAll(0.0);
        SetPositionJointLocal(goal);
        mArmState.SetCurrentState("POWERED");
        return;
    }

    const double currentTime = this->StateTable.GetTic();

    // in case we still have power but brakes are not engaged
    if (NumberOfBrakes() > 0) {
        RobotIO.BrakeEngage();
    }
    // use pots for redundancy
    if (UsePotsForSafetyCheck()) {
        RobotIO.SetPotsToEncodersTolerance(PotsToEncodersTolerance);
        RobotIO.UsePotsForSafetyCheck(true);
    }
    mHomingTimer = currentTime;
    // make sure the PID is not sending currents
    PID.Enable(false);
    // pre-load the boards with zero current
    RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfAxes(), 0.0));
    // enable power and set a flags to move to next step
    RobotIO.EnablePower();
    MessageEvents.Status(this->GetName() + " power requested");
}

void mtsIntuitiveResearchKitArm::TransitionPowering(void)
{
    const double timeToPower = 3.0 * cmn_s;
    const double currentTime = this->StateTable.GetTic();

    // second, check status
    if ((currentTime - mHomingTimer) > timeToPower) {

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
            mArmState.SetCurrentState("POWERED");
        } else {
            MessageEvents.Error(this->GetName() + " failed to enable power.");
            mArmState.SetCurrentState("UNINITIALIZED");
        }
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

void mtsIntuitiveResearchKitArm::RunPositionJoint(void)
{
    if (mHasNewPIDGoal) {
        SetPositionJointLocal(JointSet);
        // reset flag
        mHasNewPIDGoal = false;
    }
}

void mtsIntuitiveResearchKitArm::RunPositionGoalJoint(void)
{
    // check if there's anything to do
    if (!mJointTrajectory.IsWorking) {
        return;
    }

    mJointTrajectory.Reflexxes.Evaluate(JointSet,
                                        JointVelocitySet,
                                        mJointTrajectory.Goal,
                                        mJointTrajectory.GoalVelocity);
    SetPositionJointLocal(JointSet);

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
        MessageEvents.Error(this->GetName() + " error while evaluating trajectory.");
        mJointTrajectory.IsWorking = false;
        break;
    }
}

void mtsIntuitiveResearchKitArm::RunPositionCartesian(void)
{
    if (mHasNewPIDGoal) {
        // copy current position
        vctDoubleVec jointSet(JointGet.Ref(NumberOfJointsKinematics()));

        // compute desired arm position
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
        mHasNewPIDGoal = false;
    }
}

void mtsIntuitiveResearchKitArm::RunPositionGoalCartesian(void)
{
    // trajectory are computed in joint space for now
    RunPositionGoalJoint();
}

void mtsIntuitiveResearchKitArm::SetControlSpace(const ControlSpace space)
{
    // ignore if already in the same space
    if (space == mControlSpace) {
        return;
    }

    mControlSpace = space;
}

void mtsIntuitiveResearchKitArm::SetControlMode(const ControlMode mode)
{
    // ignore if already in the same mode
    if (mode == mControlMode) {
        return;
    }

    // when starting trajectory, set current position/velocity and trajectory
    // parameters
    if (mode == TRAJECTORY_MODE) {
        JointSet.Assign(JointGetDesired, NumberOfJoints());
        JointVelocitySet.Assign(JointVelocityGet, NumberOfJoints());
        mJointTrajectory.Reflexxes.Set(mJointTrajectory.Velocity,
                                       mJointTrajectory.Acceleration,
                                       StateTable.PeriodStats.GetAvg(),
                                       robReflexxes::Reflexxes_TIME);
    }

    // set flag
    mControlMode = mode;
}

void mtsIntuitiveResearchKitArm::RunEffortJoint(void)
{
    // effort required
    JointExternalEffort.Assign(mEffortJointSet.ForceTorque());

    // add gravity compensation if needed
    if (mGravityCompensation) {
        AddGravityCompensationEfforts(JointExternalEffort);
    }

    // add custom efforts
    AddCustomEfforts(JointExternalEffort);

    // convert to cisstParameterTypes
    TorqueSetParam.SetForceTorque(JointExternalEffort);

    PID.SetTorqueJoint(TorqueSetParam);
}

void mtsIntuitiveResearchKitArm::RunEffortCartesian(void)
{
    // update torques based on wrench
    vctDoubleVec force(6);

    // body wrench
    if (mWrenchType == WRENCH_BODY) {
        if (mWrenchBodyOrientationAbsolute) {
            // use forward kinematics orientation to have constant wrench orientation
            vct3 relative, absolute;
            // force
            relative.Assign(mWrenchSet.Force().Ref<3>(0));
            CartesianGet.Rotation().ApplyInverseTo(relative, absolute);
            force.Ref(3, 0).Assign(absolute);
            // torque
            relative.Assign(mWrenchSet.Force().Ref<3>(3));
            CartesianGet.Rotation().ApplyInverseTo(relative, absolute);
            force.Ref(3, 3).Assign(absolute);
        } else {
            force.Assign(mWrenchSet.Force());
        }
        JointExternalEffort.ProductOf(JacobianBody.Transpose(), force);
    }
    // spatial wrench
    else if (mWrenchType == WRENCH_SPATIAL) {
        force.Assign(mWrenchSet.Force());
        JointExternalEffort.ProductOf(JacobianSpatial.Transpose(), force);
    }

    // add gravity compensation if needed
    if (mGravityCompensation) {
        AddGravityCompensationEfforts(JointExternalEffort);
    }

    // add custom efforts
    AddCustomEfforts(JointExternalEffort);

    // pad array for PID
    vctDoubleVec torqueDesired(NumberOfAxes(), 0.0); // for PID
    torqueDesired.Assign(JointExternalEffort, NumberOfJointsKinematics());

    // convert to cisstParameterTypes
    TorqueSetParam.SetForceTorque(torqueDesired);
    PID.SetTorqueJoint(TorqueSetParam);

    // lock orientation if needed
    if (mEffortOrientationLocked) {
        RunEffortOrientationLocked();
    }
}

void mtsIntuitiveResearchKitArm::RunEffortOrientationLocked(void)
{
    CMN_LOG_CLASS_RUN_ERROR << GetName()
                            << ": RunEffortOrientationLocked, this method should never be called, MTMs are the only arms able to lock orientation and the derived implementation of this method should be called."
                            << std::endl;
}

void mtsIntuitiveResearchKitArm::SetPositionJointLocal(const vctDoubleVec & newPosition)
{
    JointSetParam.Goal().Zeros();
    JointSetParam.Goal().Assign(newPosition, NumberOfJoints());
    PID.SetPositionJoint(JointSetParam);
}

void mtsIntuitiveResearchKitArm::SetPositionJoint(const prmPositionJointSet & newPosition)
{
    if ((mControlSpace == JOINT_SPACE)
        && (mControlMode == POSITION_MODE)) {
        JointSet.Assign(newPosition.Goal(), NumberOfJoints());
        mHasNewPIDGoal = true;
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": arm not in joint control mode, current state is "
                                  << mArmState.CurrentState() << std::endl;
    }
}

void mtsIntuitiveResearchKitArm::SetPositionGoalJoint(const prmPositionJointSet & newPosition)
{
    if ((mControlSpace == JOINT_SPACE)
        && (mControlMode == TRAJECTORY_MODE)) {
        mJointTrajectory.IsWorking = true;
        mJointTrajectory.Goal.Assign(newPosition.Goal(), NumberOfJoints());
        mJointTrajectory.GoalVelocity.SetAll(0.0);
        mJointTrajectory.EndTime = 0.0;
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": arm not in joint trajectory control mode, current state is "
                                  << mArmState.CurrentState() << std::endl;
    }
}

void mtsIntuitiveResearchKitArm::SetPositionCartesian(const prmPositionCartesianSet & newPosition)
{
    if ((mControlSpace == CARTESIAN_SPACE)
        && (mControlMode == POSITION_MODE)) {
        CartesianSetParam = newPosition;
        mHasNewPIDGoal = true;
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": arm not in cartesian control mode, current state is "
                                  << mArmState.CurrentState() << std::endl;
    }
}

void mtsIntuitiveResearchKitArm::SetPositionGoalCartesian(const prmPositionCartesianSet & newPosition)
{
    if ((mControlSpace == CARTESIAN_SPACE)
        && (mControlMode == TRAJECTORY_MODE)) {

        const size_t nbJoints = this->NumberOfJoints();
        const size_t nbJointsKin = this->NumberOfJointsKinematics();

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
                const robReflexxes::ResultType trajectoryResult = mJointTrajectory.Reflexxes.ResultValue();
                if (trajectoryResult != robReflexxes::Reflexxes_FINAL_STATE_REACHED) {
                    // past any trajectory, use last desired joint position
                    jointSet.Ref(nbJoints - nbJointsKin,
                                 nbJointsKin).Assign(JointGetDesired.Ref(nbJoints - nbJointsKin,
                                                                         nbJointsKin));
                } else {
                    // there is an ongoing trajectory, use the trajectory goal
                    jointSet.Ref(nbJoints - nbJointsKin,
                                 nbJointsKin).Assign(mJointTrajectory.Goal.Ref(nbJoints - nbJointsKin,
                                                                               nbJointsKin));
                }
            }
            // set joint goals
            mJointTrajectory.IsWorking = true;
            mJointTrajectory.Goal.Assign(jointSet);
            mJointTrajectory.GoalVelocity.SetAll(0.0);
            mJointTrajectory.EndTime = 0.0;
        } else {
            MessageEvents.Error(this->GetName() + " unable to solve inverse kinematics.");
            mJointTrajectory.GoalReachedEvent(false);
        }
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": arm not in cartesian trajectory control mode, current state is "
                                  << mArmState.CurrentState() << std::endl;
    }
}

void mtsIntuitiveResearchKitArm::SetBaseFrameEventHandler(const prmPositionCartesianGet & newBaseFrame)
{
    if (newBaseFrame.Valid()) {
        this->BaseFrame.FromNormalized(newBaseFrame.Position());
        this->BaseFrameValid = true;
    } else {
        this->BaseFrameValid = false;
    }
}

void mtsIntuitiveResearchKitArm::SetBaseFrame(const prmPositionCartesianSet & newBaseFrame)
{
    if (newBaseFrame.Valid()) {
        this->BaseFrame.FromNormalized(newBaseFrame.Goal());
        this->BaseFrameValid = true;
    } else {
        this->BaseFrameValid = false;
    }
}

void mtsIntuitiveResearchKitArm::ErrorEventHandler(const std::string & message)
{
    MessageEvents.Error(this->GetName() + ": received [" + message + "]");
    mArmState.SetCurrentState("UNINITIALIZED");
}

void mtsIntuitiveResearchKitArm::JointLimitEventHandler(const vctBoolVec & CMN_UNUSED(flags))
{
    MessageEvents.Warning(this->GetName() + ": PID joint limit");
}

void mtsIntuitiveResearchKitArm::BiasEncoderEventHandler(const int & nbSamples)
{
    std::stringstream nbSamplesString;
    nbSamplesString << nbSamples;
    MessageEvents.Status(this->GetName() + ": encoders biased using " + nbSamplesString.str() + " potentiometer values");
    if (mHomingBiasEncoderRequested) {
        mHomingBiasEncoderRequested = false;
        mArmState.SetCurrentState("ENCODERS_BIASED");
    } else {
        MessageEvents.Status(this->GetName() + " encoders have been biased by another process");
    }
}

void mtsIntuitiveResearchKitArm::SetEffortJoint(const prmForceTorqueJointSet & effort)
{
    if ((mControlSpace == JOINT_SPACE)
        && (mControlMode == EFFORT_MODE)) {
        mEffortJointSet.ForceTorque().Assign(effort.ForceTorque());
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": arm not in joint effort control mode, current state is "
                                  << mArmState.CurrentState() << std::endl;
    }
}

void mtsIntuitiveResearchKitArm::SetWrenchBody(const prmForceCartesianSet & wrench)
{
    if ((mControlSpace == CARTESIAN_SPACE)
        && (mControlMode == EFFORT_MODE)) {
        mWrenchSet = wrench;
        if (mWrenchType != WRENCH_BODY) {
            mWrenchType = WRENCH_BODY;
            MessageEvents.Status(this->GetName() + " effort cartesian (body)");
        }
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": arm not in cartesian effort control mode, current state is "
                                  << mArmState.CurrentState() << std::endl;
    }
}

void mtsIntuitiveResearchKitArm::SetWrenchSpatial(const prmForceCartesianSet & wrench)
{
    if ((mControlSpace == CARTESIAN_SPACE)
        && (mControlMode == EFFORT_MODE)) {
        mWrenchSet = wrench;
        if (mWrenchType != WRENCH_SPATIAL) {
            mWrenchType = WRENCH_SPATIAL;
            MessageEvents.Status(this->GetName() + " effort cartesian (spatial)");
        }
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": arm not in cartesian effort control mode, current state is "
                                  << mArmState.CurrentState() << std::endl;
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
    gravityEfforts.ForceAssign(Manipulator.CCG(JointGet, qd));  // should this take joint velocities?
    efforts.Add(gravityEfforts);
}
