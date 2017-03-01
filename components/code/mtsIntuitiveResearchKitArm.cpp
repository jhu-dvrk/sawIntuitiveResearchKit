/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen, Zerui Wang
  Created on: 2016-02-24

  (C) Copyright 2013-2017 Johns Hopkins University (JHU), All Rights Reserved.

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
    mIsSimulated = false;
    HomedOnce = false;
    HomingGoesToZero = false; // MTM ignores this

    mWrenchBodyOrientationAbsolute = false;
    mGravityCompensation = false;

    IsGoalSet = false;
    EffortOrientationLocked = false;

    // initialize trajectory data
    JointGet.SetSize(NumberOfJoints());
    JointSet.SetSize(NumberOfJoints());
    JointVelocityGet.SetSize(NumberOfJoints());
    JointVelocitySet.SetSize(NumberOfJoints());
    JointSetParam.Goal().SetSize(NumberOfAxes());
    JointTrajectory.Velocity.SetSize(NumberOfJoints());
    JointTrajectory.Acceleration.SetSize(NumberOfJoints());
    JointTrajectory.Goal.SetSize(NumberOfJoints());
    JointTrajectory.GoalVelocity.SetSize(NumberOfJoints());
    JointTrajectory.GoalError.SetSize(NumberOfJoints());
    JointTrajectory.GoalTolerance.SetSize(NumberOfJoints());
    JointTrajectory.IsUsed = false;
    JointTrajectory.IsWorking = false;
    PotsToEncodersTolerance.SetSize(NumberOfAxes());

    // initialize velocity
    CartesianVelocityGetParam.SetVelocityLinear(vct3(0.0));
    CartesianVelocityGetParam.SetVelocityAngular(vct3(0.0));
    CartesianVelocityGetParam.SetValid(false);

    // jacobian
    mJacobianBody.SetSize(6, NumberOfJointsKinematics());
    this->StateTable.AddData(mJacobianBody, "JacobianBody");
    mJacobianSpatial.SetSize(6, NumberOfJointsKinematics());
    this->StateTable.AddData(mJacobianSpatial, "JacobianSpatial");
    mJacobianBodyTranspose.ForceAssign(mJacobianBody.Transpose());
    mJacobianPInverseData.Allocate(mJacobianBodyTranspose);
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
        RobotInterface->AddMessageEvents();

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
        RobotInterface->AddCommandReadState(this->StateTable, mJacobianBody, "GetJacobianBody");
        RobotInterface->AddCommandReadState(this->StateTable, mJacobianSpatial, "GetJacobianSpatial");
        // Set
        RobotInterface->AddCommandVoid(&mtsIntuitiveResearchKitArm::Freeze,
                                       this, "Freeze");
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
        RobotInterface->AddEventWrite(JointTrajectory.GoalReachedEvent, "GoalReached", bool());
        // Robot State
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetRobotControlState,
                                        this, "SetRobotControlState", std::string(""));
        RobotInterface->AddCommandRead(&mtsIntuitiveResearchKitArm::GetRobotControlState,
                                       this, "GetRobotControlState", std::string(""));
        // Human readable messages
        RobotInterface->AddEventWrite(MessageEvents.RobotState, "RobotState", std::string(""));

        // Stats
        RobotInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                            "GetPeriodStatistics");
    }

    // SetState will send log events, it needs to happen after the
    // provided interface has been created
    SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
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
    this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
}

void mtsIntuitiveResearchKitArm::Run(void)
{
    ProcessQueuedEvents();
    GetRobotData();

    switch (RobotState) {
    case mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED:
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_BIAS_ENCODER:
        RunHomingBiasEncoder();
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
    case mtsIntuitiveResearchKitArmTypes::DVRK_EFFORT_JOINT:
        RunEffortJoint();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_EFFORT_CARTESIAN:
        RunEffortCartesian();
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

void mtsIntuitiveResearchKitArm::SetSimulated(void)
{
    mIsSimulated = true;
    // in simulation mode, we don't need IO
    RemoveInterfaceRequired("RobotIO");
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
                              << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(state)
                              << ", current state is " << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(RobotState) << std::endl;
    return false;
}

bool mtsIntuitiveResearchKitArm::CurrentStateIs(const mtsIntuitiveResearchKitArmTypes::RobotStateType & state1,
                                                const mtsIntuitiveResearchKitArmTypes::RobotStateType & state2)
{
    if ((RobotState == state1) || (RobotState == state2)) {
        return true;
    }
    CMN_LOG_CLASS_RUN_WARNING << GetName() << ": Checking state: arm not in "
                              << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(state1)
                              << " nor "
                              << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(state2)
                              << ", current state is " << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(RobotState) << std::endl;
    return false;
}

void mtsIntuitiveResearchKitArm::GetRobotData(void)
{
    // we can start reporting some joint values after the robot is powered
    if (this->RobotState > mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_BIAS_ENCODER) {
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
            CartesianGetLocal = this->ForwardKinematics(JointGet, NumberOfJointsKinematics());
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
            this->JacobianSpatial(JointGet, mJacobianSpatial);
            this->JacobianBody(JointGet, mJacobianBody);

            // update cartesian velocity using the jacobian and joint
            // velocities.
            vctDoubleVec cartesianVelocity(6);
            cartesianVelocity.ProductOf(mJacobianBody,
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
            mJacobianBodyTranspose.Assign(mJacobianBody.Transpose());
            nmrPInverse(mJacobianBodyTranspose, mJacobianPInverseData);
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
            CartesianGetLocalDesired = this->ForwardKinematics(JointGetDesired);
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

void mtsIntuitiveResearchKitArm::RunHomingBiasEncoder(void)
{
    // if simulated, no need to bias encoders
    if (mIsSimulated || HomedOnce) {
        this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_POWERING);
        return;
    }

    // first, request bias encoder
    const double currentTime = this->StateTable.GetTic();
    const double timeToBias = 30.0 * cmn_s; // large timeout

    // first, request bias encoder
    if (!HomingBiasEncoderRequested) {
        RobotIO.BiasEncoder(1970); // birth date, state table only contain 1999 elements anyway
        HomingBiasEncoderRequested = true;
        HomingTimer = currentTime;
        return;
    }

    // second, check status
    if ((currentTime - HomingTimer) > timeToBias) {
        HomingBiasEncoderRequested = false;
        RobotInterface->SendError(this->GetName() + " failed to bias encoders (timeout).");
        this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
    }
}

void mtsIntuitiveResearchKitArm::RunHomingPower(void)
{
    if (mIsSimulated) {
        PID.EnableTrackingError(false);
        PID.Enable(true);
        vctDoubleVec goal(NumberOfJoints());
        goal.SetAll(0.0);
        SetPositionJointLocal(goal);
        this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_CALIBRATING_ARM);
        return;
    }

    const double timeToPower = 3.0 * cmn_s;

    const double currentTime = this->StateTable.GetTic();
    // first, request power to be turned on
    if (!HomingPowerRequested) {
        // in case we still have power but brakes are not engaged
        if (NumberOfBrakes() > 0) {
            RobotIO.BrakeEngage();
        }
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
        RobotInterface->SendStatus(this->GetName() + " power requested");
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
            RobotInterface->SendStatus(this->GetName() + " power on");
            if (NumberOfBrakes() > 0) {
                RobotIO.BrakeRelease();
            }
            this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_CALIBRATING_ARM);
        } else {
            // make sure the PID is not sending currents
            PID.Enable(false);
            RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfAxes(), 0.0));
            RobotIO.DisablePower();
            RobotInterface->SendError(this->GetName() + " failed to enable power.");
            this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
        }
    }
}

void mtsIntuitiveResearchKitArm::RunPositionJoint(void)
{
    if (IsGoalSet) {
        SetPositionJointLocal(JointSet);
        // reset flag
        IsGoalSet = false;
    }
}

void mtsIntuitiveResearchKitArm::RunPositionGoalJoint(void)
{
    // check if there's anything to do
    if (!JointTrajectory.IsWorking) {
        return;
    }

    JointTrajectory.Reflexxes.Evaluate(JointSet,
                                       JointVelocitySet,
                                       JointTrajectory.Goal,
                                       JointTrajectory.GoalVelocity);
    SetPositionJointLocal(JointSet);

    const robReflexxes::ResultType trajectoryResult = JointTrajectory.Reflexxes.ResultValue();
    const double currentTime = this->StateTable.GetTic();

    switch (trajectoryResult) {
    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (JointTrajectory.EndTime == 0.0) {
            JointTrajectory.EndTime = currentTime + JointTrajectory.Reflexxes.Duration();
        }
        break;
    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        JointTrajectory.GoalReachedEvent(true);
        JointTrajectory.IsWorking = false;
        break;
    default:
        RobotInterface->SendError(this->GetName() + " error while evaluating trajectory.");
        JointTrajectory.IsWorking = false;
        break;
    }
}

void mtsIntuitiveResearchKitArm::RunPositionCartesian(void)
{
    if (IsGoalSet) {
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
            RobotInterface->SendError(this->GetName() + " unable to solve inverse kinematics.");
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

void mtsIntuitiveResearchKitArm::TrajectoryIsUsed(const bool used)
{
    // ignore if the trajectory is already in use
    if (used == JointTrajectory.IsUsed) {
        return;
    }

    // when starting, set current position/velocity and trajectory
    // parameters
    if (used) {
        JointSet.Assign(JointGetDesired, NumberOfJoints());
        JointVelocitySet.Assign(JointVelocityGet, NumberOfJoints());
        JointTrajectory.Reflexxes.Set(JointTrajectory.Velocity,
                                      JointTrajectory.Acceleration,
                                      StateTable.PeriodStats.GetAvg(),
                                      robReflexxes::Reflexxes_TIME);
    }

    // set flag
    JointTrajectory.IsUsed = used;
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
        JointExternalEffort.ProductOf(mJacobianBody.Transpose(), force);
    }
    // spatial wrench
    else if (mWrenchType == WRENCH_SPATIAL) {
        force.Assign(mWrenchSet.Force());
        JointExternalEffort.ProductOf(mJacobianSpatial.Transpose(), force);
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
    if (EffortOrientationLocked) {
        RunEffortOrientationLocked();
    }
}

void mtsIntuitiveResearchKitArm::RunEffortOrientationLocked(void)
{
    CMN_LOG_CLASS_RUN_ERROR << GetName()
                            << ": RunEffortOrientationLocked, this should never happen, MTMs are the only arms able to lock orientation and the derived implementation of this method should be called."
                            << std::endl;
}

void mtsIntuitiveResearchKitArm::SetPositionJointLocal(const vctDoubleVec & newPosition)
{
    JointSetParam.Goal().Zeros();
    JointSetParam.Goal().Assign(newPosition, NumberOfJoints());
    PID.SetPositionJoint(JointSetParam);
}

void mtsIntuitiveResearchKitArm::Freeze(void)
{
    SetState(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_JOINT);
    SetPositionJointLocal(JointGetDesired);
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
        JointTrajectory.IsWorking = true;
        JointTrajectory.Goal.Assign(newPosition.Goal(), NumberOfJoints());
        JointTrajectory.GoalVelocity.SetAll(0.0);
        JointTrajectory.EndTime = 0.0;
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
                const robReflexxes::ResultType trajectoryResult = JointTrajectory.Reflexxes.ResultValue();
                if (trajectoryResult != robReflexxes::Reflexxes_FINAL_STATE_REACHED) {
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
            // set joint goals
            JointTrajectory.IsWorking = true;
            JointTrajectory.Goal.Assign(jointSet);
            JointTrajectory.GoalVelocity.SetAll(0.0);
            JointTrajectory.EndTime = 0.0;
        } else {
            RobotInterface->SendError(this->GetName() + " unable to solve inverse kinematics.");
            JointTrajectory.GoalReachedEvent(false);
        }
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

void mtsIntuitiveResearchKitArm::ErrorEventHandler(const mtsMessage & message)
{
    RobotIO.DisablePower();
    // in case there was a trajectory going on
    if (JointTrajectory.IsWorking) {
        JointTrajectory.GoalReachedEvent(false);
    }

    RobotInterface->SendError(this->GetName() + ": received [" + message.Message + "]");
    SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
}

void mtsIntuitiveResearchKitArm::JointLimitEventHandler(const vctBoolVec & CMN_UNUSED(flags))
{
    RobotInterface->SendWarning(this->GetName() + ": PID joint limit");
}

void mtsIntuitiveResearchKitArm::BiasEncoderEventHandler(const int & nbSamples)
{
    std::stringstream nbSamplesString;
    nbSamplesString << nbSamples;
    RobotInterface->SendStatus(this->GetName() + ": encoders biased using " + nbSamplesString.str() + " potentiometer values");
    if (HomingBiasEncoderRequested) {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_POWERING);
    } else {
        RobotInterface->SendStatus(this->GetName() + " encoders have been biased by another process");
    }
}

void mtsIntuitiveResearchKitArm::SetEffortJoint(const prmForceTorqueJointSet & effort)
{
    if (CurrentStateIs(mtsIntuitiveResearchKitArmTypes::DVRK_EFFORT_JOINT)) {
        mEffortJointSet.ForceTorque().Assign(effort.ForceTorque());
    }
}

void mtsIntuitiveResearchKitArm::SetWrenchBody(const prmForceCartesianSet & wrench)
{
    if (CurrentStateIs(mtsIntuitiveResearchKitArmTypes::DVRK_EFFORT_CARTESIAN,
                       mtsIntuitiveResearchKitArmTypes::DVRK_EFFORT_CARTESIAN_IMPEDANCE)) {
        mWrenchSet = wrench;
        if (mWrenchType != WRENCH_BODY) {
            mWrenchType = WRENCH_BODY;
            RobotInterface->SendStatus(this->GetName() + " effort cartesian (body)");
        }
    }
}

void mtsIntuitiveResearchKitArm::SetWrenchSpatial(const prmForceCartesianSet & wrench)
{
    if (CurrentStateIs(mtsIntuitiveResearchKitArmTypes::DVRK_EFFORT_CARTESIAN,
                       mtsIntuitiveResearchKitArmTypes::DVRK_EFFORT_CARTESIAN_IMPEDANCE)) {
        mWrenchSet = wrench;
        if (mWrenchType != WRENCH_SPATIAL) {
            mWrenchType = WRENCH_SPATIAL;
            RobotInterface->SendStatus(this->GetName() + " effort cartesian (spatial)");
        }
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
