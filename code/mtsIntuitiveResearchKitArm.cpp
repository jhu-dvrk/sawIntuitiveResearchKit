/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

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
    IsGoalSet = false;
    SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);

    // initialize trajectory data
    JointGet.SetSize(NumberOfJoints());
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

    // cartesian position are timestamped using timestamps provided by PID
    CartesianGetParam.SetAutomaticTimestamp(false);
    CartesianGetDesiredParam.SetAutomaticTimestamp(false);
    this->StateTable.AddData(CartesianGetParam, "CartesianPosition");
    this->StateTable.AddData(CartesianGetDesiredParam, "CartesianDesired");
    this->StateTable.AddData(JointGetParam, "JointPosition");

    // setup CISST Interface
    PIDInterface = AddInterfaceRequired("PID");
    if (PIDInterface) {
        PIDInterface->AddFunction("Enable", PID.Enable);
        PIDInterface->AddFunction("GetPositionJoint", PID.GetPositionJoint);
        PIDInterface->AddFunction("GetPositionJointDesired", PID.GetPositionJointDesired);
        PIDInterface->AddFunction("SetPositionJoint", PID.SetPositionJoint);
        PIDInterface->AddFunction("SetCheckJointLimit", PID.SetCheckJointLimit);
        PIDInterface->AddFunction("EnableTorqueMode", PID.EnableTorqueMode);
        PIDInterface->AddFunction("SetTorqueJoint", PID.SetTorqueJoint);
        PIDInterface->AddFunction("SetTorqueOffset", PID.SetTorqueOffset);
        PIDInterface->AddFunction("EnableTrackingError", PID.EnableTrackingError);
        PIDInterface->AddFunction("SetTrackingErrorTolerances", PID.SetTrackingErrorTolerance);
        PIDInterface->AddEventHandlerVoid(&mtsIntuitiveResearchKitArm::EventHandlerTrackingError, this, "TrackingError");
    }

    // Robot IO
    IOInterface = AddInterfaceRequired("RobotIO");
    if (IOInterface) {
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

    RobotInterface = AddInterfaceProvided("Robot");
    if (RobotInterface) {
        // Get
        RobotInterface->AddCommandReadState(this->StateTable, JointGetParam, "GetPositionJoint");
        RobotInterface->AddCommandReadState(this->StateTable, JointGetDesired, "GetPositionJointDesired");
        RobotInterface->AddCommandReadState(this->StateTable, CartesianGetParam, "GetPositionCartesian");
        RobotInterface->AddCommandReadState(this->StateTable, CartesianGetDesiredParam, "GetPositionCartesianDesired");
        // Set
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionJoint, this, "SetPositionJoint");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionGoalJoint, this, "SetPositionGoalJoint");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionCartesian, this, "SetPositionCartesian");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetPositionGoalCartesian, this, "SetPositionGoalCartesian");
        // Robot State
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetRobotControlState,
                                           this, "SetRobotControlState", std::string(""));
        RobotInterface->AddCommandRead(&mtsIntuitiveResearchKitArm::GetRobotControlState,
                                          this, "GetRobotControlState", std::string(""));
        RobotInterface->AddEventWrite(MessageEvents.RobotStatus, "RobotStatusMsg", std::string(""));
        RobotInterface->AddEventWrite(MessageEvents.RobotError, "RobotErrorMsg", std::string(""));
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

    CartesianGetPreviousParam = CartesianGetParam;
}

void mtsIntuitiveResearchKitArm::Cleanup(void)
{
    if (NumberOfBrakes() > 0) {
        RobotIO.BrakeEngage();
    }
    CMN_LOG_CLASS_INIT_VERBOSE << GetName() << ": Cleanup" << std::endl;
}

void mtsIntuitiveResearchKitArm::SetRobotControlState(const std::string & state)
{
    if (state == "Home") {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_POWERING);
    } else if ((state == "Cartesian position") || (state == "Teleop")) {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_CARTESIAN);
    } else if (state == "Manual") {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_MANUAL);
    } else {
        MessageEvents.RobotError(this->GetName() + ": unsupported state " + state);
    }
}

void mtsIntuitiveResearchKitArm::GetRobotControlState(std::string & state) const
{
    state = mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(this->RobotState);
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

        // when the robot is ready, we can compute cartesian position
        if (this->RobotState >= mtsIntuitiveResearchKitArmTypes::DVRK_READY) {
            // update cartesian position
            CartesianGet = Manipulator.ForwardKinematics(JointGet);
            CartesianGet.Rotation().NormalizedSelf();
            CartesianGetParam.SetTimestamp(JointGetParam.Timestamp());
            CartesianGetParam.SetValid(true);
            // update cartesian velocity (caveat, velocities are not updated)
            vct3 linearVelocity;
            linearVelocity.DifferenceOf(CartesianGetParam.Position().Translation(),
                                        CartesianGetPreviousParam.Position().Translation());
            linearVelocity.Divide(CartesianGetParam.Timestamp() - CartesianGetPreviousParam.Timestamp());
            CartesianVelocityGetParam.SetVelocityLinear(linearVelocity);
            CartesianVelocityGetParam.SetVelocityAngular(vct3(0.0));
            CartesianVelocityGetParam.SetTimestamp(CartesianGetParam.Timestamp());
            CartesianVelocityGetParam.SetValid(true);
            // update cartesian position desired based on joint desired
            CartesianGetDesired = Manipulator.ForwardKinematics(JointGetDesired);
            CartesianGetDesired.Rotation().NormalizedSelf();
            CartesianGetDesiredParam.SetTimestamp(JointGetParam.Timestamp());
            CartesianGetDesiredParam.SetValid(true);
        } else {
            // update cartesian position
            CartesianGet.Assign(vctFrm4x4::Identity());
            CartesianGetParam.SetValid(false);
            // update cartesian position desired
            CartesianGetDesired.Assign(vctFrm4x4::Identity());
            CartesianGetDesiredParam.SetValid(false);
        }
        CartesianGetParam.Position().From(CartesianGet);
        CartesianGetDesiredParam.Position().From(CartesianGetDesired);
    } else {
        // set joint to zeros
        JointGet.Zeros();
        JointGetParam.Position().Zeros();
        JointGetParam.SetValid(false);
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
        RobotIO.BiasEncoder();
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
        MessageEvents.RobotStatus(this->GetName() + " power requested");
        return;
    }

    // second, check status
    if (HomingPowerRequested
        && ((currentTime - HomingTimer) > timeToPower)) {

        // pre-load PID to make sure desired position has some reasonable values
        PID.GetPositionJoint(JointGetParam);
        // angles are radians but cisst uses mm.   robManipulator uses SI, so we need meters
        JointGetParam.Position().Element(2) /= 1000.0;  // convert from mm to m
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
            MessageEvents.RobotStatus(this->GetName() + " power on");
            if (NumberOfBrakes() > 0) {
                RobotIO.BrakeRelease();
            }
            this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_CALIBRATING_ARM);
        } else {
            MessageEvents.RobotError(this->GetName() + " failed to enable power.");
            this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
        }
    }
}

void mtsIntuitiveResearchKitArm::RunPositionJoint(void)
{
    if (IsGoalSet == true) {
        SetPositionJointLocal(JointSet);
    }
}

void mtsIntuitiveResearchKitArm::RunPositionGoalJoint(void)
{
    const double currentTime = this->StateTable.GetTic();
    if (currentTime <= JointTrajectory.EndTime) {
        JointTrajectory.LSPB.Evaluate(currentTime, JointSet);
        SetPositionJointLocal(JointSet);
    }
}

void mtsIntuitiveResearchKitArm::RunPositionCartesian(void)
{
    //! \todo: should prevent user to go to close to RCM!

    if (IsGoalSet == true) {
        vctDoubleVec jointSet(JointGet);

        // compute desired slave position
        CartesianPositionFrm.From(CartesianSetParam.Goal());
        Manipulator.InverseKinematics(jointSet, CartesianPositionFrm);
        jointSet.resize(4);

        // find closest solution mod 2 pi
        std::cerr << CMN_LOG_DETAILS << " ---- this is PSM/ECM specific.  Should apply to all rotation joints.  robManipulator should do that." << std::endl;
        const double difference = JointGet[3] - jointSet[3];
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        jointSet[3] = jointSet[3] + differenceInTurns * 2.0 * cmnPI;

        // finally send new joint values
        SetPositionJointLocal(jointSet);

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
    JointSetParam.Goal().Assign(newPosition, NumberOfJoints());
    PID.SetPositionJoint(JointSetParam);
}

void mtsIntuitiveResearchKitArm::SetPositionJoint(const prmPositionJointSet & newPosition)
{
    if (RobotState == mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_JOINT) {
        JointSet.Assign(newPosition.Goal());
        IsGoalSet = true;
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": SetPositionJoint: arm not in "
                                  << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_JOINT)
                                  << ", current state is " << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(RobotState) << std::endl;
    }
}

void mtsIntuitiveResearchKitArm::SetPositionGoalJoint(const prmPositionJointSet & newPosition)
{
    if (RobotState == mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_JOINT) {
        const double currentTime = this->StateTable.GetTic();
        // starting point is last requested to PID component
        JointTrajectory.Start.Assign(JointGet);
        JointTrajectory.Goal.Assign(newPosition.Goal());
        JointTrajectory.LSPB.Set(JointTrajectory.Start, JointTrajectory.Goal,
                                 JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                 currentTime, robLSPB::LSPB_DURATION);
        JointTrajectory.EndTime = currentTime + JointTrajectory.LSPB.Duration();
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": SetPositionGoalJoint: arm not in "
                                  << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_JOINT)
                                  << ", current state is " << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(RobotState) << std::endl;
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
    if (RobotState == mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN) {
        std::cerr << CMN_LOG_DETAILS << " ------------ needs to be implemented " << std::endl;
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": SetPositionGoalJoint: arm not in "
                                  << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(mtsIntuitiveResearchKitArmTypes::DVRK_POSITION_GOAL_CARTESIAN)
                                  << ", current state is " << mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(RobotState) << std::endl;
    }
}

void mtsIntuitiveResearchKitArm::EventHandlerTrackingError(void)
{
    RobotIO.DisablePower();
    MessageEvents.RobotError(this->GetName() + ": PID tracking error");
    SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
}
