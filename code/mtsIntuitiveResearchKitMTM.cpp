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
#include <cmath>
#include <iostream>

// cisst
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitMTM.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstNumerical/nmrLSMinNorm.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitMTM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsIntuitiveResearchKitMTM::mtsIntuitiveResearchKitMTM(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    Init();
}

mtsIntuitiveResearchKitMTM::mtsIntuitiveResearchKitMTM(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg)
{
    Init();
}

void mtsIntuitiveResearchKitMTM::Init(void)
{
    SetState(mtsIntuitiveResearchKitMTMTypes::MTM_UNINITIALIZED);
    RobotType = MTM_NULL; SetMTMType();

    // initialize gripper state
    GripperClosed = false;

    // initialize trajectory data
    JointGet.SetSize(NumberOfJoints);
    JointSet.SetSize(NumberOfJoints);
    JointSetParam.Goal().SetSize(NumberOfJoints + 1); // PID treats gripper as joint
    JointTrajectory.Velocity.SetSize(NumberOfJoints);
    JointTrajectory.Velocity.SetAll(720.0 * cmnPI_180); // degrees per second
    JointTrajectory.Acceleration.SetSize(NumberOfJoints);
    JointTrajectory.Acceleration.SetAll(720.0 * cmnPI_180);
    JointTrajectory.Start.SetSize(NumberOfJoints);
    JointTrajectory.Goal.SetSize(NumberOfJoints);
    JointTrajectory.GoalError.SetSize(NumberOfJoints);
    JointTrajectory.GoalTolerance.SetSize(NumberOfJoints);
    JointTrajectory.GoalTolerance.SetAll(3.0 * cmnPI / 180.0); // hard coded to 3 degrees
    JointTrajectory.EndTime = 0.0;

    this->StateTable.AddData(CartesianGetParam, "CartesianPosition");
    this->StateTable.AddData(CartesianGetDesiredParam, "CartesianDesired");
    this->StateTable.AddData(JointGetParam, "JointPosition");
    this->StateTable.AddData(GripperPosition, "GripperAngle");

    // setup cisst interfaces
    mtsInterfaceRequired * interfaceRequired;
    interfaceRequired = AddInterfaceRequired("PID");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("Enable", PID.Enable);
        interfaceRequired->AddFunction("EnableTorqueMode", PID.EnableTorqueMode);
        interfaceRequired->AddFunction("GetPositionJoint", PID.GetPositionJoint);
        interfaceRequired->AddFunction("GetPositionJointDesired", PID.GetPositionJointDesired);
        interfaceRequired->AddFunction("SetPositionJoint", PID.SetPositionJoint);
        interfaceRequired->AddFunction("SetTorqueJoint", PID.SetTorqueJoint);
        interfaceRequired->AddFunction("SetCheckJointLimit", PID.SetCheckJointLimit);
        interfaceRequired->AddFunction("SetTorqueOffset", PID.SetTorqueOffset);
    }

    // Robot IO
    interfaceRequired = AddInterfaceRequired("RobotIO");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("EnablePower", RobotIO.EnablePower);
        interfaceRequired->AddFunction("DisablePower", RobotIO.DisablePower);
        interfaceRequired->AddFunction("GetActuatorAmpStatus", RobotIO.GetActuatorAmpStatus);
        interfaceRequired->AddFunction("BiasEncoder", RobotIO.BiasEncoder);
        interfaceRequired->AddFunction("SetActuatorCurrent", RobotIO.SetActuatorCurrent);
        interfaceRequired->AddFunction("UsePotsForSafetyCheck", RobotIO.UsePotsForSafetyCheck);
        interfaceRequired->AddFunction("SetPotsToEncodersTolerance", RobotIO.SetPotsToEncodersTolerance);
        interfaceRequired->AddFunction("ResetSingleEncoder", RobotIO.ResetSingleEncoder);
        interfaceRequired->AddFunction("GetAnalogInputPosSI", RobotIO.GetAnalogInputPosSI);
    }

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Robot");
    if (interfaceProvided) {
        // Cartesian
        interfaceProvided->AddCommandReadState(this->StateTable, JointGetParam, "GetPositionJoint");
        interfaceProvided->AddCommandReadState(this->StateTable, CartesianGetParam, "GetPositionCartesian");
        interfaceProvided->AddCommandReadState(this->StateTable, CartesianGetDesiredParam, "GetPositionCartesianDesired");
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitMTM::SetPositionCartesian, this, "SetPositionCartesian");
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitMTM::SetWrench, this, "SetWrench");
        // Gripper
        interfaceProvided->AddCommandReadState(this->StateTable, GripperPosition, "GetGripperPosition");
        interfaceProvided->AddEventVoid(EventTriggers.GripperPinch, "GripperPinchEvent");
        interfaceProvided->AddEventWrite(EventTriggers.GripperClosed, "GripperClosedEvent", true);
        // Robot State
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitMTM::SetRobotControlState,
                                           this, "SetRobotControlState", std::string(""));
        interfaceProvided->AddCommandRead(&mtsIntuitiveResearchKitMTM::GetRobotControlState,
                                          this, "GetRobotControlState", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.RobotStatusMsg, "RobotStatusMsg", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.RobotErrorMsg, "RobotErrorMsg", std::string(""));
        // Stats
        interfaceProvided->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                               "GetPeriodStatistics");
    }
}

void mtsIntuitiveResearchKitMTM::Configure(const std::string & filename)
{
    robManipulator::Errno result;
    result = this->Manipulator.LoadRobot(filename);
    if (result == robManipulator::EFAILURE) {
        CMN_LOG_CLASS_INIT_ERROR << GetName() << ": Configure: failed to load manipulator configuration file \""
                                 << filename << "\"" << std::endl;
        return;
    }
}

void mtsIntuitiveResearchKitMTM::Startup(void)
{
    this->SetState(mtsIntuitiveResearchKitMTMTypes::MTM_UNINITIALIZED);
}

void mtsIntuitiveResearchKitMTM::Run(void)
{
    ProcessQueuedEvents();
    GetRobotData();

    switch (RobotState) {
    case mtsIntuitiveResearchKitMTMTypes::MTM_UNINITIALIZED:
        break;
    case mtsIntuitiveResearchKitMTMTypes::MTM_HOMING_POWERING:
        RunHomingPower();
        break;
    case mtsIntuitiveResearchKitMTMTypes::MTM_HOMING_CALIBRATING_ARM:
        RunHomingCalibrateArm();
        break;
    case mtsIntuitiveResearchKitMTMTypes::MTM_HOMING_CALIBRATING_ROLL:
        RunHomingCalibrateRoll();
        break;
    case mtsIntuitiveResearchKitMTMTypes::MTM_READY:
        break;
    case mtsIntuitiveResearchKitMTMTypes::MTM_POSITION_CARTESIAN:
        RunPositionCartesian();
        break;
    case mtsIntuitiveResearchKitMTMTypes::MTM_GRAVITY_COMPENSATION:
        RunGravityCompensation();
        break;
    case mtsIntuitiveResearchKitMTMTypes::MTM_CLUTCH:
        RunClutch();
    default:
        break;
    }

    RunEvent();
    ProcessQueuedCommands();

    // update previous value
    CartesianGetPrevious = CartesianGet;
}

void mtsIntuitiveResearchKitMTM::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << GetName() << ": Cleanup" << std::endl;
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
    // we can start reporting some joint values after the robot is powered
    if (this->RobotState > mtsIntuitiveResearchKitMTMTypes::MTM_HOMING_POWERING) {
        mtsExecutionResult executionResult;
        executionResult = PID.GetPositionJoint(JointGetParam);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to GetJointPosition failed \""
                                    << executionResult << "\"" << std::endl;
            return;
        }
        // the lower level report 8 joints, we need 7 only
        JointGet.Assign(JointGetParam.Position(), NumberOfJoints);
        JointGetParam.SetValid(true);

        // desired joints
        executionResult = PID.GetPositionJointDesired(JointGetDesired);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to GetJointPositionDesired failed \""
                                    << executionResult << "\"" << std::endl;
        }

        // when the robot is ready, we can comput cartesian position
        if (this->RobotState >= mtsIntuitiveResearchKitMTMTypes::MTM_READY) {
            // update cartesian position
            CartesianGet = Manipulator.ForwardKinematics(JointGet);
            CartesianGet.Rotation().NormalizedSelf();
            CartesianGetParam.SetValid(true);
            // update cartesian position desired based on joint desired
            CartesianGetDesired = Manipulator.ForwardKinematics(JointGetDesired);
            CartesianGetDesired.Rotation().NormalizedSelf();
            CartesianGetDesiredParam.SetValid(true);
            // update velocities
            CartesianVelocityLinear = (CartesianGet.Translation() - CartesianGetPrevious.Translation()).Divide(StateTable.Period);
            CartesianVelocityAngular.SetAll(0.0);
            CartesianVelocityParam.SetVelocityLinear(CartesianVelocityLinear);
            CartesianVelocityParam.SetVelocityAngular(CartesianVelocityAngular);
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

        // get gripper based on analog inputs
        executionResult = RobotIO.GetAnalogInputPosSI(AnalogInputPosSI);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to GetAnalogInputPosSI failed \""
                                    << executionResult << "\"" << std::endl;
            return;
        }
        GripperPosition = AnalogInputPosSI.Element(7);
        if (GripperClosed) {
            if (GripperPosition > 0.0) {
                GripperClosed = false;
                EventTriggers.GripperClosed(false);
            }
        } else {
            if (GripperPosition < 0.0) {
                GripperClosed = true;
                EventTriggers.GripperClosed(true);
                EventTriggers.GripperPinch.Execute();
            }
        }
    } else {
        // set joint to zeros
        JointGet.Zeros();
        JointGetParam.Position().Zeros();
        JointGetParam.SetValid(false);
    }
}

void mtsIntuitiveResearchKitMTM::SetState(const mtsIntuitiveResearchKitMTMTypes::RobotStateType & newState)
{
    CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": SetState: new state "
                            << mtsIntuitiveResearchKitMTMTypes::RobotStateTypeToString(newState) << std::endl;

    vctBoolVec torqueMode(8, true);

    switch (newState) {
    case mtsIntuitiveResearchKitMTMTypes::MTM_UNINITIALIZED:
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " not initialized");
        break;

    case mtsIntuitiveResearchKitMTMTypes::MTM_HOMING_POWERING:
        HomingTimer = 0.0;
        HomingPowerRequested = false;
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " powering");
        break;

    case mtsIntuitiveResearchKitMTMTypes::MTM_HOMING_CALIBRATING_ARM:
        HomingCalibrateArmStarted = false;
        RobotState = newState;
        this->EventTriggers.RobotStatusMsg(this->GetName() + " calibrating arm");
        break;

    case mtsIntuitiveResearchKitMTMTypes::MTM_HOMING_CALIBRATING_ROLL:
        HomingCalibrateRollSeekLower = false;
        HomingCalibrateRollSeekUpper = false;
        HomingCalibrateRollSeekCenter = false;
        HomingCalibrateRollLower = cmnTypeTraits<double>::MaxPositiveValue();
        HomingCalibrateRollUpper = cmnTypeTraits<double>::MinNegativeValue();
        RobotState = newState;
        this->EventTriggers.RobotStatusMsg(this->GetName() + " calibrating roll");
        break;

    case mtsIntuitiveResearchKitMTMTypes::MTM_READY:
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " ready");
        break;

    case mtsIntuitiveResearchKitMTMTypes::MTM_POSITION_CARTESIAN:
        if (this->RobotState < mtsIntuitiveResearchKitMTMTypes::MTM_READY) {
            EventTriggers.RobotErrorMsg(this->GetName() + " is not homed");
            return;
        }
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " position cartesian");

        // Disable torque mode for all joints
        torqueMode.SetAll(false);
        PID.EnableTorqueMode(torqueMode);
        PID.SetTorqueOffset(vctDoubleVec(8, 0.0));
        SetPositionJointLocal(JointGet);
        break;

    case mtsIntuitiveResearchKitMTMTypes::MTM_GRAVITY_COMPENSATION:
        if (this->RobotState < mtsIntuitiveResearchKitMTMTypes::MTM_READY) {
            EventTriggers.RobotErrorMsg(this->GetName() + " is not homed");
            return;
        }
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " gravity compensation");
        PID.EnableTorqueMode(torqueMode);
        PID.SetTorqueOffset(vctDoubleVec(8, 0.0));
        CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": SetState: set gravity compensation" << std::endl;
        break;

    case mtsIntuitiveResearchKitMTMTypes::MTM_CLUTCH:
        // check if MTM is ready
        if (this->RobotState < mtsIntuitiveResearchKitMTMTypes::MTM_READY) {
            EventTriggers.RobotErrorMsg(this->GetName() + " is not homed");
            return;
        }
        RobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " clutch mode");
        // save current cartesian position to CartesianCluted
        CartesianClutched.Assign(CartesianGet);
        // set J1-J3 to torque mode (GC) and J4-J7 to PID mode
        torqueMode.SetAll(false);
        std::fill(torqueMode.begin(), torqueMode.begin() + 3, true);
        PID.EnableTorqueMode(torqueMode);
        break;

    default:
        break;
    }
}

void mtsIntuitiveResearchKitMTM::RunHomingPower(void)
{
    const double timeToPower = 3.0 * cmn_s;

    const double currentTime = this->StateTable.GetTic();
    // first, request power to be turned on
    if (!HomingPowerRequested) {
        RobotIO.BiasEncoder();
        { // use pots for redundancy
            vctDoubleVec potsToEncodersTolerance(this->NumberOfJoints + 1); // IO level treats the gripper as joint :-)
            potsToEncodersTolerance.SetAll(10.0 * cmnPI_180); // 10 degrees for rotations
            // pots on gripper rotation are not directly mapped to encoders
            potsToEncodersTolerance.Element(6) = cmnTypeTraits<double>::PlusInfinityOrMax();
            // last joint is gripper, encoders can be anything
            potsToEncodersTolerance.Element(7) = cmnTypeTraits<double>::PlusInfinityOrMax();
            RobotIO.SetPotsToEncodersTolerance(potsToEncodersTolerance);
            RobotIO.UsePotsForSafetyCheck(true);
        }
        HomingTimer = currentTime;
        // make sure the PID is not sending currents
        PID.Enable(false);
        // pre-load the boards with zero current
        RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfJoints + 1, 0.0));
        // enable power and set a flags to move to next step
        RobotIO.EnablePower();
        HomingPowerRequested = true;
        EventTriggers.RobotStatusMsg(this->GetName() + " power requested");
        return;
    }

    // second, check status
    if (HomingPowerRequested
        && ((currentTime - HomingTimer) > timeToPower)) {

        // check power status
        vctBoolVec amplifiersStatus(NumberOfJoints + 1);
        RobotIO.GetActuatorAmpStatus(amplifiersStatus);
        if (amplifiersStatus.All()) {
            EventTriggers.RobotStatusMsg(this->GetName() + " power on");
            this->SetState(mtsIntuitiveResearchKitMTMTypes::MTM_HOMING_CALIBRATING_ARM);
        } else {
            EventTriggers.RobotErrorMsg(this->GetName() + " failed to enable power.");
            this->SetState(mtsIntuitiveResearchKitMTMTypes::MTM_UNINITIALIZED);
        }
    }
}

void mtsIntuitiveResearchKitMTM::RunHomingCalibrateArm(void)
{
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
        JointTrajectory.Goal.SetSize(NumberOfJoints);
        JointTrajectory.Goal.SetAll(0.0);
        // last joint is calibrated later
        JointTrajectory.Goal.Element(RollIndex) = JointGet.Element(RollIndex);
        JointTrajectory.LSPB.Set(JointGet, JointTrajectory.Goal,
                                 JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                 currentTime, robLSPB::LSPB_DURATION);
        HomingTimer = currentTime + JointTrajectory.LSPB.Duration();
        // set flag to indicate that homing has started
        HomingCalibrateArmStarted = true;
    }

    // compute a new set point based on time
    if (currentTime <= HomingTimer) {
        JointTrajectory.LSPB.Evaluate(currentTime, JointSet);
        SetPositionJointLocal(JointSet);
    } else {
        // request final position in case trajectory rounding prevent us to get there
        SetPositionJointLocal(JointTrajectory.Goal);

        // check position
        JointTrajectory.GoalError.DifferenceOf(JointTrajectory.Goal, JointGet);
        JointTrajectory.GoalError.AbsSelf();
        bool isHomed = !JointTrajectory.GoalError.ElementwiseGreaterOrEqual(JointTrajectory.GoalTolerance).Any();
        if (isHomed) {
            PID.SetCheckJointLimit(true);
            EventTriggers.RobotStatusMsg(this->GetName() + " arm calibrated");
            this->SetState(mtsIntuitiveResearchKitMTMTypes::MTM_HOMING_CALIBRATING_ROLL);
        } else {
            // time out
            if (currentTime > HomingTimer + extraTime) {
                CMN_LOG_CLASS_INIT_WARNING << GetName() << ": RunHomingCalibrateArm: unable to reach home position, error in degrees is "
                                           << JointTrajectory.GoalError * (180.0 / cmnPI) << std::endl;
                EventTriggers.RobotErrorMsg(this->GetName() + " unable to reach home position during calibration on pots.");
                PID.Enable(false);
                PID.SetCheckJointLimit(true);
                this->SetState(mtsIntuitiveResearchKitMTMTypes::MTM_UNINITIALIZED);
            }
        }
    }
}

void mtsIntuitiveResearchKitMTM::RunHomingCalibrateRoll(void)
{
    static const double maxTrackingError = 1.0 * cmnPI; // 1/2 turn
    static const double maxRollRange = 6.0 * cmnPI + maxTrackingError; // that actual device is limited to ~2.6 turns
    static const double extraTime = 2.0 * cmn_s;

    const double currentTime = this->StateTable.GetTic();

    // trigger search of lower limit
    if (!HomingCalibrateRollSeekLower) {
        // disable joint limits on PID
        PID.SetCheckJointLimit(false);
        // compute joint goal position, we assume PID is on from previous state
        const double currentRoll = JointGet.Element(RollIndex);
        JointTrajectory.Start.SetAll(0.0);
        JointTrajectory.Start.Element(RollIndex) = currentRoll;
        JointTrajectory.Goal.SetAll(0.0);
        JointTrajectory.Goal.Element(RollIndex) = currentRoll - maxRollRange;
        JointTrajectory.LSPB.Set(JointTrajectory.Start, JointTrajectory.Goal,
                                 JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                 currentTime, robLSPB::LSPB_DURATION);
        HomingTimer = currentTime + JointTrajectory.LSPB.Duration();
        // set flag to indicate that homing has started
        HomingCalibrateRollSeekLower = true;
        return;
    }

    // looking for lower limit has start but not found yet
    if (HomingCalibrateRollSeekLower
        && (HomingCalibrateRollLower == cmnTypeTraits<double>::MaxPositiveValue())) {
        JointTrajectory.LSPB.Evaluate(currentTime, JointSet);
        SetPositionJointLocal(JointSet);
        // detect tracking error and set lower limit
        const double trackingError =
                std::abs(JointGet.Element(RollIndex) - JointSet.Element(RollIndex));
        if (trackingError > maxTrackingError) {
            HomingCalibrateRollLower = JointGet.Element(RollIndex);
            EventTriggers.RobotStatusMsg(this->GetName() + " found roll lower limit");
        } else {
            // time out
            if (currentTime > HomingTimer + extraTime) {
                EventTriggers.RobotErrorMsg(this->GetName() + " unable to hit roll lower limit");
                PID.Enable(false);
                PID.SetCheckJointLimit(true);
                this->SetState(mtsIntuitiveResearchKitMTMTypes::MTM_UNINITIALIZED);
            }
        }
        return;
    }

    // trigger search of upper limit
    if (!HomingCalibrateRollSeekUpper) {
        // compute joint goal position, we assume PID is on from previous state
        const double currentRoll = JointGet.Element(RollIndex);
        JointTrajectory.Start.SetAll(0.0);
        JointTrajectory.Start.Element(RollIndex) = currentRoll;
        JointTrajectory.Goal.SetAll(0.0);
        JointTrajectory.Goal.Element(RollIndex) = currentRoll + maxRollRange;
        JointTrajectory.LSPB.Set(JointTrajectory.Start, JointTrajectory.Goal,
                                 JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                 currentTime, robLSPB::LSPB_DURATION);
        HomingTimer = currentTime + JointTrajectory.LSPB.Duration();
        // set flag to indicate that homing has started
        HomingCalibrateRollSeekUpper = true;
        return;
    }

    // looking for lower limit has start but not found yet
    if (HomingCalibrateRollSeekUpper
        && (HomingCalibrateRollUpper == cmnTypeTraits<double>::MinNegativeValue())) {
        JointTrajectory.LSPB.Evaluate(currentTime, JointSet);
        SetPositionJointLocal(JointSet);
        // detect tracking error and set lower limit
        const double trackingError =
                std::abs(JointGet.Element(RollIndex) - JointSet.Element(RollIndex));
        if (trackingError > maxTrackingError) {
            HomingCalibrateRollUpper = JointGet.Element(RollIndex);
            EventTriggers.RobotStatusMsg(this->GetName() + " found roll upper limit");
        } else {
            // time out
            if (currentTime > HomingTimer + extraTime) {
                EventTriggers.RobotErrorMsg(this->GetName() + " unable to hit roll upper limit");
                PID.Enable(false);
                PID.SetCheckJointLimit(true);
                this->SetState(mtsIntuitiveResearchKitMTMTypes::MTM_UNINITIALIZED);
            }
        }
        return;
    }

    // compute trajectory to go to center point
    if (!HomingCalibrateRollSeekCenter) {
        // compute joint goal position, we assume PID is on from previous state
        JointTrajectory.Start.SetAll(0.0);
        JointTrajectory.Start.Element(RollIndex) = JointGet.Element(RollIndex);
        JointTrajectory.Goal.SetAll(0.0);
        JointTrajectory.Goal.Element(RollIndex) = HomingCalibrateRollLower + 480.0 * cmnPI_180;
        JointTrajectory.LSPB.Set(JointTrajectory.Start, JointTrajectory.Goal,
                                 JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                 currentTime, robLSPB::LSPB_DURATION);
        HomingTimer = currentTime + JointTrajectory.LSPB.Duration();
        // set flag to indicate that homing has started
        HomingCalibrateRollSeekCenter = true;
        return;
    }

    // going to center position and check we have arrived
    if (currentTime <= HomingTimer) {
        JointTrajectory.LSPB.Evaluate(currentTime, JointSet);
        SetPositionJointLocal(JointSet);
    } else {
        // request final position in case trajectory rounding prevent us to get there
        SetPositionJointLocal(JointTrajectory.Goal);

        // check position
        JointTrajectory.GoalError.DifferenceOf(JointTrajectory.Goal, JointGet);
        JointTrajectory.GoalError.AbsSelf();
        bool isHomed = !JointTrajectory.GoalError.ElementwiseGreaterOrEqual(JointTrajectory.GoalTolerance).Any();
        if (isHomed) {
            // reset encoder on last joint as well as PID target position to reflect new roll position = 0
            RobotIO.ResetSingleEncoder(static_cast<int>(RollIndex));
            JointSet.SetAll(0.0);
            SetPositionJointLocal(JointSet);
            PID.SetCheckJointLimit(true);
            EventTriggers.RobotStatusMsg(this->GetName() + " roll calibrated");
            this->SetState(mtsIntuitiveResearchKitMTMTypes::MTM_READY);
        } else {
            // time out
            if (currentTime > HomingTimer + extraTime) {
                CMN_LOG_CLASS_INIT_WARNING << GetName() << ": RunHomingCalibrateRoll: unable to reach home position, error in degrees is "
                                           << JointTrajectory.GoalError * (180.0 / cmnPI) << std::endl;
                EventTriggers.RobotErrorMsg(this->GetName() + " unable to reach home position during calibration on pots.");
                PID.Enable(false);
                PID.SetCheckJointLimit(true);
                this->SetState(mtsIntuitiveResearchKitMTMTypes::MTM_UNINITIALIZED);
            }
        }
    }
}



void mtsIntuitiveResearchKitMTM::RunPositionCartesian(void)
{
    // sanity check
    if (RobotState != mtsIntuitiveResearchKitMTMTypes::MTM_POSITION_CARTESIAN) {
        CMN_LOG_CLASS_RUN_ERROR << GetName() << ": SetPositionCartesian: MTM not ready" << std::endl;
        return;
    }

    const double currentTime = this->StateTable.GetTic();
    if (currentTime <= JointTrajectory.EndTime) {
        JointTrajectory.LSPB.Evaluate(currentTime, JointSet);
        SetPositionJointLocal(JointSet);
    }
}


void mtsIntuitiveResearchKitMTM::RunGravityCompensation(void)
{
    vctDoubleVec q(7, 0.0);
    vctDoubleVec qd(7, 0.0);
    vctDoubleVec tau(7, 0.0);
    std::copy(JointGet.begin(), JointGet.begin() + 7 , q.begin());

    vctDoubleVec torqueDesired(8, 0.0);
    tau.ForceAssign(Manipulator.CCG(q, qd));
    tau[0] = q(0) * 0.0564 + 0.08;
    std::copy(tau.begin(), tau.end() , torqueDesired.begin());

    torqueDesired[3]=0.0;
    torqueDesired[4]=0.0;
    torqueDesired[5]=0.0;
    torqueDesired[6]=0.0;

    // For J7 (wrist roll) to -1.5 PI to 1.5 PI
    double gain = 0.05;
    if (JointGet[JNT_WRIST_ROLL] > 1.5 * cmnPI) {
        torqueDesired[JNT_WRIST_ROLL] = (1.5 * cmnPI - JointGet[JNT_WRIST_ROLL]) * gain;
    } else if (JointGet[JNT_WRIST_ROLL] < -1.5 * cmnPI) {
        torqueDesired[JNT_WRIST_ROLL] = (-1.5 * cmnPI - JointGet[JNT_WRIST_ROLL]) * gain;
    }

    TorqueSet.SetForceTorque(torqueDesired);
    PID.SetTorqueJoint(TorqueSet);
}

void mtsIntuitiveResearchKitMTM::RunClutch(void)
{
    // J1-J3
    vctDoubleVec q(7, 0.0);
    vctDoubleVec qd(7, 0.0);
    vctDoubleVec tau(7, 0.0);
    std::copy(JointGet.begin(), JointGet.begin() + 7 , q.begin());

    vctDoubleVec torqueDesired(8, 0.0);
    tau.ForceAssign(Manipulator.CCG(q, qd));
    tau[0] = q(0) * 0.0564 + 0.08;
    std::copy(tau.begin(), tau.end() , torqueDesired.begin());

    TorqueSet.SetForceTorque(torqueDesired);
    PID.SetTorqueJoint(TorqueSet);

    // J4-J7
    JointSet.Assign(JointGet);
    CartesianClutched.Translation().Assign(CartesianGet.Translation());
    Manipulator.InverseKinematics(JointSet, CartesianClutched);
    SetPositionJointLocal(JointSet);
}


void mtsIntuitiveResearchKitMTM::SetPositionJointLocal(const vctDoubleVec & newPosition)
{
    JointSetParam.Goal().Assign(newPosition, NumberOfJoints);
    JointSetParam.Goal().Element(7) = 0.0;
    PID.SetPositionJoint(JointSetParam);
}

void mtsIntuitiveResearchKitMTM::SetWrench(const prmForceCartesianSet & newForce)
{

    if (RobotState == mtsIntuitiveResearchKitMTMTypes::MTM_POSITION_CARTESIAN) {

        vctDoubleVec jointDesired( 7, 0.0 );
        for ( size_t i=0; i<jointDesired.size(); i++ )
            { jointDesired[i] = JointGet[i]; }

        Manipulator.JacobianBody( jointDesired );
        vctDynamicMatrix<double> J( 6, Manipulator.links.size(), VCT_COL_MAJOR );
        for( size_t r=0; r<6; r++ ){
            for( size_t c=0; c<Manipulator.links.size(); c++ ){
                J[r][c] = Manipulator.Jn[c][r];
            }
        }

        prmForceCartesianSet tmp = newForce;
        prmForceCartesianSet::ForceType tmpft;
        tmp.GetForce( tmpft );
        vctDynamicMatrix<double> ft( tmpft.size(), 1, 0.0, VCT_COL_MAJOR );
        for( size_t i=0; i<ft.size(); i++ )
            { ft[i][0] = tmpft[i]; }
        vctDynamicMatrix<double> t = nmrLSMinNorm( J, ft );


        vctDoubleVec torqueDesired(8, 0.0);
        for( size_t i=0; i<3; i++ )
            { torqueDesired[i] = t[i][0]; }

        if( torqueDesired[0] < -2.0 ) { torqueDesired[0] = -2.0; }
        if( 2.0 < torqueDesired[0]  ) { torqueDesired[0] =  2.0; }
        if( torqueDesired[1] < -2.0 ) { torqueDesired[1] = -2.0; }
        if( 2.0 < torqueDesired[1]  ) { torqueDesired[1] =  2.0; }
        if( torqueDesired[2] < -2.0 ) { torqueDesired[2] = -2.0; }
        if( 2.0 < torqueDesired[2]  ) { torqueDesired[2] =  2.0; }

        if( torqueDesired[3] < -1.0 ) { torqueDesired[3] = -0.05; }
        if( 1.0 < torqueDesired[3]  ) { torqueDesired[3] =  0.05; }
        if( torqueDesired[4] < -1.0 ) { torqueDesired[4] = -0.05; }
        if( 1.0 < torqueDesired[4]  ) { torqueDesired[4] =  0.05; }
        if( torqueDesired[5] < -1.0 ) { torqueDesired[5] = -0.05; }
        if( 1.0 < torqueDesired[5]  ) { torqueDesired[5] =  0.05; }
        if( torqueDesired[6] < -1.0 ) { torqueDesired[6] = -0.05; }
        if( 1.0 < torqueDesired[6]  ) { torqueDesired[6] =  0.05; }

        TorqueSet.SetForceTorque(torqueDesired);
        PID.SetTorqueJoint(TorqueSet);
    }
}

void mtsIntuitiveResearchKitMTM::SetPositionCartesian(const prmPositionCartesianSet & newPosition)
{
    if (RobotState == mtsIntuitiveResearchKitMTMTypes::MTM_POSITION_CARTESIAN) {
        const double currentTime = this->StateTable.GetTic();
        // starting point is last requested to PID component
        JointTrajectory.Start.Assign(JointGet);
        // end point is defined by inverse kinematics but initialize optimizer to start from current
        // and try to push L platform away from user's hand
        JointTrajectory.Goal.Assign(JointGet);
        if (RobotType == MTM_LEFT) {
            JointTrajectory.Goal[3] = -cmnPI_4;
        } else if (RobotType == MTM_RIGHT) {
            JointTrajectory.Goal[3] = cmnPI_4;
        }
        Manipulator.InverseKinematics(JointTrajectory.Goal, newPosition.Goal());
        JointTrajectory.LSPB.Set(JointTrajectory.Start, JointTrajectory.Goal,
                                 JointTrajectory.Velocity, JointTrajectory.Acceleration,
                                 currentTime, robLSPB::LSPB_DURATION);
        JointTrajectory.EndTime = currentTime + JointTrajectory.LSPB.Duration();
    } else {
        CMN_LOG_CLASS_RUN_WARNING << GetName() << ": SetPositionCartesian: MTM not ready" << std::endl;
    }
}

void mtsIntuitiveResearchKitMTM::SetRobotControlState(const std::string & state)
{
    if (state == "Home") {
        SetState(mtsIntuitiveResearchKitMTMTypes::MTM_HOMING_POWERING);
    } else if ((state == "Cartesian position") || (state == "Teleop")) {
        SetState(mtsIntuitiveResearchKitMTMTypes::MTM_POSITION_CARTESIAN);
    } else if (state == "Gravity") {
        SetState(mtsIntuitiveResearchKitMTMTypes::MTM_GRAVITY_COMPENSATION);
    } else if (state == "Clutch") {
        SetState(mtsIntuitiveResearchKitMTMTypes::MTM_CLUTCH);
    } else {
        EventTriggers.RobotErrorMsg(this->GetName() + ": unsupported state " + state);
    }

    CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": SetRobotControlState: " << state << std::endl;
}

void mtsIntuitiveResearchKitMTM::GetRobotControlState(std::string & state) const
{
    state = mtsIntuitiveResearchKitMTMTypes::RobotStateTypeToString(this->RobotState);
}
