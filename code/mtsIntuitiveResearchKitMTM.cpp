/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// cisst
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitMTM.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>


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
    SetState(MTM_UNINITIALIZED);

    // initialize trajectory data
    JointCurrent.SetSize(NumberOfJoints);
    JointDesired.SetSize(NumberOfJoints);
    JointDesiredParam.Goal().SetSize(NumberOfJoints + 1); // PID treats gripper as joint
    JointTrajectory.Velocity.SetSize(NumberOfJoints);
    JointTrajectory.Acceleration.SetSize(NumberOfJoints);
    JointTrajectory.Goal.SetSize(NumberOfJoints);
    JointTrajectory.GoalError.SetSize(NumberOfJoints);
    JointTrajectory.GoalTolerance.SetSize(NumberOfJoints);
    JointTrajectory.GoalTolerance.SetAll(2.0 * cmnPI / 180.0); // hard coded to 2 degrees
    JointTrajectory.Zero.SetSize(NumberOfJoints);

    this->StateTable.AddData(CartesianCurrentParam, "CartesianPosition");
    this->StateTable.AddData(JointCurrentParam, "JointPosition");
    this->StateTable.AddData(GripperPosition, "GripperAngle");

    // Setup cisst interfaces
    mtsInterfaceRequired * interfaceRequired;
    interfaceRequired = AddInterfaceRequired("PID");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("Enable", PID.Enable);
        interfaceRequired->AddFunction("GetPositionJoint", PID.GetPositionJoint);
        interfaceRequired->AddFunction("SetPositionJoint", PID.SetPositionJoint);
        interfaceRequired->AddFunction("SetIsCheckJointLimit", PID.SetIsCheckJointLimit);
    }

    // Robot IO
    interfaceRequired = AddInterfaceRequired("RobotIO");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("EnablePower", RobotIO.EnablePower);
        interfaceRequired->AddFunction("DisablePower", RobotIO.DisablePower);
        interfaceRequired->AddFunction("BiasEncoder", RobotIO.BiasEncoder);
        interfaceRequired->AddFunction("BiasCurrent", RobotIO.BiasCurrent);
        interfaceRequired->AddFunction("SetActuatorCurrent", RobotIO.SetActuatorCurrent);
        interfaceRequired->AddFunction("GetAnalogInputPosSI", RobotIO.GetAnalogInputPosSI);
    }

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Robot");
    if (interfaceProvided) {
        interfaceProvided->AddCommandReadState(this->StateTable, JointCurrentParam, "GetPositionJoint");
        interfaceProvided->AddCommandReadState(this->StateTable, CartesianCurrentParam, "GetPositionCartesian");
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitMTM::SetPositionCartesian, this, "SetPositionCartesian");
        interfaceProvided->AddCommandReadState(this->StateTable, GripperPosition, "GetGripperPosition");
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitMTM::SetRobotControlState,
                                           this, "SetRobotControlState", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.RobotStatusMsg, "RobotStatusMsg", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.RobotErrorMsg, "RobotErrorMsg", std::string(""));
    }
}

void mtsIntuitiveResearchKitMTM::Configure(const std::string & filename)
{
    robManipulator::Errno result;
    result = this->Manipulator.LoadRobot(filename);
    if (result == robManipulator::EFAILURE) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to load manipulator configuration file \""
                                 << filename << "\"" << std::endl;
    }
}

void mtsIntuitiveResearchKitMTM::Startup(void)
{
    this->SetState(MTM_UNINITIALIZED);
}

void mtsIntuitiveResearchKitMTM::Run(void)
{
    ProcessQueuedEvents();

    GetRobotData();

    switch (RobotState) {
    case MTM_UNINITIALIZED:
        break;
    case MTM_HOMING_POWERING:
        RunHomingPower();
        break;
    case MTM_HOMING_CALIBRATING_POTS:
        RunHomingCalibrateOnPots();
        break;
    case MTM_HOMING_CALIBRATING_LIMITS:
        RunHomingCalibrateOnLimits();
        break;
    case MTM_READY:
    case MTM_POSITION_CARTESIAN:
        break;
    default:
        break;
    }

    RunEvent();
    ProcessQueuedCommands();
}

void mtsIntuitiveResearchKitMTM::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsIntuitiveResearchKitMTM::GetRobotData(void)
{
    // we can start reporting some joint values after the robot is powered
    if (this->RobotState > MTM_HOMING_POWERING) {
        mtsExecutionResult executionResult;
        executionResult = PID.GetPositionJoint(JointCurrentParam);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << "GetRobotData: call to GetJointPosition failed \""
                                    << executionResult << "\"" << std::endl;
            return;
        }
        // the lower level report 8 joints, we need 7 only
        JointCurrent.Assign(JointCurrentParam.Position(), 0, NumberOfJoints);

        // when the robot is ready, we can comput cartesian position
        if (this->RobotState >= MTM_READY) {
            CartesianCurrent = Manipulator.ForwardKinematics(JointCurrent);
            CartesianCurrent.Rotation().NormalizedSelf();
        } else {
            CartesianCurrent.Assign(vctFrm4x4::Identity());
        }
        CartesianCurrentParam.Position().From(CartesianCurrent);

        // get gripper based on analog inputs
        executionResult = RobotIO.GetAnalogInputPosSI(AnalogInputPosSI);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << "GetRobotData: call to GetAnalogInputPosSI failed \""
                                    << executionResult << "\"" << std::endl;
            return;
        }
        GripperPosition = AnalogInputPosSI.Element(7);
    } else {
        JointCurrent.Zeros();
        JointCurrentParam.Position().Zeros();
    }
}

void mtsIntuitiveResearchKitMTM::SetState(const RobotStateType & newState)
{
    switch (newState) {
    case MTM_UNINITIALIZED:
        EventTriggers.RobotStatusMsg(this->GetName() + " not initialized");
        break;
    case MTM_HOMING_POWERING:
        RunHomingPowerTimer = 0.0;
        RunHomingPowerRequested = false;
        RunHomingPowerCurrentBiasRequested = false;
        EventTriggers.RobotStatusMsg(this->GetName() + " powering");
        break;
    case MTM_HOMING_CALIBRATING_POTS:
        RunHomingCalibrateOnPotsStarted = false;
        this->EventTriggers.RobotStatusMsg(this->GetName() + " calibrating pots");
        break;
    case MTM_HOMING_CALIBRATING_LIMITS:
        RunHomingCalibrateOnLimitsSeekLower = false;
        RunHomingCalibrateOnLimitsSeekUpper = false;
        RunHomingCalibrateOnLimitsLower = cmnTypeTraits<double>::MaxPositiveValue();
        RunHomingCalibrateOnLimitsUpper = cmnTypeTraits<double>::MinNegativeValue();
        this->EventTriggers.RobotStatusMsg(this->GetName() + " calibrating gimbal");
        break;
    case MTM_READY:
        EventTriggers.RobotStatusMsg(this->GetName() + " ready");
        break;
    case MTM_POSITION_CARTESIAN:
        if (this->RobotState < MTM_READY) {
            EventTriggers.RobotErrorMsg(this->GetName() + " is not homed");
            return;
        }
        EventTriggers.RobotStatusMsg(this->GetName() + " position cartesian");
        break;
    default:
        break;
    }
    RobotState = newState;
}

void mtsIntuitiveResearchKitMTM::RunHomingPower(void)
{
    const double currentTime = this->StateTable.GetTic();
    // first, request power to be turned on
    if (!RunHomingPowerRequested) {
        RobotIO.BiasEncoder();
        RunHomingPowerTimer = currentTime;
        // make sure the PID are not sending currents
        PID.Enable(false);
        // pre-load the boards with zero current
        RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfJoints, 0.0));
        // enable power and set a flags to move to next step
        RobotIO.EnablePower();
        RunHomingPowerRequested = true;
        RunHomingPowerCurrentBiasRequested = false;
        return;
    }

    // second, request current bias, we leave 1 second for power to stabilize
    if (!RunHomingPowerCurrentBiasRequested
        && ((currentTime - RunHomingPowerTimer) > 1.0 * cmn_s)) {
        RunHomingPowerTimer = currentTime;
        RobotIO.BiasCurrent(500); // 500 samples, this API should be changed to use time
        RunHomingPowerCurrentBiasRequested = true;
        return;
    }

    // wait another second to be ready
    if ((currentTime - RunHomingPowerTimer) > 1.0 * cmn_s) {
        std::cerr << CMN_LOG_DETAILS << " - need to add test to make sure power is OK from sawRobotIO" << std::endl;
        this->SetState(MTM_HOMING_CALIBRATING_POTS);
    }
}

void mtsIntuitiveResearchKitMTM::RunHomingCalibrateOnPots(void)
{
    const double currentTime = this->StateTable.GetTic();

    // trigger motion
    if (!RunHomingCalibrateOnPotsStarted) {
        // enable PID and start from current position
        JointDesired.ForceAssign(JointCurrent);
        SetPositionJoint(JointDesired);
        PID.Enable(true);

        // compute joint goal position
        JointTrajectory.Goal.SetSize(NumberOfJoints);
        JointTrajectory.Goal.SetAll(0.0);
        // last joint is calibrated later
        JointTrajectory.Goal.Element(NumberOfJoints - 2) = JointCurrent.Element(NumberOfJoints - 2);
        JointTrajectory.Quintic.Set(currentTime,
                                    JointCurrent, JointTrajectory.Zero, JointTrajectory.Zero,
                                    currentTime + 2.0 * cmn_s,
                                    JointTrajectory.Goal, JointTrajectory.Zero, JointTrajectory.Zero);
        // set flag to indicate that homing has started
        RunHomingCalibrateOnPotsStarted = true;
    }

    // compute a new set point based on time
    if (currentTime <= JointTrajectory.Quintic.StopTime()) {
        JointTrajectory.Quintic.Evaluate(currentTime, JointDesired,
                                         JointTrajectory.Velocity, JointTrajectory.Acceleration);
        SetPositionJoint(JointDesired);
    } else {
        // check position
        JointTrajectory.GoalError.DifferenceOf(JointTrajectory.Goal, JointCurrent);
        JointTrajectory.GoalError.AbsSelf();

        bool isHomed = !JointTrajectory.GoalError.ElementwiseGreaterOrEqual(JointTrajectory.GoalTolerance).Any();
        if (isHomed) {
            SetState(MTM_READY);
        }
    }
}

void mtsIntuitiveResearchKitMTM::RunHomingCalibrateOnLimits(void)
{
    const double currentTime = this->StateTable.GetTic();

    // trigger search of lower limit
    if (!RunHomingCalibrateOnLimitsSeekLower) {
        // compute joint goal position, we assume PID is on from previous state
        JointTrajectory.Goal.Assign(JointCurrent);
        // gimbal joint is before last
        JointTrajectory.Goal.Element(NumberOfJoints - 2) -= GimbalMaxRange;
        JointTrajectory.Quintic.Set(currentTime,
                                    JointCurrent, JointTrajectory.Zero, JointTrajectory.Zero,
                                    currentTime + 2.0 * cmn_s,
                                    JointTrajectory.Goal, JointTrajectory.Zero, JointTrajectory.Zero);
        // set flag to indicate that homing has started
        RunHomingCalibrateOnLimitsSeekLower = true;
        return;
    }

}

void mtsIntuitiveResearchKitMTM::SetPositionJoint(const vctDoubleVec & newPosition)
{
    JointDesiredParam.Goal().Assign(newPosition, 0, NumberOfJoints);
    JointDesiredParam.Goal().Element(7) = 0.0;
    PID.SetPositionJoint(JointDesiredParam);
}

void mtsIntuitiveResearchKitMTM::SetPositionCartesian(const prmPositionCartesianSet & newPosition)
{
    if (RobotState == MTM_POSITION_CARTESIAN) {
        JointDesired.Assign(JointCurrent);
        Manipulator.InverseKinematics(JointDesired, newPosition.Goal());
        // note: this directly calls the lower level to set position,
        // maybe we should cache the request in this component and later
        // in the Run method push the request.  This way, only the latest
        // request would be pushed if multiple are queued.
        SetPositionJoint(JointDesired);
    } else {
        CMN_LOG_CLASS_RUN_WARNING << "SetPositionCartesian: MTM not ready" << std::endl;
    }
}

void mtsIntuitiveResearchKitMTM::SetRobotControlState(const std::string & state)
{
    if (state == "Home") {
        SetState(MTM_HOMING_POWERING);
    } else if ((state == "Cartesian position") || (state == "Teleop")) {
        SetState(MTM_POSITION_CARTESIAN);
    } else {
        EventTriggers.RobotErrorMsg(this->GetName() + ": unsupported state " + state);
    }
}
