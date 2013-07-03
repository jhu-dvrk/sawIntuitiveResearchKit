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
    Trajectory = 0;

    this->StateTable.AddData(CartesianCurrent, "CartesianPosition");
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
        interfaceProvided->AddCommandReadState(this->StateTable, CartesianCurrent, "GetPositionCartesian");
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
    RobotState = MTM_UNINITIALIZED;
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
        executionResult = PID.GetPositionJoint(JointCurrent);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << "GetRobotData: call to GetJointPosition failed \""
                                    << executionResult << "\"" << std::endl;
            return;
        }
        // when the robot is ready, we can comput cartesian position
        if (this->RobotState >= MTM_READY) {
            vctFrm4x4 position;
            position = Manipulator.ForwardKinematics(JointCurrent.Position());
            position.Rotation().NormalizedSelf();
            CartesianCurrent.Position().From(position);
        } else {
            CartesianCurrent.Position().Assign(vctFrm3::Identity());
        }
        // get gripper based on analog inputs
        executionResult = RobotIO.GetAnalogInputPosSI(AnalogInputPosSI);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << "GetRobotData: call to GetAnalogInputPosSI failed \""
                                    << executionResult << "\"" << std::endl;
            return;
        }
        GripperPosition = AnalogInputPosSI.Element(7);
    } else {
        JointCurrent.Position().Zeros();
    }
}

void mtsIntuitiveResearchKitMTM::SetState(const RobotStateType & newState)
{
    switch (newState) {
    case MTM_UNINITIALIZED:
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
        break;
    case MTM_READY:
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
        this->RobotState = MTM_HOMING_CALIBRATING_POTS;
        this->EventTriggers.RobotStatusMsg(this->GetName() + " powered");
    }
}

void mtsIntuitiveResearchKitMTM::RunHomingCalibrateOnPots(void)
{
    const double currentTime = this->StateTable.GetTic();

    if (!RunHomingCalibrateOnPotsStarted) {
        // enable PID and start from current position
        JointDesired.Goal().ForceAssign(JointCurrent.Position());
        PID.SetPositionJoint(JointDesired);
        PID.Enable(true);

        // compute joint goal position
        HomeJointSet.SetSize(NumberOfJoints);
        HomeJointSet.SetAll(0.0);
        // last joint is not motorized and we don't want to move the last motorized so - 2
        HomeJointSet.Element(NumberOfJoints - 2) = JointCurrent.Position().Element(NumberOfJoints - 2);
        // set flag to indicate that homing has started
        RunHomingCalibrateOnPotsStarted = true;

        // this is to be replaced by trajectory generator
#ifdef OLD_PID
        JointDesired.Goal().ForceAssign(HomeJointSet);
        PID.SetPositionJoint(JointDesired);
#else
        vctDoubleVec zeros(NumberOfJoints, 0.0);
        if (Trajectory) {
            delete Trajectory;
        }
        Trajectory = new robQuintic(currentTime,
                                    JointCurrent.Position(), zeros, zeros,
                                    currentTime + 2.0 * cmn_s,
                                    HomeJointSet, zeros, zeros);
#endif
    }

    // compute a new set point based on time
    vctDoubleVec desiredJointPosition(NumberOfJoints);
    vctDoubleVec desiredJointVelocity(NumberOfJoints);
    vctDoubleVec desiredJointAcceleration(NumberOfJoints);

    if (currentTime <= Trajectory->StopTime()) {
        Trajectory->Evaluate(currentTime, desiredJointPosition, desiredJointVelocity, desiredJointAcceleration);
        JointDesired.Goal().ForceAssign(desiredJointPosition);
        PID.SetPositionJoint(JointDesired);
    }

    // check position
    vctDoubleVec homeError;
    vctDoubleVec homeErrorTolerance;
    homeError.SetSize(NumberOfJoints);
    homeError.DifferenceOf(HomeJointSet, JointCurrent.Position());
    homeError.resize(NumberOfJoints - 1);
    homeError.AbsSelf();

    homeErrorTolerance.SetSize(NumberOfJoints - 1);
    homeErrorTolerance.SetAll(2.0 * cmnPI_180); // 2 deg tolerence

    bool isHomed = !homeError.ElementwiseGreaterOrEqual(homeErrorTolerance).Any();
    if (isHomed) {
        SetState(MTM_READY);
    }
}

void mtsIntuitiveResearchKitMTM::RunHomingCalibrateOnLimits(void)
{
}

void mtsIntuitiveResearchKitMTM::SetPositionCartesian(const prmPositionCartesianSet & newPosition)
{
    if (RobotState == MTM_POSITION_CARTESIAN) {
        vctDoubleVec jointDesired;
        jointDesired.ForceAssign(JointCurrent.Position());
        const double angle = jointDesired[7];
        jointDesired.resize(7);
        Manipulator.InverseKinematics(jointDesired, newPosition.Goal());
        jointDesired.resize(8);
        jointDesired[7] = angle;
        JointDesired.Goal().ForceAssign(jointDesired);

        // note: this directly calls the lower level to set position,
        // maybe we should cache the request in this component and later
        // in the Run method push the request.  This way, only the latest
        // request would be pushed if multiple are queued.
        PID.SetPositionJoint(JointDesired);
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
