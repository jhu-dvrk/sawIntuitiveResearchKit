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
    RobotCurrentState = STATE_IDLE;
    IsHomed = false;
}

void mtsIntuitiveResearchKitMTM::Run(void)
{
    ProcessQueuedEvents();

    mtsExecutionResult executionResult;
    executionResult = PID.GetPositionJoint(JointCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Call to GetJointPosition failed \""
                                << executionResult << "\"" << std::endl;
    }
    vctFrm4x4 position;
    position = Manipulator.ForwardKinematics(JointCurrent.Position());
    position.Rotation().NormalizedSelf();
    CartesianCurrent.Position().From(position);

    executionResult = RobotIO.GetAnalogInputPosSI(AnalogInputPosSI);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Call to GetAnalogInputPosSI failed \""
                                << executionResult << "\"" << std::endl;
    }
    GripperPosition = AnalogInputPosSI.Element(7);

    switch (RobotCurrentState) {
    case STATE_TELEOP:
        break;
    case STATE_HOME:
    {
        // ZC: assume no adapter & no tool for now
        vctDoubleVec homeError;
        vctDoubleVec homeErrorTolerance;

        // check position
        PID.GetPositionJoint(JointCurrent);
        homeError.SetSize(HomeJointSet.size());
        homeError.DifferenceOf(HomeJointSet, JointCurrent.Position());
        homeError.AbsSelf();

        homeErrorTolerance.SetSize(HomeJointSet.size());
        homeErrorTolerance.SetAll(2.0 * cmnPI_180); // 2 deg tolerence
        IsHomed = true;
        for (size_t i = 0; i < HomeJointSet.size(); i++) {
            if (homeError[i] > homeErrorTolerance[i]) {
                IsHomed = false;
            }
        }
        if (IsHomed) {
            RobotCurrentState = STATE_IDLE;
            EventTriggers.RobotStatusMsg(mtsStdString("MTM Homed"));
        } else {
            JointDesired.Goal().ForceAssign(HomeJointSet);
            PID.SetPositionJoint(JointDesired);
        }
    }
        break;
    case STATE_IDLE:
        break;
    default:
        CMN_LOG_CLASS_RUN_ERROR << "Unknown control state" << std::endl;
        break;
    }

    RunEvent();
    ProcessQueuedCommands();
}

void mtsIntuitiveResearchKitMTM::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsIntuitiveResearchKitMTM::SetPositionCartesian(const prmPositionCartesianSet & newPosition)
{
    if (RobotCurrentState == STATE_TELEOP) {
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
    if (RobotCurrentState != STATE_IDLE) {
        EventTriggers.RobotErrorMsg(std::string("ERROR: MTM NOT in IDLE mode, Action cancelled"));
        return;
    }

    if (state == "Start") {
        std::cout << "YES Start" << std::endl;
    }
    else if (state == "Home"){
        EventHandlerHome();
    }
    else if (state == "Teleop"){
        EventHandlerTeleop();
    }
}


// -------------- Event Handlers ------------------------------
void mtsIntuitiveResearchKitMTM::EventHandlerHome(void)
{
    std::cerr << "MTM HOME" << std::endl;

    // Set state to HOME
    RobotCurrentState = STATE_HOME;

    // Start homing here
    RobotIO.EnablePower();
    RobotIO.BiasEncoder();

    // Home position all but last joint to 0
    PID.GetPositionJoint(JointCurrent);
    const size_t numberOfJoints = JointCurrent.Position().size();
    HomeJointSet.SetSize(numberOfJoints);
    HomeJointSet.SetAll(0.0);
    HomeJointSet.Element(numberOfJoints - 2) = JointCurrent.Position().Element(numberOfJoints - 2);

    // Enable PID
    PID.Enable(true);
}

void mtsIntuitiveResearchKitMTM::EventHandlerTeleop(void)
{
    if (!IsHomed) {
        EventTriggers.RobotErrorMsg(std::string("ERROR: Robot is not Homed"));
    } else {
        RobotCurrentState = STATE_TELEOP;
        // ZC: nothing now
        // Could be to set position & orientation to follow slave
    }
}
