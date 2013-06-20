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
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSM.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitPSM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsIntuitiveResearchKitPSM::mtsIntuitiveResearchKitPSM(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    Init();
}

mtsIntuitiveResearchKitPSM::mtsIntuitiveResearchKitPSM(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg)
{
    Init();
}

void mtsIntuitiveResearchKitPSM::Init(void)
{
    this->StateTable.AddData(CartesianCurrent, "CartesianPosition");
    this->StateTable.AddData(JointCurrent, "JointPosition");

    // Setup CISST Interface
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
    }

    // Event Adapter engage: digital input button event from PSM
    interfaceRequired = AddInterfaceRequired("Adapter");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetButton", Adapter.GetButton);
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerAdapter, this, "Button");
    }

    // Event Tool engage: digital input button event from PSM
    interfaceRequired = AddInterfaceRequired("Tool");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetButton", Tool.GetButton);
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerTool, this, "Button");
    }

    // ManipClutch: digital input button event from PSM
    interfaceRequired = AddInterfaceRequired("ManipClutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerManipClutch, this, "Button");
    }

    // SUJClutch: digital input button event from PSM
    interfaceRequired = AddInterfaceRequired("SUJClutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerSUJClutch, this, "Button");
    }


    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Robot");
    if (interfaceProvided) {
        interfaceProvided->AddCommandReadState(this->StateTable, CartesianCurrent, "GetPositionCartesian");
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetPositionCartesian, this, "SetPositionCartesian");
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetGripperPosition, this, "SetGripperPosition");

        interfaceProvided->AddCommandReadState(this->StateTable, JointCurrent, "GetPositionJoint");

        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetRobotControlState,
                                           this, "SetRobotControlState", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.RobotStatusMsg, "RobotStatusMsg", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.RobotErrorMsg, "RobotErrorMsg", std::string(""));
    }
}

void mtsIntuitiveResearchKitPSM::Configure(const std::string & filename)
{
    robManipulator::Errno result;
    result = this->Manipulator.LoadRobot(filename);
    if (result == robManipulator::EFAILURE) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to load manipulator configuration file \""
                                 << filename << "\"" << std::endl;
    }
}

void mtsIntuitiveResearchKitPSM::Startup(void)
{
    RobotCurrentState = STATE_IDLE;
    IsHomed = false;
    IsAdapterEngaged = false;
    IsToolEngaged = false;

    frame6to7.Assign(0.0, -1.0,  0.0, 0.0,
                     0.0,  0.0,  1.0, 0.0102,
                     -1.0, 0.0,  0.0, 0.0,
                     0.0,  0.0,  0.0, 1.0);
}

void mtsIntuitiveResearchKitPSM::Run(void)
{
    ProcessQueuedEvents();

    // ------ Start ----------------------
    mtsExecutionResult executionResult;
    executionResult = PID.GetPositionJoint(JointCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Call to GetJointPosition failed \""
                                << executionResult << "\"" << std::endl;
    }    

    JointCurrent.Position()[2] = JointCurrent.Position()[2] / 1000.0; // ugly hack to convert mm to meters
    vctFrm4x4 position;
    position = Manipulator.ForwardKinematics(JointCurrent.Position());
    position = position * frame6to7;

    position.Rotation().NormalizedSelf();
    CartesianCurrent.Position().From(position);

    switch (RobotCurrentState) {
    case STATE_TELEOP:
        // ZC: no thing for teleop
        break;

    case STATE_HOME:
    {
        vctDoubleVec homeError;
        vctDoubleVec homeErrorTolerance;

        // check position
        PID.GetPositionJoint(JointCurrent);
        homeError.SetSize(HomeJointSet.size());
        homeError.DifferenceOf(HomeJointSet, JointCurrent.Position());
        homeError.AbsSelf();

        homeErrorTolerance.SetSize(HomeJointSet.size());
        homeErrorTolerance.SetAll(2.0 * cmnPI_180);
        homeErrorTolerance[2] = 1.0;  // 1mm
        IsHomed = true;
        for (size_t i = 0; i < HomeJointSet.size(); i++) {
            if (homeError[i] > homeErrorTolerance[i]) {
                IsHomed = false;
            }
        }
        if (IsHomed) {
            RobotCurrentState = STATE_IDLE;
            EventTriggers.RobotStatusMsg(mtsStdString("PSM Homed"));

            Adapter.GetButton(Adapter.IsPresent);
            Tool.GetButton(Tool.IsPresent);

            if (Adapter.IsPresent && !Tool.IsPresent) {
                prmEventButton button;
                button.Type() = prmEventButton::PRESSED;
                EventHandlerAdapter(button);
            } else if (Adapter.IsPresent && Tool.IsPresent) {
                IsAdapterEngaged = true;
                prmEventButton button;
                button.Type() = prmEventButton::PRESSED;
                EventHandlerTool(button);
            }

        } else {
            JointDesired.Goal().ForceAssign(HomeJointSet);
            PID.SetPositionJoint(JointDesired);
        }
        break;
    }
    case STATE_ADAPTER:
        // PSM tool last 4 actuator coupling matrix
        // psm_m2jpos = [-1.5632  0.0000  0.0000  0.0000;
        //                0.0000  1.0186  0.0000  0.0000;
        //                0.0000 -0.8306  0.6089  0.6089;
        //                0.0000  0.0000 -1.2177  1.2177];
        // each actuator has -180 to 180 deg limits
        // these joint limit is computed as
        // joint_lim = psm_m2jpos * actuator_lim

        if (AdapterStopwatch.GetElapsedTime() > (3500 * cmn_ms)){
            AdapterJointSet[2] = 0.0;
            AdapterJointSet[3] = 0.0;
            AdapterJointSet[4] = 0.0;
            AdapterJointSet[5] = 0.0;
            AdapterJointSet[6] = 0.0;
            JointDesired.Goal().ForceAssign(AdapterJointSet);
            PID.SetIsCheckJointLimit(true);
            PID.SetPositionJoint(JointDesired);

            // Adapter engage done
            RobotCurrentState = STATE_IDLE;
            AdapterStopwatch.Reset();
            IsAdapterEngaged = true;
            EventTriggers.RobotStatusMsg(mtsStdString("PSM Adapter Engaged"));
        }
        else if (AdapterStopwatch.GetElapsedTime() > (2500 * cmn_ms)){
            AdapterJointSet[3] = -300.0 * cmnPI / 180.0;
            AdapterJointSet[4] =  170.0 * cmnPI / 180.0;
            AdapterJointSet[5] =   65.0 * cmnPI / 180.0;
            AdapterJointSet[6] =    0.0 * cmnPI / 180.0;
            JointDesired.Goal().ForceAssign(AdapterJointSet);
            PID.SetPositionJoint(JointDesired);
        }
        else if (AdapterStopwatch.GetElapsedTime() > (1500 * cmn_ms)){
            AdapterJointSet[3] =  300.0 * cmnPI / 180.0;
            AdapterJointSet[4] = -170.0 * cmnPI / 180.0;
            AdapterJointSet[5] =  -65.0 * cmnPI / 180.0;
            AdapterJointSet[6] =    0.0 * cmnPI / 180.0;
            JointDesired.Goal().ForceAssign(AdapterJointSet);
            PID.SetPositionJoint(JointDesired);
        }
        else if (AdapterStopwatch.GetElapsedTime() > (500 * cmn_ms)){
            AdapterJointSet[3] = -300.0 * cmnPI / 180.0;
            AdapterJointSet[4] =  170.0 * cmnPI / 180.0;
            AdapterJointSet[5] =   65.0 * cmnPI / 180.0;
            AdapterJointSet[6] =    0.0 * cmnPI / 180.0;
            JointDesired.Goal().ForceAssign(AdapterJointSet);
            PID.SetPositionJoint(JointDesired);
        }
        break;

    case STATE_TOOL:
        if (ToolStopwatch.GetElapsedTime() > (2500 * cmn_ms)){
            ToolJointSet.SetAll(0.0);
            ToolJointSet[6] = 10.0 * cmnPI / 180.0;
            JointDesired.Goal().ForceAssign(ToolJointSet);
            PID.SetIsCheckJointLimit(true);
            PID.SetPositionJoint(JointDesired);

            RobotCurrentState = STATE_IDLE;
            ToolStopwatch.Reset();
            IsToolEngaged = true;
            EventTriggers.RobotStatusMsg(mtsStdString("PSM Tool Engaged"));
        }
        else if (ToolStopwatch.GetElapsedTime() > (2000 * cmn_ms)){
            ToolJointSet[3] = -280.0 * cmnPI / 180.0;
            ToolJointSet[4] =  10.0 * cmnPI / 180.0;
            ToolJointSet[5] =  10.0 * cmnPI / 180.0;
            ToolJointSet[6] =  10.0 * cmnPI / 180.0;
            JointDesired.Goal().ForceAssign(ToolJointSet);
            PID.SetPositionJoint(JointDesired);
        }
        else if (ToolStopwatch.GetElapsedTime() > (1500 * cmn_ms)){
            ToolJointSet[3] = -280.0 * cmnPI / 180.0;
            ToolJointSet[4] =  10.0 * cmnPI / 180.0;
            ToolJointSet[5] = -10.0 * cmnPI / 180.0;
            ToolJointSet[6] =  10.0 * cmnPI / 180.0;
            JointDesired.Goal().ForceAssign(ToolJointSet);
            PID.SetPositionJoint(JointDesired);
        }
        else if (ToolStopwatch.GetElapsedTime() > (1000 * cmn_ms)){
            ToolJointSet[3] =  280.0 * cmnPI / 180.0;
            ToolJointSet[4] = -10.0 * cmnPI / 180.0;
            ToolJointSet[5] =  10.0 * cmnPI / 180.0;
            ToolJointSet[6] =  10.0 * cmnPI / 180.0;
            JointDesired.Goal().ForceAssign(ToolJointSet);
            PID.SetPositionJoint(JointDesired);
        }
        else if (ToolStopwatch.GetElapsedTime() > (500 * cmn_ms)){
            ToolJointSet[3] = -280.0 * cmnPI / 180.0;
            ToolJointSet[4] = -10.0 * cmnPI / 180.0;
            ToolJointSet[5] = -10.0 * cmnPI / 180.0;
            ToolJointSet[6] =  10.0 * cmnPI / 180.0;
            JointDesired.Goal().ForceAssign(ToolJointSet);
            PID.SetPositionJoint(JointDesired);
        }
        break;
    case STATE_IDLE:
        break;
    default:
        CMN_LOG_CLASS_RUN_ERROR << "Unknown control state" << std::endl;
        break;
    }


    // ------ Stop call teleop ------------
    RunEvent();

    ProcessQueuedCommands();
}

void mtsIntuitiveResearchKitPSM::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsIntuitiveResearchKitPSM::SetPositionCartesian(const prmPositionCartesianSet & newPosition)
{
    if (RobotCurrentState == STATE_TELEOP) {
        vctDoubleVec jointDesired;
        jointDesired.ForceAssign(JointCurrent.Position());
        jointDesired.resize(6);

        // compute desired slave position
        vctFrm4x4 newPositionFrm;
        newPositionFrm.FromNormalized(newPosition.Goal());

        vctMatRot3 newPositionRotation;
        newPositionRotation.From(newPosition.Goal().Rotation());
        newPositionRotation = newPositionRotation * frame6to7.Rotation().Inverse();
        newPositionFrm.Rotation().FromNormalized(newPositionRotation);

        Manipulator.InverseKinematics(jointDesired, newPositionFrm);
        // jointDesired[2] = jointDesired[2] / cmn180_PI * 1000.0; // ugly hack for translation   -   Zihan to check
        jointDesired[2] = jointDesired[2] * 1000.0; //ugly hack for translation
        jointDesired.resize(7);
        jointDesired.Element(6) = JointDesired.Goal().Element(6);
        JointDesired.Goal().ForceAssign(jointDesired);
        // note: this directly calls the lower level to set position,
        // maybe we should cache the request in this component and later
        // in the Run method push the request.  This way, only the latest
        // request would be pushed if multiple are queued.
        PID.SetPositionJoint(JointDesired);
    } else {
        CMN_LOG_RUN_WARNING << "PSM not ready" << std::endl;
    }
}

void mtsIntuitiveResearchKitPSM::SetGripperPosition(const double & gripperPosition)
{
    JointDesired.Goal().Element(6) = gripperPosition;
}

void mtsIntuitiveResearchKitPSM::SetRobotControlState(const std::string & state)
{
    if (RobotCurrentState != STATE_IDLE) {
        EventTriggers.RobotErrorMsg(std::string("ERROR: PSM NOT in IDLE mode, Action cancelled"));
        return;
    }

    if (state == "Start"){
        std::cout << "YES Start" << std::endl;
    }
    else if (state == "Home"){
        EventHandlerHome();
    }
    else if (state == "Teleop"){
        EventHandlerTeleop();
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerHome(void)
{
    // Set state to HOME
    RobotCurrentState = STATE_HOME;

    // Start homing here
    RobotIO.EnablePower();
    RobotIO.BiasEncoder();

    // ZC: TEMP home position all 0
    PID.GetPositionJoint(JointCurrent);
    HomeJointSet.ForceAssign(JointCurrent.Position());
    HomeJointSet.SetAll(0.0);

    // Enable PID
    PID.Enable(true);
}

void mtsIntuitiveResearchKitPSM::EventHandlerTeleop(void)
{

    // Anton: should not allow tele-op if tool in canula, i.e. j3 < 80 mm
    // send error message asking user to insert tool manually.   This will require
    // to have the Manipclutch working.
    if ((!IsAdapterEngaged) || (!IsToolEngaged)) {
        EventTriggers.RobotErrorMsg(mtsStdString("ERROR: Adapter or Tool NOT ready"));
    } else {
        RobotCurrentState = STATE_TELEOP;
        // Move J3 to > 80 mm
        PID.GetPositionJoint(JointCurrent);
        JointCurrent.Position()[2] = 80.0;
        JointDesired.Goal().ForceAssign(JointCurrent.Position());
        PID.SetPositionJoint(JointDesired);
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerAdapter(const prmEventButton &button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        CMN_LOG_CLASS_RUN_VERBOSE << "EventHandlerAdapter: adapter engaged" << std::endl;
        if (!IsHomed) {
            // ZC: might be redundant, because of the mechanical limit
            CMN_LOG_CLASS_RUN_WARNING << "EventHandlerAdapter: robot is not HOMED" << std::endl;
        } else {
            RobotCurrentState = STATE_ADAPTER;
            AdapterStopwatch.Reset();
            AdapterStopwatch.Start();
            PID.GetPositionJoint(JointCurrent);
            AdapterJointSet.ForceAssign(JointCurrent.Position());
            PID.SetIsCheckJointLimit(false);
        }
    } else {
        CMN_LOG_CLASS_RUN_VERBOSE << "EventHandlerAdapter: adapter disengaged" << std::endl;
        RobotCurrentState = STATE_IDLE;
        AdapterStopwatch.Reset();
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerTool(const prmEventButton &button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        CMN_LOG_RUN_ERROR << "Tool engaged" << std::endl;
        if (!IsAdapterEngaged) {
            // ZC: might be redundant, because of the mechanical limit
            CMN_LOG_CLASS_RUN_ERROR << "EventHandlerTool: adapter is not engaged" << std::endl;
        } else {
            RobotCurrentState = STATE_TOOL;
            ToolStopwatch.Reset();
            ToolStopwatch.Start();
            PID.GetPositionJoint(JointCurrent);
            ToolJointSet.ForceAssign(JointCurrent.Position());
            PID.SetIsCheckJointLimit(false);
        }
    } else {
        CMN_LOG_CLASS_RUN_VERBOSE << "EventHandlerTool: tool disengaged" << std::endl;
        RobotCurrentState = STATE_IDLE;
        ToolStopwatch.Reset();
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerManipClutch(const prmEventButton &button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        CMN_LOG_CLASS_RUN_ERROR << "ManipClutch press" << std::endl;
    } else {
        CMN_LOG_CLASS_RUN_ERROR << "ManipClutch release" << std::endl;
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerSUJClutch(const prmEventButton &button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        CMN_LOG_CLASS_RUN_ERROR << "SUJClutch press" << std::endl;
    } else {
        CMN_LOG_CLASS_RUN_ERROR << "SUJClutch release" << std::endl;
    }
}
