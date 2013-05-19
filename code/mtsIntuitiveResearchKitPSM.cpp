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

    // Setup CISST Interface
    mtsInterfaceRequired * req;
    req = AddInterfaceRequired("PID");
    if (req) {
        req->AddFunction("Enable", PID.Enable);
        req->AddFunction("GetPositionJoint", PID.GetPositionJoint);
        req->AddFunction("SetPositionJoint", PID.SetPositionJoint);
        req->AddFunction("SetIsCheckJointLimit", PID.SetIsCheckJointLimit);
    }

    // Robot IO
    req = AddInterfaceRequired("RobotIO");
    if (req) {
        req->AddFunction("EnablePower", RobotIO.EnablePower);
        req->AddFunction("DisablePower", RobotIO.DisablePower);
        req->AddFunction("BiasEncoder", RobotIO.BiasEncoder);
    }

    // Event Adapter engage: digital input button event from PSM
    req = AddInterfaceRequired("Adapter");
    if (req) {
        req->AddFunction("GetButton", Adapter.GetButton);
        req->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerAdapter, this, "Button");
    }

    // Event Tool engage: digital input button event from PSM
    req = AddInterfaceRequired("Tool");
    if (req) {
        req->AddFunction("GetButton", Tool.GetButton);
        req->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerTool, this, "Button");
    }

    // ManipClutch: digital input button event from PSM
    req = AddInterfaceRequired("ManipClutch");
    if (req) {
        req->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerManipClutch, this, "Button");
    }

    // SUJClutch: digital input button event from PSM
    req = AddInterfaceRequired("SUJClutch");
    if (req) {
        req->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerSUJClutch, this, "Button");
    }


    mtsInterfaceProvided * prov = AddInterfaceProvided("Robot");
    if (prov) {
        prov->AddCommandReadState(this->StateTable, CartesianCurrent, "GetPositionCartesian");
        prov->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetPositionCartesian, this, "SetPositionCartesian");

        prov->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetRobotControlState,
                              this, "SetRobotControlState", mtsStdString());
        prov->AddEventWrite(EventTriggers.RobotStatusMsg, "RobotStatusMsg", mtsStdString());
        prov->AddEventWrite(EventTriggers.RobotErrorMsg, "RobotErrorMsg", mtsStdString());
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

    // JointCurrent.Position()[2] = JointCurrent.Position()[2] * cmn180_PI / 1000.0; // ugly hack to convert radians to degrees to meters   - Zihan to check
    JointCurrent.Position()[2] = JointCurrent.Position()[2] / 1000.0; // ugly hack to convert mm to meters
    vctFrm4x4 position;
    position = Manipulator.ForwardKinematics(JointCurrent.Position());
    position.Rotation().NormalizedSelf();
    CartesianCurrent.Position().From(position);

    switch (RobotCurrentState) {
    case STATE_TELEOP:
        // ZC: no thing for teleop
        break;

    case STATE_HOME:
    {
        vctDoubleVec HomeError;
        vctDoubleVec HomeErrorTolerance;

        // check position
        PID.GetPositionJoint(JointCurrent);
        HomeError.SetSize(HomeJointSet.size());
        HomeError.DifferenceOf(HomeJointSet, JointCurrent.Position());
        HomeError.AbsSelf();

        HomeErrorTolerance.SetSize(HomeJointSet.size());
        HomeErrorTolerance.SetAll(2.0 * cmnPI_180);
        HomeErrorTolerance[2] = 1.0;  // 1mm
        IsHomed = true;
        for(size_t i = 0; i < HomeJointSet.size(); i++){
            if ( HomeError[i] > HomeErrorTolerance[i]){
                IsHomed = false;
            }
        }
        if(IsHomed){
            RobotCurrentState = STATE_IDLE;
            EventTriggers.RobotStatusMsg(mtsStdString("PSM Homed"));

            Adapter.GetButton(Adapter.IsPresent);
            Tool.GetButton(Tool.IsPresent);
            std::cerr << "HOME DONE" << std::endl;

            if(Adapter.IsPresent && !Tool.IsPresent){
                prmEventButton button;
                button.Type() = prmEventButton::PRESSED;
                EventHandlerAdapter(button);
            }else if (Adapter.IsPresent && Tool.IsPresent){
                IsAdapterEngaged = true;
                prmEventButton button;
                button.Type() = prmEventButton::PRESSED;
                EventHandlerTool(button);
            }

        }else{
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
        CMN_LOG_RUN_ERROR << "Unknown control state" << std::endl;
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
    if (RobotCurrentState == STATE_TELEOP){
        vctDoubleVec jointDesired;
        jointDesired.ForceAssign(JointCurrent.Position());
        jointDesired.resize(6);
        Manipulator.InverseKinematics(jointDesired, newPosition.Goal());
        // jointDesired[2] = jointDesired[2] / cmn180_PI * 1000.0; // ugly hack for translation   -   Zihan to check
        jointDesired[2] = jointDesired[2] * 1000.0; //ugly hack for translation
        jointDesired.resize(7);
        jointDesired.Element(6) = 0.5; // temporary hack to set gripper opening
        JointDesired.Goal().ForceAssign(jointDesired);
        // note: this directly calls the lower level to set position,
        // maybe we should cache the request in this component and later
        // in the Run method push the request.  This way, only the latest
        // request would be pushed if multiple are queued.
        PID.SetPositionJoint(JointDesired);
    }else{
        CMN_LOG_RUN_WARNING << "PSM not ready" << std::endl;
    }
}



void mtsIntuitiveResearchKitPSM::SetRobotControlState(const mtsStdString &state)
{
    if (RobotCurrentState != STATE_IDLE){
        EventTriggers.RobotErrorMsg(
                    mtsStdString("ERROR: PSM NOT in IDLE mode, Action cancelled"));
        return;
    }

    if (state.Data == "Start"){
        std::cout << "YES Start" << std::endl;
    }
    else if (state.Data == "Home"){
        EventHandlerHome();
    }
    else if (state.Data == "Teleop"){
        EventHandlerTeleop();
    }
}


// -------------- Event Handlers ------------------------------


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
    if( (!IsAdapterEngaged) || (!IsToolEngaged) ){
        EventTriggers.RobotErrorMsg(mtsStdString("ERROR: Adapter or Tool NOT ready"));
    }else{
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
    if(button.Type() == prmEventButton::PRESSED){
        CMN_LOG_RUN_ERROR << "Adapter engaged" << std::endl;
        if (!IsHomed){
            // ZC: might be redundant, because of the mechanical limit
            CMN_LOG_RUN_ERROR << "Robot is not HOMED" << std::endl;
        }else{
            RobotCurrentState = STATE_ADAPTER;
            AdapterStopwatch.Reset();
            AdapterStopwatch.Start();
            PID.GetPositionJoint(JointCurrent);
            AdapterJointSet.ForceAssign(JointCurrent.Position());
            PID.SetIsCheckJointLimit(false);
        }
    }else{
        CMN_LOG_RUN_ERROR << "Adapter disengaged" << std::endl;
        RobotCurrentState = STATE_IDLE;
        AdapterStopwatch.Reset();
    }
}


void mtsIntuitiveResearchKitPSM::EventHandlerTool(const prmEventButton &button)
{
    if(button.Type() == prmEventButton::PRESSED){
        CMN_LOG_RUN_ERROR << "Tool engaged" << std::endl;
        if (!IsAdapterEngaged){
            // ZC: might be redundant, because of the mechanical limit
            CMN_LOG_RUN_ERROR << "Adapter is not engaged" << std::endl;
        }else{
            RobotCurrentState = STATE_TOOL;
            ToolStopwatch.Reset();
            ToolStopwatch.Start();
            PID.GetPositionJoint(JointCurrent);
            ToolJointSet.ForceAssign(JointCurrent.Position());
            PID.SetIsCheckJointLimit(false);
        }
    }else{
        CMN_LOG_RUN_ERROR << "Tool disengaged" << std::endl;
        RobotCurrentState = STATE_IDLE;
        ToolStopwatch.Reset();
    }
}

void mtsIntuitiveResearchKitPSM::EventHandlerManipClutch(const prmEventButton &button)
{
    if(button.Type() == prmEventButton::PRESSED){
        CMN_LOG_RUN_ERROR << "ManipClutch press" << std::endl;
    }else{
        CMN_LOG_RUN_ERROR << "ManipClutch release" << std::endl;
    }
}


void mtsIntuitiveResearchKitPSM::EventHandlerSUJClutch(const prmEventButton &button)
{
    if(button.Type() == prmEventButton::PRESSED){
        CMN_LOG_RUN_ERROR << "SUJClutch press" << std::endl;
    }else{
        CMN_LOG_RUN_ERROR << "SUJClutch release" << std::endl;
    }
}






