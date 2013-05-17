/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Anton Deguet
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
        req->AddFunction("GetPositionJoint", PID.GetPositionJoint);
        req->AddFunction("SetPositionJoint", PID.SetPositionJoint);
    }

    // Event Adapter (Sterile Adatper Event)
    req = AddInterfaceRequired("Adapter");
    if (req) {
        req->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerAdapter, this, "Button");
    }

    // Event Tool engaged
    req = AddInterfaceRequired("Tool");
    if (req) {
        req->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerTool, this, "Button");
    }

    // ManipClutch
    req = AddInterfaceRequired("ManipClutch");
    if (req) {
        req->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerManipClutch, this, "Button");
    }

    // SUJClutch
    req = AddInterfaceRequired("SUJClutch");
    if (req) {
        req->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSM::EventHandlerSUJClutch, this, "Button");
    }


    mtsInterfaceProvided * prov = AddInterfaceProvided("Robot");
    if (prov) {
        prov->AddCommandReadState(this->StateTable, CartesianCurrent, "GetPositionCartesian");
        prov->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetPositionCartesian, this, "SetPositionCartesian");
        prov->AddCommandWrite(&mtsIntuitiveResearchKitPSM::SetRobotControlState, this, "SetRobotControlState", mtsInt());
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
    RobotCurrentState = STATE_START;
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
        break;
    case STATE_ADAPTER:
        if (AdapterStopwatch.GetElapsedTime() > (2000 * cmn_ms)){
            AdapterJointSet[2] = 0.0;
            AdapterJointSet[3] = 0.0;
            AdapterJointSet[4] = 0.0;
            AdapterJointSet[5] = 0.0;
            AdapterJointSet[6] = 0.0;
            JointDesired.Goal().ForceAssign(AdapterJointSet);
            PID.SetPositionJoint(JointDesired);

            RobotCurrentState = STATE_START;
            AdapterStopwatch.Reset();

        }else if (AdapterStopwatch.GetElapsedTime() > (1500 * cmn_ms)){
            AdapterJointSet[3] = -4.0;
            AdapterJointSet[4] = 2.67;
            AdapterJointSet[5] = 1.01;
            AdapterJointSet[6] = 0.0;
            JointDesired.Goal().ForceAssign(AdapterJointSet);
            PID.SetPositionJoint(JointDesired);
        }else if (AdapterStopwatch.GetElapsedTime() > (1000 * cmn_ms)){
            AdapterJointSet[3] = 4.0;
            AdapterJointSet[4] = -2.67;
            AdapterJointSet[5] = -1.01;
            AdapterJointSet[6] = 0.0;
            JointDesired.Goal().ForceAssign(AdapterJointSet);
            PID.SetPositionJoint(JointDesired);
        }else if (AdapterStopwatch.GetElapsedTime() > (500 * cmn_ms)){
            AdapterJointSet[3] = -4.0;
            AdapterJointSet[4] = 2.67;
            AdapterJointSet[5] = 1.01;
            AdapterJointSet[6] = 0.0;
            JointDesired.Goal().ForceAssign(AdapterJointSet);
            PID.SetPositionJoint(JointDesired);
        }
        break;
    default:
//        cmnThrow("mtsIntuitiveResearchKitPSM: Unknown control state");
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
    if (RobotCurrentState == STATE_READY){
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



void mtsIntuitiveResearchKitPSM::SetRobotControlState(const mtsInt &state)
{
    if(state.Data == STATE_ADAPTER){
        RobotCurrentState = STATE_ADAPTER;
    }
}


// -------------- Event Handlers ------------------------------

void mtsIntuitiveResearchKitPSM::EventHandlerAdapter(const prmEventButton &button)
{
    if(button.Type() == prmEventButton::PRESSED){
        CMN_LOG_RUN_ERROR << "Adapter engaged" << std::endl;
        RobotCurrentState = STATE_ADAPTER;
        AdapterStopwatch.Reset();
        AdapterStopwatch.Start();
        PID.GetPositionJoint(JointCurrent);
        AdapterJointSet.ForceAssign(JointCurrent.Position());
    }else{
        RobotCurrentState = STATE_START;
        AdapterStopwatch.Reset();
        CMN_LOG_RUN_ERROR << "Adapter disengaged" << std::endl;
    }
}


void mtsIntuitiveResearchKitPSM::EventHandlerTool(const prmEventButton &button)
{
    if(button.Type() == prmEventButton::PRESSED){
        CMN_LOG_RUN_ERROR << "Tool engaged" << std::endl;
    }else{
        CMN_LOG_RUN_ERROR << "Tool disengaged" << std::endl;
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






