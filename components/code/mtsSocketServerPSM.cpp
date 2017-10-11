/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Pretham Chalasani, Anton Deguet
  Created on: 2016-11-04

  (C) Copyright 2016-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/mtsSocketServerPSM.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

CMN_IMPLEMENT_SERVICES_DERIVED(mtsSocketServerPSM, mtsTaskPeriodic);

mtsSocketServerPSM::mtsSocketServerPSM(const std::string & componentName, const double periodInSeconds,
                                       const std::string & ip, const unsigned int port) :
    mtsSocketBasePSM(componentName, periodInSeconds, ip, port, true),
    mIsHoming(false),
    mIsHomed(false)
{
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("PSM");
    if(interfaceRequired) {
        interfaceRequired->AddFunction("GetPositionCartesian", GetPositionCartesian);
        interfaceRequired->AddFunction("SetPositionCartesian", SetPositionCartesian);
        interfaceRequired->AddFunction("SetPositionJaw", SetPositionJaw);
        interfaceRequired->AddFunction("GetCurrentState", GetCurrentState);
        interfaceRequired->AddFunction("SetDesiredState", SetDesiredState);
        interfaceRequired->AddEventHandlerWrite(&mtsSocketServerPSM::ErrorEventHandler,
                                                this, "Error");
    }
}

void mtsSocketServerPSM::Configure(const std::string & CMN_UNUSED(fileName))
{
    DesiredState = socketMessages::SCK_UNINITIALIZED;
    CurrentState = socketMessages::SCK_UNINITIALIZED;
    State.Data.Header.Size = SERVER_MSG_SIZE;
    State.Socket->SetDestination(IpAddress, State.IpPort);
    Command.Socket->AssignPort(Command.IpPort);
}

void mtsSocketServerPSM::Run(void)
{
    ProcessQueuedEvents();
    ProcessQueuedCommands();

    ReceivePSMCommandData();
    UpdateStatistics();
    SendPSMStateData();
}

void mtsSocketServerPSM::ExecutePSMCommands(void)
{
    if (DesiredState != Command.Data.RobotControlState) {
        std::string state;
        DesiredState = Command.Data.RobotControlState;
        switch (DesiredState) {
        case socketMessages::SCK_UNINITIALIZED:
            state = "UNINITIALIZED";
            break;
        case socketMessages::SCK_HOMED:
            if (CurrentState != socketMessages::SCK_HOMING) {
                state = "READY";
            }
            break;
        case socketMessages::SCK_CART_POS:
            if (CurrentState != socketMessages::SCK_HOMING) {
                state = "READY";
            }
            CurrentState = socketMessages::SCK_CART_POS;
            break;
        default:
            std::cerr << CMN_LOG_DETAILS << Command.Data.RobotControlState << " state not supported. " << std::endl;
            break;
        }
        SetDesiredState(state);
    }

    // Only send when in cartesian mode
    switch (CurrentState) {
    case socketMessages::SCK_CART_POS:
        // send cartesian goal
        PositionCartesianSet.Goal().From(Command.Data.GoalPose);
        SetPositionCartesian(PositionCartesianSet);
        // send jaw goal
        PositionJointSet.Goal().SetSize(1);
        PositionJointSet.Goal().Element(0) = Command.Data.GoalJaw;
        SetPositionJaw(PositionJointSet);
        break;
    default:
        break;
    }
}

void mtsSocketServerPSM::ReceivePSMCommandData(void)
{
    // Recv Socket Data
    int bytesRead = 0;
    bytesRead = Command.Socket->Receive(Command.Buffer, BUFFER_SIZE, TIMEOUT);
    if (bytesRead > 0) {
        if (bytesRead != Command.Data.Header.Size) {
            std::cerr << CMN_LOG_DETAILS << "Incorrect bytes read " << bytesRead << ". Looking for " << Command.Data.Header.Size << " bytes." << std::endl;
        }

        std::stringstream ss;
        cmnDataFormat local, remote;
        ss.write(Command.Buffer, bytesRead);

        // Dequeue all the datagrams and only use the latest one.
        int readCounter = 0;
        int dataLeft = bytesRead;
        while (dataLeft > 0) {
            dataLeft = Command.Socket->Receive(Command.Buffer, BUFFER_SIZE, 0);
            if (dataLeft != 0) {
                bytesRead = dataLeft;
            }
            readCounter++;
        }

        if (readCounter > 1) {
            std::cerr << CMN_LOG_DETAILS << "Catching up : " << readCounter << std::endl;
        }

        ss.write(Command.Buffer, bytesRead);
        cmnData<socketCommandPSM>::DeSerializeBinary(Command.Data, ss, local, remote);

        Command.Data.GoalPose.NormalizedSelf();
        ExecutePSMCommands();

    } else {
        CMN_LOG_CLASS_RUN_DEBUG << "RecvPSMCommandData: UDP receive failed" << std::endl;
    }
}

void mtsSocketServerPSM::UpdatePSMState(void)
{
    // Update PSM State
    mtsExecutionResult executionResult;

    // Get Cartesian position
    executionResult = GetPositionCartesian(PositionCartesianCurrent);
    State.Data.CurrentPose.Assign(PositionCartesianCurrent.Position());

    // Get Robot State
    mtsStdString psmState;
    GetCurrentState(psmState);

    // Switch to socket states
    if (psmState.Data == "UNINITIALIZED") {
        CurrentState = socketMessages::SCK_UNINITIALIZED;
    } else if ((psmState.Data == "READY")) {
        if ((CurrentState != socketMessages::SCK_HOMED)
            && (CurrentState != socketMessages::SCK_CART_POS)) {
            CurrentState = socketMessages::SCK_HOMED;
        }
    } else {
        CurrentState = socketMessages::SCK_HOMING;
    }
}

void mtsSocketServerPSM::SendPSMStateData(void)
{
    UpdatePSMState();

    // Update Header
    State.Data.Header.Id++;
    State.Data.Header.Timestamp = mTimeServer.GetRelativeTime();
    State.Data.Header.LastId = Command.Data.Header.Id;
    State.Data.Header.LastTimestamp = Command.Data.Header.Timestamp;
    State.Data.RobotControlState = CurrentState;

    // Send Socket Data
    std::stringstream ss;
    cmnData<socketStatePSM>::SerializeBinary(State.Data, ss);
    memcpy(State.Buffer, ss.str().c_str(), ss.str().length());

    State.Socket->Send(State.Buffer, ss.str().size());
}

void mtsSocketServerPSM::ErrorEventHandler(const mtsMessage & CMN_UNUSED(message))
{
    // Send error message to the client
    //State.Data.Error = message;
    State.Data.RobotControlState = socketMessages::SCK_UNINITIALIZED;
}
