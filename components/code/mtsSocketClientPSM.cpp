/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Pretham Chalasani, Anton Deguet
  Created on: 2016-11-04

  (C) Copyright 2016-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/mtsSocketClientPSM.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

CMN_IMPLEMENT_SERVICES_DERIVED(mtsSocketClientPSM, mtsTaskPeriodic);

mtsSocketClientPSM::mtsSocketClientPSM(const std::string & componentName, const double periodInSeconds,
                                       const std::string & ip, const unsigned int port) :
    mtsSocketBasePSM(componentName, periodInSeconds, ip, port, false)
{
    this->StateTable.AddData(PositionCartesianCurrent, "PositionCartesianCurrent");
    StateJaw.Position().resize(1);
    this->StateTable.AddData(StateJaw, "StateJaw");

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Robot");
    if (interfaceProvided) {
        interfaceProvided->AddMessageEvents();
        interfaceProvided->AddCommandReadState(this->StateTable, PositionCartesianCurrent, "measured_cp");
        interfaceProvided->AddCommandReadState(this->StateTable, StateJaw, "GetStateJaw");

        interfaceProvided->AddCommandVoid(&mtsSocketClientPSM::Freeze,
                                          this, "Freeze");
        interfaceProvided->AddCommandWrite(&mtsSocketClientPSM::servo_cp,
                                           this , "servo_cp");
        interfaceProvided->AddCommandWrite(&mtsSocketClientPSM::SetPositionJaw,
                                           this , "SetPositionJaw");
        interfaceProvided->AddCommandWrite(&mtsSocketClientPSM::SetDesiredState,
                                           this , "SetDesiredState");
        interfaceProvided->AddCommandRead(&mtsSocketClientPSM::GetDesiredState,
                                           this , "GetDesiredState");
        interfaceProvided->AddCommandRead(&mtsSocketClientPSM::GetCurrentState,
                                           this , "GetCurrentState");
    }
}

void mtsSocketClientPSM::Configure(const std::string & CMN_UNUSED(fileName))
{
    DesiredState = socketMessages::SCK_UNINITIALIZED;
    CurrentState = socketMessages::SCK_UNINITIALIZED;
    Command.Data.Header.Size = CLIENT_MSG_SIZE;
    Command.Socket->SetDestination(IpAddress, Command.IpPort);
    State.Socket->AssignPort(State.IpPort);
}

void mtsSocketClientPSM::Run(void)
{
    ProcessQueuedEvents();
    ProcessQueuedCommands();

    ReceivePSMStateData();
    UpdateStatistics();
    SendPSMCommandData();
}

void mtsSocketClientPSM::UpdateApplication(void)
{
    CurrentState = State.Data.RobotControlState;
    PositionCartesianCurrent.Valid() = (CurrentState >= socketMessages::SCK_HOMED);
    PositionCartesianCurrent.Position().FromNormalized(State.Data.CurrentPose);
    StateJaw.Position().at(0) = State.Data.CurrentJaw;
}

void mtsSocketClientPSM::Freeze(void)
{
    DesiredState = socketMessages::SCK_CART_POS;
    Command.Data.GoalPose.From(State.Data.CurrentPose);
    Command.Data.GoalJaw = State.Data.CurrentJaw;
}

void mtsSocketClientPSM::SetDesiredState(const std::string & state)
{
    if (state == "DISABLED") {
        DesiredState = socketMessages::SCK_UNINITIALIZED;
    } else if (state == "READY") {
        DesiredState = socketMessages::SCK_HOMED;
    } else {
        std::cerr << CMN_LOG_DETAILS << state << " state not supported." << std::endl;
    }

    Command.Data.RobotControlState = DesiredState;
    Command.Data.GoalPose.From(State.Data.CurrentPose);
    Command.Data.GoalJaw = State.Data.CurrentJaw;
}

void mtsSocketClientPSM::servo_cp(const prmPositionCartesianSet & position)
{
    DesiredState = socketMessages::SCK_CART_POS;
    Command.Data.GoalPose.From(position.Goal());
}

void mtsSocketClientPSM::SetPositionJaw(const prmPositionJointSet & position)
{
    DesiredState = socketMessages::SCK_CART_POS;
    Command.Data.GoalJaw = position.Goal().at(0);
}

void mtsSocketClientPSM::GetDesiredState(std::string & state) const
{
    switch (DesiredState) {
    case socketMessages::SCK_UNINITIALIZED:
        state = "DISABLED";
        break;
    case socketMessages::SCK_HOMING:
    case socketMessages::SCK_HOMED:
    case socketMessages::SCK_CART_POS:
        state = "READY";
        break;
    default:
        std::cerr << CMN_LOG_DETAILS << state << " state not supported." << std::endl;
        break;
    }
}

void mtsSocketClientPSM::GetCurrentState(std::string & state) const
{
    switch (CurrentState) {
    case socketMessages::SCK_UNINITIALIZED:
        state = "DISABLED";
        break;
    case socketMessages::SCK_HOMING:
    case socketMessages::SCK_HOMED:
    case socketMessages::SCK_CART_POS:
        state = "READY";
        break;
    default:
        std::cerr << CMN_LOG_DETAILS << state << " state not supported." << std::endl;
        break;
    }
}

void mtsSocketClientPSM::ReceivePSMStateData(void)
{
    // Recv Scoket Data
    size_t bytesRead = 0;
    bytesRead = State.Socket->Receive(State.Buffer, BUFFER_SIZE, TIMEOUT);
    if (bytesRead > 0) {
        if (bytesRead != State.Data.Header.Size){
            std::cerr << CMN_LOG_DETAILS << "Incorrect bytes read " << bytesRead << ". Looking for " << State.Data.Header.Size << " bytes." << std::endl;
        }

        std::stringstream ss;
        cmnDataFormat local, remote;
        ss.write(State.Buffer, bytesRead);

        // Dequeue all the datagrams and only use the latest one.
        int readCounter = 0;
        int dataLeft = bytesRead;
        while (dataLeft > 0) {
            dataLeft = State.Socket->Receive(State.Buffer, BUFFER_SIZE, 0);
            if (dataLeft != 0) {
                bytesRead = dataLeft;
            }

            readCounter++;
        }

        if (readCounter > 1) {
            std::cerr << CMN_LOG_DETAILS << "Catching up : " << readCounter << std::endl;
        }

        ss.write(State.Buffer, bytesRead);
        cmnData<socketStatePSM>::DeSerializeBinary(State.Data, ss, local, remote);

        State.Data.CurrentPose.NormalizedSelf();
        UpdateApplication();
    } else {
        CMN_LOG_CLASS_RUN_DEBUG << "RecvPSMStateData: UDP receive failed" << std::endl;
    }
}

void mtsSocketClientPSM::SendPSMCommandData(void)
{
    // Update Header
    Command.Data.Header.Id++;
    Command.Data.Header.Timestamp = mTimeServer.GetRelativeTime();
    Command.Data.Header.LastId = State.Data.Header.Id;
    Command.Data.Header.LastTimestamp = State.Data.Header.Timestamp;
    Command.Data.RobotControlState = DesiredState;

    // Send Socket Data
    std::stringstream ss;
    cmnData<socketCommandPSM>::SerializeBinary(Command.Data, ss);
    memcpy(Command.Buffer, ss.str().c_str(), ss.str().length());

    Command.Socket->Send(Command.Buffer, ss.str().size());
}
