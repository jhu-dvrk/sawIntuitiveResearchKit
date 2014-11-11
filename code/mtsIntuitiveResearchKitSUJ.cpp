/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-11-07

  (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitSUJ.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitSUJ, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsIntuitiveResearchKitSUJ::mtsIntuitiveResearchKitSUJ(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    Init();
}

mtsIntuitiveResearchKitSUJ::mtsIntuitiveResearchKitSUJ(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg)
{
    Init();
}

void mtsIntuitiveResearchKitSUJ::Init(void)
{
    mCounter = 0;
    mMuxTimer = 0.0;
    mMuxUp = false;

    SetState(SUJ_UNINITIALIZED);

    // Robot IO
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("RobotIO");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("EnablePower", RobotIO.EnablePower);
        interfaceRequired->AddFunction("DisablePower", RobotIO.DisablePower);
        interfaceRequired->AddFunction("GetEncoderChannelA", RobotIO.GetEncoderChannelA);
        interfaceRequired->AddFunction("GetActuatorAmpStatus", RobotIO.GetActuatorAmpStatus);
        interfaceRequired->AddFunction("SetActuatorCurrent", RobotIO.SetActuatorCurrent);
    }
    interfaceRequired = AddInterfaceRequired("MuxReset");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetValue", MuxReset.GetValue);
        interfaceRequired->AddFunction("SetValue", MuxReset.SetValue);
    }
    interfaceRequired = AddInterfaceRequired("MuxIncrement");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetValue", MuxIncrement.GetValue);
        interfaceRequired->AddFunction("SetValue", MuxIncrement.SetValue);
    }

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Robot");
    if (interfaceProvided) {
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitSUJ::SetRobotControlState,
                                           this, "SetRobotControlState", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.RobotStatusMsg, "RobotStatusMsg", std::string(""));
        interfaceProvided->AddEventWrite(EventTriggers.RobotErrorMsg, "RobotErrorMsg", std::string(""));
        // Stats
        interfaceProvided->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                               "GetPeriodStatistics");
    }
}

void mtsIntuitiveResearchKitSUJ::Configure(const std::string & filename)
{
    std::cerr << CMN_LOG_DETAILS << " - need to load JSON config file with DH and ..." << std::endl;
}

void mtsIntuitiveResearchKitSUJ::Startup(void)
{
    this->SetState(SUJ_UNINITIALIZED);
}

void mtsIntuitiveResearchKitSUJ::Run(void)
{
    mCounter++;

    ProcessQueuedEvents();
    GetRobotData();

    switch (mRobotState) {
    case SUJ_UNINITIALIZED:
        break;
    case SUJ_HOMING_POWERING:
        RunHomingPower();
        break;
    case SUJ_READY:
        break;
    case SUJ_MANUAL:
        break;
    default:
        break;
    }

    RunEvent();
    ProcessQueuedCommands();
}

void mtsIntuitiveResearchKitSUJ::Cleanup(void)
{
    std::cerr << CMN_LOG_DETAILS << " we should make sure current is turned off and brakes are engaged" << std::endl;
    CMN_LOG_CLASS_INIT_VERBOSE << GetName() << ": Cleanup" << std::endl;
}

void mtsIntuitiveResearchKitSUJ::GetRobotData(void)
{
    const double muxCycle = 0.5 * cmn_s;
    const double muxIncrementDownTime = 0.1 * cmn_s;

    // we can start reporting some joint values after the robot is powered
    if (this->mRobotState > SUJ_HOMING_POWERING) {
        const double currentTime = this->StateTable.GetTic();
        mtsExecutionResult executionResult;

        // toggle
        if (currentTime > mMuxTimer) {
            mMuxTimer = currentTime + muxCycle;
            executionResult = MuxIncrement.SetValue(true);
            mMuxUp = true;
        } else {
            if (mMuxUp && (currentTime > (mMuxTimer - muxIncrementDownTime))) {
                executionResult = MuxIncrement.SetValue(false);
                mMuxUp = false;
            }
        }
    }
    if (mCounter%100 == 0) {
        vctBoolVec encA(4);
        RobotIO.GetEncoderChannelA(encA);
        std::cerr << "enc A: " << encA << std::endl;
    }
}

void mtsIntuitiveResearchKitSUJ::SetState(const RobotStateType & newState)
{
    CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": SetState: new state " << newState << std::endl;

    switch (newState) {

    case SUJ_UNINITIALIZED:
        mRobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " not initialized");
        break;

    case SUJ_HOMING_POWERING:
        mHomingTimer = 0.0;
        mHomingPowerRequested = false;
        mRobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " powering");
        break;

    case SUJ_READY:
        // when returning from manual mode, need to re-enable PID
        mRobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " ready");
        break;

    case SUJ_MANUAL:
        if (this->mRobotState < SUJ_READY) {
            EventTriggers.RobotErrorMsg(this->GetName() + " is not ready yet");
            return;
        }
        std::cerr << CMN_LOG_DETAILS << " should release breaks now" << std::endl;
        mRobotState = newState;
        EventTriggers.RobotStatusMsg(this->GetName() + " in manual mode");
        break;
    default:
        break;
    }
}

void mtsIntuitiveResearchKitSUJ::RunHomingPower(void)
{
    const double timeToPower = 3.0 * cmn_s;

    const double currentTime = this->StateTable.GetTic();
    // first, request power to be turned on
    if (!mHomingPowerRequested) {
        mHomingTimer = currentTime;
        // pre-load the boards with zero current
        RobotIO.SetActuatorCurrent(vctDoubleVec(NumberOfJoints, 0.0));
        // enable power and set a flags to move to next step
        RobotIO.EnablePower();
        mHomingPowerRequested = true;
        EventTriggers.RobotStatusMsg(this->GetName() + " power requested");
        return;
    }

    // second, check status
    if (mHomingPowerRequested
        && ((currentTime - mHomingTimer) > timeToPower)) {
        // check power status
        vctBoolVec actuatorAmplifiersStatus(NumberOfJoints);
        RobotIO.GetActuatorAmpStatus(actuatorAmplifiersStatus);
        if (actuatorAmplifiersStatus.All()) {
            EventTriggers.RobotStatusMsg(this->GetName() + " power on");
            this->SetState(SUJ_READY);
        } else {
            EventTriggers.RobotErrorMsg(this->GetName() + " failed to enable power.");
            this->SetState(SUJ_UNINITIALIZED);
        }
    }
}

void mtsIntuitiveResearchKitSUJ::SetRobotControlState(const std::string & state)
{
    if (state == "Home") {
        SetState(SUJ_HOMING_POWERING);
    } else if (state == "Manual") {
        SetState(SUJ_MANUAL);
    } else {
        EventTriggers.RobotErrorMsg(this->GetName() + ": unsupported state " + state);
    }
}
