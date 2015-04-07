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

class mtsIntuitiveResearchKitSUJArmData
{
public:
    mtsIntuitiveResearchKitSUJArmData(const std::string & name,
                                      mtsInterfaceProvided * interfaceProvided):
        mName(name),
        mStateTable(500, name)
    {
        // state table doesn't always advance, only when pots are stable
        mStateTable.SetAutomaticAdvance(false);

        for (size_t potArray = 0; potArray < 2; ++potArray) {
            mVoltages[potArray].SetSize(8);
            mPositions[potArray].SetSize(8);
            mVoltageToPositionScales[potArray].SetSize(8);
            mVoltageToPositionOffsets[potArray].SetSize(8);
        }

        mStateTable.AddData(this->mVoltages[0], "Voltages[0]");
        mStateTable.AddData(this->mVoltages[1], "Voltages[1]");
        CMN_ASSERT(interfaceProvided);
        interfaceProvided->AddCommandReadState(mStateTable, mVoltages[0], "Voltages[0]");
        interfaceProvided->AddCommandReadState(mStateTable, mVoltages[1], "Voltages[1]");
    }

    // name of this SUJ arm
    std::string mName;

    // state of this SUJ arm
    mtsStateTable mStateTable;

    // 2 arrays, one for each set of potentiometers
    vctFixedSizeVector<vctDoubleVec, 2>
        mVoltages,
        mPositions,
        mVoltageToPositionScales,
        mVoltageToPositionOffsets;
};

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
    mMuxState.SetSize(4);
    mVoltages.SetSize(4);

    SetState(SUJ_UNINITIALIZED);

    // Robot IO
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("RobotIO");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("EnablePower", RobotIO.EnablePower);
        interfaceRequired->AddFunction("DisablePower", RobotIO.DisablePower);
        interfaceRequired->AddFunction("GetEncoderChannelA", RobotIO.GetEncoderChannelA);
        interfaceRequired->AddFunction("GetActuatorAmpStatus", RobotIO.GetActuatorAmpStatus);
        interfaceRequired->AddFunction("SetActuatorCurrent", RobotIO.SetActuatorCurrent);
        interfaceRequired->AddFunction("GetAnalogInputVolts", RobotIO.GetAnalogInputVolts);
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
        interfaceProvided->AddEventWrite(MessagesEvents.RobotStatus, "RobotStatusMsg", std::string(""));
        interfaceProvided->AddEventWrite(MessagesEvents.RobotError, "RobotErrorMsg", std::string(""));
        // Stats
        interfaceProvided->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                               "GetPeriodStatistics");
    }

    // allocate data for each SUJ and setup interfaces
    for (size_t i = 0; i < 4; ++i) {
        std::stringstream interfaceNameStream;
        interfaceNameStream << "SUJ" << i + 1;
        std::string interfaceName = interfaceNameStream.str();
        mtsInterfaceProvided * interface = this->AddInterfaceProvided(interfaceName);
        Arms[i] = new mtsIntuitiveResearchKitSUJArmData(interfaceName, interface);
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
    // make sure requested current is back to 0
    vctDoubleVec zero(4, 0.0);
    RobotIO.SetActuatorCurrent(zero);
    // turn off amplifiers
    RobotIO.DisablePower();
}

void mtsIntuitiveResearchKitSUJ::GetRobotData(void)
{
    // every 100 ms, set mux increment signal up for 1 ms, this is arbitrary slow, all we need is to make
    // 100 ms is to make sure A2D stabilizes
    const double muxCycle = 100.0 * cmn_ms;
    const double muxIncrementDownTime = 1.0 * cmn_ms;

    mtsExecutionResult executionResult;

    // we can start reporting some joint values after the robot is powered
    if (this->mRobotState > SUJ_HOMING_POWERING) {
        const double currentTime = this->StateTable.GetTic();

        // toggle
        if (currentTime > mMuxTimer) {
            // time to raise signal to increment mux
            mMuxTimer = currentTime + muxCycle;
            executionResult = MuxIncrement.SetValue(true);
            mMuxUp = true;
        } else {
            // time to lower signal to increment mux
            if (mMuxUp && (currentTime > (mMuxTimer - muxIncrementDownTime))) {
                executionResult = MuxIncrement.SetValue(false);
                mMuxUp = false;
                mMuxIndexExpected = (mMuxIndexExpected + 1) % 16;
                // pot values should be stable by now, get pots values
                GetAndConvertPotentiometerValues();
            }
        }
    }
}

void mtsIntuitiveResearchKitSUJ::GetAndConvertPotentiometerValues(void)
{
    // read encoder channel A to get the mux state
    mtsExecutionResult executionResult = RobotIO.GetEncoderChannelA(mMuxState);
    // compute pot index
    mMuxIndex = (mMuxState[0]?1:0) + (mMuxState[1]?2:0) + (mMuxState[2]?4:0) + (mMuxState[3]?8:0);
    if (mMuxIndex != mMuxIndexExpected) {
        std::cerr << CMN_LOG_DETAILS << "mux from IO board: " << mMuxIndex << " expected: " << mMuxIndexExpected << std::endl;
        return;
    }
    // array index, 0 or 1, primary or secondary pots
    const size_t arrayIndex = mMuxIndex / 8;
    executionResult = RobotIO.GetAnalogInputVolts(mVoltages);
    // for each arm, i.e. SUJ1, SUJ2, SUJ3, ...
    for (size_t armIndex = 0; armIndex < 4; ++armIndex) {
        if (mMuxState.Equal(0)) {
            Arms[armIndex]->mStateTable.Start();
        }
        Arms[armIndex]->mVoltages[arrayIndex][mMuxIndex%8] = mVoltages[armIndex];
        if (mMuxState.Equal(1)) {
            Arms[armIndex]->mStateTable.Advance();
        }
    }
    // debug code
    // for (size_t armIndex = 0; armIndex < 4; ++armIndex) {
        if (mMuxState.Equal(1)) {
            size_t armIndex = 2;
            std::cerr << "Arm " << armIndex << " A " << Arms[armIndex]->mVoltages[0] << " B " << Arms[armIndex]->mVoltages[1] << std::endl;
        }
    // }
}

void mtsIntuitiveResearchKitSUJ::SetState(const RobotStateType & newState)
{
    CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": SetState: new state " << newState << std::endl;

    switch (newState) {

    case SUJ_UNINITIALIZED:
        mRobotState = newState;
        MessagesEvents.RobotStatus(this->GetName() + " not initialized");
        break;

    case SUJ_HOMING_POWERING:
        mHomingTimer = 0.0;
        mHomingPowerRequested = false;
        mRobotState = newState;
        MessagesEvents.RobotStatus(this->GetName() + " powering");
        break;

    case SUJ_READY:
        // when returning from manual mode, need to re-enable PID
        mRobotState = newState;
        MessagesEvents.RobotStatus(this->GetName() + " ready");
        break;

    case SUJ_MANUAL:
        if (this->mRobotState < SUJ_READY) {
            MessagesEvents.RobotError(this->GetName() + " is not ready yet");
            return;
        }
        std::cerr << CMN_LOG_DETAILS << " should release breaks now" << std::endl;
        mRobotState = newState;
        MessagesEvents.RobotStatus(this->GetName() + " in manual mode");
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
        MessagesEvents.RobotStatus(this->GetName() + " power requested");
        return;
    }

    // second, check status
    if (mHomingPowerRequested
        && ((currentTime - mHomingTimer) > timeToPower)) {
        // check power status
        vctBoolVec actuatorAmplifiersStatus(NumberOfJoints);
        RobotIO.GetActuatorAmpStatus(actuatorAmplifiersStatus);
        if (actuatorAmplifiersStatus.All()) {
            MessagesEvents.RobotStatus(this->GetName() + " power on");
            this->SetState(SUJ_READY);

            // hack to reset mux
            std::cerr << "Add new state to reset mux? to avoid blocking code?" << std::endl;
            MuxReset.SetValue(true);
            this->Sleep(3.0 * cmn_ms);
            MuxReset.SetValue(false);
            mMuxIndexExpected = 0;
        } else {
            MessagesEvents.RobotError(this->GetName() + " failed to enable power.");
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
        MessagesEvents.RobotError(this->GetName() + ": unsupported state " + state);
    }
}
