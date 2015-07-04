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
#include <cisstParameterTypes/prmPositionCartesianGet.h>

const size_t MUX_ARRAY_SIZE = 6;
const size_t MUX_MAX_INDEX = 11;

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitSUJ, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

class mtsIntuitiveResearchKitSUJArmData
{
public:
    inline mtsIntuitiveResearchKitSUJArmData(const std::string & name,
                                             mtsInterfaceProvided * interfaceProvided):
        mName(name),
        mStateTable(500, name),
        mStateTableConfiguration(100, name + "Configuration")
    {
        // state table doesn't always advance, only when pots are stable
        mStateTable.SetAutomaticAdvance(false);

        for (size_t potArray = 0; potArray < 2; ++potArray) {
            mVoltages[potArray].SetSize(MUX_ARRAY_SIZE);
            mPositions[potArray].SetSize(MUX_ARRAY_SIZE);
            mVoltageToPositionScales[potArray].SetSize(MUX_ARRAY_SIZE);
            mVoltageToPositionOffsets[potArray].SetSize(MUX_ARRAY_SIZE);
        }

        mPositionJoint.SetSize(MUX_ARRAY_SIZE);

        mStateTable.AddData(this->mVoltages[0], "Voltages[0]");
        mStateTable.AddData(this->mVoltages[1], "Voltages[1]");
        mStateTable.AddData(this->mPositionJoint, "PositionJoint");
        mStateTable.AddData(this->mPositionCartesianParam, "PositionCartesian");
        mStateTableConfiguration.AddData(this->mName, "Name");
        mStateTableConfiguration.AddData(this->mSerialNumber, "SerialNumber");
        mStateTableConfiguration.AddData(this->mPlugNumber, "PlugNumber");

        CMN_ASSERT(interfaceProvided);
        interfaceProvided->AddCommandReadState(mStateTable, mPositionCartesianParam, "GetPositionCartesian");
        interfaceProvided->AddCommandReadState(mStateTable, mVoltages[0], "GetVoltagesPrimary");
        interfaceProvided->AddCommandReadState(mStateTable, mVoltages[1], "GetVoltagesSecondary");
        interfaceProvided->AddCommandReadState(mStateTableConfiguration, mName, "GetName");
        interfaceProvided->AddCommandReadState(mStateTableConfiguration, mSerialNumber, "GetSerialNumber");
        interfaceProvided->AddCommandReadState(mStateTableConfiguration, mPlugNumber, "GetPlugNumber");
        // Stats
        interfaceProvided->AddCommandReadState(mStateTable, mStateTable.PeriodStats,
                                               "GetPeriodStatistics");
    }

    // name of this SUJ arm (ECM, PSM1, ...)
    mtsStdString mName;
    // serial number
    mtsStdString mSerialNumber;
    // plug on back of controller, 1 to 4
    unsigned int mPlugNumber;

    // state of this SUJ arm
    mtsStateTable mStateTable;
    mtsStateTable mStateTableConfiguration;

    // 2 arrays, one for each set of potentiometers
    vctDoubleVec mVoltages[2];
    vctDoubleVec mPositions[2];
    vctDoubleVec mVoltageToPositionScales[2];
    vctDoubleVec mVoltageToPositionOffsets[2];
    vctDoubleVec mPositionJoint;
    prmPositionCartesianGet mPositionCartesianParam;
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
    mMuxState.SetSize(4);
    mVoltages.SetSize(4);

    SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);

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
        interfaceRequired->AddFunction("DownUpDown", MuxReset.DownUpDown);
    }
    interfaceRequired = AddInterfaceRequired("MuxIncrement");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetValue", MuxIncrement.GetValue);
        interfaceRequired->AddFunction("SetValue", MuxIncrement.SetValue);
        interfaceRequired->AddFunction("DownUpDown", MuxIncrement.DownUpDown);
    }

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Robot");
    if (interfaceProvided) {
        // Robot State
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitSUJ::SetRobotControlState,
                                           this, "SetRobotControlState", std::string(""));
        interfaceProvided->AddCommandRead(&mtsIntuitiveResearchKitSUJ::GetRobotControlState,
                                          this, "GetRobotControlState", std::string(""));
        // Events
        interfaceProvided->AddEventWrite(MessageEvents.Status, "Status", std::string(""));
        interfaceProvided->AddEventWrite(MessageEvents.Warning, "Warning", std::string(""));
        interfaceProvided->AddEventWrite(MessageEvents.Error, "Error", std::string(""));
        interfaceProvided->AddEventWrite(MessageEvents.RobotState, "RobotState", std::string(""));
        // Stats
        interfaceProvided->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                               "GetPeriodStatistics");
    }

    // allocate data for each SUJ and setup interfaces
    for (size_t i = 0; i < 4; ++i) {
        std::cerr << CMN_LOG_DETAILS << " -- name need to be found in configuration file" << std::endl;
        std::string name;
        if (i == 0) {
            name = "PSM2";
        } else if (i == 1) {
            name = "ECM";
        } else if (i == 2) {
            name = "PSM1";
        } else {
            name = "PSM3";
        }
        mtsInterfaceProvided * armInterface = this->AddInterfaceProvided(name);
        // Robot State
        armInterface->AddCommandWrite(&mtsIntuitiveResearchKitSUJ::SetRobotControlState,
                                      this, "SetRobotControlState", std::string(""));
        armInterface->AddCommandRead(&mtsIntuitiveResearchKitSUJ::GetRobotControlState,
                                     this, "GetRobotControlState", std::string(""));
        // Events
        armInterface->AddEventWrite(MessageEvents.Status, "Status", std::string(""));
        armInterface->AddEventWrite(MessageEvents.Warning, "Warning", std::string(""));
        armInterface->AddEventWrite(MessageEvents.Error, "Error", std::string(""));
        armInterface->AddEventWrite(MessageEvents.RobotState, "RobotState", std::string(""));
        Arms[i] = new mtsIntuitiveResearchKitSUJArmData(name, armInterface);
        Arms[i]->mPlugNumber = i + 1;
    }

    std::cerr << CMN_LOG_DETAILS << " -- arm settings need to be loaded from configuration file" << std::endl;

    // ARM ID: SE054270 (a.k.a. PSM 2 - RIGHT PSM SUJ)
    Arms[0]->mSerialNumber = "SE054270";
    Arms[0]->mVoltageToPositionOffsets[0].Assign( -0.043291, -2.3611, -3.0623, 3.069, -3.0623, -3.0564 );
    Arms[0]->mVoltageToPositionScales[0].Assign( 0.00027822, 0.0011525, 0.0014967, -0.0014975, 0.0014944, 0.0014902 );
    Arms[0]->mVoltageToPositionOffsets[1].Assign(  -0.043744, -2.3726, -3.0707, 3.072, -3.0734, -3.0697 );
    Arms[0]->mVoltageToPositionScales[1].Assign( 0.00027827, 0.0011578, 0.0015001, -0.0015, 0.0014992, 0.0014984 );

    // ARM ID: SE054269 (a.k.a. ECM - ECM SUJ)
    Arms[1]->mSerialNumber = "SE054269";
    Arms[1]->mVoltageToPositionOffsets[0].Assign(-0.098202, -2.3641, -3.0619, 6.2089, 0.0, 0.0 );
    Arms[1]->mVoltageToPositionScales[0].Assign( 0.00027813, 0.0011549, 0.0014946, -0.0014971, 0.0, 0.0 );
    Arms[1]->mVoltageToPositionOffsets[1].Assign( -0.097423, -2.3663, -3.0692, 6.2094, 0.0, 0.0 );
    Arms[1]->mVoltageToPositionScales[1].Assign(0.00027794, 0.0011545, 0.0014989, -0.0014968, 0.0, 0.0 );

    // ARM ID: SE054268 (a.k.a. PSM1 - LEFT PSM SUJ)
    Arms[2]->mSerialNumber = "SE054268";
    Arms[2]->mVoltageToPositionOffsets[0].Assign( -0.043675, -2.3642, -3.0699, 3.068, -3.0613, -3.0712 );
    Arms[2]->mVoltageToPositionScales[0].Assign( 0.00027819, 0.0011544, 0.001499, -0.0014988, 0.0014943, 0.0014989 );
    Arms[2]->mVoltageToPositionOffsets[1].Assign( -0.044521, -2.3689, -3.0687, 3.0611, -3.0717, -3.0692 );
    Arms[2]->mVoltageToPositionScales[1].Assign( 0.00027826, 0.0011544, 0.0014972, -0.0014935, 0.0014991, 0.001498 );

    // ARM ID: SE054271 (a.k.a. PSM 3 - AUX PSM SUJ)
    Arms[3]->mSerialNumber = "SE054271";
    Arms[3]->mVoltageToPositionOffsets[0].Assign( -0.035785, -2.3599, 3.0577, -3.0829, -3.0683, -3.0628 );
    Arms[3]->mVoltageToPositionScales[0].Assign( 0.00027957, 0.0011502, -0.0014949, 0.0015033, 0.0014982, 0.0014955 );
    Arms[3]->mVoltageToPositionOffsets[1].Assign(  -0.29097, -2.3539, 3.0768, -3.0571, -3.0675, -3.0686 );
    Arms[3]->mVoltageToPositionScales[1].Assign(  0.00027978, 0.0011505, -0.0014994, 0.0014929, 0.0014978, 0.0014979 );

    // mV vs V?
    for (size_t armIndex = 0; armIndex < 4; armIndex++) {
        for (size_t potIndex = 0; potIndex < 2; potIndex++) {
            Arms[armIndex]->mVoltageToPositionScales[potIndex].Multiply(1000.0);
        }
    }
}

void mtsIntuitiveResearchKitSUJ::Configure(const std::string & filename)
{
    std::cerr << CMN_LOG_DETAILS << " - need to load JSON config file with DH and ..." << std::endl;
}

void mtsIntuitiveResearchKitSUJ::Startup(void)
{
    this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
}

void mtsIntuitiveResearchKitSUJ::Run(void)
{
    mCounter++;

    ProcessQueuedEvents();

    GetRobotData();

    switch (mRobotState) {
    case mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED:
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_POWERING:
        RunHomingPower();
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_READY:
        break;
    case mtsIntuitiveResearchKitArmTypes::DVRK_MANUAL:
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
    // every 100 ms all we need is to make
    // 100 ms is to make sure A2D stabilizes
    const double muxCycle = 100.0 * cmn_ms;

    // we can start reporting some joint values after the robot is powered
    if (this->mRobotState > mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_POWERING) {
        const double currentTime = this->StateTable.GetTic();

        // time to toggle
        if (currentTime > mMuxTimer) {
            // pot values should be stable by now, get pots values
            GetAndConvertPotentiometerValues();
            // toggle mux
            mMuxTimer = currentTime + muxCycle;
            if (mMuxIndexExpected == MUX_MAX_INDEX) {
                MuxReset.DownUpDown();
                mMuxIndexExpected = 0;
            } else {
                MuxIncrement.DownUpDown();
                mMuxIndexExpected += 1;
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
    const size_t arrayIndex = mMuxIndex / MUX_ARRAY_SIZE; // 0 or 1: mux index 0 to 5 goes to first array of data, 6 to 11 goes to second array
    const size_t indexInArray = mMuxIndex % MUX_ARRAY_SIZE; // pot index in array, 0 to 5
    executionResult = RobotIO.GetAnalogInputVolts(mVoltages);
    mtsIntuitiveResearchKitSUJArmData * arm;
    // for each arm, i.e. SUJ1, SUJ2, SUJ3, ...
    for (size_t armIndex = 0; armIndex < 4; ++armIndex) {
        arm = Arms[armIndex];
        // start stable when reading 1st joint on all arms
        if (mMuxIndex == 0) {
            arm->mStateTable.Start();
        }
        // all 4 analog inputs are send to all 4 arm data structures
        arm->mVoltages[arrayIndex][indexInArray] = mVoltages[armIndex];
        // advance state table when all joints have been read
        if (mMuxIndex == MUX_MAX_INDEX) {
            arm->mPositions[0].Assign(arm->mVoltageToPositionOffsets[0]);
            arm->mPositions[0].AddElementwiseProductOf(arm->mVoltageToPositionScales[0], arm->mVoltages[0]);
            arm->mPositions[1].Assign(arm->mVoltageToPositionOffsets[1]);
            arm->mPositions[1].AddElementwiseProductOf(arm->mVoltageToPositionScales[1], arm->mVoltages[1]);
            // temporary hack to build a vector of positions from pots that seem to work
            arm->mPositionJoint[0] = arm->mPositions[1][0];
            arm->mPositionJoint[1] = arm->mPositions[0][1];
            arm->mPositionJoint[2] = arm->mPositions[0][2];
            arm->mPositionJoint[3] = arm->mPositions[1][3];
            arm->mPositionJoint[4] = arm->mPositions[0][4];
            arm->mPositionJoint[5] = arm->mPositions[0][5];
            // advance this arm state table
            arm->mStateTable.Advance();
        }
    }
    // debug code
    for (size_t armIndex = 0; armIndex < 4; ++armIndex) {
        //std::cerr << "Arm " << armIndex << std::endl;
        if (mMuxIndex == MUX_MAX_INDEX) {
            //size_t armIndex = 2;
            std::cerr << "Arm " << armIndex << std::endl;
            //std::cerr << "A " << Arms[armIndex]->mVoltages[0] << std::endl << "B " << Arms[armIndex]->mVoltages[1] << std::endl;
            std::cerr<< Arms[armIndex]->mPositionJoint * 180.0 / 3.14159 << std::endl;
        }
    }
    if (mMuxIndex == MUX_MAX_INDEX) {
        std::cerr<<std::endl;
    }
}

void mtsIntuitiveResearchKitSUJ::SetState(const mtsIntuitiveResearchKitArmTypes::RobotStateType & newState)
{
    CMN_LOG_CLASS_RUN_DEBUG << GetName() << ": SetState: new state " << newState << std::endl;

    switch (newState) {

    case mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED:
        mRobotState = newState;
        MessageEvents.Status(this->GetName() + " not initialized");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_POWERING:
        mHomingTimer = 0.0;
        mHomingPowerRequested = false;
        mRobotState = newState;
        MessageEvents.Status(this->GetName() + " powering");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_READY:
        // when returning from manual mode, need to re-enable PID
        mRobotState = newState;
        MessageEvents.Status(this->GetName() + " ready");
        break;

    case mtsIntuitiveResearchKitArmTypes::DVRK_MANUAL:
        if (this->mRobotState < mtsIntuitiveResearchKitArmTypes::DVRK_READY) {
            MessageEvents.Error(this->GetName() + " is not ready yet");
            return;
        }
        std::cerr << CMN_LOG_DETAILS << " should release breaks now" << std::endl;
        mRobotState = newState;
        MessageEvents.Status(this->GetName() + " in manual mode");
        break;
    default:
        break;
    }

    // Emit event with current state
    MessageEvents.RobotState(mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(this->mRobotState));
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
        MessageEvents.Status(this->GetName() + " power requested");
        return;
    }

    // second, check status
    if (mHomingPowerRequested
        && ((currentTime - mHomingTimer) > timeToPower)) {
        // check power status
        vctBoolVec actuatorAmplifiersStatus(NumberOfJoints);
        RobotIO.GetActuatorAmpStatus(actuatorAmplifiersStatus);
        if (actuatorAmplifiersStatus.All()) {
            MessageEvents.Status(this->GetName() + " power on");
            this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_READY);

            // reset mux
            MuxReset.DownUpDown();
            Sleep(10.0 * cmn_ms);
            mMuxIndexExpected = 0;
        } else {
            MessageEvents.Error(this->GetName() + " failed to enable power.");
            this->SetState(mtsIntuitiveResearchKitArmTypes::DVRK_UNINITIALIZED);
        }
    }
}

void mtsIntuitiveResearchKitSUJ::SetRobotControlState(const std::string & state)
{
    if (state == "Home") {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_HOMING_POWERING);
    } else if (state == "Manual") {
        SetState(mtsIntuitiveResearchKitArmTypes::DVRK_MANUAL);
    } else {
        mtsIntuitiveResearchKitArmTypes::RobotStateType stateEnum;
        try {
            stateEnum = mtsIntuitiveResearchKitArmTypes::RobotStateTypeFromString(state);
        } catch (std::exception e) {
            MessageEvents.Error(this->GetName() + ": SUJ unsupported state " + state + ": " + e.what());
            return;
        }
        SetState(stateEnum);
    }
}

void mtsIntuitiveResearchKitSUJ::GetRobotControlState(std::string & state) const
{
    state = mtsIntuitiveResearchKitArmTypes::RobotStateTypeToString(this->mRobotState);
}
