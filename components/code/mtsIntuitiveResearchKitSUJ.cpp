/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Youri Tan
  Created on: 2014-11-07

  (C) Copyright 2014-2017 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstRobot/robManipulator.h>

const size_t MUX_ARRAY_SIZE = 6;
const size_t MUX_MAX_INDEX = 15;

// empirical value, tradeoff between speed and stability of analog
// input
const size_t ANALOG_SAMPLE_NUMBER = 60;

// DO NOT set value below 2, this value should probably go
// down when the QLA/dSIB are properly grounded.
const size_t NUMBER_OF_MUX_CYCLE_BEFORE_STABLE = 4;

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitSUJ, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg)

class mtsIntuitiveResearchKitSUJArmData
{
public:

    typedef enum {SUJ_UNDEFINED, SUJ_PSM, SUJ_ECM, SUJ_MOTORIZED_PSM} SujType;

    inline mtsIntuitiveResearchKitSUJArmData(const std::string & name,
                                             const SujType type,
                                             const unsigned int plugNumber,
                                             mtsInterfaceProvided * interfaceProvided):
        mName(name),
        mType(type),
        mPlugNumber(plugNumber),
        mStateTable(500, name),
        mStateTableConfiguration(100, name + "Configuration"),
        mStateTableBrakeCurrent(100, name + "BrakeCurrent")
    {
        // brake info
        mClutched = 0;
        mBrakeDesiredCurrent = 0.0;

        // emit an event the first time we have a valid position
        mNeedToUpdatePositionCartesianDesired = 1;

        // joints
        mJointGet.SetSize(6);
        mJointGet.Zeros();

        // base frame
        mBaseFrameValid = true;

        // recalibration matrix
        mRecalibrationMatrix.SetSize(6,6);
        mRecalibrationMatrix.Zeros();
        mNewJointScales[0].SetSize(6);
        mNewJointScales[0].Zeros();
        mNewJointScales[1].SetSize(6);
        mNewJointScales[1].Zeros();

        mNewJointOffsets[0].SetSize(6);
        mNewJointOffsets[0].Zeros();
        mNewJointOffsets[1].SetSize(6);
        mNewJointOffsets[1].Zeros();

        // state table doesn't always advance, only when pots are stable
        mStateTable.SetAutomaticAdvance(false);
        mStateTableConfiguration.SetAutomaticAdvance(false);
        mStateTableBrakeCurrent.SetAutomaticAdvance(false);

        for (size_t potArray = 0; potArray < 2; ++potArray) {
            mVoltages[potArray].SetSize(MUX_ARRAY_SIZE);
            mPositions[potArray].SetSize(MUX_ARRAY_SIZE);
            mVoltageToPositionScales[potArray].SetSize(MUX_ARRAY_SIZE);
            mVoltageToPositionOffsets[potArray].SetSize(MUX_ARRAY_SIZE);
        }
        mPositionDifference.SetSize(MUX_ARRAY_SIZE);
        mPotsAgree = true;
        mVoltagesExtra.SetSize(MUX_MAX_INDEX - 2 * MUX_ARRAY_SIZE + 1);

        mStateJoint.Position().SetSize(MUX_ARRAY_SIZE);
        mStateJoint.Name().SetSize(MUX_ARRAY_SIZE);
        std::stringstream jointName;
        for (size_t index = 0; index < MUX_ARRAY_SIZE; ++index) {
            jointName.str("");
            jointName << "SUJ-" << name << "-j" << index;
            mStateJoint.Name().at(index) = jointName.str();
        }
        mStateTable.AddData(this->mVoltages[0], "Voltages[0]");
        mStateTable.AddData(this->mVoltages[1], "Voltages[1]");
        mStateTable.AddData(this->mVoltagesExtra, "VoltagesExtra");
        mStateTable.AddData(this->mStateJoint, "PositionJoint");
        mStateTable.AddData(this->mPositionCartesianParam,             "PositionCartesian");
        mStateTable.AddData(this->mPositionCartesianLocalParam,        "PositionCartesianLocal");
        mStateTable.AddData(this->mPositionCartesianDesiredParam,      "PositionCartesianDesired");
        mStateTable.AddData(this->mPositionCartesianLocalDesiredParam, "PositionCartesianLocalDesired");
        mStateTable.AddData(this->mBaseFrame, "BaseFrame");
        mStateTableConfiguration.AddData(this->mName, "Name");
        mStateTableConfiguration.AddData(this->mSerialNumber, "SerialNumber");
        mStateTableConfiguration.AddData(this->mPlugNumber, "PlugNumber");
        mStateTableConfiguration.AddData(this->mVoltageToPositionOffsets[0], "PrimaryJointOffset");
        mStateTableConfiguration.AddData(this->mVoltageToPositionOffsets[1], "SecondaryJointOffset");
        mStateTableBrakeCurrent.AddData(this->mBrakeDesiredCurrent, "BrakeCurrent");

        CMN_ASSERT(interfaceProvided);
        mInterface = interfaceProvided;
        // read commands
        mInterface->AddCommandReadState(mStateTable, mStateJoint, "GetStateJoint");
        mInterface->AddCommandReadState(mStateTableConfiguration, mVoltageToPositionOffsets[0],
                                        "GetPrimaryJointOffset");
        mInterface->AddCommandReadState(mStateTableConfiguration, mVoltageToPositionOffsets[1],
                                        "GetSecondaryJointOffset");
        mInterface->AddCommandReadState(mStateTable, mPositionCartesianParam,
                                        "GetPositionCartesian");
        mInterface->AddCommandReadState(mStateTable, mPositionCartesianLocalParam,
                                        "GetPositionCartesianLocal");
        mInterface->AddCommandReadState(mStateTable, mPositionCartesianDesiredParam,
                                        "GetPositionCartesianDesired");
        mInterface->AddCommandReadState(mStateTable, mPositionCartesianLocalDesiredParam,
                                        "GetPositionCartesianLocalDesired");
        mInterface->AddCommandReadState(mStateTable, mBaseFrame, "GetBaseFrame");
        mInterface->AddCommandReadState(mStateTable, mVoltages[0], "GetVoltagesPrimary");
        mInterface->AddCommandReadState(mStateTable, mVoltages[1], "GetVoltagesSecondary");
        mInterface->AddCommandReadState(mStateTable, mVoltagesExtra, "GetVoltagesExtra");
        mInterface->AddCommandReadState(mStateTableConfiguration, mName, "GetName");
        mInterface->AddCommandReadState(mStateTableConfiguration, mSerialNumber, "GetSerialNumber");
        mInterface->AddCommandReadState(mStateTableConfiguration, mPlugNumber, "GetPlugNumber");
        mInterface->AddCommandReadState(mStateTableBrakeCurrent, mBrakeDesiredCurrent, "GetBrakeCurrent");

        // write commands
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitSUJArmData::ClutchCommand, this,
                                    "Clutch", false);
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitSUJArmData::CalibratePotentiometers, this,
                                    "SetRecalibrationMatrix", mRecalibrationMatrix);

        // cartesian position events
        // BaseFrame is send everytime the mux has found all joint values
        mInterface->AddEventWrite(EventPositionCartesian, "PositionCartesian", prmPositionCartesianGet());
        mInterface->AddEventWrite(EventPositionCartesianLocal, "PositionCartesianLocal", prmPositionCartesianGet());
        // BaseFrameDesired is sent only when the brakes are released, constant value afterwards
        mInterface->AddEventWrite(EventPositionCartesianDesired, "PositionCartesianDesired", prmPositionCartesianGet());
        mInterface->AddEventWrite(EventPositionCartesianLocalDesired, "PositionCartesianLocalDesired", prmPositionCartesianGet());

        // Events
        mInterface->AddEventWrite(MessageEvents.CurrentState, "CurrentState", std::string(""));
        mInterface->AddEventWrite(MessageEvents.DesiredState, "DesiredState", std::string(""));
        mInterface->AddMessageEvents();
        // Stats
        mInterface->AddCommandReadState(mStateTable, mStateTable.PeriodStats,
                                        "GetPeriodStatistics");
    }

    inline void ClutchCallback(const prmEventButton & button) {
        if (button.Type() == prmEventButton::PRESSED) {
            mClutched += 1;
            if (mClutched == 1) {
                // clutch is pressed, arm is moving around and we know the pots are slow, we mark position as invalid
                mInterface->SendStatus(mName.Data + ": SUJ clutched");
                mPositionCartesianDesiredParam.SetTimestamp(mStateJoint.Timestamp());
                mPositionCartesianDesiredParam.SetValid(false);
                EventPositionCartesianDesired(mPositionCartesianDesiredParam);
                mPositionCartesianLocalDesiredParam.SetTimestamp(mStateJoint.Timestamp());
                mPositionCartesianLocalDesiredParam.SetValid(false);
                EventPositionCartesianLocalDesired(mPositionCartesianLocalDesiredParam);
            }
        } else {
            mClutched -= 1;
            if (mClutched == 0) {
                mInterface->SendStatus(mName.Data + ": SUJ not clutched");
            }
        }
    }

    inline void ClutchCommand(const bool & clutch) {
        prmEventButton button;
        if (clutch) {
            button.SetType(prmEventButton::PRESSED);
        } else {
            button.SetType(prmEventButton::RELEASED);
        }
        ClutchCallback(button);
    }

    inline void CalibratePotentiometers(const vctMat & mat) {
        for (size_t col = 0; col < 6; col++) {
            // IF:                                      Pi = Offset + Vi * Scale
            // Given P1 / V1 & P2 / V2, THEN:           Scale = (P1 - P2) / (V1 - V2)

            // Delta_P = P1 - P2
            const double deltaJointPosition = mat.Element(0, col) - mat.Element(3, col);

            // Delta_V = V1 - V2 (primary)
            const double deltaPrimaryVoltage = mat.Element(1, col) - mat.Element(4, col);

            // V1 - V2 (secondary)
            const double deltaSecondaryVoltage = mat.Element(2, col) - mat.Element(5, col);

            // Scale = Delta_P / Delta_V
            mNewJointScales[0][col] = deltaJointPosition / deltaPrimaryVoltage;
            mNewJointScales[1][col] = deltaJointPosition / deltaSecondaryVoltage;

            mNewJointOffsets[0][col] = mat.Element(0, col) - mat.Element(1, col) * mNewJointScales[0][col];
            mNewJointOffsets[1][col] = mat.Element(0, col) - mat.Element(2, col) * mNewJointScales[1][col];
            mNewJointScales[0][col] /= 1000.0;
            mNewJointScales[1][col] /= 1000.0;
        }


        std::cerr << "SUJ scales and offsets for arm: " << mName << std::endl
                  << "Please update your suj.json file using these values" << std::endl
                  << "\"primary-offsets\": [ "
                  << mNewJointOffsets[0][0] << ", "
                  << mNewJointOffsets[0][1] << ", "
                  << mNewJointOffsets[0][2] << ", "
                  << mNewJointOffsets[0][3] << ", "
                  << mNewJointOffsets[0][4] << ", "
                  << mNewJointOffsets[0][5] << "],"  << std::endl
                  << "\"primary-scales\": [ "
                  << mNewJointScales[0][0] << ", "
                  << mNewJointScales[0][1] << ", "
                  << mNewJointScales[0][2] << ", "
                  << mNewJointScales[0][3] << ", "
                  << mNewJointScales[0][4] << ", "
                  << mNewJointScales[0][5] << "],"  << std::endl
                  << "\"secondary-offsets\": [ "
                  << mNewJointOffsets[1][0] << ", "
                  << mNewJointOffsets[1][1] << ", "
                  << mNewJointOffsets[1][2] << ", "
                  << mNewJointOffsets[1][3] << ", "
                  << mNewJointOffsets[1][4] << ", "
                  << mNewJointOffsets[1][5] << "]," << std::endl
                  << "\"secondary-scales\": [ "
                  << mNewJointScales[1][0] << ", "
                  << mNewJointScales[1][1] << ", "
                  << mNewJointScales[1][2] << ", "
                  << mNewJointScales[1][3] << ", "
                  << mNewJointScales[1][4] << ", "
                  << mNewJointScales[1][5] << "],"  << std::endl;
    }

    // name of this SUJ arm (ECM, PSM1, ...)
    mtsStdString mName;
    // suj type
    SujType mType;
    // serial number
    mtsStdString mSerialNumber;
    // plug on back of controller, 1 to 4
    unsigned int mPlugNumber;

    // interface provided
    mtsInterfaceProvided * mInterface;

    // state of this SUJ arm
    mtsStateTable mStateTable; // for positions, fairly slow, i.e 12 * delay for a2d
    mtsStateTable mStateTableConfiguration; // changes only at config and if recalibrate
    mtsStateTable mStateTableBrakeCurrent; // changes when requested current changes

    // 2 arrays, one for each set of potentiometers
    vctDoubleVec mVoltages[2];
    vctDoubleVec mPositions[2];
    vctDoubleVec mPositionDifference;
    bool mPotsAgree;

    vctDoubleVec mVoltageToPositionScales[2];
    vctDoubleVec mVoltageToPositionOffsets[2];
    prmStateJoint mStateJoint;
    prmPositionCartesianGet mPositionCartesianParam;
    prmPositionCartesianGet mPositionCartesianLocalParam;
    unsigned int mNeedToUpdatePositionCartesianDesired; // 0 is no, 1 tells we need to send, 2 is for first full mux cycle has started
    vctFrm4x4 mPositionCartesianLocalDesired;
    prmPositionCartesianGet mPositionCartesianDesiredParam; // set when user releases brakes
    prmPositionCartesianGet mPositionCartesianLocalDesiredParam;

    // Exta analog feedback
    // plugs 1-3:  spare1, spare2, brake-voltage, gnd
    // plug 4: I_MOT+, I_MOT-, VA_BIAS, brake-voltage
    vctDoubleVec mVoltagesExtra;

    // Kinematics
    robManipulator mManipulator;
    vctDoubleVec mJointGet;
    vctMat mRecalibrationMatrix;
    vctDoubleVec mNewJointScales[2];
    vctDoubleVec mNewJointOffsets[2];

    // Setup transformations from json file
    vctFrame4x4<double> mWorldToSUJ;
    vctFrame4x4<double> mSUJToArmBase;

    // Base frame
    vctFrame4x4<double> mBaseFrame;
    bool mBaseFrameValid;

    // clutch data
    unsigned int mClutched;
    double mBrakeDesiredCurrent;
    double mBrakeReleaseCurrent;
    double mBrakeEngagedCurrent;
    double mBrakeDirectionCurrent;

    // Functions for events
    mtsFunctionWrite EventPositionCartesian;
    mtsFunctionWrite EventPositionCartesianLocal;
    mtsFunctionWrite EventPositionCartesianDesired;
    mtsFunctionWrite EventPositionCartesianLocalDesired;

    struct {
        mtsFunctionWrite CurrentState;
        mtsFunctionWrite DesiredState;
    } MessageEvents;
};

mtsIntuitiveResearchKitSUJ::mtsIntuitiveResearchKitSUJ(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    mArmState(componentName, "UNINITIALIZED"),
    mStateTableState(100, "State"),
    mVoltageSamplesNumber(ANALOG_SAMPLE_NUMBER)
{
    Init();
}

mtsIntuitiveResearchKitSUJ::mtsIntuitiveResearchKitSUJ(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    mArmState(arg.Name, "UNINITIALIZED"),
    mStateTableState(100, "State"),
    mVoltageSamplesNumber(ANALOG_SAMPLE_NUMBER)
{
    Init();
}

void mtsIntuitiveResearchKitSUJ::Init(void)
{
    // configure state machine common to all arms (ECM/MTM/PSM)
    // possible states
    mArmState.AddState("POWERING");
    mArmState.AddState("POWERED");
    mArmState.AddState("READY");

    // possible desired states
    mArmState.AddAllowedDesiredState("UNINITIALIZED");
    mArmState.AddAllowedDesiredState("POWERED");
    mArmState.AddAllowedDesiredState("READY");

    mFallbackState = "UNINITIALIZED";

    // state change, to convert to string events for users (Qt, ROS)
    mArmState.SetStateChangedCallback(&mtsIntuitiveResearchKitSUJ::StateChanged,
                                      this);

    // run for all states
    mArmState.SetRunCallback(&mtsIntuitiveResearchKitSUJ::RunAllStates,
                             this);

    // unitialized
    mArmState.SetEnterCallback("UNINITIALIZED",
                               &mtsIntuitiveResearchKitSUJ::EnterUninitialized,
                               this);

    mArmState.SetTransitionCallback("UNINITIALIZED",
                                    &mtsIntuitiveResearchKitSUJ::TransitionUninitialized,
                                    this);

    // power
    mArmState.SetEnterCallback("POWERING",
                               &mtsIntuitiveResearchKitSUJ::EnterPowering,
                               this);

    mArmState.SetTransitionCallback("POWERING",
                                    &mtsIntuitiveResearchKitSUJ::TransitionPowering,
                                    this);

    // powered
    mArmState.SetTransitionCallback("POWERED",
                                    &mtsIntuitiveResearchKitSUJ::TransitionPowered,
                                    this);

    // state between ARM_HOMED and READY depends on the arm type, see
    // derived classes
    mArmState.SetEnterCallback("READY",
                               &mtsIntuitiveResearchKitSUJ::EnterReady,
                               this);

    mArmState.SetRunCallback("READY",
                             &mtsIntuitiveResearchKitSUJ::RunReady,
                             this);

    // state table to maintain state :-)
    mStateTableState.AddData(mStateTableStateCurrent, "Current");
    mStateTableState.AddData(mStateTableStateDesired, "Desired");
    AddStateTable(&mStateTableState);
    mStateTableState.SetAutomaticAdvance(false);

    // default values
    mIsSimulated = false;
    mMuxTimer = 0.0;
    mMuxState.SetSize(4);
    mVoltages.SetSize(4);
    mBrakeCurrents.SetSize(4);
    mVoltageSamples.SetSize(mVoltageSamplesNumber);
    mVoltageSamplesCounter = 0;

    // Robot IO
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("RobotIO");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("EnablePower", RobotIO.EnablePower);
        interfaceRequired->AddFunction("DisablePower", RobotIO.DisablePower);
        interfaceRequired->AddFunction("GetEncoderChannelA", RobotIO.GetEncoderChannelA);
        interfaceRequired->AddFunction("GetActuatorAmpStatus", RobotIO.GetActuatorAmpStatus);
        interfaceRequired->AddFunction("SetActuatorCurrent", RobotIO.SetActuatorCurrent);
        interfaceRequired->AddFunction("GetAnalogInputVolts", RobotIO.GetAnalogInputVolts);
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitSUJ::ErrorEventHandler, this, "Error");
    }
    interfaceRequired = AddInterfaceRequired("NoMuxReset");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetValue", NoMuxReset.GetValue);
        interfaceRequired->AddFunction("SetValue", NoMuxReset.SetValue);
    }
    interfaceRequired = AddInterfaceRequired("MuxIncrement");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetValue", MuxIncrement.GetValue);
        interfaceRequired->AddFunction("SetValue", MuxIncrement.SetValue);
    }
    interfaceRequired = AddInterfaceRequired("ControlPWM");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("SetPWMDutyCycle", PWM.SetPWMDutyCycle);
    }
    interfaceRequired = AddInterfaceRequired("DisablePWM");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("SetValue", PWM.DisablePWM);
    }
    interfaceRequired = AddInterfaceRequired("MotorUp");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitSUJ::MotorUpEventHandler, this, "Button");
    }
    interfaceRequired = AddInterfaceRequired("MotorDown");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitSUJ::MotorDownEventHandler, this, "Button");
    }
    interfaceRequired = AddInterfaceRequired("BaseFrame", MTS_OPTIONAL);
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitSUJ::SetBaseFrame, this, "PositionCartesianDesired");
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitSUJ::ErrorEventHandler, this, "Error");
    }

    mInterface = AddInterfaceProvided("Robot");
    if (mInterface) {
        // Robot State
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitSUJ::SetDesiredState,
                                    this, "SetDesiredState", std::string(""));
        mInterface->AddCommandReadState(this->mStateTableState,
                                        mStateTableStateCurrent, "GetCurrentState");
        mInterface->AddCommandReadState(this->mStateTableState,
                                        mStateTableStateDesired, "GetDesiredState");
        // Events
        mInterface->AddMessageEvents();
        mInterface->AddEventWrite(MessageEvents.DesiredState, "DesiredState", std::string(""));
        mInterface->AddEventWrite(MessageEvents.CurrentState, "CurrentState", std::string(""));
        // Stats
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                        "GetPeriodStatistics");
    }
}


void mtsIntuitiveResearchKitSUJ::Configure(const std::string & filename)
{
    std::ifstream jsonStream;
    jsonStream.open(filename.c_str());

    Json::Value jsonConfig;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration\n"
                                 << jsonReader.getFormattedErrorMessages();
        return;
    }

    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << this->GetName()
                               << " using file \"" << filename << "\"" << std::endl
                               << "----> content of configuration file: " << std::endl
                               << jsonConfig << std::endl
                               << "<----" << std::endl;

    // find all arms, there should be 4 of them
    const Json::Value jsonArms = jsonConfig["arms"];
    if (jsonArms.size() != 4) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to find 4 SUJ arms" << std::endl;
        return;
    }

    mtsIntuitiveResearchKitSUJArmData * arm;
    for (unsigned int index = 0; index < jsonArms.size(); ++index) {
        // name
        Json::Value jsonArm = jsonArms[index];
        std::string name = jsonArm["name"].asString();
        if (!((name == "ECM") || (name == "PSM1") || (name == "PSM2") || (name == "PSM3"))) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: incorrect arm name for SUJ \""
                                     << name << "\", must be one of \"PSM1\", \"PSM2\", \"PSM3\" or \"ECM\""
                                     << std::endl;
            return;
        }
        std::string typeString = jsonArm["type"].asString();
        mtsIntuitiveResearchKitSUJArmData::SujType type;
        if (typeString == "ECM") {
            type = mtsIntuitiveResearchKitSUJArmData::SUJ_ECM;
        }
        else if (typeString == "PSM") {
            type = mtsIntuitiveResearchKitSUJArmData::SUJ_PSM;
        }
        else if (typeString == "Motorized PSM") {
            type = mtsIntuitiveResearchKitSUJArmData::SUJ_MOTORIZED_PSM;
        }
        else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: incorrect arm type for SUJ \""
                                     << name << "\", must be one of \"PSM\", \"ECM\" or \"Motorized PSM\""
                                     << std::endl;
            return;
        }

        if (!jsonArm["plug-number"]) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: plug number is missing for SUJ \""
                                     << name << "\", must be an integer between 1 and 4"
                                     << std::endl;
            return;
        }
        unsigned int plugNumber = jsonArm["plug-number"].asInt();
        unsigned int armIndex = plugNumber - 1;

        mtsInterfaceProvided * armInterface = this->AddInterfaceProvided(name);
        arm = new mtsIntuitiveResearchKitSUJArmData(name, type, plugNumber, armInterface);
        Arms[armIndex] = arm;

        // Robot State so GUI widget for each arm can set/get state
        armInterface->AddCommandWrite(&mtsIntuitiveResearchKitSUJ::SetDesiredState,
                                      this, "SetDesiredState", std::string(""));

        // Add motor up/down for the motorized arm
        if (type == mtsIntuitiveResearchKitSUJArmData::SUJ_MOTORIZED_PSM) {
            armInterface->AddCommandWrite(&mtsIntuitiveResearchKitSUJ::SetLiftVelocity, this,
                                          "SetLiftVelocity", 0.0);
        }

        // create a required interface for each arm to handle clutch button
        if (!mIsSimulated) {
            std::stringstream interfaceName;
            interfaceName << "SUJ-Clutch-" << plugNumber;
            mtsInterfaceRequired * requiredInterface = this->AddInterfaceRequired(interfaceName.str());
            if (requiredInterface) {
                requiredInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitSUJArmData::ClutchCallback, arm,
                                                        "Button");
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: can't add required interface for SUJ \""
                                         << name << "\" clutch button because this interface already exists: \""
                                         << interfaceName.str() << "\".  Make sure all arms have a different plug number."
                                         << std::endl;
                return;
            }
        }

        // find serial number
        arm->mSerialNumber = jsonArm["serial-number"].asString();

        // read brake current configuration
        // all math for ramping up/down current is done on positive values
        // negate only when applying
        double brakeCurrent = jsonArm["brake-release-current"].asFloat();
        if (brakeCurrent > 0.0) {
            arm->mBrakeReleaseCurrent = brakeCurrent;
            arm->mBrakeDirectionCurrent = 1.0;
        } else {
            arm->mBrakeReleaseCurrent = -brakeCurrent;
            arm->mBrakeDirectionCurrent = -1.0;
        }

        brakeCurrent = 0.0;
        if (!jsonArm["brake-engaged-current"].isNull()) {
            brakeCurrent = jsonArm["brake-engaged-current"].asFloat();
        }
        arm->mBrakeEngagedCurrent = brakeCurrent;

        // read pot settings
        cmnDataJSON<vctDoubleVec>::DeSerializeText(arm->mVoltageToPositionOffsets[0], jsonArm["primary-offsets"]);
        cmnDataJSON<vctDoubleVec>::DeSerializeText(arm->mVoltageToPositionOffsets[1], jsonArm["secondary-offsets"]);
        cmnDataJSON<vctDoubleVec>::DeSerializeText(arm->mVoltageToPositionScales[0], jsonArm["primary-scales"]);
        cmnDataJSON<vctDoubleVec>::DeSerializeText(arm->mVoltageToPositionScales[1], jsonArm["secondary-scales"]);

        // look for DH
        arm->mManipulator.LoadRobot(jsonArm["DH"]);

        // mV vs V?
        arm->mStateTableConfiguration.Start();
        for (size_t potIndex = 0; potIndex < 2; potIndex++) {
            arm->mVoltageToPositionScales[potIndex].Multiply(1000.0);
        }
        arm->mStateTableConfiguration.Advance();

        // Read setup transforms
        vctFrm3 transform;
        cmnDataJSON<vctFrm3>::DeSerializeText(transform, jsonArm["world-origin-to-suj"]);
        arm->mWorldToSUJ.From(transform);
        cmnDataJSON<vctFrm3>::DeSerializeText(transform, jsonArm["suj-tip-to-tool-origin"]);
        arm->mSUJToArmBase.From(transform);
    }
}

void mtsIntuitiveResearchKitSUJ::StateChanged(void)
{
    const std::string newState = mArmState.CurrentState();
    // update state table
    mStateTableState.Start();
    mStateTableStateCurrent = newState;
    mStateTableState.Advance();
    // event
    DispatchStatus(this->GetName() + ": current state " + newState);
    DispatchCurrentState(newState);
}

void mtsIntuitiveResearchKitSUJ::RunAllStates(void)
{
    // get robot data, i.e. process mux/pots
    GetRobotData();

    // always allow to go to unitialized
    if (mArmState.DesiredStateIsNotCurrent()) {
        if (mArmState.DesiredState() == "UNINITIALIZED") {
            mArmState.SetCurrentState("UNINITIALIZED");
        } else {
            // error handling will require to swith to fallback state
            if (mArmState.DesiredState() == mFallbackState) {
                mArmState.SetCurrentState(mFallbackState);
            }
        }
    }
}

void mtsIntuitiveResearchKitSUJ::ResetMux(void)
{
    mMuxTimer = this->StateTable.GetTic();
    MuxIncrement.SetValue(false);
    NoMuxReset.SetValue(false);
    Sleep(30.0 * cmn_ms);
    mMuxIndexExpected = 0;
}

void mtsIntuitiveResearchKitSUJ::EnterUninitialized(void)
{
    // power off brakes
    RobotIO.SetActuatorCurrent(vctDoubleVec(4, 0.0));
    RobotIO.DisablePower();

    // disable power on PWM
    PWM.DisablePWM(true);
    // set lift velocity
    SetLiftVelocity(0.0);

    // reset mux
    ResetMux();

    mFallbackState = "UNINITIALIZED";
}

void mtsIntuitiveResearchKitSUJ::TransitionUninitialized(void)
{
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState("POWERING");
    }
}

void mtsIntuitiveResearchKitSUJ::EnterPowering(void)
{
    if (mIsSimulated) {
        return;
    }
    const double currentTime = this->StateTable.GetTic();
    mHomingTimer = currentTime;
    // pre-load the boards with zero current
    RobotIO.SetActuatorCurrent(vctDoubleVec(4, 0.0));
    // enable power and set a flags to move to next step
    RobotIO.EnablePower();

    DispatchStatus(this->GetName() + ": power requested");
}

void mtsIntuitiveResearchKitSUJ::TransitionPowering(void)
{
    if (mIsSimulated) {
        mArmState.SetCurrentState("POWERED");
        return;
    }

    const double currentTime = this->StateTable.GetTic();

    // check status
    if ((currentTime - mHomingTimer) > mtsIntuitiveResearchKit::TimeToPower) {
        // check power status
        vctBoolVec actuatorAmplifiersStatus(4);
        RobotIO.GetActuatorAmpStatus(actuatorAmplifiersStatus);
        if (actuatorAmplifiersStatus.All()) {
            DispatchStatus(this->GetName() + ": power on");
            mArmState.SetCurrentState("POWERED");
        } else {
            DispatchError(this->GetName() + ": failed to enable power");
            this->SetDesiredState(mFallbackState);
        }
    }
}

void mtsIntuitiveResearchKitSUJ::TransitionPowered(void)
{
    // move to next stage if desired state is different
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState(mArmState.DesiredState());
    }
}

void mtsIntuitiveResearchKitSUJ::EnterReady(void)
{
    if (mIsSimulated) {
        return;
    }

    // enable power on PWM
    PWM.DisablePWM(false);

    // make sure motor current is zero (brakes)
    RobotIO.SetActuatorCurrent(vctDoubleVec(4, 0.0));

    // when returning from manual mode, make sure brakes are not released
    mtsIntuitiveResearchKitSUJArmData * arm;
    for (size_t armIndex = 0; armIndex < 4; ++armIndex) {
        arm = Arms[armIndex];
        arm->mClutched = 0;
        arm->mBrakeDesiredCurrent = 0.0;
        mPreviousTic = 0.0;
    }
}

void mtsIntuitiveResearchKitSUJ::Startup(void)
{
    this->SetDesiredState("UNINITIALIZED");
}

void mtsIntuitiveResearchKitSUJ::Run(void)
{
    // collect data from required interfaces
    ProcessQueuedEvents();
    try {
        mArmState.Run();
    } catch (std::exception & e) {
        DispatchError(this->GetName() + ": in state " + mArmState.CurrentState()
                      + ", caught exception \"" + e.what() + "\"");
        this->SetDesiredState("UNINITIALIZED");
    }
    // trigger ExecOut event
    RunEvent();
    ProcessQueuedCommands();
}

void mtsIntuitiveResearchKitSUJ::Cleanup(void)
{
    // Disable PWM
    SetLiftVelocity(0.0);
    PWM.DisablePWM(true);
    // make sure requested current is back to 0
    RobotIO.SetActuatorCurrent(vctDoubleVec(4, 0.0));
    // turn off amplifiers
    RobotIO.DisablePower();
}

void mtsIntuitiveResearchKitSUJ::SetSimulated(void)
{
    mIsSimulated = true;
    // in simulation mode, we don't need IOs
    RemoveInterfaceRequired("RobotIO");
    RemoveInterfaceRequired("NoMuxReset");
    RemoveInterfaceRequired("MuxIncrement");
    RemoveInterfaceRequired("ControlPWM");
    RemoveInterfaceRequired("DisablePWM");
    RemoveInterfaceRequired("MotorUp");
    RemoveInterfaceRequired("MotorDown");
}

void mtsIntuitiveResearchKitSUJ::GetRobotData(void)
{
    if (mIsSimulated) {
        return;
    }

    // 30 ms is to make sure A2D stabilizes
    const double muxCycle = 30.0 * cmn_ms;

    // we can start reporting some joint values after the robot is powered
    const double currentTime = this->StateTable.GetTic();

    // we assume the analog in is now stable
    if (currentTime > mMuxTimer) {
        // pot values should be stable by now, get pots values
        GetAndConvertPotentiometerValues();

        // time to toggle
        if (mVoltageSamplesCounter == mVoltageSamplesNumber) {
            // toggle mux
            mMuxTimer = currentTime + muxCycle;
            if (mMuxIndexExpected == MUX_MAX_INDEX) {
                NoMuxReset.SetValue(false);
                mMuxIndexExpected = 0;
            } else {
                MuxIncrement.SetValue(true);
                mMuxIndexExpected += 1;
            }
            // reset sample counter
            mVoltageSamplesCounter = 0;
        }
    }
}

void mtsIntuitiveResearchKitSUJ::GetAndConvertPotentiometerValues(void)
{
    mtsIntuitiveResearchKitSUJArmData * arm;

    // read encoder channel A to get the mux state
    mtsExecutionResult executionResult = RobotIO.GetEncoderChannelA(mMuxState);
    // compute pot index
    mMuxIndex = (mMuxState[0]?1:0) + (mMuxState[1]?2:0) + (mMuxState[2]?4:0) + (mMuxState[3]?8:0);
    if (mMuxIndex != mMuxIndexExpected) {
        DispatchWarning(this->GetName() + ": unexpected multiplexer value.");
        CMN_LOG_CLASS_RUN_ERROR << "GetAndConvertPotentiometerValues: mux from IO board: " << mMuxIndex << " expected: " << mMuxIndexExpected << std::endl;
        ResetMux();
        return;
    }
    // array index, 0 or 1, primary or secondary pots
    const size_t arrayIndex = mMuxIndex / MUX_ARRAY_SIZE; // 0 or 1: mux index 0 to 5 goes to first array of data, 6 to 11 goes to second array
                                                          // 12 to 15 goes to third array (misc. voltages)
    const size_t indexInArray = mMuxIndex % MUX_ARRAY_SIZE; // pot index in array, 0 to 5 (0 to 3 for third array)

    executionResult = RobotIO.GetAnalogInputVolts(mVoltages);
    mVoltageSamples[mVoltageSamplesCounter].ForceAssign(mVoltages);
    mVoltageSamplesCounter++;

    // if we have enough samples
    if (mVoltageSamplesCounter == mVoltageSamplesNumber) {
        // use mVoltages to store average
        mVoltages.Zeros();
        for (size_t index = 0;
             index < mVoltageSamplesNumber;
             ++index) {
            mVoltages.Add(mVoltageSamples[index]);
        }
        mVoltages.Divide(mVoltageSamplesNumber);
        // for each arm, i.e. SUJ1, SUJ2, SUJ3, ...
        for (size_t armIndex = 0; armIndex < 4; ++armIndex) {
            arm = Arms[armIndex];
            // start stable when reading 1st joint on all arms
            if (mMuxIndex == 0) {
                arm->mStateTable.Start();
            }
            // all 4 analog inputs are sent to all 4 arm data structures
            if (arrayIndex < 2) {
                arm->mVoltages[arrayIndex][indexInArray] = mVoltages[armIndex];
            } else {
                if (indexInArray == 2) {
                    if ((armIndex == 0) || (armIndex == 1) || (armIndex == 2)) {
                        Arms[3 - armIndex]->mVoltagesExtra[indexInArray] = mVoltages[armIndex];
                    } else if (armIndex == 3) {

                    }
                } else if ((indexInArray == 3) && (armIndex == 3)) {
                    Arms[0 /* 3 - armIndex */]->mVoltagesExtra[2] = mVoltages[armIndex];
                } else {
                    // normal case
                    arm->mVoltagesExtra[indexInArray] = mVoltages[armIndex];
                }
            }
            // advance state table when all joints have been read
            if (mMuxIndex == MUX_MAX_INDEX) {
                arm->mPositions[0].Assign(arm->mVoltageToPositionOffsets[0]);
                arm->mPositions[0].AddElementwiseProductOf(arm->mVoltageToPositionScales[0], arm->mVoltages[0]);
                arm->mPositions[1].Assign(arm->mVoltageToPositionOffsets[1]);
                arm->mPositions[1].AddElementwiseProductOf(arm->mVoltageToPositionScales[1], arm->mVoltages[1]);

                // ignore values on ECM arm
                if (arm->mType == mtsIntuitiveResearchKitSUJArmData::SUJ_ECM) {
                    // ECM has only 4 joints
                    arm->mPositions[0][4] = 0.0;
                    arm->mPositions[0][5] = 0.0;
                    arm->mPositions[1][4] = 0.0;
                    arm->mPositions[1][5] = 0.0;
                }

                // use average of positions reported by potentiometers
                arm->mStateJoint.Position().SumOf(arm->mPositions[0],
                                                  arm->mPositions[1]);
                arm->mStateJoint.Position().Divide(2.0);
                arm->mStateJoint.SetValid(true);

                // Joint forward kinematics
                arm->mJointGet.Assign(arm->mStateJoint.Position(),arm->mManipulator.links.size());
                // forward kinematic
                vctFrame4x4<double> suj = arm->mManipulator.ForwardKinematics(arm->mJointGet, 6);
                // pre and post transformations loaded from JSON file, base frame updated using events
                vctFrm4x4 armLocal = arm->mWorldToSUJ * suj * arm->mSUJToArmBase;
                // apply base frame
                vctFrm4x4 armBase = arm->mBaseFrame * armLocal;
                // emit events for continuous positions
                // - with base
                arm->mPositionCartesianParam.Position().From(armBase);
                arm->mPositionCartesianParam.SetTimestamp(arm->mStateJoint.Timestamp());
                arm->mPositionCartesianParam.SetValid(arm->mBaseFrameValid);
                arm->EventPositionCartesian(arm->mPositionCartesianParam);
                // - local
                arm->mPositionCartesianLocalParam.Position().From(armLocal);
                arm->mPositionCartesianLocalParam.SetTimestamp(arm->mStateJoint.Timestamp());
                arm->mPositionCartesianLocalParam.SetValid(arm->mBaseFrameValid);
                arm->EventPositionCartesianLocal(arm->mPositionCartesianLocalParam);
                // this mux cycle might have started before brakes where engaged
                if (arm->mNeedToUpdatePositionCartesianDesired > 0) {
                    if (arm->mNeedToUpdatePositionCartesianDesired < NUMBER_OF_MUX_CYCLE_BEFORE_STABLE) {
                        arm->mNeedToUpdatePositionCartesianDesired++;
                    } else {
                        // at that point we know there has been a full mux cycle with brakes engaged
                        // so we treat this as a fixed transformation until the SUJ move again (user clutch)
                        arm->mPositionCartesianLocalDesired = armLocal;
                        arm->mPositionCartesianLocalDesiredParam.SetValid(true);
                        arm->mPositionCartesianLocalDesiredParam.SetTimestamp(arm->mStateJoint.Timestamp());
                        arm->mPositionCartesianLocalDesiredParam.Position().From(arm->mPositionCartesianLocalDesired);
                        arm->mNeedToUpdatePositionCartesianDesired = 0;
                    }
                }

                // check pots when the SUJ is not clutch and if the
                // counter for update cartesian desired position is
                // back to zero (pot values should now be stable).
                if ((arm->mClutched == 0) && (arm->mNeedToUpdatePositionCartesianDesired == 0)) {
                    // compare primary and secondary pots when arm is not clutched
                    const double angleTolerance = 1.0 * cmnPI / 180.0;
                    const double distanceTolerance = 2.0 * cmn_mm;
                    arm->mPositionDifference.DifferenceOf(arm->mPositions[0], arm->mPositions[1]);
                    if ((arm->mPositionDifference[0] > distanceTolerance) ||
                        (arm->mPositionDifference.Ref(5, 1).MaxAbsElement() > angleTolerance)) {
                        // send messages if this is new
                        if (arm->mPotsAgree) {
                            mInterface->SendWarning(this->GetName() + ": " + arm->mName.Data + " primary and secondary potentiometers don't seem to agree.");
                            CMN_LOG_CLASS_RUN_WARNING << "GetAndConvertPotentiometerValues, error: " << std::endl
                                                      << " - " << this->GetName() << ": " << arm->mName.Data << std::endl
                                                      << " - primary:   " << arm->mPositions[0] << std::endl
                                                      << " - secondary: " << arm->mPositions[1] << std::endl;
                            arm->mPotsAgree = false;
                        }
                    } else {
                        if (!arm->mPotsAgree) {
                            mInterface->SendStatus(this->GetName() + ": " + arm->mName.Data + " primary and secondary potentiometers agree.");
                            CMN_LOG_CLASS_RUN_VERBOSE << "GetAndConvertPotentiometerValues recovery" << std::endl
                                                      << " - " << this->GetName() << ": " << arm->mName.Data << std::endl;
                            arm->mPotsAgree = true;
                        }
                    }
                }

                // update and send desired position
                arm->mPositionCartesianDesiredParam.Position().From(arm->mBaseFrame * arm->mPositionCartesianLocalDesired);
                arm->mPositionCartesianDesiredParam.SetTimestamp(arm->mStateJoint.Timestamp());
                arm->mPositionCartesianDesiredParam.SetValid(arm->mBaseFrameValid * arm->mPositionCartesianLocalDesiredParam.Valid());
                arm->EventPositionCartesianDesired(arm->mPositionCartesianDesiredParam);
                // local is constant, no computation needed here
                arm->EventPositionCartesianLocalDesired(arm->mPositionCartesianLocalDesiredParam);
                // advance this arm state table
                arm->mStateTable.Advance();
            }
        }
    }
}

void mtsIntuitiveResearchKitSUJ::SetDesiredState(const std::string & state)
{
    // try to find the state in state machine
    if (!mArmState.StateExists(state)) {
        DispatchError(this->GetName() + ": unsupported state " + state);
        return;
    }
    // setting desired state triggers a new event so user nows which state is current
    DispatchCurrentState(mArmState.CurrentState());
    // try to set the desired state
    try {
        mArmState.SetDesiredState(state);
    } catch (...) {
        DispatchError(this->GetName() + ": " + state + " is not an allowed desired state");
        return;
    }
    // update state table
    mStateTableState.Start();
    mStateTableStateDesired = state;
    mStateTableState.Advance();

    DispatchDesiredState(state);
    DispatchStatus(this->GetName() + ": desired state " + state);
}

void mtsIntuitiveResearchKitSUJ::RunReady(void)
{
    if (mIsSimulated) {
        return;
    }

    double currentTic = this->StateTable.GetTic();
    const double timeDelta = currentTic - mPreviousTic;

    const double brakeCurrentRate = 8.0; // rate = 8 A/s, about 1/4 second to get up/down

    mtsIntuitiveResearchKitSUJArmData * arm;
    for (size_t armIndex = 0; armIndex < 4; ++armIndex) {
        arm = Arms[armIndex];
        // brakes
        // increase current for brakes
        if (arm->mClutched > 0) {
            arm->mStateTableBrakeCurrent.Start();
            if (((brakeCurrentRate * timeDelta) + arm->mBrakeDesiredCurrent) < arm->mBrakeReleaseCurrent) {
                arm->mBrakeDesiredCurrent += brakeCurrentRate * timeDelta;
            } else {
                arm->mBrakeDesiredCurrent = arm->mBrakeReleaseCurrent;
            }
            arm->mStateTableBrakeCurrent.Advance();
        }
        // decrease current for brakes
        else {
            if (arm->mBrakeDesiredCurrent != arm->mBrakeEngagedCurrent) {
                arm->mStateTableBrakeCurrent.Start();
                if ((arm->mBrakeDesiredCurrent - (brakeCurrentRate * timeDelta)) >= arm->mBrakeEngagedCurrent) {
                    arm->mBrakeDesiredCurrent -= brakeCurrentRate * timeDelta;
                    // if by any luck we have reached arm->mBrakeEngagedCurrent, need to update cartesian desired
                    if (arm->mBrakeDesiredCurrent == arm->mBrakeEngagedCurrent) {
                        arm->mNeedToUpdatePositionCartesianDesired = 1;
                    }
                } else {
                    arm->mBrakeDesiredCurrent = arm->mBrakeEngagedCurrent;
                    arm->mNeedToUpdatePositionCartesianDesired = 1;
                }
                arm->mStateTableBrakeCurrent.Advance();
            }
        }
        mBrakeCurrents[armIndex] = arm->mBrakeDirectionCurrent * arm->mBrakeDesiredCurrent;
    }
    RobotIO.SetActuatorCurrent(mBrakeCurrents);
    mPreviousTic = currentTic;
}

void mtsIntuitiveResearchKitSUJ::SetLiftVelocity(const double & velocity)
{
    if ((velocity >= -1.0) && (velocity <= 1.0)) {
        const double dutyCyle = 0.5 + velocity * 0.1;  // 0.1 determines max velocity
        PWM.SetPWMDutyCycle(dutyCyle);
    } else {
        CMN_LOG_CLASS_RUN_ERROR << "MotorVelocity: value must be between -1.0 and 1.0" << std::endl;
    }
}

void mtsIntuitiveResearchKitSUJ::SetBaseFrame(const prmPositionCartesianGet & newBaseFrame)
{
    mtsIntuitiveResearchKitSUJArmData * arm;
    for (size_t armIndex = 0; armIndex < 4; ++armIndex) {
        arm = Arms[armIndex];
        if (arm->mType != mtsIntuitiveResearchKitSUJArmData::SUJ_ECM) {
            vctFrm4x4 base;
            base.From(newBaseFrame.Position());
            arm->mBaseFrame = base;
            arm->mBaseFrameValid = newBaseFrame.Valid();
        }
    }
}

void mtsIntuitiveResearchKitSUJ::MotorDownEventHandler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        SetLiftVelocity(-1.0);
    } else {
        SetLiftVelocity(0.0);
    }
}

void mtsIntuitiveResearchKitSUJ::MotorUpEventHandler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        SetLiftVelocity(1.0);
    } else {
        SetLiftVelocity(0.0);
    }
}

void mtsIntuitiveResearchKitSUJ::ErrorEventHandler(const mtsMessage & message)
{
    RobotIO.DisablePower();
    DispatchError(this->GetName() + ": received [" + message.Message + "]");
    mArmState.SetCurrentState("UNINITIALIZED");
}

void mtsIntuitiveResearchKitSUJ::DispatchError(const std::string & message)
{
    mInterface->SendError(message);
    for (size_t armIndex = 0; armIndex < 4; ++armIndex) {
        Arms[armIndex]->mInterface->SendError(Arms[armIndex]->mName.Data + " " + message);
    }
}

void mtsIntuitiveResearchKitSUJ::DispatchWarning(const std::string & message)
{
    mInterface->SendWarning(message);
    for (size_t armIndex = 0; armIndex < 4; ++armIndex) {
        Arms[armIndex]->mInterface->SendWarning(Arms[armIndex]->mName.Data + " " + message);
    }
}

void mtsIntuitiveResearchKitSUJ::DispatchStatus(const std::string & message)
{
    mInterface->SendStatus(message);
    for (size_t armIndex = 0; armIndex < 4; ++armIndex) {
        Arms[armIndex]->mInterface->SendStatus(Arms[armIndex]->mName.Data + " " + message);
    }
}

void mtsIntuitiveResearchKitSUJ::DispatchCurrentState(const std::string & state)
{
    MessageEvents.CurrentState(state);
    for (size_t armIndex = 0; armIndex < 4; ++armIndex) {
        Arms[armIndex]->MessageEvents.CurrentState(state);
    }
}

void mtsIntuitiveResearchKitSUJ::DispatchDesiredState(const std::string & state)
{
    MessageEvents.DesiredState(state);
    for (size_t armIndex = 0; armIndex < 4; ++armIndex) {
        Arms[armIndex]->MessageEvents.DesiredState(state);
    }
}
