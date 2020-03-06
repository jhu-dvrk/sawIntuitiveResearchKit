/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// cisst
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationPSM.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperationPSM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsTeleOperationPSM::mtsTeleOperationPSM(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    mTeleopState(componentName, "DISABLED")
{
    Init();
}

mtsTeleOperationPSM::mtsTeleOperationPSM(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    mTeleopState(arg.Name, "DISABLED")
{
    Init();
}

mtsTeleOperationPSM::~mtsTeleOperationPSM()
{
}

void mtsTeleOperationPSM::Init(void)
{
    // configure state machine
    mTeleopState.AddState("SETTING_ARMS_STATE");
    mTeleopState.AddState("ALIGNING_MTM");
    mTeleopState.AddState("ENABLED");
    mTeleopState.AddAllowedDesiredState("ENABLED");
    mTeleopState.AddAllowedDesiredState("ALIGNING_MTM");
    mTeleopState.AddAllowedDesiredState("DISABLED");

    // state change, to convert to string events for users (Qt, ROS)
    mTeleopState.SetStateChangedCallback(&mtsTeleOperationPSM::StateChanged,
                                         this);

    // run for all states
    mTeleopState.SetRunCallback(&mtsTeleOperationPSM::RunAllStates,
                                this);

    // disabled
    mTeleopState.SetTransitionCallback("DISABLED",
                                       &mtsTeleOperationPSM::TransitionDisabled,
                                       this);

    // setting arms state
    mTeleopState.SetEnterCallback("SETTING_ARMS_STATE",
                                  &mtsTeleOperationPSM::EnterSettingArmsState,
                                  this);
    mTeleopState.SetTransitionCallback("SETTING_ARMS_STATE",
                                       &mtsTeleOperationPSM::TransitionSettingArmsState,
                                       this);

    // aligning MTM
    mTeleopState.SetEnterCallback("ALIGNING_MTM",
                                  &mtsTeleOperationPSM::EnterAligningMTM,
                                  this);
    mTeleopState.SetRunCallback("ALIGNING_MTM",
                                  &mtsTeleOperationPSM::RunAligningMTM,
                                  this);
    mTeleopState.SetTransitionCallback("ALIGNING_MTM",
                                       &mtsTeleOperationPSM::TransitionAligningMTM,
                                       this);

    // enabled
    mTeleopState.SetEnterCallback("ENABLED",
                                  &mtsTeleOperationPSM::EnterEnabled,
                                  this);
    mTeleopState.SetRunCallback("ENABLED",
                                &mtsTeleOperationPSM::RunEnabled,
                                this);
    mTeleopState.SetTransitionCallback("ENABLED",
                                       &mtsTeleOperationPSM::TransitionEnabled,
                                       this);

    mPSM.PositionJointSet.Goal().SetSize(1);

    this->StateTable.AddData(mMTM.PositionCartesianCurrent, "MTMCartesianPositionCurrent");
    this->StateTable.AddData(mMTM.PositionCartesianDesired, "MTMCartesianPositionDesired");
    this->StateTable.AddData(mPSM.PositionCartesianCurrent, "PSMCartesianPosition");
    this->StateTable.AddData(mAlignOffset, "AlignOffset");

    mConfigurationStateTable = new mtsStateTable(100, "Configuration");
    mConfigurationStateTable->SetAutomaticAdvance(false);
    this->AddStateTable(mConfigurationStateTable);
    mConfigurationStateTable->AddData(mScale, "Scale");
    mConfigurationStateTable->AddData(mRegistrationRotation, "RegistrationRotation");
    mConfigurationStateTable->AddData(mRotationLocked, "RotationLocked");
    mConfigurationStateTable->AddData(mTranslationLocked, "TranslationLocked");
    mConfigurationStateTable->AddData(mAlignMTM, "AlignMTM");

    // setup cisst interfaces
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("MTM");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetPositionCartesian", mMTM.GetPositionCartesian);
        interfaceRequired->AddFunction("GetPositionCartesianDesired", mMTM.GetPositionCartesianDesired);
        interfaceRequired->AddFunction("SetPositionGoalCartesian", mMTM.SetPositionGoalCartesian);
        interfaceRequired->AddFunction("GetStateGripper", mMTM.GetStateGripper);
        interfaceRequired->AddFunction("LockOrientation", mMTM.LockOrientation);
        interfaceRequired->AddFunction("UnlockOrientation", mMTM.UnlockOrientation);
        interfaceRequired->AddFunction("SetWrenchBody", mMTM.SetWrenchBody);
        interfaceRequired->AddFunction("SetGravityCompensation", mMTM.SetGravityCompensation);
        interfaceRequired->AddFunction("GetCurrentState", mMTM.GetCurrentState);
        interfaceRequired->AddFunction("GetDesiredState", mMTM.GetDesiredState);
        interfaceRequired->AddFunction("SetDesiredState", mMTM.SetDesiredState);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSM::MTMErrorEventHandler,
                                                this, "Error");
    }

    interfaceRequired = AddInterfaceRequired("PSM");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetPositionCartesian", mPSM.GetPositionCartesian);
        interfaceRequired->AddFunction("SetPositionCartesian", mPSM.SetPositionCartesian);
        interfaceRequired->AddFunction("Freeze", mPSM.Freeze);
        interfaceRequired->AddFunction("GetStateJaw", mPSM.GetStateJaw, MTS_OPTIONAL);
        interfaceRequired->AddFunction("GetConfigurationJaw", mPSM.GetConfigurationJaw, MTS_OPTIONAL);
        interfaceRequired->AddFunction("SetPositionJaw", mPSM.SetPositionJaw, MTS_OPTIONAL);
        interfaceRequired->AddFunction("GetCurrentState", mPSM.GetCurrentState);
        interfaceRequired->AddFunction("GetDesiredState", mPSM.GetDesiredState);
        interfaceRequired->AddFunction("SetDesiredState", mPSM.SetDesiredState);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSM::PSMErrorEventHandler,
                                                this, "Error");
    }

    // footpedal events
    interfaceRequired = AddInterfaceRequired("Clutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSM::ClutchEventHandler, this, "Button");
    }

    interfaceRequired = AddInterfaceRequired("PSM-base-frame", MTS_OPTIONAL);
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetPositionCartesian", mBaseFrame.GetPositionCartesian);
    }

    mInterface = AddInterfaceProvided("Setting");
    if (mInterface) {
        mInterface->AddMessageEvents();
        // commands
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                        "GetPeriodStatistics"); // mtsIntervalStatistics

        mInterface->AddCommandWrite(&mtsTeleOperationPSM::SetDesiredState, this,
                                    "SetDesiredState", mTeleopState.CurrentState());
        mInterface->AddCommandWrite(&mtsTeleOperationPSM::SetScale, this,
                                    "SetScale", mScale);
        mInterface->AddCommandWrite(&mtsTeleOperationPSM::SetRegistrationRotation, this,
                                    "SetRegistrationRotation", vctMatRot3());
        mInterface->AddCommandWrite(&mtsTeleOperationPSM::LockRotation, this,
                                    "LockRotation", mRotationLocked);
        mInterface->AddCommandWrite(&mtsTeleOperationPSM::LockTranslation, this,
                                    "LockTranslation", mTranslationLocked);
        mInterface->AddCommandWrite(&mtsTeleOperationPSM::SetAlignMTM, this,
                                    "SetAlignMTM", mAlignMTM);
        mInterface->AddCommandReadState(*(mConfigurationStateTable),
                                        mScale,
                                        "GetScale");
        mInterface->AddCommandReadState(*(mConfigurationStateTable),
                                        mRegistrationRotation,
                                        "GetRegistrationRotation");
        mInterface->AddCommandReadState(*(mConfigurationStateTable),
                                        mRotationLocked, "GetRotationLocked");
        mInterface->AddCommandReadState(*(mConfigurationStateTable),
                                        mTranslationLocked, "GetTranslationLocked");
        mInterface->AddCommandReadState(*(mConfigurationStateTable),
                                        mAlignMTM, "GetAlignMTM");
        mInterface->AddCommandReadState(this->StateTable,
                                        mMTM.PositionCartesianCurrent,
                                        "GetPositionCartesianMTM");
        mInterface->AddCommandReadState(this->StateTable,
                                        mPSM.PositionCartesianCurrent,
                                        "GetPositionCartesianPSM");
        mInterface->AddCommandReadState(this->StateTable,
                                        mAlignOffset,
                                        "GetAlignOffset");
        // events
        mInterface->AddEventWrite(MessageEvents.DesiredState,
                                  "DesiredState", std::string(""));
        mInterface->AddEventWrite(MessageEvents.CurrentState,
                                  "CurrentState", std::string(""));
        mInterface->AddEventWrite(MessageEvents.Following,
                                  "Following", false);
        // configuration
        mInterface->AddEventWrite(ConfigurationEvents.Scale,
                                  "Scale", mScale);
        mInterface->AddEventWrite(ConfigurationEvents.RotationLocked,
                                  "RotationLocked", mRotationLocked);
        mInterface->AddEventWrite(ConfigurationEvents.TranslationLocked,
                                  "TranslationLocked", mTranslationLocked);
        mInterface->AddEventWrite(ConfigurationEvents.AlignMTM,
                                  "AlignMTM", mAlignMTM);
    }
}

void mtsTeleOperationPSM::Configure(const std::string & CMN_UNUSED(filename))
{
}

void mtsTeleOperationPSM::Configure(const Json::Value & jsonConfig)
{
    Json::Value jsonValue;

    // read scale if present
    jsonValue = jsonConfig["scale"];
    if (!jsonValue.empty()) {
        mScale = jsonValue.asDouble();
    }
    if (mScale <= 0.0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": \"scale\" must be a positive number.  Found " << mScale << std::endl;
        exit(EXIT_FAILURE);
    }

    // read orientation if present
    jsonValue = jsonConfig["rotation"];
    if (!jsonValue.empty()) {
        vctMatRot3 orientation; // identity by default
        cmnDataJSON<vctMatRot3>::DeSerializeText(orientation, jsonConfig["rotation"]);
        SetRegistrationRotation(orientation);
    }

    // rotation locked
    jsonValue = jsonConfig["rotation-locked"];
    if (!jsonValue.empty()) {
        mRotationLocked = jsonValue.asBool();
    }

    // rotation locked
    jsonValue = jsonConfig["translation-locked"];
    if (!jsonValue.empty()) {
        mTranslationLocked = jsonValue.asBool();
    }

    // ignore jaw if needed
    jsonValue = jsonConfig["ignore-jaw"];
    if (!jsonValue.empty()) {
        mIgnoreJaw = jsonValue.asBool();
    }

    // jaw rate of opening-closing
    jsonValue = jsonConfig["jaw-rate"];
    if (!jsonValue.empty()) {
        mJawRate = jsonValue.asDouble();
    }
    if (mJawRate <= 0.0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": \"jaw-rate\" must be a positive number.  Found " << mJawRate << std::endl;
        exit(EXIT_FAILURE);
    }

    // jaw rate of opening-closing after clutch
    jsonValue = jsonConfig["jaw-rate-back-from-clutch"];
    if (!jsonValue.empty()) {
        mJawRateBackFromClutch = jsonValue.asDouble();
    }
    if (mJawRateBackFromClutch <= 0.0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": \"jaw-rate-back-from-clutch\" must be a positive number.  Found " << mJawRate << std::endl;
        exit(EXIT_FAILURE);
    }

    // orientation tolerance to start teleop
    jsonValue = jsonConfig["start-orientation-tolerance"];
    if (!jsonValue.empty()) {
        mOperator.OrientationTolerance = jsonValue.asDouble();
    }
    if (mOperator.OrientationTolerance < 0.0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": \"start-orientation-tolerance\" must be a positive number.  Found "
                                 << mOperator.OrientationTolerance << std::endl;
        exit(EXIT_FAILURE);
    }

    // gripper threshold to start teleop
    jsonValue = jsonConfig["start-gripper-threshold"];
    if (!jsonValue.empty()) {
        mOperator.GripperThreshold = jsonValue.asDouble();
    }
    if (mOperator.GripperThreshold < 0.0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": \"start-gripper-threshold\" must be a positive number.  Found "
                                 << mOperator.GripperThreshold << std::endl;
        exit(EXIT_FAILURE);
    }

    // roll threshold to start teleop
    jsonValue = jsonConfig["start-roll-threshold"];
    if (!jsonValue.empty()) {
        mOperator.RollThreshold = jsonValue.asDouble();
    }
    if (mOperator.RollThreshold < 0.0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": \"start-roll-threshold\" must be a positive number.  Found "
                                 << mOperator.RollThreshold << std::endl;
        exit(EXIT_FAILURE);
    }

    // align MTM if needed
    jsonValue = jsonConfig["align-mtm"];
    if (!jsonValue.empty()) {
        mAlignMTM = jsonValue.asBool();
    }
}

void mtsTeleOperationPSM::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
    SetScale(mScale);
    SetFollowing(false);
    LockRotation(mRotationLocked);
    LockTranslation(mTranslationLocked);
    SetAlignMTM(mAlignMTM);

    // check if functions for jaw are connected
    if (!mIgnoreJaw) {
        if (!mPSM.GetStateJaw.IsValid()
            || !mPSM.SetPositionJaw.IsValid()) {
            mInterface->SendError(this->GetName() + ": optional functions \"SetPositionJaw\" and \"GetStateJaw\" are not connected, setting \"ignore-jaw\" to true");
            mIgnoreJaw = true;
        }
    }
}

void mtsTeleOperationPSM::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    // run based on state
    mTeleopState.Run();
}

void mtsTeleOperationPSM::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsTeleOperationPSM::MTMErrorEventHandler(const mtsMessage & message)
{
    mTeleopState.SetDesiredState("DISABLED");
    mInterface->SendError(this->GetName() + ": received from MTM [" + message.Message + "]");
}

void mtsTeleOperationPSM::PSMErrorEventHandler(const mtsMessage & message)
{
    mTeleopState.SetDesiredState("DISABLED");
    mInterface->SendError(this->GetName() + ": received from PSM [" + message.Message + "]");
}

void mtsTeleOperationPSM::ClutchEventHandler(const prmEventButton & button)
{
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        mIsClutched = true;
        break;
    case prmEventButton::RELEASED:
        mIsClutched = false;
        break;
    default:
        break;
    }

    // if the teleoperation is activated
    if (mTeleopState.DesiredState() == "ENABLED") {
        Clutch(mIsClutched);
    }
}

void mtsTeleOperationPSM::Clutch(const bool & clutch)
{
    // if the teleoperation is activated
    if (clutch) {
        // keep track of last follow mode
        mOperator.WasActiveBeforeClutch = mOperator.IsActive;
        SetFollowing(false);
        mMTM.PositionCartesianSet.Goal().Rotation().FromNormalized(mPSM.PositionCartesianCurrent.Position().Rotation());
        mMTM.PositionCartesianSet.Goal().Translation().Assign(mMTM.PositionCartesianCurrent.Position().Translation());
        mInterface->SendStatus(this->GetName() + ": console clutch pressed");

        // no force applied but gravity and locked orientation
        prmForceCartesianSet wrench;
        mMTM.SetWrenchBody(wrench);
        mMTM.SetGravityCompensation(true);
        if (mAlignMTM || mRotationLocked) {
            // lock in current position
            mMTM.LockOrientation(mMTM.PositionCartesianCurrent.Position().Rotation());
        } else {
            // make sure it is freed
            mMTM.UnlockOrientation();
        }

        // make sure PSM stops moving
        mPSM.Freeze();
    } else {
        mInterface->SendStatus(this->GetName() + ": console clutch released");
        mTeleopState.SetCurrentState("SETTING_ARMS_STATE");
        mBackFromClutch = true;
        mJawCaughtUpAfterClutch = false;
    }
}

void mtsTeleOperationPSM::SetDesiredState(const std::string & state)
{
    // try to find the state in state machine
    if (!mTeleopState.StateExists(state)) {
        mInterface->SendError(this->GetName() + ": unsupported state " + state);
        return;
    }
    // try to set the desired state
    try {
        mTeleopState.SetDesiredState(state);
    } catch (...) {
        mInterface->SendError(this->GetName() + ": " + state + " is not an allowed desired state");
        return;
    }
    // force operator to indicate they are present
    mOperator.IsActive = false;
    MessageEvents.DesiredState(state);
    mInterface->SendStatus(this->GetName() + ": set desired state to " + state);
}

void mtsTeleOperationPSM::SetScale(const double & scale)
{
    // set scale
    mConfigurationStateTable->Start();
    mScale = scale;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.Scale(mScale);

    // update MTM/PSM previous position to prevent jumps
    mMTM.CartesianInitial.From(mMTM.PositionCartesianCurrent.Position());
    mPSM.CartesianInitial.From(mPSM.PositionCartesianCurrent.Position());
}

void mtsTeleOperationPSM::SetRegistrationRotation(const vctMatRot3 & rotation)
{
    mConfigurationStateTable->Start();
    mRegistrationRotation = rotation;
    mConfigurationStateTable->Advance();
}

void mtsTeleOperationPSM::LockRotation(const bool & lock)
{
    mConfigurationStateTable->Start();
    mRotationLocked = lock;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.RotationLocked(mRotationLocked);
    // when releasing the orientation, MTM orientation is likely off
    // so force re-align
    if (lock == false) {
        SetFollowing(false);
        mTeleopState.SetCurrentState("DISABLED");
    } else {
        // update MTM/PSM previous position
        mMTM.CartesianInitial.From(mMTM.PositionCartesianDesired.Position());
        mPSM.CartesianInitial.From(mPSM.PositionCartesianCurrent.Position());
        // lock orientation is the arm is running
        if (mTeleopState.CurrentState() == "ENABLED") {
            mMTM.LockOrientation(mMTM.PositionCartesianCurrent.Position().Rotation());
        }
    }
}

void mtsTeleOperationPSM::LockTranslation(const bool & lock)
{
    mConfigurationStateTable->Start();
    mTranslationLocked = lock;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.TranslationLocked(mTranslationLocked);
    // update MTM/PSM previous position
    mMTM.CartesianInitial.From(mMTM.PositionCartesianDesired.Position());
    mPSM.CartesianInitial.From(mPSM.PositionCartesianCurrent.Position());
}

void mtsTeleOperationPSM::SetAlignMTM(const bool & alignMTM)
{
    mConfigurationStateTable->Start();
    mAlignMTM = alignMTM;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.AlignMTM(mAlignMTM);
}

void mtsTeleOperationPSM::StateChanged(void)
{
    const std::string newState = mTeleopState.CurrentState();
    MessageEvents.CurrentState(newState);
    mInterface->SendStatus(this->GetName() + ": current state is " + newState);
}

void mtsTeleOperationPSM::RunAllStates(void)
{
    mtsExecutionResult executionResult;

    // get MTM Cartesian position
    executionResult = mMTM.GetPositionCartesian(mMTM.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTM.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from MTM");
        this->SetDesiredState("DISABLED");
    }
    executionResult = mMTM.GetPositionCartesianDesired(mMTM.PositionCartesianDesired);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTM.GetPositionCartesianDesired failed \""
                                << executionResult << "\"" << std::endl;
    }

    // get PSM Cartesian position
    executionResult = mPSM.GetPositionCartesian(mPSM.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to PSM.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from PSM");
        this->SetDesiredState("DISABLED");
    }

    // get base-frame cartesian position if available
    if (mBaseFrame.GetPositionCartesian.IsValid()) {
        executionResult = mBaseFrame.GetPositionCartesian(mBaseFrame.PositionCartesianCurrent);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << "Run: call to BaseFrame.GetPositionCartesian failed \""
                                    << executionResult << "\"" << std::endl;
            mInterface->SendError(this->GetName() + ": unable to get cartesian position from base frame");
            this->SetDesiredState("DISABLED");
        }
    }

    // check if anyone wanted to disable anyway
    if ((mTeleopState.DesiredState() == "DISABLED")
        && (mTeleopState.CurrentState() != "DISABLED")) {
        SetFollowing(false);
        mTeleopState.SetCurrentState("DISABLED");
        return;
    }

    // monitor state of arms if needed
    if ((mTeleopState.CurrentState() != "DISABLED")
        && (mTeleopState.CurrentState() != "SETTING_ARMS_STATE")) {
        std::string armState;
        mPSM.GetDesiredState(armState);
        if (armState != "READY") {
            this->SetDesiredState("DISABLED");
            mInterface->SendError(this->GetName() + ": PSM is not in state \"READY\" anymore");
        }
        mMTM.GetDesiredState(armState);
        if (armState != "READY") {
            this->SetDesiredState("DISABLED");
            mInterface->SendError(this->GetName() + ": MTM is not in state \"READY\" anymore");
        }
    }
}

void mtsTeleOperationPSM::TransitionDisabled(void)
{
    if (mTeleopState.DesiredStateIsNotCurrent()) {
        mTeleopState.SetCurrentState("SETTING_ARMS_STATE");
    }
}

void mtsTeleOperationPSM::EnterSettingArmsState(void)
{
    // reset timer
    mInStateTimer = StateTable.GetTic();

    // request state if needed
    std::string armState;
    mPSM.GetDesiredState(armState);
    if (armState != "READY") {
        mPSM.SetDesiredState(std::string("READY"));
    }
    mMTM.GetDesiredState(armState);
    if (armState != "READY") {
        mMTM.SetDesiredState(std::string("READY"));
    }
}

void mtsTeleOperationPSM::TransitionSettingArmsState(void)
{
    // check state
    std::string psmState, mtmState;
    mPSM.GetCurrentState(psmState);
    mMTM.GetCurrentState(mtmState);
    if ((psmState == "READY")
        && (mtmState == "READY")) {
        mTeleopState.SetCurrentState("ALIGNING_MTM");
        return;
    }
    // check timer
    if ((StateTable.GetTic() - mInStateTimer) > 60.0 * cmn_s) {
        mInterface->SendError(this->GetName() + ": timed out while setting up MTM state");
        this->SetDesiredState("DISABLED");
    }
}

void mtsTeleOperationPSM::EnterAligningMTM(void)
{
    // update user GUI re. scale
    ConfigurationEvents.Scale(mScale);

    // reset timer
    mInStateTimer = StateTable.GetTic();
    mTimeSinceLastAlign = 0.0;

    // if we don't align MTM, just stay in same position
    if (!mAlignMTM) {
        // convert to prm type
        mMTM.PositionCartesianSet.Goal().Assign(mMTM.PositionCartesianDesired.Position());
        mMTM.SetPositionGoalCartesian(mMTM.PositionCartesianSet);
    }

    if (mBackFromClutch) {
        mOperator.IsActive = mOperator.WasActiveBeforeClutch;
        mBackFromClutch = false;
    }

    if (!mIgnoreJaw) {
        // figure out the mapping between the MTM gripper angle and the PSM jaw angle
        UpdateGripperToJawConfiguration();
    }

    // set min/max for roll outside bounds
    mOperator.RollMin = cmnPI * 100.0;
    mOperator.RollMax = -cmnPI * 100.0;
    mOperator.GripperMin = cmnPI * 100.0;
    mOperator.GripperMax = -cmnPI * 100.0;
}

void mtsTeleOperationPSM::RunAligningMTM(void)
{
    // if clutched or align not needed, do nothing
    if (mIsClutched || !mAlignMTM) {
        return;
    }

    // set trajectory goal periodically, this will track PSM motion
    const double currentTime = StateTable.GetTic();
    if ((currentTime - mTimeSinceLastAlign) > 10.0 * cmn_ms) {
        mTimeSinceLastAlign = currentTime;
        // Orientate MTM with PSM
        vctFrm4x4 mtmCartesianGoal;
        mtmCartesianGoal.Translation().Assign(mMTM.PositionCartesianDesired.Position().Translation());
        vctMatRot3 mtmRotation;
        mtmRotation = mRegistrationRotation.Inverse() * mPSM.PositionCartesianCurrent.Position().Rotation();
        mtmCartesianGoal.Rotation().FromNormalized(mtmRotation);
        // convert to prm type
        mMTM.PositionCartesianSet.Goal().From(mtmCartesianGoal);
        mMTM.SetPositionGoalCartesian(mMTM.PositionCartesianSet);
    }
}

void mtsTeleOperationPSM::TransitionAligningMTM(void)
{
    // check psm state
    std::string armState;
    mPSM.GetCurrentState(armState);
    if ((armState != "READY") && (armState != "MANUAL")) {
        mInterface->SendWarning(this->GetName() + ": PSM state has changed to [" + armState + "]");
        mTeleopState.SetDesiredState("DISABLED");
        return;
    }

    // if the desired state is aligning MTM, just stay here
    if (!mTeleopState.DesiredStateIsNotCurrent()) {
        return;
    }

    // check difference of orientation between mtm and PSM to enable
    vctMatRot3 desiredOrientation;
    mRegistrationRotation.ApplyInverseTo(mPSM.PositionCartesianCurrent.Position().Rotation(),
                                         desiredOrientation);
    mMTM.PositionCartesianCurrent.Position().Rotation().ApplyInverseTo(desiredOrientation, mAlignOffset);
    vctAxAnRot3 axisAngle(mAlignOffset, VCT_NORMALIZE);
    double orientationError = 0.0;
    // set error only if we need to align MTM to PSM
    if (mAlignMTM) {
        orientationError = axisAngle.Angle();
    }

    // if not active, use gripper and/or roll to detect if the user is ready
    if (!mOperator.IsActive) {
        // update gripper values
        mMTM.GetStateGripper(mMTM.StateGripper);
        const double gripper = mMTM.StateGripper.Position()[0];
        if (gripper > mOperator.GripperMax) {
            mOperator.GripperMax = gripper;
        } else if (gripper < mOperator.GripperMin) {
            mOperator.GripperMin = gripper;
        }
        const double gripperRange = mOperator.GripperMax - mOperator.GripperMin;

        // checking roll
        const double roll = acos(vctDotProduct(desiredOrientation.Column(1),
                                               mMTM.PositionCartesianCurrent.Position().Rotation().Column(1)));
        if (roll > mOperator.RollMax) {
            mOperator.RollMax = roll;
        } else if (roll < mOperator.RollMin) {
            mOperator.RollMin = roll;
        }
        const double rollRange = mOperator.RollMax - mOperator.RollMin;

        // different conditions to set operator active
        if (gripperRange >= mOperator.GripperThreshold) {
            mOperator.IsActive = true;
        } else if (rollRange >= mOperator.RollThreshold) {
            mOperator.IsActive = true;
        } else if ((gripperRange + rollRange)
                   > 0.8 * (mOperator.GripperThreshold
                            + mOperator.RollThreshold)) {
            mOperator.IsActive = true;
        }
    }

    // finally check for transition
    if ((orientationError <= mOperator.OrientationTolerance)
        && mOperator.IsActive) {
        if (mTeleopState.DesiredState() == "ENABLED") {
            mTeleopState.SetCurrentState("ENABLED");
        }
    } else {
        // check timer and issue a message
        if ((StateTable.GetTic() - mInStateTimer) > 2.0 * cmn_s) {
            std::stringstream message;
            if (orientationError >= mOperator.OrientationTolerance) {
                message << this->GetName() + ": unable to align MTM, angle error is "
                        << orientationError * cmn180_PI << " (deg)";
            } else if (!mOperator.IsActive) {
                message << this->GetName() + ": pinch/twist MTM gripper a bit";
            }
            mInterface->SendWarning(message.str());
            mInStateTimer = StateTable.GetTic();
        }
    }
}

void mtsTeleOperationPSM::EnterEnabled(void)
{
    // update MTM/PSM previous position
    mMTM.CartesianInitial.From(mMTM.PositionCartesianCurrent.Position());
    mPSM.CartesianInitial.From(mPSM.PositionCartesianCurrent.Position());
    mAlignOffsetInitial = mAlignOffset;
    if (mBaseFrame.GetPositionCartesian.IsValid()) {
        mBaseFrame.CartesianInitial.From(mBaseFrame.PositionCartesianCurrent.Position());
    }

    // set gripper ghost if needed
    if (!mIgnoreJaw) {
        mJawCaughtUpAfterClutch = false;
        // gripper ghost
        mPSM.GetStateJaw(mPSM.StateJaw);
        double currentJaw = mPSM.StateJaw.Position()[0];
        mGripperGhost = JawToGripper(currentJaw);
    }

    // set MTM/PSM to Teleop (Cartesian Position Mode)
    mMTM.SetGravityCompensation(true);
    // set forces to zero and lock/unlock orientation as needed
    prmForceCartesianSet wrench;
    mMTM.SetWrenchBody(wrench);
    if (mRotationLocked) {
        mMTM.LockOrientation(mMTM.PositionCartesianCurrent.Position().Rotation());
    } else {
        mMTM.UnlockOrientation();
    }
    // check if by any chance the clutch pedal is pressed
    if (mIsClutched) {
        Clutch(true);
    } else {
        SetFollowing(true);
    }
}

void mtsTeleOperationPSM::RunEnabled(void)
{
    if (mMTM.PositionCartesianCurrent.Valid()
        && mPSM.PositionCartesianCurrent.Valid()) {
        // follow mode
        if (!mIsClutched) {
            // compute mtm Cartesian motion
            vctFrm4x4 mtmPosition(mMTM.PositionCartesianCurrent.Position());

            // translation
            vct3 mtmTranslation;
            vct3 psmTranslation;
            if (mTranslationLocked) {
                psmTranslation = mPSM.CartesianInitial.Translation();
            } else {
                mtmTranslation = (mtmPosition.Translation() - mMTM.CartesianInitial.Translation());
                psmTranslation = mtmTranslation * mScale;
                psmTranslation = mRegistrationRotation * psmTranslation + mPSM.CartesianInitial.Translation();
            }
            // rotation
            vctMatRot3 psmRotation;
            if (mRotationLocked) {
                psmRotation.From(mPSM.CartesianInitial.Rotation());
            } else {
                psmRotation = mRegistrationRotation * mtmPosition.Rotation() * mAlignOffsetInitial;
            }

            // compute desired psm position
            vctFrm4x4 psmCartesianGoal;
            psmCartesianGoal.Translation().Assign(psmTranslation);
            psmCartesianGoal.Rotation().FromNormalized(psmRotation);

            // take into account changes in PSM base frame if any
            if (mBaseFrame.GetPositionCartesian.IsValid()) {
                vctFrm4x4 baseFrame(mBaseFrame.PositionCartesianCurrent.Position());
                vctFrm4x4 baseFrameChange = baseFrame.Inverse() * mBaseFrame.CartesianInitial;
                // update PSM position goal
                psmCartesianGoal = baseFrameChange * psmCartesianGoal;
                // update alignment offset
                mtmPosition.Rotation().ApplyInverseTo(psmCartesianGoal.Rotation(), mAlignOffset);
            }

            // PSM go this cartesian position
            mPSM.PositionCartesianSet.Goal().FromNormalized(psmCartesianGoal);
            mPSM.SetPositionCartesian(mPSM.PositionCartesianSet);

            if (!mIgnoreJaw) {
                // gripper
                if (mMTM.GetStateGripper.IsValid()) {
                    mMTM.GetStateGripper(mMTM.StateGripper);
                    const double currentGripper = mMTM.StateGripper.Position()[0];
                    // see if we caught up
                    if (!mJawCaughtUpAfterClutch) {
                        const double error = std::abs(currentGripper - mGripperGhost);
                        if (error < mtsIntuitiveResearchKit::TeleOperationPSM::ToleranceBackFromClutch) {
                            mJawCaughtUpAfterClutch = true;
                        }
                    }
                    // pick the rate based on back from clutch or not
                    const double delta = mJawCaughtUpAfterClutch ?
                        mJawRate * StateTable.PeriodStats.PeriodAvg()
                        : mJawRateBackFromClutch * StateTable.PeriodStats.PeriodAvg();
                    // gripper ghost below, add to catch up
                    if (mGripperGhost <= (currentGripper - delta)) {
                        mGripperGhost += delta;
                    } else {
                        // gripper ghost above, subtract to catch up
                        if (mGripperGhost >= (currentGripper + delta)) {
                            mGripperGhost -= delta;
                        }
                    }
                    mPSM.PositionJointSet.Goal()[0] = GripperToJaw(mGripperGhost);
                    // make sure we don't send goal past joint limits
                    if (mPSM.PositionJointSet.Goal()[0] < mGripperToJaw.PositionMin) {
                        mPSM.PositionJointSet.Goal()[0] = mGripperToJaw.PositionMin;
                        mGripperGhost = JawToGripper(mGripperToJaw.PositionMin);
                    }
                    mPSM.SetPositionJaw(mPSM.PositionJointSet);
                } else {
                    mPSM.PositionJointSet.Goal()[0] = 45.0 * cmnPI_180;
                    mPSM.SetPositionJaw(mPSM.PositionJointSet);
                }
            }
        }
    }
}

void mtsTeleOperationPSM::TransitionEnabled(void)
{
    std::string armState;

    // check psm state
    mPSM.GetCurrentState(armState);
    if (armState != "READY") {
        mInterface->SendWarning(this->GetName() + ": PSM state has changed to [" + armState + "]");
        mTeleopState.SetDesiredState("DISABLED");
    }

    // check mtm state
    mMTM.GetCurrentState(armState);
    if (armState != "READY") {
        mInterface->SendWarning(this->GetName() + ": MTM state has changed to [" + armState + "]");
        mTeleopState.SetDesiredState("DISABLED");
    }

    if (mTeleopState.DesiredStateIsNotCurrent()) {
        SetFollowing(false);
        mTeleopState.SetCurrentState(mTeleopState.DesiredState());
    }
}

double mtsTeleOperationPSM::GripperToJaw(const double & gripperAngle) const
{
    return mGripperToJaw.Scale * gripperAngle + mGripperToJaw.Offset;
}

double mtsTeleOperationPSM::JawToGripper(const double & jawAngle) const
{
    return (jawAngle - mGripperToJaw.Offset) / mGripperToJaw.Scale;
}

void mtsTeleOperationPSM::UpdateGripperToJawConfiguration(void)
{
    // default values
    mGripperToJaw.Scale = 1.0;
    mGripperToJaw.Offset = 0.0;
    mGripperToJaw.PositionMin = cmnTypeTraits<double>::MinNegativeValue();
    // get the PSM jaw configuration if possible to find range
    if (mPSM.GetConfigurationJaw.IsValid()) {
        mPSM.GetConfigurationJaw(mPSM.ConfigurationJaw);
        if ((mPSM.ConfigurationJaw.PositionMin().size() == 1)
            && (mPSM.ConfigurationJaw.PositionMax().size() == 1)) {
            // for now we assume MTM is from 0 to 60 degrees
            double min = mPSM.ConfigurationJaw.PositionMin()[0];
            double max = mPSM.ConfigurationJaw.PositionMax()[0];
            // save min for later so we never ask PSM to close jaws more than min
            mGripperToJaw.PositionMin = min;
            // if the PSM can close its jaws past 0 (tighter), we map from 0 to qmax
            // negative values just mean tighter jaws
            if (min < 0.0) {
                min = 0.0;
            }
            mGripperToJaw.Scale = (max - min) / (60.0 * cmnPI_180);
            mGripperToJaw.Offset = min;
        }
    }
}

void mtsTeleOperationPSM::SetFollowing(const bool following)
{
    MessageEvents.Following(following);
    mIsFollowing = following;
}
