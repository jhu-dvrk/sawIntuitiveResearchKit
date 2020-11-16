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
#include <cisstParameterTypes/prmOperatingState.h>
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
        interfaceRequired->AddFunction("measured_cp", mMTM.measured_cp);
        interfaceRequired->AddFunction("setpoint_cp", mMTM.setpoint_cp);
        interfaceRequired->AddFunction("move_cp", mMTM.move_cp);
        interfaceRequired->AddFunction("gripper/measured_js", mMTM.gripper_measured_js);
        interfaceRequired->AddFunction("lock_orientation", mMTM.lock_orientation);
        interfaceRequired->AddFunction("unlock_orientation", mMTM.unlock_orientation);
        interfaceRequired->AddFunction("body/servo_cf", mMTM.servo_cf_body);
        interfaceRequired->AddFunction("use_gravity_compensation", mMTM.use_gravity_compensation);
        interfaceRequired->AddFunction("operating_state", mMTM.operating_state);
        interfaceRequired->AddFunction("state_command", mMTM.state_command);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSM::MTMErrorEventHandler,
                                                this, "error");
    }

    interfaceRequired = AddInterfaceRequired("PSM");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("measured_cp", mPSM.measured_cp);
        interfaceRequired->AddFunction("servo_cp", mPSM.servo_cp);
        interfaceRequired->AddFunction("Freeze", mPSM.Freeze);
        interfaceRequired->AddFunction("jaw/measured_js", mPSM.jaw_measured_js, MTS_OPTIONAL);
        interfaceRequired->AddFunction("jaw/configuration_js", mPSM.jaw_configuration_js, MTS_OPTIONAL);
        interfaceRequired->AddFunction("jaw/servo_jp", mPSM.jaw_servo_jp, MTS_OPTIONAL);
        interfaceRequired->AddFunction("operating_state", mPSM.operating_state);
        interfaceRequired->AddFunction("state_command", mPSM.state_command);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSM::PSMErrorEventHandler,
                                                this, "error");
    }

    // footpedal events
    interfaceRequired = AddInterfaceRequired("Clutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSM::ClutchEventHandler, this, "Button");
    }

    interfaceRequired = AddInterfaceRequired("PSM-base-frame", MTS_OPTIONAL);
    if (interfaceRequired) {
        interfaceRequired->AddFunction("measured_cp", mBaseFrame.measured_cp);
    }

    mInterface = AddInterfaceProvided("Setting");
    if (mInterface) {
        mInterface->AddMessageEvents();
        // commands
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                        "period_statistics"); // mtsIntervalStatistics

        mInterface->AddCommandWrite(&mtsTeleOperationPSM::state_command, this,
                                    "state_command", mTeleopState.CurrentState());
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
        mInterface->AddEventWrite(MessageEvents.desired_state,
                                  "desired_state", std::string(""));
        mInterface->AddEventWrite(MessageEvents.current_state,
                                  "current_state", std::string(""));
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

    // so sent commands can be used with ros-bridge
    mPSM.PositionCartesianSet.Valid() = true;
    mPSM.PositionJointSet.Valid() = true;
}

void mtsTeleOperationPSM::Configure(const std::string & CMN_UNUSED(filename))
{
}

void mtsTeleOperationPSM::Configure(const Json::Value & jsonConfig)
{
    Json::Value jsonValue;

    // base component configuration
    mtsComponent::ConfigureJSON(jsonConfig);

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
        if (!mPSM.jaw_measured_js.IsValid()
            || !mPSM.jaw_servo_jp.IsValid()) {
            mInterface->SendError(this->GetName() + ": optional functions \"jaw/servo_jp\" and \"jaw/measured_js\" are not connected, setting \"ignore-jaw\" to true");
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
        mMTM.servo_cf_body(wrench);
        mMTM.use_gravity_compensation(true);
        if (mAlignMTM || mRotationLocked) {
            // lock in current position
            mMTM.lock_orientation(mMTM.PositionCartesianCurrent.Position().Rotation());
        } else {
            // make sure it is freed
            mMTM.unlock_orientation();
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


void mtsTeleOperationPSM::state_command(const std::string & command)
{
    if (command == "enable") {
        SetDesiredState("ENABLED");
        return;
    }
    if (command == "disable") {
        SetDesiredState("DISABLED");
        return;
    }
    if (command == "align_mtm") {
        SetDesiredState("ALIGNING_MTM");
        return;
    }
    mInterface->SendWarning(this->GetName() + ": " + command + " doesn't seem to be a valid state_command");
}


void mtsTeleOperationPSM::SetDesiredState(const std::string & state)
{
    // try to find the state in state machine
    if (!mTeleopState.StateExists(state)) {
        mInterface->SendError(this->GetName() + ": unsupported state " + state);
        return;
    }
    // return is already the desired state
    if (mTeleopState.DesiredState() == state) {
        MessageEvents.desired_state(state);
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
    MessageEvents.desired_state(state);
    mInterface->SendStatus(this->GetName() + ": set desired state to " + state);
}

vctMatRot3 mtsTeleOperationPSM::UpdateAlignOffset(void)
{
    vctMatRot3 desiredOrientation;
    mRegistrationRotation.ApplyInverseTo(mPSM.PositionCartesianCurrent.Position().Rotation(),
                                         desiredOrientation);
    mMTM.PositionCartesianCurrent.Position().Rotation().ApplyInverseTo(desiredOrientation, mAlignOffset);
    return desiredOrientation;
}

void mtsTeleOperationPSM::UpdateInitialState(void)
{
    mMTM.CartesianInitial.From(mMTM.PositionCartesianCurrent.Position());
    mPSM.CartesianInitial.From(mPSM.PositionCartesianCurrent.Position());
    UpdateAlignOffset();
    mAlignOffsetInitial = mAlignOffset;
    if (mBaseFrame.measured_cp.IsValid()) {
        mBaseFrame.CartesianInitial.From(mBaseFrame.PositionCartesianCurrent.Position());
    }
}

void mtsTeleOperationPSM::SetScale(const double & scale)
{
    // set scale
    mConfigurationStateTable->Start();
    mScale = scale;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.Scale(mScale);

    // update MTM/PSM previous position to prevent jumps
    UpdateInitialState();
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
        UpdateInitialState();
        // lock orientation if the arm is running
        if (mTeleopState.CurrentState() == "ENABLED") {
            mMTM.lock_orientation(mMTM.PositionCartesianCurrent.Position().Rotation());
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
    UpdateInitialState();
}

void mtsTeleOperationPSM::SetAlignMTM(const bool & alignMTM)
{
    mConfigurationStateTable->Start();
    mAlignMTM = alignMTM;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.AlignMTM(mAlignMTM);
    // force re-align if the teleop is already enabled
    if (mTeleopState.CurrentState() == "ENABLED") {
        mTeleopState.SetCurrentState("DISABLED");
    }
}

void mtsTeleOperationPSM::StateChanged(void)
{
    const std::string newState = mTeleopState.CurrentState();
    MessageEvents.current_state(newState);
    mInterface->SendStatus(this->GetName() + ": current state is " + newState);
}

void mtsTeleOperationPSM::RunAllStates(void)
{
    mtsExecutionResult executionResult;

    // get MTM Cartesian position
    executionResult = mMTM.measured_cp(mMTM.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTM.measured_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from MTM");
        mTeleopState.SetDesiredState("DISABLED");
    }
    executionResult = mMTM.setpoint_cp(mMTM.PositionCartesianDesired);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTM.setpoint_cp failed \""
                                << executionResult << "\"" << std::endl;
    }

    // get PSM Cartesian position
    executionResult = mPSM.measured_cp(mPSM.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to PSM.measured_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from PSM");
        mTeleopState.SetDesiredState("DISABLED");
    }

    // get base-frame cartesian position if available
    if (mBaseFrame.measured_cp.IsValid()) {
        executionResult = mBaseFrame.measured_cp(mBaseFrame.PositionCartesianCurrent);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << "Run: call to m_base_frame.measured_cp failed \""
                                    << executionResult << "\"" << std::endl;
            mInterface->SendError(this->GetName() + ": unable to get cartesian position from base frame");
            mTeleopState.SetDesiredState("DISABLED");
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
        prmOperatingState state;
        mPSM.operating_state(state);
        if ((state.State() != prmOperatingState::ENABLED)
            || !state.IsHomed()) {
            mTeleopState.SetDesiredState("DISABLED");
            mInterface->SendError(this->GetName() + ": PSM is not in state \"ENABLED\" anymore");
        }
        mMTM.operating_state(state);
        if ((state.State() != prmOperatingState::ENABLED)
            || !state.IsHomed()) {
            mTeleopState.SetDesiredState("DISABLED");
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
    prmOperatingState state;
    mPSM.operating_state(state);
    if (state.State() != prmOperatingState::ENABLED) {
        mPSM.state_command(std::string("enable"));
    } else if (!state.IsHomed()) {
        mPSM.state_command(std::string("home"));
    }

    mMTM.operating_state(state);
    if (state.State() != prmOperatingState::ENABLED) {
        mMTM.state_command(std::string("enable"));
    } else if (!state.IsHomed()) {
        mMTM.state_command(std::string("home"));
    }
}

void mtsTeleOperationPSM::TransitionSettingArmsState(void)
{
    // check state
    prmOperatingState psmState, mtmState;
    mPSM.operating_state(psmState);
    mMTM.operating_state(mtmState);
    if ((psmState.State() == prmOperatingState::ENABLED) && psmState.IsHomed()
        && (mtmState.State() == prmOperatingState::ENABLED) && mtmState.IsHomed()) {
        mTeleopState.SetCurrentState("ALIGNING_MTM");
        return;
    }
    // check timer
    if ((StateTable.GetTic() - mInStateTimer) > 60.0 * cmn_s) {
        if (!((psmState.State() == prmOperatingState::ENABLED) && psmState.IsHomed())) {
            mInterface->SendError(this->GetName() + ": timed out while setting up PSM state");
        }
        if (!((mtmState.State() == prmOperatingState::ENABLED) && mtmState.IsHomed())) {
            mInterface->SendError(this->GetName() + ": timed out while setting up MTM state");
        }
        mTeleopState.SetDesiredState("DISABLED");
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
        mMTM.move_cp(mMTM.PositionCartesianSet);
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
        mMTM.move_cp(mMTM.PositionCartesianSet);
    }
}

void mtsTeleOperationPSM::TransitionAligningMTM(void)
{
    // if the desired state is aligning MTM, just stay here
    if (!mTeleopState.DesiredStateIsNotCurrent()) {
        return;
    }

    // check difference of orientation between mtm and PSM to enable
    vctMatRot3 desiredOrientation = UpdateAlignOffset();
    vctAxAnRot3 axisAngle(mAlignOffset, VCT_NORMALIZE);
    double orientationError = 0.0;
    // set error only if we need to align MTM to PSM
    if (mAlignMTM) {
        orientationError = axisAngle.Angle();
    }

    // if not active, use gripper and/or roll to detect if the user is ready
    if (!mOperator.IsActive) {
        // update gripper values
        mMTM.gripper_measured_js(mMTM.m_gripper_measured_js);
        const double gripper = mMTM.m_gripper_measured_js.Position()[0];
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
    UpdateInitialState();

    // set gripper ghost if needed
    if (!mIgnoreJaw) {
        mJawCaughtUpAfterClutch = false;
        // gripper ghost
        mPSM.jaw_measured_js(mPSM.m_jaw_measured_js);
        double currentJaw = mPSM.m_jaw_measured_js.Position()[0];
        mGripperGhost = JawToGripper(currentJaw);
    }

    // set MTM/PSM to Teleop (Cartesian Position Mode)
    mMTM.use_gravity_compensation(true);
    // set forces to zero and lock/unlock orientation as needed
    prmForceCartesianSet wrench;
    mMTM.servo_cf_body(wrench);
    if (mRotationLocked) {
        mMTM.lock_orientation(mMTM.PositionCartesianCurrent.Position().Rotation());
    } else {
        mMTM.unlock_orientation();
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
            if (mBaseFrame.measured_cp.IsValid()) {
                vctFrm4x4 baseFrame(mBaseFrame.PositionCartesianCurrent.Position());
                vctFrm4x4 baseFrameChange = baseFrame.Inverse() * mBaseFrame.CartesianInitial;
                // update PSM position goal
                psmCartesianGoal = baseFrameChange * psmCartesianGoal;
                // update alignment offset
                mtmPosition.Rotation().ApplyInverseTo(psmCartesianGoal.Rotation(), mAlignOffset);
            }

            // PSM go this cartesian position
            mPSM.PositionCartesianSet.Goal().FromNormalized(psmCartesianGoal);
            mPSM.servo_cp(mPSM.PositionCartesianSet);

            if (!mIgnoreJaw) {
                // gripper
                if (mMTM.gripper_measured_js.IsValid()) {
                    mMTM.gripper_measured_js(mMTM.m_gripper_measured_js);
                    const double currentGripper = mMTM.m_gripper_measured_js.Position()[0];
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
                    mPSM.jaw_servo_jp(mPSM.PositionJointSet);
                } else {
                    mPSM.PositionJointSet.Goal()[0] = 45.0 * cmnPI_180;
                    mPSM.jaw_servo_jp(mPSM.PositionJointSet);
                }
            }
        }
    }
}

void mtsTeleOperationPSM::TransitionEnabled(void)
{
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
    if (mPSM.jaw_configuration_js.IsValid()) {
        mPSM.jaw_configuration_js(mPSM.m_jaw_configuration_js);
        if ((mPSM.m_jaw_configuration_js.PositionMin().size() == 1)
            && (mPSM.m_jaw_configuration_js.PositionMax().size() == 1)) {
            // for now we assume MTM is from 0 to 60 degrees
            double min = mPSM.m_jaw_configuration_js.PositionMin()[0];
            double max = mPSM.m_jaw_configuration_js.PositionMax()[0];
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
