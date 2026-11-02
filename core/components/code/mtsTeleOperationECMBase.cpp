/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-01-21

  (C) Copyright 2016-2026 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/mtsTeleOperationECMBase.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmOperatingState.h>

CMN_IMPLEMENT_SERVICES_DERIVED(mtsTeleOperationECMBase, mtsTaskPeriodic);

mtsTeleOperationECMBase::mtsTeleOperationECMBase(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    mTeleopState(componentName, "DISABLED")
{
}

mtsTeleOperationECMBase::mtsTeleOperationECMBase(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    mTeleopState(arg.Name, "DISABLED")
{
}

mtsTeleOperationECMBase::~mtsTeleOperationECMBase()
{
}

void mtsTeleOperationECMBase::Init(void)
{
    m_following = false;
    m_clutched = false;

    // state machine
    mTeleopState.AddState("SETTING_ARMS_STATE");
    mTeleopState.AddState("CHECKING_DATA");
    mTeleopState.AddState("ENABLED");
    mTeleopState.AddAllowedDesiredState("DISABLED");
    mTeleopState.AddAllowedDesiredState("ENABLED");

    mTeleopState.SetStateChangedCallback(&mtsTeleOperationECMBase::StateChanged,
                                         this);

    // run for all states
    mTeleopState.SetRunCallback(&mtsTeleOperationECMBase::RunAllStates,
                                this);

    // disabled
    mTeleopState.SetTransitionCallback("DISABLED",
                                       &mtsTeleOperationECMBase::TransitionDisabled,
                                       this);

    // setting arms state
    mTeleopState.SetEnterCallback("SETTING_ARMS_STATE",
                                  &mtsTeleOperationECMBase::EnterSettingArmsState,
                                  this);
    mTeleopState.SetTransitionCallback("SETTING_ARMS_STATE",
                                       &mtsTeleOperationECMBase::TransitionSettingArmsState,
                                       this);

    // checking data
    mTeleopState.SetEnterCallback("CHECKING_DATA",
                                  &mtsTeleOperationECMBase::EnterCheckingData,
                                  this);
    mTeleopState.SetTransitionCallback("CHECKING_DATA",
                                       &mtsTeleOperationECMBase::TransitionCheckingData,
                                       this);

    // enabled
    mTeleopState.SetEnterCallback("ENABLED",
                                  &mtsTeleOperationECMBase::EnterEnabled,
                                  this);
    mTeleopState.SetRunCallback("ENABLED",
                                &mtsTeleOperationECMBase::RunEnabled,
                                this);
    mTeleopState.SetTransitionCallback("ENABLED",
                                       &mtsTeleOperationECMBase::TransitionEnabled,
                                       this);

    mConfigurationStateTable = new mtsStateTable(100, "Configuration");
    mConfigurationStateTable->SetAutomaticAdvance(false);
    AddStateTable(mConfigurationStateTable);
    mConfigurationStateTable->AddData(m_config.scale, "scale");

    StateTable.AddData(mECM.m_measured_cp, "ECM/measured_cp");

    // ECM base interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("ECM");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("measured_cp", mECM.measured_cp);
        interfaceRequired->AddFunction("operating_state", mECM.operating_state);
        interfaceRequired->AddFunction("state_command", mECM.state_command);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECMBase::arm_error_event_handler,
                                                this, "error");
    }

    interfaceRequired = AddInterfaceRequired("clutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECMBase::clutch_event_handler, this, "Button");
    }

    mInterface = AddInterfaceProvided("Setting");
    if (mInterface) {
        mInterface->AddMessageEvents();
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                        "period_statistics");

        mInterface->AddCommandWrite(&mtsTeleOperationECMBase::state_command, this,
                                    "state_command", std::string("DISABLED"));
        mInterface->AddCommandWrite(&mtsTeleOperationECMBase::set_scale, this,
                                    "set_scale", 0.5);
        mInterface->AddCommandReadState(*mConfigurationStateTable,
                                        m_config.scale,
                                        "scale");

        mInterface->AddCommandReadState(StateTable,
                                        mECM.m_measured_cp,
                                        "ECM/measured_cp");

        // events
        mInterface->AddEventWrite(MessageEvents.desired_state,
                                  "desired_state", std::string(""));
        mInterface->AddEventWrite(MessageEvents.current_state,
                                  "current_state", std::string(""));
        mInterface->AddEventWrite(MessageEvents.following,
                                  "following", false);
        // configuration
        mInterface->AddEventWrite(ConfigurationEvents.scale,
                                  "scale", 0.5);
    }
}

void mtsTeleOperationECMBase::Configure(const std::string & filename)
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
                               << "== content:" << std::endl << jsonConfig << std::endl
                               << "==" << std::endl;

    this->Configure(jsonConfig);
}

void mtsTeleOperationECMBase::Configure(const Json::Value & _json_config)
{
    Json::Value json_config = _json_config;

    try {
        m_config.DeSerializeTextJSON(json_config);
    } catch (std::exception & std_error) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to deserialize configuration: "
                                 << std_error.what() << std::endl;
        return;
    }

    if (!m_config.MTM_is_haptic) {
        m_config.check_MTMs_workspace = false;
    }
}

void mtsTeleOperationECMBase::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.scale(m_config.scale);
    SetDesiredState("DISABLED");
}

void mtsTeleOperationECMBase::set_scale(const double & scale)
{
    mConfigurationStateTable->Start();
    m_config.scale = scale;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.scale(m_config.scale);
}

void mtsTeleOperationECMBase::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    mTeleopState.Run();
}

void mtsTeleOperationECMBase::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsTeleOperationECMBase::StateChanged(void)
{
    const std::string newState = mTeleopState.CurrentState();
    MessageEvents.current_state(newState);
    mInterface->SendStatus(this->GetName() + ": current state is " + newState);
}

void mtsTeleOperationECMBase::RunAllStates(void)
{
    mtsExecutionResult executionResult;

    // get ECM Cartesian position for GUI / teleop
    executionResult = mECM.measured_cp(mECM.m_measured_cp);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to ECM.measured_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from ECM");
        mTeleopState.SetDesiredState("DISABLED");
    }

    // check if anyone wanted to disable anyway
    if ((mTeleopState.DesiredState() == "DISABLED")
        && (mTeleopState.CurrentState() != "DISABLED")) {
        set_following(false);
        mTeleopState.SetCurrentState("DISABLED");
        return;
    }

    // monitor state of ECM if needed
    if ((mTeleopState.CurrentState() != "DISABLED")
        && (mTeleopState.CurrentState() != "SETTING_ARMS_STATE")) {
        prmOperatingState state;
        mECM.operating_state(state);
        if ((state.State() != prmOperatingState::ENABLED)
            || !state.IsHomed()) {
            mTeleopState.SetDesiredState("DISABLED");
            mInterface->SendError(this->GetName() + ": ECM is not in state \"READY\" anymore");
        }
    }
}

void mtsTeleOperationECMBase::TransitionDisabled(void)
{
    if (mTeleopState.DesiredStateIsNotCurrent()) {
        mTeleopState.SetCurrentState("SETTING_ARMS_STATE");
    }
}

void mtsTeleOperationECMBase::EnterSettingArmsState(void)
{
    mECM.state_command(std::string("enable"));
    mInStateTimer = StateTable.GetTic();
}

void mtsTeleOperationECMBase::EnterCheckingData(void)
{
    mInStateTimer = StateTable.GetTic();
    m_data_check_warning_sent = false;
}

void mtsTeleOperationECMBase::TransitionEnabled(void)
{
    if (mTeleopState.DesiredStateIsNotCurrent()) {
        set_following(false);
        mTeleopState.SetCurrentState("DISABLED");
    }
}

void mtsTeleOperationECMBase::arm_error_event_handler(const mtsMessage & message)
{
    mInterface->SendError(this->GetName() + ": received from arm \"" + message.Message + "\"");
    SetDesiredState("DISABLED");
}

void mtsTeleOperationECMBase::clutch_event_handler(const prmEventButton & button)
{
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        m_clutched = true;
        break;
    case prmEventButton::RELEASED:
        m_clutched = false;
        break;
    default:
        break;
    }

    if (mTeleopState.DesiredState() == "ENABLED") {
        Clutch(m_clutched);
    }
}

void mtsTeleOperationECMBase::Clutch(const bool & clutch)
{
    m_clutched = clutch;
    if (clutch) {
        set_following(false);
        mInterface->SendStatus(this->GetName() + ": console clutch pressed");
    } else {
        mInterface->SendStatus(this->GetName() + ": console clutch released");
        mTeleopState.SetCurrentState("SETTING_ARMS_STATE");
    }
}

void mtsTeleOperationECMBase::state_command(const std::string & command)
{
    if (command == "enable") {
        SetDesiredState("ENABLED");
        return;
    }
    if (command == "disable") {
        SetDesiredState("DISABLED");
        return;
    }
    mInterface->SendWarning(this->GetName() + ": " + command + " doesn't seem to be a valid state_command");
}

void mtsTeleOperationECMBase::SetDesiredState(const std::string & state)
{
    // try to find the state in state machine
    if (!mTeleopState.StateExists(state)) {
        mInterface->SendError(this->GetName() + ": unsupported state " + state);
        return;
    }
    // return if already the desired state
    if (mTeleopState.DesiredState() == state) {
        MessageEvents.desired_state(state);
        return;
    }
    // try to set the desired state
    try {
        mTeleopState.SetDesiredState(state);
    } catch (...) {
        mInterface->SendError(this->GetName() + ": " + state + " doesn't seem to be a valid state");
        return;
    }
    MessageEvents.desired_state(state);
    mInterface->SendStatus(this->GetName() + ": set desired state to " + state);
}

void mtsTeleOperationECMBase::set_following(const bool following)
{
    if (following != m_following) {
        m_following = following;
        MessageEvents.following(m_following);
        if (m_following) {
            mInterface->SendStatus(this->GetName() + ": following");
        } else {
            mInterface->SendStatus(this->GetName() + ": not following");
        }
    }
}
