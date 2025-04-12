/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2023 Johns Hopkins University (JHU), All Rights Reserved.

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

    mPSM.m_jaw_servo_jp.Goal().SetSize(1);

    this->StateTable.AddData(mMTM.m_measured_cp, "MTM/measured_cp");
    this->StateTable.AddData(mMTM.m_measured_cv, "MTM/measured_cv");
    this->StateTable.AddData(mMTM.m_setpoint_cp, "MTM/setpoint_cp");
    this->StateTable.AddData(mPSM.m_setpoint_cp, "PSM/setpoint_cp");
    this->StateTable.AddData(m_alignment_offset, "alignment_offset");

    mConfigurationStateTable = new mtsStateTable(100, "Configuration");
    mConfigurationStateTable->SetAutomaticAdvance(false);
    this->AddStateTable(mConfigurationStateTable);
    mConfigurationStateTable->AddData(m_scale, "scale");
    mConfigurationStateTable->AddData(m_rotation_locked, "rotation_locked");
    mConfigurationStateTable->AddData(m_translation_locked, "translation_locked");
    mConfigurationStateTable->AddData(m_align_mtm, "align_mtm");

    // setup cisst interfaces
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("MTM");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("measured_cp", mMTM.measured_cp);
        interfaceRequired->AddFunction("measured_cv", mMTM.measured_cv, MTS_OPTIONAL);
        interfaceRequired->AddFunction("setpoint_cp", mMTM.setpoint_cp);
        interfaceRequired->AddFunction("move_cp", mMTM.move_cp);
        interfaceRequired->AddFunction("gripper/measured_js", mMTM.gripper_measured_js, MTS_OPTIONAL);
        interfaceRequired->AddFunction("lock_orientation", mMTM.lock_orientation, MTS_OPTIONAL);
        interfaceRequired->AddFunction("unlock_orientation", mMTM.unlock_orientation, MTS_OPTIONAL);
        interfaceRequired->AddFunction("body/servo_cf", mMTM.body_servo_cf);
        interfaceRequired->AddFunction("use_gravity_compensation", mMTM.use_gravity_compensation);
        interfaceRequired->AddFunction("operating_state", mMTM.operating_state);
        interfaceRequired->AddFunction("state_command", mMTM.state_command);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSM::MTMErrorEventHandler,
                                                this, "error");
    }

    interfaceRequired = AddInterfaceRequired("PSM");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("setpoint_cp", mPSM.setpoint_cp);
        interfaceRequired->AddFunction("servo_cp", mPSM.servo_cp);
        interfaceRequired->AddFunction("hold", mPSM.hold);
        interfaceRequired->AddFunction("jaw/setpoint_js", mPSM.jaw_setpoint_js, MTS_OPTIONAL);
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

    interfaceRequired = AddInterfaceRequired("PSM_base_frame", MTS_OPTIONAL);
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
                                    "state_command", std::string());
        mInterface->AddCommandWrite(&mtsTeleOperationPSM::set_scale, this,
                                    "set_scale", m_scale);
        mInterface->AddCommandWrite(&mtsTeleOperationPSM::lock_rotation, this,
                                    "lock_rotation", m_rotation_locked);
        mInterface->AddCommandWrite(&mtsTeleOperationPSM::lock_translation, this,
                                    "lock_translation", m_translation_locked);
        mInterface->AddCommandWrite(&mtsTeleOperationPSM::set_align_mtm, this,
                                    "set_align_mtm", m_align_mtm);
        mInterface->AddCommandWrite(&mtsTeleOperationPSM::following_mtm_body_servo_cf, this,
                                    "following/mtm/body/servo_cf");
        mInterface->AddCommandReadState(*(mConfigurationStateTable),
                                        m_scale,
                                        "scale");
        mInterface->AddCommandReadState(*(mConfigurationStateTable),
                                        m_rotation_locked, "rotation_locked");
        mInterface->AddCommandReadState(*(mConfigurationStateTable),
                                        m_translation_locked, "translation_locked");
        mInterface->AddCommandReadState(*(mConfigurationStateTable),
                                        m_align_mtm, "align_mtm");
        mInterface->AddCommandReadState(this->StateTable,
                                        mMTM.m_measured_cp,
                                        "MTM/measured_cp");
        mInterface->AddCommandReadState(this->StateTable,
                                        mMTM.m_measured_cv,
                                        "MTM/measured_cv");
        mInterface->AddCommandReadState(this->StateTable,
                                        mPSM.m_setpoint_cp,
                                        "PSM/setpoint_cp");
        mInterface->AddCommandReadState(this->StateTable,
                                        m_alignment_offset,
                                        "alignment_offset");
        // events
        mInterface->AddEventWrite(MessageEvents.desired_state,
                                  "desired_state", std::string(""));
        mInterface->AddEventWrite(MessageEvents.current_state,
                                  "current_state", std::string(""));
        mInterface->AddEventWrite(MessageEvents.following,
                                  "following", false);
        // configuration
        mInterface->AddEventWrite(ConfigurationEvents.scale,
                                  "scale", m_scale);
        mInterface->AddEventWrite(ConfigurationEvents.rotation_locked,
                                  "rotation_locked", m_rotation_locked);
        mInterface->AddEventWrite(ConfigurationEvents.translation_locked,
                                  "translation_locked", m_translation_locked);
        mInterface->AddEventWrite(ConfigurationEvents.align_mtm,
                                  "align_mtm", m_align_mtm);
    }

    // so sent commands can be used with ros-bridge
    mPSM.m_servo_cp.Valid() = true;
    mPSM.m_jaw_servo_jp.Valid() = true;
}

void mtsTeleOperationPSM::Configure(const std::string & filename)
{
    std::ifstream jsonStream;
    Json::Value jsonConfig;
    Json::Reader jsonReader;

    if (filename == "") {
        return;
    }

    jsonStream.open(filename.c_str());
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": failed to parse configuration file \""
                                 << filename << "\"\n"
                                 << jsonReader.getFormattedErrorMessages();
        exit(EXIT_FAILURE);
    }

    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << this->GetName()
                               << " using file \"" << filename << "\"" << std::endl
                               << "----> content of configuration file: " << std::endl
                               << jsonConfig << std::endl
                               << "<----" << std::endl;

    // base component configuration
    mtsComponent::ConfigureJSON(jsonConfig);

    // JSON part
    mtsTeleOperationPSM::Configure(jsonConfig);
}

void mtsTeleOperationPSM::Configure(const Json::Value & jsonConfig)
{
    Json::Value jsonValue;

    // base component configuration
    mtsComponent::ConfigureJSON(jsonConfig);

    // read scale if present
    jsonValue = jsonConfig["scale"];
    if (!jsonValue.empty()) {
        m_scale = jsonValue.asDouble();
    }
    if (m_scale <= 0.0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": \"scale\" must be a positive number.  Found " << m_scale << std::endl;
        exit(EXIT_FAILURE);
    }

    // read orientation if present
    jsonValue = jsonConfig["rotation"];
    if (!jsonValue.empty()) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": \"rotation\" is deprecated"<< std::endl;
        exit(EXIT_FAILURE);
    }

    // rotation locked
    jsonValue = jsonConfig["rotation-locked"];
    if (!jsonValue.empty()) {
        m_rotation_locked = jsonValue.asBool();
    }

    // rotation locked
    jsonValue = jsonConfig["translation-locked"];
    if (!jsonValue.empty()) {
        m_translation_locked = jsonValue.asBool();
    }

    // ignore jaw if needed
    jsonValue = jsonConfig["ignore-jaw"];
    if (!jsonValue.empty()) {
        m_jaw.ignore = jsonValue.asBool();
    }

    // jaw rate of opening-closing
    jsonValue = jsonConfig["jaw-rate"];
    if (!jsonValue.empty()) {
        m_jaw.rate = jsonValue.asDouble();
    }
    if (m_jaw.rate <= 0.0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": \"jaw-rate\" must be a positive number.  Found " << m_jaw.rate << std::endl;
        exit(EXIT_FAILURE);
    }

    // jaw rate of opening-closing after clutch
    jsonValue = jsonConfig["jaw-rate-back-from-clutch"];
    if (!jsonValue.empty()) {
        m_jaw.rate_back_from_clutch = jsonValue.asDouble();
    }
    if (m_jaw.rate_back_from_clutch <= 0.0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": \"jaw-rate-back-from-clutch\" must be a positive number.  Found " << m_jaw.rate << std::endl;
        exit(EXIT_FAILURE);
    }

    // gripper scaling
    Json::Value jsonGripper = jsonConfig["gripper-scaling"];
    if (!jsonGripper.empty()) {
        jsonValue = jsonGripper["max"];
        if (!jsonValue.empty()) {
            m_gripper.max = jsonValue.asDouble();
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": \"gripper-scaling\": { \"max\": } is missing" << std::endl;
            exit(EXIT_FAILURE);
        }
        jsonValue = jsonGripper["zero"];
        if (!jsonValue.empty()) {
            m_gripper.zero = jsonValue.asDouble();
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": \"gripper-scaling\": { \"zero\": } is missing" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // orientation tolerance to start teleop
    jsonValue = jsonConfig["start-orientation-tolerance"];
    if (!jsonValue.empty()) {
        m_operator.orientation_tolerance = jsonValue.asDouble();
    }
    if (m_operator.orientation_tolerance < 0.0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": \"start-orientation-tolerance\" must be a positive number.  Found "
                                 << m_operator.orientation_tolerance << std::endl;
        exit(EXIT_FAILURE);
    }

    // Gripper threshold to start teleop
    jsonValue = jsonConfig["start-gripper-threshold"];
    if (!jsonValue.empty()) {
        m_operator.gripper_threshold = jsonValue.asDouble();
    }
    if (m_operator.gripper_threshold < 0.0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": \"start-gripper-threshold\" must be a positive number.  Found "
                                 << m_operator.gripper_threshold << std::endl;
        exit(EXIT_FAILURE);
    }

    // roll threshold to start teleop
    jsonValue = jsonConfig["start-roll-threshold"];
    if (!jsonValue.empty()) {
        m_operator.roll_threshold = jsonValue.asDouble();
    }
    if (m_operator.roll_threshold < 0.0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": \"start-roll-threshold\" must be a positive number.  Found "
                                 << m_operator.roll_threshold << std::endl;
        exit(EXIT_FAILURE);
    }

    // align MTM if needed
    jsonValue = jsonConfig["align-mtm"];
    if (!jsonValue.empty()) {
        m_align_mtm = jsonValue.asBool();
    }

    // use MTM cv and send to PSM
    mMTM.use_measured_cv = true; // better by default
    jsonValue = jsonConfig["use-mtm-velocity"];
    if (!jsonValue.empty()) {
        mMTM.use_measured_cv = jsonValue.asBool();
    }
}

void mtsTeleOperationPSM::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
    set_scale(m_scale);
    set_following(false);
    lock_rotation(m_rotation_locked);
    lock_translation(m_translation_locked);
    set_align_mtm(m_align_mtm);

    // check if functions for jaw are connected
    if (!m_jaw.ignore) {
        if (!mPSM.jaw_setpoint_js.IsValid()
            || !mPSM.jaw_servo_jp.IsValid()) {
            mInterface->SendError(this->GetName() + ": optional functions \"jaw/servo_jp\" and \"jaw/setpoint_js\" are not connected, setting \"ignore-jaw\" to true");
            m_jaw.ignore = true;
        }
    }

    // check if MTM has measured_cv as needed
    if (mMTM.use_measured_cv &&
        !mMTM.measured_cv.IsValid()) {
        mMTM.use_measured_cv = false;
        mInterface->SendWarning(this->GetName() + ": MTM doesn't provide measured_cv, you can avoid this warning by setting \"use-mtm-velocity\" to false");
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
        m_clutched = true;
        break;
    case prmEventButton::RELEASED:
        m_clutched = false;
        break;
    default:
        break;
    }

    // if the teleoperation is activated
    if (mTeleopState.DesiredState() == "ENABLED") {
        Clutch(m_clutched);
    }
}

void mtsTeleOperationPSM::Clutch(const bool & clutch)
{
    // if the teleoperation is activated
    if (clutch) {
        // keep track of last follow mode
        m_operator.was_active_before_clutch = m_operator.is_active;
        set_following(false);
        mMTM.m_move_cp.Goal().Rotation().FromNormalized(mPSM.m_setpoint_cp.Position().Rotation());
        mMTM.m_move_cp.Goal().Translation().Assign(mMTM.m_measured_cp.Position().Translation());
        mInterface->SendStatus(this->GetName() + ": console clutch pressed");

        // no force applied but gravity and locked orientation
        prmForceCartesianSet wrench;
        mMTM.body_servo_cf(wrench);
        mMTM.use_gravity_compensation(true);
        if ((m_align_mtm || m_rotation_locked)
            && mMTM.lock_orientation.IsValid()) {
            // lock in current position
            mMTM.lock_orientation(mMTM.m_measured_cp.Position().Rotation());
        } else {
            // make sure it is freed
            if (mMTM.unlock_orientation.IsValid()) {
                mMTM.unlock_orientation();
            }
        }

        // make sure PSM stops moving
        mPSM.hold();
    } else {
        mInterface->SendStatus(this->GetName() + ": console clutch released");
        mTeleopState.SetCurrentState("SETTING_ARMS_STATE");
        m_back_from_clutch = true;
        m_jaw_caught_up_after_clutch = false;
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
    m_operator.is_active = false;
    MessageEvents.desired_state(state);
    mInterface->SendStatus(this->GetName() + ": set desired state to " + state);
}

vctMatRot3 mtsTeleOperationPSM::UpdateAlignOffset(void)
{
    vctMatRot3 desiredOrientation = mPSM.m_setpoint_cp.Position().Rotation();
    mMTM.m_measured_cp.Position().Rotation().ApplyInverseTo(desiredOrientation, m_alignment_offset);
    return desiredOrientation;
}

void mtsTeleOperationPSM::UpdateInitialState(void)
{
    mMTM.CartesianInitial.From(mMTM.m_measured_cp.Position());
    mPSM.CartesianInitial.From(mPSM.m_setpoint_cp.Position());
    UpdateAlignOffset();
    m_alignment_offset_initial = m_alignment_offset;
    if (mBaseFrame.measured_cp.IsValid()) {
        mBaseFrame.CartesianInitial.From(mBaseFrame.m_measured_cp.Position());
    }
}

void mtsTeleOperationPSM::set_scale(const double & scale)
{
    // set scale
    mConfigurationStateTable->Start();
    m_scale = scale;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.scale(m_scale);

    // update MTM/PSM previous position to prevent jumps
    UpdateInitialState();
}

void mtsTeleOperationPSM::lock_rotation(const bool & lock)
{
    mConfigurationStateTable->Start();
    m_rotation_locked = lock;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.rotation_locked(m_rotation_locked);
    // when releasing the orientation, MTM orientation is likely off
    // so force re-align
    if (lock == false) {
        set_following(false);
        mTeleopState.SetCurrentState("DISABLED");
    } else {
        // update MTM/PSM previous position
        UpdateInitialState();
        // lock orientation if the arm is running
        if ((mTeleopState.CurrentState() == "ENABLED")
            && mMTM.lock_orientation.IsValid()) {
            mMTM.lock_orientation(mMTM.m_measured_cp.Position().Rotation());
        }
    }
}

void mtsTeleOperationPSM::lock_translation(const bool & lock)
{
    mConfigurationStateTable->Start();
    m_translation_locked = lock;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.translation_locked(m_translation_locked);
    // update MTM/PSM previous position
    UpdateInitialState();
}

void mtsTeleOperationPSM::set_align_mtm(const bool & alignMTM)
{
    mConfigurationStateTable->Start();
    // make sure we have access to lock/unlock
    if ((mMTM.lock_orientation.IsValid()
         && mMTM.unlock_orientation.IsValid())) {
        m_align_mtm = alignMTM;
    } else {
        if (alignMTM) {
            mInterface->SendWarning(this->GetName() + ": unable to force MTM alignment, the device doesn't provide commands to lock/unlock orientation");
        }
        m_align_mtm = false;
    }
    mConfigurationStateTable->Advance();
    ConfigurationEvents.align_mtm(m_align_mtm);
    // force re-align if the teleop is already enabled
    if (mTeleopState.CurrentState() == "ENABLED") {
        mTeleopState.SetCurrentState("DISABLED");
    }
}

void mtsTeleOperationPSM::following_mtm_body_servo_cf(const prmForceCartesianSet & wrench)
{
    m_following_mtm_body_servo_cf = wrench;
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
    executionResult = mMTM.measured_cp(mMTM.m_measured_cp);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTM.measured_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from MTM");
        mTeleopState.SetDesiredState("DISABLED");
    }
    if (mMTM.use_measured_cv) {
        executionResult = mMTM.measured_cv(mMTM.m_measured_cv);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTM.measured_cv failed \""
                                    << executionResult << "\"" << std::endl;
            mInterface->SendError(this->GetName() + ": unable to get cartesian velocity from MTM");
            mTeleopState.SetDesiredState("DISABLED");
        }
    }

    executionResult = mMTM.setpoint_cp(mMTM.m_setpoint_cp);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTM.setpoint_cp failed \""
                                << executionResult << "\"" << std::endl;
    }

    // get PSM Cartesian position
    executionResult = mPSM.setpoint_cp(mPSM.m_setpoint_cp);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to PSM.setpoint_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from PSM");
        mTeleopState.SetDesiredState("DISABLED");
    }

    // get base-frame cartesian position if available
    if (mBaseFrame.measured_cp.IsValid()) {
        executionResult = mBaseFrame.measured_cp(mBaseFrame.m_measured_cp);
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
        set_following(false);
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
    }
    if (!state.IsHomed()) {
        mPSM.state_command(std::string("home"));
    }

    mMTM.operating_state(state);
    if (state.State() != prmOperatingState::ENABLED) {
        mMTM.state_command(std::string("enable"));
    }
    if (!state.IsHomed()) {
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
    ConfigurationEvents.scale(m_scale);

    // reset timer
    mInStateTimer = StateTable.GetTic();
    mTimeSinceLastAlign = 0.0;

    // if we don't align MTM, just stay in same position
    if (!m_align_mtm) {
        // convert to prm type
        mMTM.m_move_cp.Goal().Assign(mMTM.m_setpoint_cp.Position());
        mMTM.move_cp(mMTM.m_move_cp);
    }

    if (m_back_from_clutch) {
        m_operator.is_active = m_operator.was_active_before_clutch;
        m_back_from_clutch = false;
    }

    if (!m_jaw.ignore) {
        // figure out the mapping between the MTM gripper angle and the PSM jaw angle
        UpdateGripperToJawConfiguration();
    }

    // set min/max for roll outside bounds
    m_operator.roll_min = cmnPI * 100.0;
    m_operator.roll_max = -cmnPI * 100.0;
    m_operator.gripper_min = cmnPI * 100.0;
    m_operator.gripper_max = -cmnPI * 100.0;
}

void mtsTeleOperationPSM::RunAligningMTM(void)
{
    // if clutched or align not needed, do nothing
    if (m_clutched || !m_align_mtm) {
        return;
    }

    // set trajectory goal periodically, this will track PSM motion
    const double currentTime = StateTable.GetTic();
    if ((currentTime - mTimeSinceLastAlign) > 10.0 * cmn_ms) {
        mTimeSinceLastAlign = currentTime;
        // Orientate MTM with PSM
        vctFrm4x4 mtmCartesianGoal;
        mtmCartesianGoal.Translation().Assign(mMTM.m_setpoint_cp.Position().Translation());
        mtmCartesianGoal.Rotation().FromNormalized(mPSM.m_setpoint_cp.Position().Rotation());
        // convert to prm type
        mMTM.m_move_cp.Goal().From(mtmCartesianGoal);
        mMTM.move_cp(mMTM.m_move_cp);
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
    vctAxAnRot3 axisAngle(m_alignment_offset, VCT_NORMALIZE);
    double orientationError = 0.0;
    // set error only if we need to align MTM to PSM
    if (m_align_mtm) {
        orientationError = axisAngle.Angle();
    }

    // if not active, use gripper and/or roll to detect if the user is ready
    if (!m_operator.is_active) {
        // update gripper values
        double gripperRange = 0.0;
        if (mMTM.gripper_measured_js.IsValid()) {
            mMTM.gripper_measured_js(mMTM.m_gripper_measured_js);
            const double gripper = mMTM.m_gripper_measured_js.Position()[0];
            if (gripper > m_operator.gripper_max) {
                m_operator.gripper_max = gripper;
            } else if (gripper < m_operator.gripper_min) {
                m_operator.gripper_min = gripper;
            }
            gripperRange = m_operator.gripper_max - m_operator.gripper_min;
        }

        // checking roll
        const double roll = acos(vctDotProduct(desiredOrientation.Column(1),
                                               mMTM.m_measured_cp.Position().Rotation().Column(1)));
        if (roll > m_operator.roll_max) {
            m_operator.roll_max = roll;
        } else if (roll < m_operator.roll_min) {
            m_operator.roll_min = roll;
        }
        const double rollRange = m_operator.roll_max - m_operator.roll_min;

        // different conditions to set operator active
        if (gripperRange >= m_operator.gripper_threshold) {
            m_operator.is_active = true;
        } else if (rollRange >= m_operator.roll_threshold) {
            m_operator.is_active = true;
        } else if ((gripperRange + rollRange)
                   > 0.8 * (m_operator.gripper_threshold
                            + m_operator.roll_threshold)) {
            m_operator.is_active = true;
        }
    }

    // finally check for transition
    if ((orientationError <= m_operator.orientation_tolerance)
        && m_operator.is_active) {
        if (mTeleopState.DesiredState() == "ENABLED") {
            mTeleopState.SetCurrentState("ENABLED");
        }
    } else {
        // check timer and issue a message
        if ((StateTable.GetTic() - mInStateTimer) > 2.0 * cmn_s) {
            std::stringstream message;
            if (orientationError >= m_operator.orientation_tolerance) {
                message << this->GetName() + ": unable to align MTM, angle error is "
                        << orientationError * cmn180_PI << " (deg)";
            } else if (!m_operator.is_active) {
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
    if (!m_jaw.ignore) {
        m_jaw_caught_up_after_clutch = false;
        // gripper ghost
        mPSM.jaw_setpoint_js(mPSM.m_jaw_setpoint_js);
        if (mPSM.m_jaw_setpoint_js.Position().size() != 1) {
            mInterface->SendWarning(this->GetName() + ": unable to get jaw position.  Make sure there is an instrument on the PSM");
            mTeleopState.SetDesiredState("DISABLE");
        }
        double currentJaw = mPSM.m_jaw_setpoint_js.Position()[0];
        m_gripper_ghost = JawToGripper(currentJaw);
    }

    // set MTM/PSM to Teleop (Cartesian Position Mode)
    mMTM.use_gravity_compensation(true);
    // set forces to zero and lock/unlock orientation as needed
    prmForceCartesianSet wrench;
    mMTM.body_servo_cf(wrench);
    // reset user wrench
    m_following_mtm_body_servo_cf = wrench;

    // orientation locked or not
    if (m_rotation_locked
        && mMTM.lock_orientation.IsValid()) {
        mMTM.lock_orientation(mMTM.m_measured_cp.Position().Rotation());
    } else {
        if (mMTM.unlock_orientation.IsValid()) {
            mMTM.unlock_orientation();
        }
    }
    // check if by any chance the clutch pedal is pressed
    if (m_clutched) {
        Clutch(true);
    } else {
        set_following(true);
    }
}

void mtsTeleOperationPSM::RunEnabled(void)
{
    if (mMTM.m_measured_cp.Valid()
        && mPSM.m_setpoint_cp.Valid()) {
        // follow mode
        if (!m_clutched) {

            // on MTM, just apply user provided effort
            if (m_following_mtm_body_servo_cf.Valid()) {
                mMTM.body_servo_cf(m_following_mtm_body_servo_cf);
            }

            // compute mtm Cartesian motion
            vctFrm4x4 mtmPosition(mMTM.m_measured_cp.Position());

            // translation
            vct3 mtmTranslation;
            vct3 psmTranslation;
            if (m_translation_locked) {
                psmTranslation = mPSM.CartesianInitial.Translation();
            } else {
                mtmTranslation = (mtmPosition.Translation() - mMTM.CartesianInitial.Translation());
                psmTranslation = mtmTranslation * m_scale;
                psmTranslation = psmTranslation + mPSM.CartesianInitial.Translation();
            }
            // rotation
            vctMatRot3 psmRotation;
            if (m_rotation_locked) {
                psmRotation.From(mPSM.CartesianInitial.Rotation());
            } else {
                psmRotation = mtmPosition.Rotation() * m_alignment_offset_initial;
            }

            // compute desired psm position
            vctFrm4x4 psmCartesianGoal;
            psmCartesianGoal.Translation().Assign(psmTranslation);
            psmCartesianGoal.Rotation().FromNormalized(psmRotation);

            // take into account changes in PSM base frame if any
            if (mBaseFrame.measured_cp.IsValid()) {
                vctFrm4x4 baseFrame(mBaseFrame.m_measured_cp.Position());
                vctFrm4x4 baseFrameChange = baseFrame.Inverse() * mBaseFrame.CartesianInitial;
                // update PSM position goal
                psmCartesianGoal = baseFrameChange * psmCartesianGoal;
                // update alignment offset
                mtmPosition.Rotation().ApplyInverseTo(psmCartesianGoal.Rotation(), m_alignment_offset);
            }

            // PSM go this cartesian position -> m_servo_cp
            mPSM.m_servo_cp.Goal().FromNormalized(psmCartesianGoal);

            // Add desired velocity if needed
            if (mMTM.use_measured_cv) {
                // linear is scaled and re-oriented
                mPSM.m_servo_cp.Velocity() = m_scale * mMTM.m_measured_cv.VelocityLinear();
                // angular is not scaled
                mPSM.m_servo_cp.VelocityAngular() = mMTM.m_measured_cv.VelocityAngular();
            } else {
                mPSM.m_servo_cp.Velocity().Assign(vct3(0));
                mPSM.m_servo_cp.VelocityAngular().Assign(vct3(0));
            }

            mPSM.servo_cp(mPSM.m_servo_cp);

            if (!m_jaw.ignore) {
                // gripper
                if (mMTM.gripper_measured_js.IsValid()) {
                    mMTM.gripper_measured_js(mMTM.m_gripper_measured_js);
                    const double currentGripper = mMTM.m_gripper_measured_js.Position()[0];
                    // see if we caught up
                    if (!m_jaw_caught_up_after_clutch) {
                        const double error = std::abs(currentGripper - m_gripper_ghost);
                        if (error < mtsIntuitiveResearchKit::TeleOperationPSM::ToleranceBackFromClutch) {
                            m_jaw_caught_up_after_clutch = true;
                        }
                    }
                    // pick the rate based on back from clutch or not
                    const double delta = m_jaw_caught_up_after_clutch ?
                        m_jaw.rate * StateTable.PeriodStats.PeriodAvg()
                        : m_jaw.rate_back_from_clutch * StateTable.PeriodStats.PeriodAvg();
                    // gripper ghost below, add to catch up
                    if (m_gripper_ghost <= (currentGripper - delta)) {
                        m_gripper_ghost += delta;
                    } else {
                        // gripper ghost above, subtract to catch up
                        if (m_gripper_ghost >= (currentGripper + delta)) {
                            m_gripper_ghost -= delta;
                        }
                    }
                    mPSM.m_jaw_servo_jp.Goal()[0] = GripperToJaw(m_gripper_ghost);
                    // make sure we don't send goal past joint limits
                    if (mPSM.m_jaw_servo_jp.Goal()[0] < m_gripper_to_jaw.position_min) {
                        mPSM.m_jaw_servo_jp.Goal()[0] = m_gripper_to_jaw.position_min;
                        m_gripper_ghost = JawToGripper(m_gripper_to_jaw.position_min);
                    }
                    mPSM.jaw_servo_jp(mPSM.m_jaw_servo_jp);
                } else {
                    mPSM.m_jaw_servo_jp.Goal()[0] = 45.0 * cmnPI_180;
                    mPSM.jaw_servo_jp(mPSM.m_jaw_servo_jp);
                }
            }
        }
    }
}

void mtsTeleOperationPSM::TransitionEnabled(void)
{
    if (mTeleopState.DesiredStateIsNotCurrent()) {
        set_following(false);
        mTeleopState.SetCurrentState(mTeleopState.DesiredState());
    }
}

double mtsTeleOperationPSM::GripperToJaw(const double & gripperAngle) const
{
    return m_gripper_to_jaw.scale * gripperAngle + m_gripper_to_jaw.offset;
}

double mtsTeleOperationPSM::JawToGripper(const double & jawAngle) const
{
    return (jawAngle - m_gripper_to_jaw.offset) / m_gripper_to_jaw.scale;
}

void mtsTeleOperationPSM::UpdateGripperToJawConfiguration(void)
{
    // default values, assumes jaws match gripper
    double _jaw_min = 0.0;
    double _jaw_max = m_gripper.max;

    m_gripper_to_jaw.position_min = 0.0;
    // get the PSM jaw configuration if possible to find range
    if (mPSM.jaw_configuration_js.IsValid()) {
        mPSM.jaw_configuration_js(mPSM.m_jaw_configuration_js);
        if ((mPSM.m_jaw_configuration_js.PositionMin().size() == 1)
            && (mPSM.m_jaw_configuration_js.PositionMax().size() == 1)) {
            _jaw_min = mPSM.m_jaw_configuration_js.PositionMin()[0];
            _jaw_max = mPSM.m_jaw_configuration_js.PositionMax()[0];
            // save min for later so we never ask PSM to close jaws more than min
            m_gripper_to_jaw.position_min = _jaw_min;
        }
    }
    // if the PSM can close its jaws past 0 (tighter), we map from 0 to qmax
    // negative values just mean tighter jaws
    m_gripper_to_jaw.scale = (_jaw_max) / (m_gripper.max - m_gripper.zero);
    m_gripper_to_jaw.offset = -m_gripper.zero / m_gripper_to_jaw.scale;
}

void mtsTeleOperationPSM::set_following(const bool following)
{
    MessageEvents.following(following);
    m_following = following;
    // reset user servo_cf at each transition
    m_following_mtm_body_servo_cf.SetValid(false);
}
