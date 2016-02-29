/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-01-21

  (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// cisst
#include <sawIntuitiveResearchKit/mtsTeleOperationECM.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperationECM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsTeleOperationECM::mtsTeleOperationECM(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    mTeleopState(mtsTeleOperationECMTypes::DISABLED)
{
    Init();
}

mtsTeleOperationECM::mtsTeleOperationECM(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    mTeleopState(mtsTeleOperationECMTypes::DISABLED)
{
    Init();
}

void mtsTeleOperationECM::Init(void)
{
    // configure state machine
    mTeleopState.AddAllowedDesiredStates(mtsTeleOperationECMTypes::DISABLED);
    mTeleopState.AddAllowedDesiredStates(mtsTeleOperationECMTypes::ENABLED);

    // state change, to convert to string events for users (Qt, ROS)
    mTeleopState.SetStateChangedCallback(&mtsTeleOperationECM::StateChanged,
                                         this);

    // run for all states
    mTeleopState.SetRunCallback(&mtsTeleOperationECM::RunAll,
                                this);
    // disabled
    mTeleopState.SetTransitionCallback(mtsTeleOperationECMTypes::DISABLED,
                                       &mtsTeleOperationECM::TransitionDisabled,
                                       this);

    // setting ECM state
    mTeleopState.SetEnterCallback(mtsTeleOperationECMTypes::SETTING_ECM_STATE,
                                  &mtsTeleOperationECM::EnterSettingECMState,
                                  this);
    mTeleopState.SetTransitionCallback(mtsTeleOperationECMTypes::SETTING_ECM_STATE,
                                       &mtsTeleOperationECM::TransitionSettingECMState,
                                       this);

    // setting MTMs state
    mTeleopState.SetEnterCallback(mtsTeleOperationECMTypes::SETTING_MTMS_STATE,
                                  &mtsTeleOperationECM::EnterSettingMTMsState,
                                  this);
    mTeleopState.SetTransitionCallback(mtsTeleOperationECMTypes::SETTING_MTMS_STATE,
                                       &mtsTeleOperationECM::TransitionSettingMTMsState,
                                       this);

    // enabled
    mTeleopState.SetEnterCallback(mtsTeleOperationECMTypes::ENABLED,
                                  &mtsTeleOperationECM::EnterEnabled,
                                  this);
    mTeleopState.SetRunCallback(mtsTeleOperationECMTypes::ENABLED,
                                &mtsTeleOperationECM::RunEnabled,
                                this);
    mTeleopState.SetTransitionCallback(mtsTeleOperationECMTypes::ENABLED,
                                       &mtsTeleOperationECM::TransitionEnabled,
                                       this);

    mScale = 0.2;

    StateTable.AddData(mMasterLeft.PositionCartesianCurrent, "MasterLeftCartesianPosition");
    StateTable.AddData(mMasterRight.PositionCartesianCurrent, "MasterRightCartesianPosition");
    StateTable.AddData(mSlave.PositionCartesianCurrent, "SlaveCartesianPosition");

    mConfigurationStateTable = new mtsStateTable(100, "Configuration");
    mConfigurationStateTable->SetAutomaticAdvance(false);
    AddStateTable(mConfigurationStateTable);
    mConfigurationStateTable->AddData(mScale, "Scale");
    mConfigurationStateTable->AddData(mRegistrationRotation, "RegistrationRotation");

    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("MasterLeft");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetPositionCartesian", mMasterLeft.GetPositionCartesian);
        interfaceRequired->AddFunction("SetPositionCartesian", mMasterLeft.SetPositionCartesian);
        interfaceRequired->AddFunction("GetRobotControlState", mMasterLeft.GetRobotControlState);
        interfaceRequired->AddFunction("SetRobotControlState", mMasterLeft.SetRobotControlState);
        interfaceRequired->AddFunction("LockOrientation", mMasterLeft.LockOrientation);
        interfaceRequired->AddFunction("UnlockOrientation", mMasterLeft.UnlockOrientation);
        interfaceRequired->AddFunction("SetWrenchBody", mMasterLeft.SetWrenchBody);
        interfaceRequired->AddFunction("SetWrenchBodyOrientationAbsolute", mMasterLeft.SetWrenchBodyOrientationAbsolute);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::MasterLeftErrorEventHandler,
                                                this, "Error");
    }

    interfaceRequired = AddInterfaceRequired("MasterRight");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetPositionCartesian", mMasterRight.GetPositionCartesian);
        interfaceRequired->AddFunction("SetPositionCartesian", mMasterRight.SetPositionCartesian);
        interfaceRequired->AddFunction("GetRobotControlState", mMasterRight.GetRobotControlState);
        interfaceRequired->AddFunction("SetRobotControlState", mMasterRight.SetRobotControlState);
        interfaceRequired->AddFunction("LockOrientation", mMasterRight.LockOrientation);
        interfaceRequired->AddFunction("UnlockOrientation", mMasterRight.UnlockOrientation);
        interfaceRequired->AddFunction("SetWrenchBody", mMasterRight.SetWrenchBody);
        interfaceRequired->AddFunction("SetWrenchBodyOrientationAbsolute", mMasterRight.SetWrenchBodyOrientationAbsolute);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::MasterRightErrorEventHandler,
                                                this, "Error");
    }

    interfaceRequired = AddInterfaceRequired("Slave");
    if (interfaceRequired) {
        // ECM, use PID desired position to make sure there is no jump when engaging
        interfaceRequired->AddFunction("GetPositionCartesianDesired", mSlave.GetPositionCartesian);
        interfaceRequired->AddFunction("SetPositionCartesian", mSlave.SetPositionCartesian);
        interfaceRequired->AddFunction("GetRobotControlState", mSlave.GetRobotControlState);
        interfaceRequired->AddFunction("SetRobotControlState", mSlave.SetRobotControlState);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::SlaveErrorEventHandler,
                                                this, "Error");
    }

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Setting");
    if (interfaceProvided) {
        // commands
        interfaceProvided->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                              "GetPeriodStatistics"); // mtsIntervalStatistics

        interfaceProvided->AddCommandWrite(&mtsTeleOperationECM::SetDesiredState, this,
                                           "SetDesiredState", std::string("DISABLED"));
        interfaceProvided->AddCommandWrite(&mtsTeleOperationECM::SetScale, this,
                                           "SetScale", 0.5);
        interfaceProvided->AddCommandWrite(&mtsTeleOperationECM::SetRegistrationRotation, this,
                                           "SetRegistrationRotation", vctMatRot3());
        interfaceProvided->AddCommandReadState(*mConfigurationStateTable,
                                               mScale,
                                               "GetScale");
        interfaceProvided->AddCommandReadState(*mConfigurationStateTable,
                                               mRegistrationRotation,
                                               "GetRegistrationRotation");
        interfaceProvided->AddCommandReadState(StateTable,
                                               mMasterLeft.PositionCartesianCurrent,
                                               "GetPositionCartesianMasterLeft");
        interfaceProvided->AddCommandReadState(StateTable,
                                               mMasterRight.PositionCartesianCurrent,
                                               "GetPositionCartesianMasterRight");
        interfaceProvided->AddCommandReadState(StateTable,
                                               mSlave.PositionCartesianCurrent,
                                               "GetPositionCartesianSlave");
        // events
        interfaceProvided->AddEventWrite(MessageEvents.Status,
                                         "Status", std::string(""));
        interfaceProvided->AddEventWrite(MessageEvents.Warning,
                                         "Warning", std::string(""));
        interfaceProvided->AddEventWrite(MessageEvents.Error,
                                         "Error", std::string(""));
        interfaceProvided->AddEventWrite(MessageEvents.DesiredState,
                                         "DesiredState", std::string(""));
        interfaceProvided->AddEventWrite(MessageEvents.CurrentState,
                                         "CurrentState", std::string(""));
        // configuration
        interfaceProvided->AddEventWrite(ConfigurationEvents.Scale,
                                         "Scale", 0.5);
    }
}

void mtsTeleOperationECM::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsTeleOperationECM::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
}

void mtsTeleOperationECM::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    // run based on state
    mTeleopState.Run();
}

void mtsTeleOperationECM::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsTeleOperationECM::StateChanged(void)
{
    MessageEvents.Status(this->GetName() + ", current state "
                         + mtsTeleOperationECMTypes::StateTypeToString(mTeleopState.CurrentState()));
}

void mtsTeleOperationECM::RunAll(void)
{
    mtsExecutionResult executionResult;

    // get master left Cartesian position
    executionResult = mMasterLeft.GetPositionCartesian(mMasterLeft.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MasterLeft.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
        MessageEvents.Error(this->GetName() + ": unable to get cartesian position from master left");
        mTeleopState.SetDesiredState(mtsTeleOperationECMTypes::DISABLED);
    }
    // shift position to take into account distance between MTMs
    mMasterLeft.PositionCartesianCurrent.Position().Translation()[0] -= 0.2 * cmn_m;

    // get master right Cartesian position
    executionResult = mMasterRight.GetPositionCartesian(mMasterRight.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MasterRight.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
        MessageEvents.Error(this->GetName() + ": unable to get cartesian position from master right");
        mTeleopState.SetDesiredState(mtsTeleOperationECMTypes::DISABLED);
    }
    // shift position to take into account distance between MTMs
    mMasterRight.PositionCartesianCurrent.Position().Translation()[0] += 0.2 * cmn_m;

    // get slave Cartesian position
    executionResult = mSlave.GetPositionCartesian(mSlave.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to Slave.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
        MessageEvents.Error(this->GetName() + ": unable to get cartesian position from slave");
        mTeleopState.SetDesiredState(mtsTeleOperationECMTypes::DISABLED);
    }

    // check if anyone wanted to disable anyway
    if ((mTeleopState.DesiredState() == mtsTeleOperationECMTypes::DISABLED)
        && (mTeleopState.CurrentState() != mtsTeleOperationECMTypes::DISABLED)) {
        mTeleopState.SetCurrentState(mtsTeleOperationECMTypes::DISABLED);
        return;
    }
}

void mtsTeleOperationECM::TransitionDisabled(void)
{
    if (mTeleopState.DesiredState() == mtsTeleOperationECMTypes::ENABLED) {
        mTeleopState.SetCurrentState(mtsTeleOperationECMTypes::SETTING_ECM_STATE);
    }
}

void mtsTeleOperationECM::EnterSettingECMState(void)
{
    // reset timer
    mInStateTimer = StateTable.GetTic();

    // request state if needed
    mtsStdString armState;
    mSlave.GetRobotControlState(armState);
    if (armState.Data != "DVRK_POSITION_CARTESIAN") {
        mSlave.SetRobotControlState(mtsStdString("DVRK_POSITION_CARTESIAN"));
    }
}

void mtsTeleOperationECM::TransitionSettingECMState(void)
{
    // check if anyone wanted to disable anyway
    if (mTeleopState.DesiredState() == mtsTeleOperationECMTypes::DISABLED) {
        mTeleopState.SetCurrentState(mtsTeleOperationECMTypes::DISABLED);
        return;
    }
    // check state
    mtsStdString armState;
    mSlave.GetRobotControlState(armState);
    if (armState.Data == "DVRK_POSITION_CARTESIAN") {
        mTeleopState.SetCurrentState(mtsTeleOperationECMTypes::SETTING_MTMS_STATE);
        return;
    }
    // check timer
    if ((StateTable.GetTic() - mInStateTimer) > 60.0 * cmn_s) {
        MessageEvents.Error(this->GetName() + ": timed out while setting up ECM state");
        mTeleopState.SetDesiredState(mtsTeleOperationECMTypes::DISABLED);
    }
}

void mtsTeleOperationECM::EnterSettingMTMsState(void)
{
    // reset timer
    mInStateTimer = StateTable.GetTic();

    // request state if needed
    mtsStdString armState;
    mMasterLeft.GetRobotControlState(armState);
    if (armState.Data != "DVRK_EFFORT_CARTESIAN") {
        mMasterLeft.SetRobotControlState(mtsStdString("DVRK_EFFORT_CARTESIAN"));
    }
    mMasterRight.GetRobotControlState(armState);
    if (armState.Data != "DVRK_EFFORT_CARTESIAN") {
        mMasterRight.SetRobotControlState(mtsStdString("DVRK_EFFORT_CARTESIAN"));
    }
}

void mtsTeleOperationECM::TransitionSettingMTMsState(void)
{
    // check if anyone wanted to disable anyway
    if (mTeleopState.DesiredState() == mtsTeleOperationECMTypes::DISABLED) {
        mTeleopState.SetCurrentState(mtsTeleOperationECMTypes::DISABLED);
        return;
    }
    // check state
    mtsStdString leftArmState, rightArmState;
    mMasterLeft.GetRobotControlState(leftArmState);
    mMasterRight.GetRobotControlState(rightArmState);
    if ((leftArmState.Data == "DVRK_EFFORT_CARTESIAN") &&
        (rightArmState.Data == "DVRK_EFFORT_CARTESIAN")) {
        mTeleopState.SetCurrentState(mtsTeleOperationECMTypes::ENABLED);
        return;
    }
    // check timer
    if ((StateTable.GetTic() - mInStateTimer) > 60.0 * cmn_s) {
        MessageEvents.Error(this->GetName() + ": timed out while setting up MTMs state");
        mTeleopState.SetDesiredState(mtsTeleOperationECMTypes::DISABLED);
    }
}

void mtsTeleOperationECM::EnterEnabled(void)
{
    // set cartesian effort parameters
    mMasterLeft.SetWrenchBodyOrientationAbsolute(true);
    mMasterLeft.LockOrientation(mMasterLeft.PositionCartesianCurrent.Position().Rotation());
    mMasterRight.SetWrenchBodyOrientationAbsolute(true);
    mMasterRight.LockOrientation(mMasterRight.PositionCartesianCurrent.Position().Rotation());

    // store inital state
    vct3 masterOffset;
    masterOffset.DifferenceOf(mMasterRight.PositionCartesianCurrent.Position().Translation(),
                              mMasterLeft.PositionCartesianCurrent.Position().Translation());
    mMasterDistance = masterOffset.Norm();
}

void mtsTeleOperationECM::RunEnabled(void)
{
    // compute force to maintain constant distance between masters
    vct3 masterOffset;
    masterOffset.DifferenceOf(mMasterRight.PositionCartesianCurrent.Position().Translation(),
                              mMasterLeft.PositionCartesianCurrent.Position().Translation());
    // get distance and normalize the vector
    double newDistance = masterOffset.Norm();
    masterOffset.Divide(newDistance);

    // compute force to apply on each master to maintain distance
    double error = mMasterDistance - newDistance;
    vct3 force(masterOffset);
    force.Multiply(error * 100.0); // this gain should come from JSON file

    // apply forces
    prmForceCartesianSet wrench;
    wrench.Force().Ref<3>(0).Assign(force);
    mMasterRight.SetWrenchBody(wrench);
    // flip force
    wrench.Force().Multiply(-1.0);
    mMasterLeft.SetWrenchBody(wrench);
}

void mtsTeleOperationECM::TransitionEnabled(void)
{
    if (mTeleopState.DesiredState() == mtsTeleOperationECMTypes::DISABLED) {
        mTeleopState.SetCurrentState(mtsTeleOperationECMTypes::DISABLED);
    }
}

void mtsTeleOperationECM::MasterLeftErrorEventHandler(const std::string & message)
{
    mTeleopState.SetDesiredState(mtsTeleOperationECMTypes::DISABLED);
    MessageEvents.Error(this->GetName() + ": received from left master [" + message + "]");
}

void mtsTeleOperationECM::MasterRightErrorEventHandler(const std::string & message)
{
    mTeleopState.SetDesiredState(mtsTeleOperationECMTypes::DISABLED);
    MessageEvents.Error(this->GetName() + ": received from right master [" + message + "]");
}

void mtsTeleOperationECM::SlaveErrorEventHandler(const std::string & message)
{
    mTeleopState.SetDesiredState(mtsTeleOperationECMTypes::DISABLED);
    MessageEvents.Error(this->GetName() + ": received from slave [" + message + "]");
}

void mtsTeleOperationECM::SetDesiredState(const std::string & state)
{
    mtsTeleOperationECMTypes::StateType stateEnum;
    try {
        stateEnum = mtsTeleOperationECMTypes::StateTypeFromString(state);
    } catch (std::exception e) {
        MessageEvents.Error(this->GetName() + ": unsupported state " + state + ": " + e.what());
        return;
    }
    if (!mTeleopState.SetDesiredState(stateEnum)) {
        MessageEvents.Error(this->GetName() + ": " + state + " is not an allowed desired state");
        return;
    }
    MessageEvents.DesiredState(state);
    MessageEvents.Status(this->GetName() + ": set desired state to " + state);
}

void mtsTeleOperationECM::SetScale(const double & scale)
{
    mConfigurationStateTable->Start();
    mScale = scale;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.Scale(mScale);
}

void mtsTeleOperationECM::SetRegistrationRotation(const vctMatRot3 & rotation)
{
    mConfigurationStateTable->Start();
    mRegistrationRotation = rotation;
    mConfigurationStateTable->Advance();
}
