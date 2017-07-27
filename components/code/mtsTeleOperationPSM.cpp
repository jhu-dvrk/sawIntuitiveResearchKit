/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// cisst
#include <sawIntuitiveResearchKit/mtsTeleOperationPSM.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperationPSM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsTeleOperationPSM::mtsTeleOperationPSM(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    mMTM(0),
    mPSM(0),
    mTeleopState(componentName, "DISABLED")
{
    Init();
}

mtsTeleOperationPSM::mtsTeleOperationPSM(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    mMTM(0),
    mPSM(0),
    mTeleopState(arg.Name, "DISABLED")
{
    Init();
}

mtsTeleOperationPSM::~mtsTeleOperationPSM()
{
    if (mMTM) {
        delete mMTM;
    }
    if (mPSM) {
        delete mPSM;
    }
}

void mtsTeleOperationPSM::Init(void)
{
    if (!mMTM) {
        mMTM = new RobotMTM;
    }
    if (!mPSM) {
        mPSM = new RobotPSM;
    }
}

void mtsTeleOperationPSM::Configure(const std::string & CMN_UNUSED(filename))
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

    mScale = 0.2;
    mIsClutched = false;

    mRotationLocked = false;
    mTranslationLocked = false;

    this->StateTable.AddData(mMTM->PositionCartesianCurrent, "MTMCartesianPositionCurrent");
    this->StateTable.AddData(mMTM->PositionCartesianDesired, "MTMCartesianPositionDesired");
    this->StateTable.AddData(mPSM->PositionCartesianCurrent, "PSMCartesianPosition");

    mConfigurationStateTable = new mtsStateTable(100, "Configuration");
    mConfigurationStateTable->SetAutomaticAdvance(false);
    this->AddStateTable(mConfigurationStateTable);
    mConfigurationStateTable->AddData(mScale, "Scale");
    mConfigurationStateTable->AddData(mRegistrationRotation, "RegistrationRotation");
    mConfigurationStateTable->AddData(mRotationLocked, "RotationLocked");
    mConfigurationStateTable->AddData(mTranslationLocked, "TranslationLocked");

    // setup cisst interfaces
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("MTM");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetPositionCartesian", mMTM->GetPositionCartesian);
        interfaceRequired->AddFunction("GetPositionCartesianDesired", mMTM->GetPositionCartesianDesired);
        interfaceRequired->AddFunction("SetPositionGoalCartesian", mMTM->SetPositionGoalCartesian);
        interfaceRequired->AddFunction("GetStateGripper", mMTM->GetStateGripper);
        interfaceRequired->AddFunction("LockOrientation", mMTM->LockOrientation);
        interfaceRequired->AddFunction("UnlockOrientation", mMTM->UnlockOrientation);
        interfaceRequired->AddFunction("SetWrenchBody", mMTM->SetWrenchBody);
        interfaceRequired->AddFunction("SetGravityCompensation", mMTM->SetGravityCompensation);
        interfaceRequired->AddFunction("GetCurrentState", mMTM->GetCurrentState);
        interfaceRequired->AddFunction("GetDesiredState", mMTM->GetDesiredState);
        interfaceRequired->AddFunction("SetDesiredState", mMTM->SetDesiredState);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSM::MTMErrorEventHandler,
                                                this, "Error");
    }

    interfaceRequired = AddInterfaceRequired("PSM");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetPositionCartesian", mPSM->GetPositionCartesian);
        interfaceRequired->AddFunction("SetPositionCartesian", mPSM->SetPositionCartesian);
        interfaceRequired->AddFunction("SetJawPosition", mPSM->SetJawPosition);
        interfaceRequired->AddFunction("GetCurrentState", mPSM->GetCurrentState);
        interfaceRequired->AddFunction("GetDesiredState", mPSM->GetDesiredState);
        interfaceRequired->AddFunction("SetDesiredState", mPSM->SetDesiredState);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSM::PSMErrorEventHandler,
                                                this, "Error");
    }

    // footpedal events
    interfaceRequired = AddInterfaceRequired("Clutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSM::ClutchEventHandler, this, "Button");
    }

    mInterface = AddInterfaceProvided("Setting");
    if (mInterface) {
        mInterface->AddMessageEvents();
        // commands
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                        "GetPeriodStatistics"); // mtsIntervalStatistics

        mInterface->AddCommandWrite(&mtsTeleOperationPSM::SetDesiredState, this,
                                    "SetDesiredState", std::string("DISABLED"));
        mInterface->AddCommandWrite(&mtsTeleOperationPSM::SetScale, this,
                                    "SetScale", 0.5);
        mInterface->AddCommandWrite(&mtsTeleOperationPSM::SetRegistrationRotation, this,
                                    "SetRegistrationRotation", vctMatRot3());
        mInterface->AddCommandWrite(&mtsTeleOperationPSM::LockRotation, this,
                                    "LockRotation", false);
        mInterface->AddCommandWrite(&mtsTeleOperationPSM::LockTranslation, this,
                                    "LockTranslation", false);
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
        mInterface->AddCommandReadState(this->StateTable,
                                        mMTM->PositionCartesianCurrent,
                                        "GetPositionCartesianMTM");
        mInterface->AddCommandReadState(this->StateTable,
                                        mPSM->PositionCartesianCurrent,
                                        "GetPositionCartesianPSM");
        // events
        mInterface->AddEventWrite(MessageEvents.DesiredState,
                                  "DesiredState", std::string(""));
        mInterface->AddEventWrite(MessageEvents.CurrentState,
                                  "CurrentState", std::string(""));
        // configuration
        mInterface->AddEventWrite(ConfigurationEvents.Scale,
                                  "Scale", 0.5);
        mInterface->AddEventWrite(ConfigurationEvents.RotationLocked,
                                  "RotationLocked", false);
        mInterface->AddEventWrite(ConfigurationEvents.TranslationLocked,
                                  "TranslationLocked", false);
    }
}

void mtsTeleOperationPSM::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
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
    mInterface->SendError(this->GetName() + ": received from master [" + message.Message + "]");
}

void mtsTeleOperationPSM::PSMErrorEventHandler(const mtsMessage & message)
{
    mTeleopState.SetDesiredState("DISABLED");
    mInterface->SendError(this->GetName() + ": received from slave [" + message.Message + "]");
}

void mtsTeleOperationPSM::ClutchEventHandler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        mIsClutched = true;
    } else {
        mIsClutched = false;
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
        mMTM->PositionCartesianSet.Goal().Rotation().FromNormalized(mPSM->PositionCartesianCurrent.Position().Rotation());
        mMTM->PositionCartesianSet.Goal().Translation().Assign(mMTM->PositionCartesianCurrent.Position().Translation());
        mInterface->SendStatus(this->GetName() + ": console clutch pressed");

        // no force applied but gravity and locked orientation
        prmForceCartesianSet wrench;
        mMTM->SetWrenchBody(wrench);
        mMTM->SetGravityCompensation(true);
        mMTM->LockOrientation(mMTM->PositionCartesianCurrent.Position().Rotation());
    } else {
        mInterface->SendStatus(this->GetName() + ": console clutch released");
        mTeleopState.SetCurrentState("SETTING_ARMS_STATE");
    }
}

void mtsTeleOperationPSM::SetDesiredState(const std::string & state)
{
    // try to find the state in state machine
    if (!mTeleopState.StateExists(state)) {
        mInterface->SendError(this->GetName() + ": unsupported state " + state);
        return;
    }
    // if state is same as current, return
    if (mTeleopState.DesiredState() == state) {
        return;
    }
    // try to set the desired state
    try {
        mTeleopState.SetDesiredState(state);
    } catch (...) {
        mInterface->SendError(this->GetName() + ": " + state + " is not an allowed desired state");
        return;
    }
    MessageEvents.DesiredState(state);
    mInterface->SendStatus(this->GetName() + ": set desired state to " + state);
}

void mtsTeleOperationPSM::SetScale(const double & scale)
{
    mConfigurationStateTable->Start();
    mScale = scale;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.Scale(mScale);
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
    // when releasing the orientation, master orientation is likely off
    // so force re-align
    if (lock == false) {
        mTeleopState.SetCurrentState("DISABLED");
    } else {
        // update MTM/PSM previous position
        mMTM->CartesianPrevious.From(mMTM->PositionCartesianDesired.Position());
        mPSM->CartesianPrevious.From(mPSM->PositionCartesianCurrent.Position());
        // lock orientation is the arm is running
        if (mTeleopState.CurrentState() == "ENABLED") {
            mMTM->LockOrientation(mMTM->PositionCartesianCurrent.Position().Rotation());
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
    mMTM->CartesianPrevious.From(mMTM->PositionCartesianDesired.Position());
    mPSM->CartesianPrevious.From(mPSM->PositionCartesianCurrent.Position());
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

    // get master Cartesian position
    executionResult = mMTM->GetPositionCartesian(mMTM->PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTM.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from master");
        this->SetDesiredState("DISABLED");
    }
    executionResult = mMTM->GetPositionCartesianDesired(mMTM->PositionCartesianDesired);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTM.GetPositionCartesianDesired failed \""
                                << executionResult << "\"" << std::endl;
    }

    // get slave Cartesian position
    executionResult = mPSM->GetPositionCartesian(mPSM->PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to PSM.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from slave");
        this->SetDesiredState("DISABLED");
    }

    // check if anyone wanted to disable anyway
    if ((mTeleopState.DesiredState() == "DISABLED")
        && (mTeleopState.CurrentState() != "DISABLED")) {
        mTeleopState.SetCurrentState("DISABLED");
        return;
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
    mPSM->GetDesiredState(armState);
    if (armState != "READY") {
        mPSM->SetDesiredState(std::string("READY"));
    }
    mMTM->GetDesiredState(armState);
    if (armState != "READY") {
        mMTM->SetDesiredState(std::string("READY"));
    }
}

void mtsTeleOperationPSM::TransitionSettingArmsState(void)
{
    // check state
    std::string psmState, mtmState;
    mPSM->GetCurrentState(psmState);
    mMTM->GetCurrentState(mtmState);
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
    // reset timer
    mInStateTimer = StateTable.GetTic();

    // Orientate MTM with PSM
    vctFrm4x4 masterCartesianGoal;
    masterCartesianGoal.Translation().Assign(mMTM->PositionCartesianDesired.Position().Translation());
    vctMatRot3 masterRotation;
    masterRotation = mRegistrationRotation.Inverse() * mPSM->PositionCartesianCurrent.Position().Rotation();
    masterCartesianGoal.Rotation().FromNormalized(masterRotation);
    // convert to prm type
    mMTM->PositionCartesianSet.Goal().From(masterCartesianGoal);
    mMTM->SetPositionGoalCartesian(mMTM->PositionCartesianSet);
}

void mtsTeleOperationPSM::TransitionAligningMTM(void)
{
    // check psm state
    std::string armState;
    mPSM->GetCurrentState(armState);
    if (armState != "READY") {
        mInterface->SendWarning(this->GetName() + ": PSM state has changed to [" + armState + "]");
        mTeleopState.SetDesiredState("DISABLED");
        mTeleopState.SetCurrentState("DISABLED");
        return;
    }

    // if the desired state is aligning MTM, just stay here
    if (!mTeleopState.DesiredStateIsNotCurrent()) {
        return;
    }

    // check difference of orientation between master and slave to enable
    vctMatRot3 desiredOrientation, difference;
    mRegistrationRotation.ApplyInverseTo(mPSM->PositionCartesianCurrent.Position().Rotation(),
                                         desiredOrientation);
    mMTM->PositionCartesianCurrent.Position().Rotation().ApplyInverseTo(desiredOrientation, difference);
    vctAxAnRot3 axisAngle(difference, VCT_NORMALIZE);
    const double angleInDegrees = axisAngle.Angle() * 180.0 / cmnPI;
    if (angleInDegrees <= 5.0) {
        mTeleopState.SetCurrentState("ENABLED");
    } else {
        // check timer and issue a message
        if ((StateTable.GetTic() - mInStateTimer) > 2.0 * cmn_s) {
            std::stringstream message;
            message << this->GetName() + ": unable to align master, current angle error is " << angleInDegrees;
            mInterface->SendWarning(message.str());
            mInStateTimer = StateTable.GetTic();
        }
    }
}

void mtsTeleOperationPSM::EnterEnabled(void)
{
    // update MTM/PSM previous position
    mMTM->CartesianPrevious.From(mMTM->PositionCartesianCurrent.Position());
    mPSM->CartesianPrevious.From(mPSM->PositionCartesianCurrent.Position());

    // set MTM/PSM to Teleop (Cartesian Position Mode)
    mMTM->SetGravityCompensation(true);
    // set forces to zero and lock/unlock orientation as needed
    prmForceCartesianSet wrench;
    mMTM->SetWrenchBody(wrench);
    if (mRotationLocked) {
        mMTM->LockOrientation(mMTM->PositionCartesianCurrent.Position().Rotation());
    } else {
        mMTM->UnlockOrientation();
    }
    // check if by any chance the clutch pedal is pressed
    if (mIsClutched) {
        Clutch(true);
    }
}

void mtsTeleOperationPSM::RunEnabled(void)
{
    if (mMTM->PositionCartesianCurrent.Valid()
        && mPSM->PositionCartesianCurrent.Valid()) {
        // follow mode
        if (!mIsClutched) {
            // compute master Cartesian motion
            vctFrm4x4 masterCartesianMotion;
            vctFrm4x4 masterPosition(mMTM->PositionCartesianCurrent.Position());
            masterCartesianMotion = mMTM->CartesianPrevious.Inverse() * masterPosition;

            // translation
            vct3 masterTranslation;
            vct3 slaveTranslation;
            if (mTranslationLocked) {
                slaveTranslation = mPSM->CartesianPrevious.Translation();
            } else {
                masterTranslation = (masterPosition.Translation() - mMTM->CartesianPrevious.Translation());
                slaveTranslation = masterTranslation * mScale;
                slaveTranslation = mRegistrationRotation * slaveTranslation + mPSM->CartesianPrevious.Translation();
            }
            // rotation
            vctMatRot3 slaveRotation;
            if (mRotationLocked) {
                slaveRotation.From(mPSM->CartesianPrevious.Rotation());
            } else {
                slaveRotation = mRegistrationRotation * masterPosition.Rotation();
            }

            // compute desired slave position
            vctFrm4x4 slaveCartesianGoal;
            slaveCartesianGoal.Translation().Assign(slaveTranslation);
            slaveCartesianGoal.Rotation().FromNormalized(slaveRotation);
            mPSM->PositionCartesianSet.Goal().FromNormalized(slaveCartesianGoal);

            // PSM go this cartesian position
            mPSM->SetPositionCartesian(mPSM->PositionCartesianSet);

            // Gripper
            if (mMTM->GetStateGripper.IsValid()) {
                prmStateJoint gripper;
                mMTM->GetStateGripper(gripper);
                mPSM->SetJawPosition(gripper.Position()[0]);
            } else {
                mPSM->SetJawPosition(45.0 * cmnPI_180);
            }
        }
    }
}

void mtsTeleOperationPSM::TransitionEnabled(void)
{
    std::string armState;

    // check psm state
    mPSM->GetCurrentState(armState);
    if (armState != "READY") {
        mInterface->SendWarning(this->GetName() + ": PSM state has changed to [" + armState + "]");
        mTeleopState.SetDesiredState("DISABLED");
    }

    // check mtm state
    mMTM->GetCurrentState(armState);
    if (armState != "READY") {
        mInterface->SendWarning(this->GetName() + ": MTM state has changed to [" + armState + "]");
        mTeleopState.SetDesiredState("DISABLED");
    }

    if (mTeleopState.DesiredStateIsNotCurrent()) {
        mTeleopState.SetCurrentState(mTeleopState.DesiredState());
    }
}
