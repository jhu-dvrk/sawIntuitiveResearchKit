/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2016 Johns Hopkins University (JHU), All Rights Reserved.

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


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperationPSM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsTeleOperationPSM::mtsTeleOperationPSM(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    mTeleopState(mtsTeleOperationPSMTypes::DISABLED)
{
    Init();
}

mtsTeleOperationPSM::mtsTeleOperationPSM(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    mTeleopState(mtsTeleOperationPSMTypes::DISABLED)
{
    Init();
}

void mtsTeleOperationPSM::Init(void)
{
    // configure state machine
    mTeleopState.AddAllowedDesiredStates(mtsTeleOperationPSMTypes::ENABLED);
    mTeleopState.AddAllowedDesiredStates(mtsTeleOperationPSMTypes::DISABLED);

    mScale = 0.2;

    // Initialize states
    mIsClutched = false;
    mIsOperatorPresent = false;
    mIsEnabled = false;
    mSlave.IsManipClutched = false;

    mRotationLocked = false;
    mTranslationLocked = false;

    this->StateTable.AddData(mMaster.PositionCartesianCurrent, "MasterCartesianPosition");
    this->StateTable.AddData(mSlave.PositionCartesianCurrent, "SlaveCartesianPosition");

    mConfigurationStateTable = new mtsStateTable(100, "Configuration");
    mConfigurationStateTable->SetAutomaticAdvance(false);
    this->AddStateTable(mConfigurationStateTable);
    mConfigurationStateTable->AddData(mScale, "Scale");
    mConfigurationStateTable->AddData(mRegistrationRotation, "RegistrationRotation");
    mConfigurationStateTable->AddData(mRotationLocked, "RotationLocked");
    mConfigurationStateTable->AddData(mTranslationLocked, "TranslationLocked");

    // Setup CISST Interface
    mtsInterfaceRequired * masterRequired = AddInterfaceRequired("Master");
    if (masterRequired) {
        masterRequired->AddFunction("GetPositionCartesian", mMaster.GetPositionCartesian);
        masterRequired->AddFunction("SetPositionCartesian", mMaster.SetPositionCartesian);
        masterRequired->AddFunction("SetPositionGoalCartesian", mMaster.SetPositionGoalCartesian);
        masterRequired->AddFunction("GetGripperPosition", mMaster.GetGripperPosition);
        masterRequired->AddFunction("SetRobotControlState", mMaster.SetRobotControlState);
        masterRequired->AddEventHandlerWrite(&mtsTeleOperationPSM::MasterErrorEventHandler, this, "Error");
    }

    mtsInterfaceRequired * slaveRequired = AddInterfaceRequired("Slave");
    if (slaveRequired) {
        slaveRequired->AddFunction("GetPositionCartesian", mSlave.GetPositionCartesian);
        slaveRequired->AddFunction("SetPositionCartesian", mSlave.SetPositionCartesian);
        slaveRequired->AddFunction("SetJawPosition", mSlave.SetJawPosition);
        slaveRequired->AddFunction("SetRobotControlState", mSlave.SetRobotControlState);

        slaveRequired->AddEventHandlerWrite(&mtsTeleOperationPSM::SlaveErrorEventHandler, this, "Error");
        slaveRequired->AddEventHandlerWrite(&mtsTeleOperationPSM::SlaveClutchEventHandler, this, "ManipClutch");
    }

    // Footpedal events
    mtsInterfaceRequired * clutchRequired = AddInterfaceRequired("Clutch");
    if (clutchRequired) {
        clutchRequired->AddEventHandlerWrite(&mtsTeleOperationPSM::ClutchEventHandler, this, "Button");
    }

    mtsInterfaceRequired * headRequired = AddInterfaceRequired("OperatorPresent");
    if (headRequired) {
        headRequired->AddEventHandlerWrite(&mtsTeleOperationPSM::OperatorPresentEventHandler, this, "Button");
    }

    mtsInterfaceProvided * providedSettings = AddInterfaceProvided("Setting");
    if (providedSettings) {
        // commands
        providedSettings->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                              "GetPeriodStatistics"); // mtsIntervalStatistics

        providedSettings->AddCommandWrite(&mtsTeleOperationPSM::Enable, this, "Enable", false);
        providedSettings->AddCommandWrite(&mtsTeleOperationPSM::SetScale, this, "SetScale", 0.5);
        providedSettings->AddCommandWrite(&mtsTeleOperationPSM::SetRegistrationRotation, this,
                                          "SetRegistrationRotation", vctMatRot3());
        providedSettings->AddCommandWrite(&mtsTeleOperationPSM::LockRotation, this, "LockRotation", false);
        providedSettings->AddCommandWrite(&mtsTeleOperationPSM::LockTranslation, this, "LockTranslation", false);
        providedSettings->AddCommandReadState(*(mConfigurationStateTable), mScale, "GetScale");
        providedSettings->AddCommandReadState(*(mConfigurationStateTable), mRegistrationRotation, "GetRegistrationRotation");
        providedSettings->AddCommandReadState(*(mConfigurationStateTable), mRotationLocked, "GetRotationLocked");
        providedSettings->AddCommandReadState(*(mConfigurationStateTable), mTranslationLocked, "GetTranslationLocked");

        providedSettings->AddCommandReadState(this->StateTable, mMaster.PositionCartesianCurrent, "GetPositionCartesianMaster");
        providedSettings->AddCommandReadState(this->StateTable, mSlave.PositionCartesianCurrent, "GetPositionCartesianSlave");
        // events
        providedSettings->AddEventWrite(MessageEvents.Status, "Status", std::string(""));
        providedSettings->AddEventWrite(MessageEvents.Warning, "Warning", std::string(""));
        providedSettings->AddEventWrite(MessageEvents.Error, "Error", std::string(""));
        providedSettings->AddEventWrite(MessageEvents.Enabled, "Enabled", false);
        // configuration
        providedSettings->AddEventWrite(ConfigurationEvents.Scale, "Scale", 0.5);
        providedSettings->AddEventWrite(ConfigurationEvents.RotationLocked, "RotationLocked", false);
        providedSettings->AddEventWrite(ConfigurationEvents.TranslationLocked, "TranslationLocked", false);
    }
}

void mtsTeleOperationPSM::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsTeleOperationPSM::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
}

void mtsTeleOperationPSM::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    // get master Cartesian position
    mtsExecutionResult executionResult;
    executionResult = mMaster.GetPositionCartesian(mMaster.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to Master.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
        MessageEvents.Error(this->GetName() + ": unable to get cartesian position from master");
        this->Enable(false);
    }
    vctFrm4x4 masterPosition(mMaster.PositionCartesianCurrent.Position());

    // get slave Cartesian position
    executionResult = mSlave.GetPositionCartesian(mSlave.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to Slave.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
        MessageEvents.Error(this->GetName() + ": unable to get cartesian position from slave");
        this->Enable(false);
    }

    /*!
      mtsTeleOperationPSM can run in 4 control modes, which is controlled by
      footpedal Clutch & OperatorPresent.

      Mode 1: OperatorPresent = False, Clutch = False
              MTM and PSM stop at their current position. If PSM ManipClutch is
              pressed, then the user can manually move PSM.
              NOTE: MTM always tries to allign its orientation with PSM's orientation

      Mode 2/3: OperatorPresent = False/True, Clutch = True
              MTM can move freely in workspace, however its orientation is locked
              PSM can not move

      Mode 4: OperatorPresent = True, Clutch = False
              PSM follows MTM motion
    */
    if (mIsEnabled
        && mMaster.PositionCartesianCurrent.Valid()
        && mSlave.PositionCartesianCurrent.Valid()) {
        // follow mode
        if (!mIsClutched && mIsOperatorPresent) {
            // compute master Cartesian motion
            vctFrm4x4 masterCartesianMotion;
            masterCartesianMotion = mMaster.CartesianPrevious.Inverse() * masterPosition;

            // translation
            vct3 masterTranslation;
            vct3 slaveTranslation;
            if (mTranslationLocked) {
                slaveTranslation = mSlave.CartesianPrevious.Translation();
            } else {
                masterTranslation = (masterPosition.Translation() - mMaster.CartesianPrevious.Translation());
                slaveTranslation = masterTranslation * mScale;
                slaveTranslation = mRegistrationRotation * slaveTranslation + mSlave.CartesianPrevious.Translation();
            }
            // rotation
            vctMatRot3 slaveRotation;
            if (mRotationLocked) {
                slaveRotation.From(mSlave.CartesianPrevious.Rotation());
            } else {
                slaveRotation = mRegistrationRotation * masterPosition.Rotation();
            }

            // compute desired slave position
            vctFrm4x4 slaveCartesianDesired;
            slaveCartesianDesired.Translation().Assign(slaveTranslation);
            slaveCartesianDesired.Rotation().FromNormalized(slaveRotation);
            mSlave.PositionCartesianDesired.Goal().FromNormalized(slaveCartesianDesired);

            // Slave go this cartesian position
            mSlave.SetPositionCartesian(mSlave.PositionCartesianDesired);

            // Gripper
            if (mMaster.GetGripperPosition.IsValid()) {
                double gripperPosition;
                mMaster.GetGripperPosition(gripperPosition);
                mSlave.SetJawPosition(gripperPosition);
            } else {
                mSlave.SetJawPosition(5.0 * cmnPI_180);
            }
        } else if (!mIsClutched && !mIsOperatorPresent) {
            // Do nothing
        }
    } else {
        CMN_LOG_CLASS_RUN_DEBUG << "mtsTeleOperationPSM disabled" << std::endl;
    }
}

void mtsTeleOperationPSM::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsTeleOperationPSM::MasterErrorEventHandler(const std::string & message)
{
    this->Enable(false);
    MessageEvents.Error(this->GetName() + ": received from master [" + message + "]");
}

void mtsTeleOperationPSM::SlaveErrorEventHandler(const std::string & message)
{
    this->Enable(false);
    MessageEvents.Error(this->GetName() + ": received from slave [" + message + "]");
}

void mtsTeleOperationPSM::SlaveClutchEventHandler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        mSlave.IsManipClutched = true;
        MessageEvents.Status(this->GetName() + ": slave clutch pressed");
    } else {
        mSlave.IsManipClutched = false;
        MessageEvents.Status(this->GetName() + ": slave clutch released");
    }

    // Slave State
    if (mIsEnabled && !mIsOperatorPresent && mSlave.IsManipClutched) {
        mSlave.SetRobotControlState(mtsStdString("Manual"));
    } else if (mIsEnabled) {
        mSlave.SetRobotControlState(mtsStdString("Teleop"));
    }

    // Align master
    StartAlignMaster();
}

void mtsTeleOperationPSM::StartAlignMaster(void)
{
    // Master
    if (mIsEnabled && !mSlave.IsManipClutched) {
        vctFrm4x4 masterCartesianDesired;
        masterCartesianDesired.Translation().Assign(mMasterLockTranslation);
        vctMatRot3 masterRotation;
        masterRotation = mRegistrationRotation.Inverse() * mSlave.PositionCartesianCurrent.Position().Rotation();
        masterCartesianDesired.Rotation().FromNormalized(masterRotation);

        // Send Master command position
        mMaster.SetRobotControlState(mtsStdString("DVRK_POSITION_GOAL_CARTESIAN"));
        mMaster.PositionCartesianDesired.Goal().FromNormalized(masterCartesianDesired);
        mMaster.SetPositionGoalCartesian(mMaster.PositionCartesianDesired);
    }
}

void mtsTeleOperationPSM::ClutchEventHandler(const prmEventButton & button)
{
    mtsExecutionResult executionResult;
    executionResult = mMaster.GetPositionCartesian(mMaster.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "EventHandlerClutched: call to Master.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
    }
    executionResult = mSlave.GetPositionCartesian(mSlave.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "EventHandlerClutched: call to Slave.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
    }

    if (button.Type() == prmEventButton::PRESSED) {
        mIsClutched = true;
        mMaster.PositionCartesianDesired.Goal().Rotation().FromNormalized(
                    mSlave.PositionCartesianCurrent.Position().Rotation());
        mMaster.PositionCartesianDesired.Goal().Translation().Assign(
                    mMaster.PositionCartesianCurrent.Position().Translation());
        MessageEvents.Status(this->GetName() + ": master clutch pressed");
    } else {
        mIsClutched = false;
        MessageEvents.Status(this->GetName() + ": master clutch released");
    }
    SetMasterControlState();
}

void mtsTeleOperationPSM::OperatorPresentEventHandler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        mIsOperatorPresent = true;
        MessageEvents.Status(this->GetName() + ": operator present");
        CMN_LOG_CLASS_RUN_DEBUG << "EventHandlerOperatorPresent: OperatorPresent pressed" << std::endl;
    } else {
        mIsOperatorPresent = false;
        MessageEvents.Status(this->GetName() + ": operator not present");
        CMN_LOG_CLASS_RUN_DEBUG << "EventHandlerOperatorPresent: OperatorPresent released" << std::endl;
    }
    SetMasterControlState();
}

void mtsTeleOperationPSM::Enable(const bool & enable)
{
    mIsEnabled = enable;

    if (mIsEnabled) {
        // Set Master/Slave to Teleop (Cartesian Position Mode)
        SetMasterControlState();
        mSlave.SetRobotControlState(mtsStdString("Teleop"));

        // Orientate Master with Slave
        vctFrm4x4 masterCartesianDesired;
        masterCartesianDesired.Translation().Assign(mMasterLockTranslation);
        vctMatRot3 masterRotation;
        masterRotation = mRegistrationRotation.Inverse() * mSlave.PositionCartesianCurrent.Position().Rotation();
        masterCartesianDesired.Rotation().FromNormalized(masterRotation);

        // Send Master command postion
        mMaster.SetRobotControlState(mtsStdString("DVRK_POSITION_GOAL_CARTESIAN"));
        mMaster.PositionCartesianDesired.Goal().FromNormalized(masterCartesianDesired);
        mMaster.SetPositionGoalCartesian(mMaster.PositionCartesianDesired);
    }

    // Send event for GUI
    MessageEvents.Enabled(mIsEnabled);
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
    // so disable to force a re-enable with master align
    if (lock == false) {
        Enable(false);
    } else {
        mMaster.CartesianPrevious.From(mMaster.PositionCartesianCurrent.Position());
        mSlave.CartesianPrevious.From(mSlave.PositionCartesianCurrent.Position());
    }
}

void mtsTeleOperationPSM::LockTranslation(const bool & lock)
{
    mConfigurationStateTable->Start();
    mTranslationLocked = lock;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.TranslationLocked(mTranslationLocked);
    mMaster.CartesianPrevious.From(mMaster.PositionCartesianCurrent.Position());
    mSlave.CartesianPrevious.From(mSlave.PositionCartesianCurrent.Position());
}

void mtsTeleOperationPSM::SetMasterControlState(void)
{
    if (mIsEnabled == false) {
        CMN_LOG_CLASS_RUN_WARNING << "TeleOperationPSM is NOT enabled" << std::endl;
        return;
    }

    if (mIsClutched) {
        mMaster.SetRobotControlState(mtsStdString("Clutch"));
    } else {
        if (mIsOperatorPresent) {
            mMaster.SetRobotControlState(mtsStdString("Gravity"));
        } else {
            mMasterLockTranslation.Assign(mMaster.PositionCartesianCurrent.Position().Translation());
            // Master.SetRobotControlState(mtsStdString("DVRK_POSITION_CARTESIAN"));
            // Master.PositionCartesianDesired.SetGoal(Master.PositionCartesianCurrent.Position());
            // Master.SetPositionCartesian(Master.PositionCartesianDesired);
        }
    }

    // Update MTM/PSM previous position
    mMaster.CartesianPrevious.From(mMaster.PositionCartesianCurrent.Position());
    mSlave.CartesianPrevious.From(mSlave.PositionCartesianCurrent.Position());
}
