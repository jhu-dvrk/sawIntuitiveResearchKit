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


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperationECM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsTeleOperationECM::mtsTeleOperationECM(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    mTeleopState(componentName,
                 mtsTeleOperationECMTypes::DISABLED)
{
    Init();
}

mtsTeleOperationECM::mtsTeleOperationECM(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    mTeleopState(arg.Name,
                 mtsTeleOperationECMTypes::DISABLED)
{
    Init();
}

void mtsTeleOperationECM::Init(void)
{
    // configure state machine
    mTeleopState.AddAllowedDesiredStates(mtsTeleOperationECMTypes::ENABLED);
    mTeleopState.AddAllowedDesiredStates(mtsTeleOperationECMTypes::DISABLED);

    mScale = 0.2;

    // Initialize states
    mSlave.IsManipClutched = false;

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
        interfaceRequired->AddFunction("SetRobotControlState", mSlave.SetRobotControlState);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::SlaveErrorEventHandler,
                                                this, "Error");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::SlaveClutchEventHandler,
                                                this, "ManipClutch");
    }

    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Setting");
    if (interfaceProvided) {
        // commands
        interfaceProvided->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                              "GetPeriodStatistics"); // mtsIntervalStatistics

        interfaceProvided->AddCommandWrite(&mtsTeleOperationECM::Enable, this,
                                           "Enable", false);
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
        interfaceProvided->AddEventWrite(MessageEvents.Enabled,
                                         "Enabled", false);
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

    std::cerr << "+" << std::flush;

#if 0
    // get master Cartesian position
    mtsExecutionResult executionResult;
    executionResult = Master.GetPositionCartesian(Master.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to Master.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
        MessageEvents.Error(this->GetName() + ": unable to get cartesian position from master");
        this->Enable(false);
    }
    vctFrm4x4 masterPosition(Master.PositionCartesianCurrent.Position());

    // get slave Cartesian position
    executionResult = Slave.GetPositionCartesian(Slave.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to Slave.GetPositionCartesian failed \""
                                << executionResult << "\"" << std::endl;
        MessageEvents.Error(this->GetName() + ": unable to get cartesian position from slave");
        this->Enable(false);
    }

    /*!
      mtsTeleOperationECM can run in 4 control modes, which is controlled by
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
    if (IsEnabled
        && Master.PositionCartesianCurrent.Valid()
        && Slave.PositionCartesianCurrent.Valid()) {
        // follow mode
        if (!IsClutched && IsOperatorPresent) {
            // compute master Cartesian motion
            vctFrm4x4 masterCartesianMotion;
            masterCartesianMotion = Master.CartesianPrevious.Inverse() * masterPosition;

            // translation
            vct3 masterTranslation;
            vct3 slaveTranslation;
            if (this->TranslationLocked) {
                slaveTranslation = Slave.CartesianPrevious.Translation();
            } else {
                masterTranslation = (masterPosition.Translation() - Master.CartesianPrevious.Translation());
                slaveTranslation = masterTranslation * this->Scale;
                slaveTranslation = RegistrationRotation * slaveTranslation + Slave.CartesianPrevious.Translation();
            }
            // rotation
            vctMatRot3 slaveRotation;
            if (this->RotationLocked) {
                slaveRotation.From(Slave.CartesianPrevious.Rotation());
            } else {
                slaveRotation = RegistrationRotation * masterPosition.Rotation();
            }

            // compute desired slave position
            vctFrm4x4 slaveCartesianDesired;
            slaveCartesianDesired.Translation().Assign(slaveTranslation);
            slaveCartesianDesired.Rotation().FromNormalized(slaveRotation);
            Slave.PositionCartesianDesired.Goal().FromNormalized(slaveCartesianDesired);

            // Slave go this cartesian position
            Slave.SetPositionCartesian(Slave.PositionCartesianDesired);

            // Gripper
            if (Master.GetGripperPosition.IsValid()) {
                double gripperPosition;
                Master.GetGripperPosition(gripperPosition);
                Slave.SetJawPosition(gripperPosition);
            } else {
                Slave.SetJawPosition(5.0 * cmnPI_180);
            }
        } else if (!IsClutched && !IsOperatorPresent) {
            // Do nothing
        }
    } else {
        CMN_LOG_CLASS_RUN_DEBUG << "mtsTeleOperationECM disabled" << std::endl;
    }
#endif
}

void mtsTeleOperationECM::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

#if 0
void mtsTeleOperationECM::UpdateTransition(void)
{
    // condition to be operating
    if (mIsEnabled && mIsOperatorPresent && !mSlave.IsManipClutched) {
        // is this a transition?
        if (!mIsOperating) {
            mIsOperating = true;
            // record position of each arm
            mMasterLeft.GetPositionCartesian(mMasterLeft.PositionCartesianCurrent);
            mMasterLeft.PositionCartesianInitial.Assign(mMasterLeft.PositionCartesianCurrent.Position());
            mMasterRight.GetPositionCartesian(mMasterRight.PositionCartesianCurrent);
            mMasterRight.PositionCartesianInitial.Assign(mMasterRight.PositionCartesianCurrent.Position());
            mSlave.GetPositionCartesian(mSlave.PositionCartesianCurrent);
            mSlave.PositionCartesianInitial.Assign(mSlave.PositionCartesianCurrent.Position());
            // set the masters on wrench body mode
            mMasterLeft.SetRobotControlState(mtsStdString("DVRK_EFFORT_CARTESIAN"));
            mMasterLeft.SetWrenchBodyOrientationAbsolute(true);
            mMasterLeft.LockOrientation(mMasterLeft.PositionCartesianInitial.Rotation());
            mMasterRight.SetRobotControlState(mtsStdString("DVRK_EFFORT_CARTESIAN"));
            mMasterRight.SetWrenchBodyOrientationAbsolute(true);
            mMasterRight.LockOrientation(mMasterRight.PositionCartesianInitial.Rotation());
            // set ECM in cartesian position mode
            mSlave.SetRobotControlState(mtsStdString("DVRK_POSITION_CARTESIAN"));
            return;
        }
    }
    // all other cases, not operating
    // is this a transition?
    if (mIsOperating) {
        mIsOperating = false;
    }
}
#endif

void mtsTeleOperationECM::MasterLeftErrorEventHandler(const std::string & message)
{
    Enable(false);
    MessageEvents.Error(this->GetName() + ": received from left master [" + message + "]");
}

void mtsTeleOperationECM::MasterRightErrorEventHandler(const std::string & message)
{
    Enable(false);
    MessageEvents.Error(this->GetName() + ": received from right master [" + message + "]");
}

void mtsTeleOperationECM::SlaveErrorEventHandler(const std::string & message)
{
    Enable(false);
    MessageEvents.Error(this->GetName() + ": received from slave [" + message + "]");
}

void mtsTeleOperationECM::SlaveClutchEventHandler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        mSlave.IsManipClutched = true;
        MessageEvents.Status(this->GetName() + ": slave clutch pressed");
    } else {
        mSlave.IsManipClutched = false;
        MessageEvents.Status(this->GetName() + ": slave clutch released");
    }
}

void mtsTeleOperationECM::Enable(const bool & enable)
{
    if (enable) {
        mTeleopState.SetDesiredState(mtsTeleOperationECMTypes::ENABLED);
    } else {
        mTeleopState.SetDesiredState(mtsTeleOperationECMTypes::DISABLED);
    }
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
