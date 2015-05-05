/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-17

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <cisstParameterTypes/prmEventButton.h>

#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawControllers/mtsPID.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitMTM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitECM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>

#include <json/json.h>

CMN_IMPLEMENT_SERVICES(mtsIntuitiveResearchKitConsole);


mtsIntuitiveResearchKitConsole::Arm::Arm(const std::string & name,
                                         const std::string & ioComponentName):
    mName(name),
    mIOComponentName(ioComponentName)
{}

void mtsIntuitiveResearchKitConsole::Arm::ConfigurePID(const std::string & configFile,
                                                       const double & periodInSeconds)
{
    mPIDComponentName = mName + "-PID";
    mPIDConfigurationFile = configFile;

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    mtsPID * pidMaster = new mtsPID(mPIDComponentName,
                                    (periodInSeconds != 0.0) ? periodInSeconds : 1.0 * cmn_s);
    pidMaster->Configure(mPIDConfigurationFile);
    componentManager->AddComponent(pidMaster);
    componentManager->Connect(PIDComponentName(), "RobotJointTorqueInterface", IOComponentName(), Name());
    if (periodInSeconds == 0.0) {
        componentManager->Connect(PIDComponentName(), "ExecIn",
                                  IOComponentName(), "ExecOut");
    }
}

void mtsIntuitiveResearchKitConsole::Arm::ConfigureArm(const ArmType armType,
                                                       const std::string & configFile,
                                                       const double & periodInSeconds,
                                                       mtsComponent * existingArm)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    mArmConfigurationFile = configFile;
    // for research kit arms, create, add to manager and connect to
    // extra IO, PID, etc.  For generic arms, do nothing.
    switch (armType) {
    case ARM_MTM:
        {
            if (!existingArm) {
                mtsIntuitiveResearchKitMTM * master = new mtsIntuitiveResearchKitMTM(Name(), periodInSeconds);
                master->Configure(mArmConfigurationFile);
                componentManager->AddComponent(master);
            }
        }
        break;
    case ARM_PSM:
        {
            if (!existingArm) {
                mtsIntuitiveResearchKitPSM * slave = new mtsIntuitiveResearchKitPSM(Name(), periodInSeconds);
                slave->Configure(mArmConfigurationFile);
                componentManager->AddComponent(slave);
            }
            componentManager->Connect(Name(), "Adapter",
                                      IOComponentName(), Name() + "-Adapter");
            componentManager->Connect(Name(), "Tool",
                                      IOComponentName(), Name() + "-Tool");
            componentManager->Connect(Name(), "ManipClutch",
                                      IOComponentName(), Name() + "-ManipClutch");
        }
        break;
    case ARM_ECM:
        {
            if (!existingArm) {
                mtsIntuitiveResearchKitECM * ecm = new mtsIntuitiveResearchKitECM(Name(), periodInSeconds);
                ecm->Configure(mArmConfigurationFile);
                componentManager->AddComponent(ecm);
            }
            componentManager->Connect(Name(), "ManipClutch",
                                      IOComponentName(), Name() + "-ManipClutch");
        }
        break;
    default:
        break;
    }

    // if the arm is a research kit arm
    if ((armType == ARM_PSM) || (armType == ARM_MTM) || (armType == ARM_ECM)) {
        // Connect arm to IO and PID
        componentManager->Connect(Name(), "RobotIO",
                                  IOComponentName(), Name());
        componentManager->Connect(Name(), "PID",
                                  PIDComponentName(), "Controller");

    }
}

const std::string & mtsIntuitiveResearchKitConsole::Arm::Name(void) const {
    return mName;
}

const std::string & mtsIntuitiveResearchKitConsole::Arm::IOComponentName(void) const {
    return mIOComponentName;
}

const std::string & mtsIntuitiveResearchKitConsole::Arm::PIDComponentName(void) const {
    return mPIDComponentName;
}



mtsIntuitiveResearchKitConsole::TeleOp::TeleOp(const std::string & name):
    mName(name)
{
}

const std::string & mtsIntuitiveResearchKitConsole::TeleOp::Name(void) const {
    return mName;
}



mtsIntuitiveResearchKitConsole::mtsIntuitiveResearchKitConsole(const std::string & componentName):
    mtsTaskFromSignal(componentName, 100),
    mConfigured(false)
{
    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Main");
    if (interfaceProvided) {
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitConsole::SetRobotsControlState, this,
                                           "SetRobotsControlState", std::string(""));
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitConsole::TeleopEnable, this,
                                           "TeleopEnable", false);
        interfaceProvided->AddEventWrite(MessageEvents.Error, "Error", std::string(""));
        interfaceProvided->AddEventWrite(MessageEvents.Warning, "Warning", std::string(""));
        interfaceProvided->AddEventWrite(MessageEvents.Status, "Status", std::string(""));
    }

    // Footpedal events, receive
    mtsInterfaceRequired * clutchRequired = AddInterfaceRequired("Clutch");
    if (clutchRequired) {
        clutchRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ClutchEventHandler, this, "Button");
    }

    mtsInterfaceRequired * cameraRequired = AddInterfaceRequired("Camera");
    if (cameraRequired) {
        cameraRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::CameraEventHandler, this, "Button");
    }

    mtsInterfaceRequired * headRequired = AddInterfaceRequired("OperatorPresent");
    if (headRequired) {
        headRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::OperatorPresentEventHandler, this, "Button");
    }

    // Console events, send
    interfaceProvided = AddInterfaceProvided("Clutch");
    if (interfaceProvided) {
        interfaceProvided->AddEventWrite(ConsoleEvents.Clutch, "Button", prmEventButton());
    }
    interfaceProvided = AddInterfaceProvided("Camera");
    if (interfaceProvided) {
        interfaceProvided->AddEventWrite(ConsoleEvents.Camera, "Button", prmEventButton());
    }
    interfaceProvided = AddInterfaceProvided("OperatorPresent");
    if (interfaceProvided) {
        interfaceProvided->AddEventWrite(ConsoleEvents.OperatorPresent, "Button", prmEventButton());
    }
}

void mtsIntuitiveResearchKitConsole::Configure(const std::string & filename)
{
    std::ifstream jsonStream;
    jsonStream.open(filename.c_str());

    Json::Value jsonConfig, jsonValue;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration\n"
                                 << jsonReader.getFormattedErrorMessages();
        this->mConfigured = false;
        return;
    }

    // IO default settings
    double periodIO = 0.5 * cmn_ms;
    int firewirePort = 0;
    // get user preferences
    jsonValue = jsonConfig["io"]["period"];
    if (!jsonValue.empty()) {
        periodIO = jsonValue.asInt();
    }
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: period IO is " << periodIO << std::endl;
    jsonValue = jsonConfig["io"]["port"];
    if (!jsonValue.empty()) {
        firewirePort = jsonValue.asInt();
    }
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: FireWire port is " << firewirePort << std::endl;
    // create IO
    mtsRobotIO1394 * io = new mtsRobotIO1394("io", periodIO, firewirePort);

    // setup io defined in the json configuration file
    const Json::Value pairs = jsonConfig["pairs"];
    for (unsigned int index = 0; index < pairs.size(); ++index) {
        // master
        Json::Value jsonMaster = pairs[index]["master"];
        std::string armName = jsonMaster["name"].asString();
        std::string ioFile = jsonMaster["io"].asString();
        fileExists(armName + " IO", ioFile);
        io->Configure(ioFile);
        // slave
        Json::Value jsonSlave = pairs[index]["slave"];
        armName = jsonSlave["name"].asString();
        ioFile = jsonSlave["io"].asString();
        fileExists(armName + " IO", ioFile);
        io->Configure(ioFile);
    }

    componentManager->AddComponent(io);

}

const bool & mtsIntuitiveResearchKitConsole::Configured(void) const
{
    return mConfigured;
}

void mtsIntuitiveResearchKitConsole::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
}

void mtsIntuitiveResearchKitConsole::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();
}

void mtsIntuitiveResearchKitConsole::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

bool mtsIntuitiveResearchKitConsole::AddArm(Arm * newArm)
{
    if (newArm->mPIDConfigurationFile.empty() || newArm->mArmConfigurationFile.empty()) {
        CMN_LOG_CLASS_INIT_ERROR << GetName() << ": AddArm, "
                                 << newArm->Name() << " must be configured first (PID and Arm config)." << std::endl;
        return false;
    }
    if (SetupAndConnectInterfaces(newArm)) {
        mArms.push_back(newArm);
        return true;
    }
    CMN_LOG_CLASS_INIT_ERROR << GetName() << ": AddArm, unable to add new arm.  Are you adding two arms with the same name? "
                             << newArm->Name() << std::endl;
    return false;
}

bool mtsIntuitiveResearchKitConsole::AddArm(mtsComponent * genericArm, const mtsIntuitiveResearchKitConsole::Arm::ArmType CMN_UNUSED(armType))
{
    // create new required interfaces to communicate with the components we created
    Arm * newArm = new Arm(genericArm->GetName(), "");
    if (SetupAndConnectInterfaces(newArm)) {
        mArms.push_back(newArm);
        return true;
    }
    CMN_LOG_CLASS_INIT_ERROR << GetName() << ": AddArm, unable to add new arm.  Are you adding two arms with the same name? "
                             << newArm->Name() << std::endl;
    delete newArm;
    return false;
}

bool mtsIntuitiveResearchKitConsole::AddTeleOperation(const std::string & name)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    TeleOp * teleOp = new TeleOp(name);
    teleOp->InterfaceRequired = this->AddInterfaceRequired(name);
    if (teleOp->InterfaceRequired) {
        teleOp->InterfaceRequired->AddFunction("Enable", teleOp->Enable);
        teleOp->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler, this, "Error");
        teleOp->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler, this, "Warning");
        teleOp->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler, this, "Status");
        componentManager->Connect(this->GetName(), name,
                                  name, "Setting");
        mTeleOps.push_back(teleOp);
        return true;
    }
    delete teleOp;
    return false;
}

bool mtsIntuitiveResearchKitConsole::SetupAndConnectInterfaces(Arm * arm)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    // create interfaces
    const std::string interfaceNameIO = "IO" + arm->Name();
    arm->IOInterfaceRequired = AddInterfaceRequired(interfaceNameIO);
    const std::string interfaceNamePID = "PID" + arm->Name();
    arm->PIDInterfaceRequired = AddInterfaceRequired(interfaceNamePID);
    const std::string interfaceNameArm = arm->Name();
    arm->ArmInterfaceRequired = AddInterfaceRequired(interfaceNameArm);

    // check if all interfaces are correct
    if (arm->ArmInterfaceRequired && arm->PIDInterfaceRequired && arm->IOInterfaceRequired) {
        // IO
        arm->IOInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler, this, "Error");
        arm->IOInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler, this, "Warning");
        arm->IOInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler, this, "Status");
        componentManager->Connect(this->GetName(), interfaceNameIO,
                                  arm->IOComponentName(), arm->Name());
        // PID
        arm->PIDInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler, this, "Error");
        arm->PIDInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler, this, "Warning");
        arm->PIDInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler, this, "Status");
        componentManager->Connect(this->GetName(), interfaceNamePID,
                                  arm->PIDComponentName(), "Controller");
        // arm interface
        arm->ArmInterfaceRequired->AddFunction("SetRobotControlState", arm->SetRobotControlState);
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler, this, "Error");
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler, this, "Warning");
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler, this, "Status");
        componentManager->Connect(this->GetName(), interfaceNameArm,
                                  arm->Name(), "Robot");
        return true;
    }
    return false;
}

void mtsIntuitiveResearchKitConsole::SetRobotsControlState(const std::string & newState)
{
    mtsExecutionResult result;
    const ArmList::iterator end = mArms.end();
    for (ArmList::iterator arm = mArms.begin();
         arm != end;
         ++arm) {
        result = (*arm)->SetRobotControlState(newState);
        if (!result) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": SetRobotControlState: failed to set state \""
                                    << newState << "\" for arm \"" << (*arm)->Name()
                                    << "\"" << std::endl;
        }
    }
}

void mtsIntuitiveResearchKitConsole::TeleopEnable(const bool & enable)
{
    mtsExecutionResult result;
    const TeleOpList::iterator end = mTeleOps.end();
    for (TeleOpList::iterator teleOp = mTeleOps.begin();
         teleOp != end;
         ++teleOp) {
        result = (*teleOp)->Enable(enable);
        if (!result) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": Enable: failed to set \""
                                    << enable << "\" for tele-op \"" << (*teleOp)->Name()
                                    << "\"" << std::endl;
        }
    }
}

void mtsIntuitiveResearchKitConsole::ClutchEventHandler(const prmEventButton & button)
{
    ConsoleEvents.Clutch(button);
    if (button.Type() == prmEventButton::PRESSED) {
        MessageEvents.Status(this->GetName() + ": clutch pressed");
    } else {
        MessageEvents.Status(this->GetName() + ": clutch released");
    }
}

void mtsIntuitiveResearchKitConsole::CameraEventHandler(const prmEventButton & button)
{
    ConsoleEvents.Camera(button);
    if (button.Type() == prmEventButton::PRESSED) {
        MessageEvents.Status(this->GetName() + ": camera pressed");
    } else {
        MessageEvents.Status(this->GetName() + ": camera released");
    }
}

void mtsIntuitiveResearchKitConsole::OperatorPresentEventHandler(const prmEventButton & button)
{
    ConsoleEvents.OperatorPresent(button);
    if (button.Type() == prmEventButton::PRESSED) {
        MessageEvents.Status(this->GetName() + ": operator present");
    } else {
        MessageEvents.Status(this->GetName() + ": operator not present");
    }
}

void mtsIntuitiveResearchKitConsole::ErrorEventHandler(const std::string & message) {
    MessageEvents.Error(message);
}

void mtsIntuitiveResearchKitConsole::WarningEventHandler(const std::string & message) {
    MessageEvents.Warning(message);
}

void mtsIntuitiveResearchKitConsole::StatusEventHandler(const std::string & message) {
    MessageEvents.Status(message);
}
