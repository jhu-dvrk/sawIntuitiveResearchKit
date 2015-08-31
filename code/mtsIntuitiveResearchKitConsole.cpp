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
#include <cisstCommon/cmnPath.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawRobotIO1394/mtsRobotIO1394.h>

#include <sawControllers/mtsPID.h>
#include <sawControllers/mtsTeleOperation.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitMTM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitECM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitSUJ.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>


CMN_IMPLEMENT_SERVICES(mtsIntuitiveResearchKitConsole);


mtsIntuitiveResearchKitConsole::Arm::Arm(const std::string & name,
                                         const std::string & ioComponentName):
    mName(name),
    mIOComponentName(ioComponentName),
    IOInterfaceRequired(0),
    PIDInterfaceRequired(0),
    ArmInterfaceRequired(0)
{}

void mtsIntuitiveResearchKitConsole::Arm::ConfigurePID(const std::string & configFile,
                                                       const double & periodInSeconds)
{
    mPIDConfigurationFile = configFile;
    mPIDComponentName = mName + "-PID";

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
    mType = armType;
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
        }
        break;
    case ARM_ECM:
        {
            if (!existingArm) {
                mtsIntuitiveResearchKitECM * ecm = new mtsIntuitiveResearchKitECM(Name(), periodInSeconds);
                ecm->Configure(mArmConfigurationFile);
                componentManager->AddComponent(ecm);
            }
        }
        break;
    case ARM_SUJ:
        {
            mtsIntuitiveResearchKitSUJ * suj = new mtsIntuitiveResearchKitSUJ(Name(), periodInSeconds);
            suj->Configure(mArmConfigurationFile);
            componentManager->AddComponent(suj);
        }
        break;

    default:
        break;
    }
}

bool mtsIntuitiveResearchKitConsole::Arm::Connect(void)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    // for research kit arms, create, add to manager and connect to
    // extra IO, PID, etc.  For generic arms, do nothing.
    switch (mType) {
    case ARM_MTM:
        break;
    case ARM_PSM:
        componentManager->Connect(Name(), "Adapter",
                                  IOComponentName(), Name() + "-Adapter");
        componentManager->Connect(Name(), "Tool",
                                  IOComponentName(), Name() + "-Tool");
        componentManager->Connect(Name(), "ManipClutch",
                                  IOComponentName(), Name() + "-ManipClutch");
        break;
    case ARM_ECM:
        componentManager->Connect(Name(), "ManipClutch",
                                  IOComponentName(), Name() + "-ManipClutch");
        break;
    case ARM_SUJ:
        componentManager->Connect(Name(), "RobotIO",
                                  IOComponentName(), Name());
        componentManager->Connect(Name(), "MuxReset",
                                  IOComponentName(), "MuxReset");
        componentManager->Connect(Name(), "MuxIncrement",
                                  IOComponentName(), "MuxIncrement");
        componentManager->Connect(Name(), "SUJ-Clutch-1",
                                  IOComponentName(), "SUJ-Clutch-1");
        componentManager->Connect(Name(), "SUJ-Clutch-2",
                                  IOComponentName(), "SUJ-Clutch-2");
        componentManager->Connect(Name(), "SUJ-Clutch-3",
                                  IOComponentName(), "SUJ-Clutch-3");
        componentManager->Connect(Name(), "SUJ-Clutch-4",
                                  IOComponentName(), "SUJ-Clutch-4");
        break;
    default:
        break;
    }

    // if the arm is a research kit arm
    if ((mType == ARM_PSM) || (mType == ARM_MTM) || (mType == ARM_ECM)) {
        // Connect arm to IO and PID
        componentManager->Connect(Name(), "RobotIO",
                                  IOComponentName(), Name());
        componentManager->Connect(Name(), "PID",
                                  PIDComponentName(), "Controller");
        if ((mBaseFrameComponentName != "") && (mBaseFrameInterfaceName != "")) {
            componentManager->Connect(Name(), "BaseFrame", mBaseFrameComponentName, mBaseFrameInterfaceName);
        }
    }
    return true;
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


mtsIntuitiveResearchKitConsole::Teleop::Teleop(const std::string & name,
                                               const std::string & masterName,
                                               const std::string & slaveName,
                                               const std::string & consoleName):
    mName(name),
    mMasterName(masterName),
    mSlaveName(slaveName),
    mConsoleName(consoleName)
{
}

void mtsIntuitiveResearchKitConsole::Teleop::ConfigureTeleop(const vctMatRot3 & orientation,
                                                             const double & periodInSeconds)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    mtsTeleOperation * teleop = new mtsTeleOperation(mName, periodInSeconds);
    teleop->SetRegistrationRotation(orientation);
    componentManager->AddComponent(teleop);
}

bool mtsIntuitiveResearchKitConsole::Teleop::Connect(void)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(mName, "Master", mMasterName, "Robot");
    componentManager->Connect(mName, "Slave", mSlaveName, "Robot");
    componentManager->Connect(mName, "Clutch", mConsoleName, "Clutch");
    componentManager->Connect(mName, "OperatorPresent", mConsoleName, "OperatorPresent");
    componentManager->Connect(mConsoleName, mName, mName, "Setting");
    return true;
}

const std::string & mtsIntuitiveResearchKitConsole::Teleop::Name(void) const {
    return mName;
}



mtsIntuitiveResearchKitConsole::mtsIntuitiveResearchKitConsole(const std::string & componentName):
    mtsTaskFromSignal(componentName, 100),
    mConfigured(false),
    mSUJECMInterfaceRequired(0),
    mECMBaseFrameInterfaceProvided(0)
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
}

void mtsIntuitiveResearchKitConsole::Configure(const std::string & filename)
{
    mConfigured = false;

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
        periodIO = jsonValue.asDouble();
    }
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: period IO is " << periodIO << std::endl;
    jsonValue = jsonConfig["io"]["port"];
    if (!jsonValue.empty()) {
        firewirePort = jsonValue.asInt();
    }
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: FireWire port is " << firewirePort << std::endl;
    // create IO
    mtsRobotIO1394 * io = new mtsRobotIO1394("io", periodIO, firewirePort);

    const Json::Value arms = jsonConfig["arms"];
    for (unsigned int index = 0; index < arms.size(); ++index) {
        if (!ConfigureArmJSON(arms[index], io->GetName())) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure arms[" << index << "]" << std::endl;
            return;
        }
    }

    // loop over all arms to configure IO only
    const ArmList::iterator end = mArms.end();
    ArmList::iterator iter;
    for (iter = mArms.begin(); iter != end; ++iter) {
        std::string ioConfig = iter->second->mIOConfigurationFile;
        if (ioConfig != "") {
            io->Configure(ioConfig);
        }
    }

    mtsComponentManager::GetInstance()->AddComponent(io);

    // now can configure PID and Arms
    for (iter = mArms.begin(); iter != end; ++iter) {
        std::string pidConfig = iter->second->mPIDConfigurationFile;
        if (pidConfig != "") {
            iter->second->ConfigurePID(pidConfig);
        }
        std::string armConfig = iter->second->mArmConfigurationFile;
        if (armConfig != "") {
            iter->second->ConfigureArm(iter->second->mType, armConfig);
        }
    }

    bool hasSUJ = false;
    bool hasECM = false;

    // now load all PSM teleops
    const Json::Value psmTeleops = jsonConfig["psm-teleops"];
    for (unsigned int index = 0; index < psmTeleops.size(); ++index) {
        if (!ConfigurePSMTeleopJSON(psmTeleops[index])) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure psm-teleops[" << index << "]" << std::endl;
            return;
        }
    }

    // see which event is used for operator present
    // find name of button event used to detect if operator is present
    mOperatorPresentComponent = jsonConfig["operator-present"]["component"].asString();
    mOperatorPresentInterface = jsonConfig["operator-present"]["interface"].asString();
    //set defaults
    if (mOperatorPresentComponent == "") {
        mOperatorPresentComponent = "io";
    }
    if (mOperatorPresentInterface == "") {
        mOperatorPresentInterface = "COAG";
    }

    // if we have any teleoperation component, we need to add the interfaces for the foot pedals
    if (mTeleops.size() > 0) {
        this->AddFootpedalInterfaces();
    }

    // interface to ecm to get ECM frame and then push to PSM SUJs as base frame
    mtsInterfaceRequired * ecmArmInterface = 0;
    for (iter = mArms.begin(); iter != end; ++iter) {
        if (iter->second->mType == Arm::ARM_ECM) {
            hasECM = true;
            ecmArmInterface = iter->second->ArmInterfaceRequired;
        }
        else if (iter->second->mType == Arm::ARM_SUJ) {
            hasSUJ = true;
        }
    }

    // add required and provided interfaces to grab positions from ECM SUJ and ECM
    if (hasSUJ && hasECM) {
        mSUJECMInterfaceRequired = AddInterfaceRequired("BaseFrame");
        if (mSUJECMInterfaceRequired) {
            mSUJECMInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::SUJECMBaseFrameHandler, this, "BaseFrameDesired");
        }
        if (ecmArmInterface) {
            ecmArmInterface->AddFunction("GetPositionCartesianLocal", mGetPositionCartesianLocalFromECM);
        } else {
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: arm interface not yet added for ECM" << std::endl;
        }
        mECMBaseFrameInterfaceProvided = AddInterfaceProvided("ECMBaseFrame");
        if (mECMBaseFrameInterfaceProvided) {
            mECMBaseFrameInterfaceProvided->AddEventWrite(mECMBaseFrameEvent, "BaseFrameDesired", prmPositionCartesianGet());
        }
    }

    // connect arm SUJ clutch button to SUJ
    if (hasSUJ) {
        for (iter = mArms.begin(); iter != end; ++iter) {
            Arm * arm = iter->second;
            if ((arm->mType == Arm::ARM_ECM) || (arm->mType == Arm::ARM_PSM)) {
                arm->SUJInterfaceRequiredFromIO = this->AddInterfaceRequired("SUJ-" + arm->Name() + "-IO");
                arm->SUJInterfaceRequiredFromIO->AddEventHandlerWrite(&Arm::SUJClutchEventHandlerFromIO, arm, "Button");
                arm->SUJInterfaceRequiredToSUJ = this->AddInterfaceRequired("SUJ-" + arm->Name());
                arm->SUJInterfaceRequiredToSUJ->AddFunction("Clutch", arm->SUJClutch);
            } else {
                arm->SUJInterfaceRequiredFromIO = 0;
                arm->SUJInterfaceRequiredToSUJ = 0;
            }
        }
    }
    mConfigured = true;
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
    if (newArm->mType != Arm::ARM_SUJ) {
        if (newArm->mPIDConfigurationFile.empty()) {
            CMN_LOG_CLASS_INIT_ERROR << GetName() << ": AddArm, "
                                     << newArm->Name() << " must be configured first (PID)." << std::endl;
            return false;
        }
    }
    if (newArm->mArmConfigurationFile.empty()) {
        CMN_LOG_CLASS_INIT_ERROR << GetName() << ": AddArm, "
                                 << newArm->Name() << " must be configured first (Arm config)." << std::endl;
        return false;
    }
    if (AddArmInterfaces(newArm)) {
        ArmList::iterator armIterator = mArms.find(newArm->mName);
        if (armIterator == mArms.end()) {
            mArms[newArm->mName] = newArm;
            return true;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << GetName() << ": AddArm, "
                                     << newArm->Name() << " seems to already exist (Arm config)." << std::endl;
        }
    }
    CMN_LOG_CLASS_INIT_ERROR << GetName() << ": AddArm, unable to add new arm.  Are you adding two arms with the same name? "
                             << newArm->Name() << std::endl;
    return false;
}

bool mtsIntuitiveResearchKitConsole::AddArm(mtsComponent * genericArm, const mtsIntuitiveResearchKitConsole::Arm::ArmType CMN_UNUSED(armType))
{
    // create new required interfaces to communicate with the components we created
    Arm * newArm = new Arm(genericArm->GetName(), "");
    if (AddArmInterfaces(newArm)) {
        ArmList::iterator armIterator = mArms.find(newArm->mName);
        if (armIterator != mArms.end()) {
            mArms[newArm->mName] = newArm;
            return true;
        }
    }
    CMN_LOG_CLASS_INIT_ERROR << GetName() << ": AddArm, unable to add new arm.  Are you adding two arms with the same name? "
                             << newArm->Name() << std::endl;
    delete newArm;
    return false;
}

bool mtsIntuitiveResearchKitConsole::AddTeleOperation(const std::string & name,
                                                      const std::string & masterName,
                                                      const std::string & slaveName)
{
    Teleop * teleop = new Teleop(name, masterName, slaveName, this->GetName());
    if (AddTeleopInterfaces(teleop)) {
        return true;
    }
    delete teleop;
    return false;
}

bool mtsIntuitiveResearchKitConsole::AddTeleopInterfaces(Teleop * teleop)
{
    teleop->InterfaceRequired = this->AddInterfaceRequired(teleop->Name());
    if (teleop->InterfaceRequired) {
        teleop->InterfaceRequired->AddFunction("Enable", teleop->Enable);
        teleop->InterfaceRequired->AddFunction("CameraClutch", teleop->ManipClutch);
        teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler, this, "Error");
        teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler, this, "Warning");
        teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler, this, "Status");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "AddTeleopInterfaces: failed to add Main interface for teleop \""
                                 << teleop->Name() << "\"" << std::endl;
        return false;
    }
    return true;
}

bool mtsIntuitiveResearchKitConsole::AddFootpedalInterfaces(void)
{
    // Footpedal events, receive
    mtsInterfaceRequired * clutchRequired = AddInterfaceRequired("Clutch");
    if (clutchRequired) {
        clutchRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ClutchEventHandler, this, "Button");
    } else {
        return false;
    }
    mtsInterfaceRequired * cameraRequired = AddInterfaceRequired("Camera");
    if (cameraRequired) {
        cameraRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::CameraEventHandler, this, "Button");
    } else {
        return false;
    }
    mtsInterfaceRequired * headRequired = AddInterfaceRequired("OperatorPresent");
    if (headRequired) {
        headRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::OperatorPresentEventHandler, this, "Button");
    } else {
        return false;
    }

    // Console events, send
    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Clutch");
    if (interfaceProvided) {
        interfaceProvided->AddEventWrite(ConsoleEvents.Clutch, "Button", prmEventButton());
    } else {
        return false;
    }
    interfaceProvided = AddInterfaceProvided("Camera");
    if (interfaceProvided) {
        interfaceProvided->AddEventWrite(ConsoleEvents.Camera, "Button", prmEventButton());
    } else {
        return false;
    }
    interfaceProvided = AddInterfaceProvided("OperatorPresent");
    if (interfaceProvided) {
        interfaceProvided->AddEventWrite(ConsoleEvents.OperatorPresent, "Button", prmEventButton());
    } else {
        return false;
    }
    return true;
}

bool mtsIntuitiveResearchKitConsole::ConnectFootpedalInterfaces(void)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    // connect console to IO
    componentManager->Connect(this->GetName(), "Clutch",
                              "io", "CLUTCH");
    componentManager->Connect(this->GetName(), "Camera",
                              "io", "CAMERA");
    componentManager->Connect(this->GetName(), "OperatorPresent",
                              this->mOperatorPresentComponent, this->mOperatorPresentInterface);
    return true;
}

bool mtsIntuitiveResearchKitConsole::FileExists(const std::string & description, const std::string & filename) const
{
    if (!cmnPath::Exists(filename)) {
        CMN_LOG_CLASS_INIT_ERROR << this->GetName() << ", file not found for " << description
                                 << ": " << filename << std::endl;
        return false;
    } else {
        CMN_LOG_CLASS_INIT_VERBOSE << this->GetName() << ", file found for " << description
                                   << ": " << filename << std::endl;
        return true;
    }
}

bool mtsIntuitiveResearchKitConsole::ConfigureArmJSON(const Json::Value & jsonArm,
                                                      const std::string & ioComponentName)
{
    std::string armName = jsonArm["name"].asString();
    ArmList::iterator armIterator = mArms.find(armName);
    Arm * armPointer = 0;
    if (armIterator == mArms.end()) {
        // create a new arm if needed
        armPointer = new Arm(armName, ioComponentName);
        mArms[armName] = armPointer;
    } else {
        armPointer = armIterator->second;
    }

    // read from JSON and check if configuration files exist
    Json::Value jsonValue;
    jsonValue = jsonArm["type"];
    if (!jsonValue.empty()) {
        std::string typeString = jsonValue.asString();
        if (typeString == "MTM") {
            armPointer->mType = Arm::ARM_MTM;
        } else if (typeString == "PSM") {
            armPointer->mType = Arm::ARM_PSM;
        } else if (typeString == "ECM") {
            armPointer->mType = Arm::ARM_ECM;
        } else if (typeString == "SUJ") {
            armPointer->mType = Arm::ARM_SUJ;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: arm " << armName << ": invalid type \""
                                     << typeString << "\", needs to be MTM, PSM or ECM" << std::endl;
            return false;
        }
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: arm " << armName
                                 << ": doesn't have a \"type\" specified, needs to be MTM, PSM or ECM" << std::endl;
        return false;
    }
    jsonValue = jsonArm["io"];
    if (!jsonValue.empty()) {
        armPointer->mIOConfigurationFile = jsonValue.asString();
        if (!FileExists(armName + " io", armPointer->mIOConfigurationFile)) {
            return false;
        }
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"io\" setting for arm \""
                                 << armName << "\"" << std::endl;
        return false;
    }
    // PID only required for MTM, PSM and ECM
    if ((armPointer->mType == Arm::ARM_MTM)
        || (armPointer->mType == Arm::ARM_PSM)
        || (armPointer->mType == Arm::ARM_ECM)) {
        jsonValue = jsonArm["pid"];
        if (!jsonValue.empty()) {
            armPointer->mPIDConfigurationFile = jsonValue.asString();
            if (!FileExists(armName + " PID", armPointer->mPIDConfigurationFile)) {
                return false;
            }
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"pid\" setting for arm \""
                                     << armName << "\"" << std::endl;
            return false;
        }
    }
    jsonValue = jsonArm["kinematic"];
    if (!jsonValue.empty()) {
        armPointer->mArmConfigurationFile = jsonValue.asString();
        if (!FileExists(armName + " kinematic", armPointer->mArmConfigurationFile)) {
            return false;
        }
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"kinematic\" setting for arm \""
                                 << armName << "\"" << std::endl;
        return false;
    }
    jsonValue = jsonArm["base-frame"];
    if (!jsonValue.empty()) {
        armPointer->mBaseFrameComponentName = jsonValue.get("component", "").asString();
        armPointer->mBaseFrameInterfaceName = jsonValue.get("interface", "").asString();
        if ((armPointer->mBaseFrameComponentName == "")
            || (armPointer->mBaseFrameInterfaceName == "")) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: both \"component\" and \"interface\" must be provided with \"base-frame\" for arm \""
                                     << armName << "\"" << std::endl;
            return false;
        }
    }
    return true;
}

bool mtsIntuitiveResearchKitConsole::ConfigurePSMTeleopJSON(const Json::Value & jsonTeleop)
{
    const std::string masterName = jsonTeleop["master"].asString();
    const std::string slaveName = jsonTeleop["slave"].asString();
    if ((masterName == "") || (slaveName == "")) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: both \"master\" and \"slave\" must be provided as strings" << std::endl;
        return false;
    }

    // check that both arms have been defined and have correct type
    Arm * armPointer;
    ArmList::iterator armIterator = mArms.find(masterName);
    if (armIterator == mArms.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: master \""
                                 << masterName << "\" is not defined in \"arms\"" << std::endl;
        return false;
    } else {
        armPointer = armIterator->second;
        if (!((armPointer->mType == Arm::ARM_GENERIC_MTM) ||
              (armPointer->mType == Arm::ARM_MTM))) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: master \""
                                     << masterName << "\" type must be \"MTM\" or \"GENERIC_MTM\"" << std::endl;
            return false;
        }
    }
    armIterator = mArms.find(slaveName);
    if (armIterator == mArms.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: slave \""
                                 << slaveName << "\" is not defined in \"arms\"" << std::endl;
        return false;
    } else {
        armPointer = armIterator->second;
        if (!((armPointer->mType == Arm::ARM_GENERIC_PSM) ||
              (armPointer->mType == Arm::ARM_PSM))) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: slave \""
                                     << masterName << "\" type must be \"PSM\" or \"GENERIC_PSM\"" << std::endl;
            return false;
        }
    }

    // check if pair already exist and then add
    const std::string name = masterName + "-" + slaveName;
    TeleopList::iterator teleopIterator = mTeleops.find(name);
    Teleop * teleopPointer = 0;
    if (teleopIterator == mTeleops.end()) {
        // create a new teleop if needed
        teleopPointer = new Teleop(name, masterName, slaveName, this->GetName());
        mTeleops[name] = teleopPointer;
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: there is already a teleop for the pair \""
                                 << name << "\"" << std::endl;
        return false;
    }

    // read orientation if present
    vctMatRot3 orientation; // identity by default
    Json::Value jsonValue = jsonTeleop["rotation"];
    if (!jsonValue.empty()) {
        cmnDataJSON<vctMatRot3>::DeSerializeText(orientation, jsonTeleop["rotation"]);
    }

    // read period if present
    double period = 2.0 * cmn_ms;
    jsonValue = jsonTeleop["period"];
    if (!jsonValue.empty()) {
        period = jsonValue.asFloat();
    }
    teleopPointer->ConfigureTeleop(orientation, period);
    AddTeleopInterfaces(teleopPointer);
    return true;
}

bool mtsIntuitiveResearchKitConsole::AddArmInterfaces(Arm * arm)
{
    // IO
    const std::string interfaceNameIO = "IO-" + arm->Name();
    arm->IOInterfaceRequired = AddInterfaceRequired(interfaceNameIO);
    if (arm->IOInterfaceRequired) {
        arm->IOInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler, this, "Error");
        arm->IOInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler, this, "Warning");
        arm->IOInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler, this, "Status");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "AddArmInterfaces: failed to add IO interface for arm \""
                                 << arm->Name() << "\"" << std::endl;
        return false;
    }

    // PID
    if (arm->mType != Arm::ARM_SUJ) {
        const std::string interfaceNamePID = "PID-" + arm->Name();
        arm->PIDInterfaceRequired = AddInterfaceRequired(interfaceNamePID);
        if (arm->PIDInterfaceRequired) {
            arm->PIDInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler, this, "Error");
            arm->PIDInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler, this, "Warning");
            arm->PIDInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler, this, "Status");
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "AddArmInterfaces: failed to add PID interface for arm \""
                                     << arm->Name() << "\"" << std::endl;
            return false;
        }
    }

    // arm interface
    const std::string interfaceNameArm = arm->Name();
    arm->ArmInterfaceRequired = AddInterfaceRequired(interfaceNameArm);
    if (arm->ArmInterfaceRequired) {
        arm->ArmInterfaceRequired->AddFunction("SetRobotControlState", arm->SetRobotControlState);
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler, this, "Error");
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler, this, "Warning");
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler, this, "Status");
        // for ECM, we need to know when clutched so we can tell teleops to update master orientation
        if (arm->mType == Arm::ARM_ECM) {
            arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ECMManipClutchEventHandler, this, "ManipClutch");
        }
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "AddArmInterfaces: failed to add Main interface for arm \""
                                 << arm->Name() << "\"" << std::endl;
        return false;
    }

    return true;
}

bool mtsIntuitiveResearchKitConsole::Connect(void)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    const ArmList::iterator armsEnd = mArms.end();
    for (ArmList::iterator armIter = mArms.begin();
         armIter != armsEnd;
         ++armIter) {
        Arm * arm = armIter->second;

        // IO
        if (arm->IOInterfaceRequired) {
            componentManager->Connect(this->GetName(), "IO-" + arm->Name(),
                                      arm->IOComponentName(), arm->Name());
        }
        // PID
        if (arm->mType != Arm::ARM_SUJ) {
            if (arm->PIDInterfaceRequired) {
                componentManager->Connect(this->GetName(), "PID-" + arm->Name(),
                                          arm->PIDComponentName(), "Controller");
            }
        }
        // arm interface
        if (arm->ArmInterfaceRequired) {
            componentManager->Connect(this->GetName(), arm->Name(),
                                      arm->Name(), "Robot");
        }
        // arm specific interfaces
        arm->Connect();
        // connect to SUJ if needed
        if (arm->SUJInterfaceRequiredFromIO && arm->SUJInterfaceRequiredToSUJ) {
            componentManager->Connect(this->GetName(), arm->SUJInterfaceRequiredToSUJ->GetName(),
                                      "SUJ", arm->Name());
            componentManager->Connect(this->GetName(), arm->SUJInterfaceRequiredFromIO->GetName(),
                                      arm->IOComponentName(), arm->Name() + "-SUJClutch");
        }
    }

    const TeleopList::iterator teleopsEnd = mTeleops.end();
    for (TeleopList::iterator teleopIter = mTeleops.begin();
         teleopIter != teleopsEnd;
         ++teleopIter) {
        Teleop * teleop = teleopIter->second;
        teleop->Connect();
    }

    // if we have any teleoperation component, we need to connect the foot pedals
    if (mTeleops.size() > 0) {
        this->ConnectFootpedalInterfaces();
    }

    // connect interfaces to retrieve base frame from ECM SUJ and send event to SUJ
    if (mSUJECMInterfaceRequired
        && mECMBaseFrameInterfaceProvided) {
        componentManager->Connect(this->GetName(), "BaseFrame", "SUJ", "ECM");
        componentManager->Connect("SUJ", "BaseFrame", this->GetName(), "ECMBaseFrame");
    }

    return true;
}

void mtsIntuitiveResearchKitConsole::SetRobotsControlState(const std::string & newState)
{
    mtsExecutionResult result;
    const ArmList::iterator end = mArms.end();
    for (ArmList::iterator arm = mArms.begin();
         arm != end;
         ++arm) {
        result = arm->second->SetRobotControlState(newState);
        if (!result) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": SetRobotControlState: failed to set state \""
                                    << newState << "\" for arm \"" << arm->second->Name()
                                    << "\"" << std::endl;
        }
    }
}

void mtsIntuitiveResearchKitConsole::TeleopEnable(const bool & enable)
{
    mtsExecutionResult result;
    const TeleopList::iterator end = mTeleops.end();
    for (TeleopList::iterator teleOp = mTeleops.begin();
         teleOp != end;
         ++teleOp) {
        result = teleOp->second->Enable(enable);
        if (!result) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": Enable: failed to set \""
                                    << enable << "\" for tele-op \"" << teleOp->second->Name()
                                    << "\": " << result << std::endl;
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

void mtsIntuitiveResearchKitConsole::ECMManipClutchEventHandler(const prmEventButton & button)
{
    mtsExecutionResult result;
    const TeleopList::iterator end = mTeleops.end();
    for (TeleopList::iterator teleOp = mTeleops.begin();
         teleOp != end;
         ++teleOp) {
        result = teleOp->second->ManipClutch(button);
        if (!result) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": ManipClutch: failed to send \""
                                    << button << "\" for tele-op \"" << teleOp->second->Name()
                                    << "\": " << result << std::endl;
        }
    }
}

void mtsIntuitiveResearchKitConsole::SUJECMBaseFrameHandler(const prmPositionCartesianGet & baseFrameParam)
{
    // get position from ECM and convert to useful type
    prmPositionCartesianGet positionECMLocalParam;
    mGetPositionCartesianLocalFromECM(positionECMLocalParam);
    vctFrm3 positionECM = baseFrameParam.Position() * positionECMLocalParam.Position();

    // compute and send new base frame for all SUJs (SUJ will handle ECM differently)
    prmPositionCartesianGet baseFrameSUJParam;
    baseFrameSUJParam.Position().From(positionECM.Inverse());
    baseFrameSUJParam.SetValid(baseFrameParam.Valid() && positionECMLocalParam.Valid());
    baseFrameSUJParam.SetTimestamp(positionECMLocalParam.Timestamp());
    mECMBaseFrameEvent(baseFrameSUJParam);
}
