/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-17

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
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnClassRegister.h>
#include <cisstOSAbstraction/osaDynamicLoader.h>

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawRobotIO1394/mtsRobotIO1394.h>

#include <sawControllers/mtsPID.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitRevision.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitConfig.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitMTM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitECM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitSUJ.h>
#include <sawIntuitiveResearchKit/mtsSocketClientPSM.h>
#include <sawIntuitiveResearchKit/mtsSocketServerPSM.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationPSM.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationECM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>

#include <json/json.h>

CMN_IMPLEMENT_SERVICES(mtsIntuitiveResearchKitConsole);


mtsIntuitiveResearchKitConsole::Arm::Arm(const std::string & name,
                                         const std::string & ioComponentName):
    mName(name),
    mIOComponentName(ioComponentName),
    mArmPeriod(1.0 * cmn_ms),
    IOInterfaceRequired(0),
    PIDInterfaceRequired(0),
    ArmInterfaceRequired(0),
    mSUJClutched(false)
{}

void mtsIntuitiveResearchKitConsole::Arm::ConfigurePID(const std::string & configFile,
                                                       const double & periodInSeconds)
{
    mPIDConfigurationFile = configFile;
    mPIDComponentName = mName + "-PID";

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    mtsPID * pid = new mtsPID(mPIDComponentName,
                              (periodInSeconds != 0.0) ? periodInSeconds : 0.5 * cmn_ms);
    bool hasIO = true;
    pid->Configure(mPIDConfigurationFile);
    if (mSimulation == SIMULATION_KINEMATIC) {
        pid->SetSimulated();
        hasIO = false;
    }
    componentManager->AddComponent(pid);
    if (hasIO) {
        componentManager->Connect(PIDComponentName(), "RobotJointTorqueInterface", IOComponentName(), Name());
        if (periodInSeconds == 0.0) {
            componentManager->Connect(PIDComponentName(), "ExecIn",
                                      IOComponentName(), "ExecOut");
        }
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
                if (mSimulation == SIMULATION_KINEMATIC) {
                    master->SetSimulated();
                }
                master->Configure(mArmConfigurationFile);
                componentManager->AddComponent(master);
            }
        }
        break;
    case ARM_PSM:
        {
            if (!existingArm) {
                mtsIntuitiveResearchKitPSM * slave = new mtsIntuitiveResearchKitPSM(Name(), periodInSeconds);
                if (mSimulation == SIMULATION_KINEMATIC) {
                    slave->SetSimulated();
                }
                slave->Configure(mArmConfigurationFile);
                componentManager->AddComponent(slave);

                if (mSocketServer) {
                    mtsSocketServerPSM *serverPSM = new mtsSocketServerPSM(SocketComponentName(), periodInSeconds, mIp, mPort);
                    serverPSM->Configure();
                    componentManager->AddComponent(serverPSM);
                }
            }
        }
        break;
    case ARM_PSM_SOCKET:
        {
            if (!existingArm) {
                mtsSocketClientPSM * clientPSM = new mtsSocketClientPSM(Name(), periodInSeconds, mIp, mPort);
                clientPSM->Configure();
                componentManager->AddComponent(clientPSM);
            }
        }
        break;
    case ARM_ECM:
        {
            if (!existingArm) {
                mtsIntuitiveResearchKitECM * ecm = new mtsIntuitiveResearchKitECM(Name(), periodInSeconds);
                if (mSimulation == SIMULATION_KINEMATIC) {
                    ecm->SetSimulated();
                }
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
    case ARM_MTM_DERIVED:
        {
            if (!existingArm) {
                mtsComponent * component;
                component = componentManager->GetComponent(Name());
                if (component) {
                    mtsIntuitiveResearchKitMTM * master = dynamic_cast<mtsIntuitiveResearchKitMTM *>(component);
                    if (master) {
                        master->Configure(mArmConfigurationFile);
                    } else {
                        CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::Arm::ConfigureArm: component \""
                                           << Name() << "\" doesn't seem to be derived from mtsIntuitiveResearchKitMTM."
                                           << std::endl;
                    }
                } else {
                    CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::Arm::ConfigureArm: component \""
                                       << Name() << "\" not found."
                                       << std::endl;
                }
            }
        }
        break;
    case ARM_PSM_DERIVED:
        {
            if (!existingArm) {
                mtsComponent * component;
                component = componentManager->GetComponent(Name());
                if (component) {
                    mtsIntuitiveResearchKitPSM * slave = dynamic_cast<mtsIntuitiveResearchKitPSM *>(component);
                    if (slave) {
                        slave->Configure(mArmConfigurationFile);
                    } else {
                        CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::Arm::ConfigureArm: component \""
                                           << Name() << "\" doesn't seem to be derived from mtsIntuitiveResearchKitPSM."
                                           << std::endl;
                    }
                } else {
                    CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::Arm::ConfigureArm: component \""
                                       << Name() << "\" not found."
                                       << std::endl;
                }
            }
        }
        break;
     case ARM_ECM_DERIVED:
        {
            if (!existingArm) {
                mtsComponent * component;
                component = componentManager->GetComponent(Name());
                if (component) {
                    mtsIntuitiveResearchKitECM * ecm = dynamic_cast<mtsIntuitiveResearchKitECM *>(component);
                    if (ecm) {
                        ecm->Configure(mArmConfigurationFile);
                    } else {
                        CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::Arm::ConfigureArm: component \""
                                           << Name() << "\" doesn't seem to be derived from mtsIntuitiveResearchKitECM."
                                           << std::endl;
                    }
                } else {
                    CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::Arm::ConfigureArm: component \""
                                       << Name() << "\" not found."
                                       << std::endl;
                }
            }
        }
        break;

    case ARM_ECM_GENERIC:
    case ARM_MTM_GENERIC:
    case ARM_PSM_GENERIC:
        {
            if (existingArm) {
                if (mArmConfigurationFile.empty()) {
                    existingArm->Configure();
                } else {
                    existingArm->Configure(mArmConfigurationFile);
                }
                componentManager->AddComponent(existingArm);
            }
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
    case ARM_MTM_DERIVED:
        break;
    case ARM_PSM:
    case ARM_PSM_DERIVED:
        if (mSimulation == SIMULATION_NONE) {
            componentManager->Connect(Name(), "Adapter",
                                      IOComponentName(), Name() + "-Adapter");
            componentManager->Connect(Name(), "Tool",
                                      IOComponentName(), Name() + "-Tool");
            componentManager->Connect(Name(), "ManipClutch",
                                      IOComponentName(), Name() + "-ManipClutch");
        }
        if (mSocketServer) {
            componentManager->Connect(SocketComponentName(), "PSM",
                                      Name(), "Robot");
        }
        break;
    case ARM_ECM:
    case ARM_ECM_DERIVED:
        if (mSimulation == SIMULATION_NONE) {
            componentManager->Connect(Name(), "ManipClutch",
                                      IOComponentName(), Name() + "-ManipClutch");
        }
        break;
    case ARM_SUJ:
        componentManager->Connect(Name(), "RobotIO",
                                  IOComponentName(), Name());
        componentManager->Connect(Name(), "NoMuxReset",
                                  IOComponentName(), "NoMuxReset");
        componentManager->Connect(Name(), "MuxIncrement",
                                  IOComponentName(), "MuxIncrement");
        componentManager->Connect(Name(), "ControlPWM",
                                  IOComponentName(), "ControlPWM");
        componentManager->Connect(Name(), "DisablePWM",
                                  IOComponentName(), "DisablePWM");
        componentManager->Connect(Name(), "MotorUp",
                                  IOComponentName(), "MotorUp");
        componentManager->Connect(Name(), "MotorDown",
                                  IOComponentName(), "MotorDown");
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
    if ((mType == ARM_PSM) ||
        (mType == ARM_PSM_DERIVED) ||
        (mType == ARM_MTM) ||
        (mType == ARM_MTM_DERIVED) ||
        (mType == ARM_ECM) ||
        (mType == ARM_ECM_DERIVED)) {
        // Connect arm to IO if not simulated
        if (mSimulation == SIMULATION_NONE) {
            componentManager->Connect(Name(), "RobotIO",
                                      IOComponentName(), Name());
        }
    }
    if ((mType == ARM_PSM) ||
        (mType == ARM_PSM_DERIVED) ||
        (mType == ARM_MTM) ||
        (mType == ARM_MTM_DERIVED) ||
        (mType == ARM_ECM) ||
        (mType == ARM_ECM_DERIVED)) {
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

const std::string & mtsIntuitiveResearchKitConsole::Arm::SocketComponentName(void) const {
    return mSocketComponentName;
}

const std::string & mtsIntuitiveResearchKitConsole::Arm::IOComponentName(void) const {
    return mIOComponentName;
}

const std::string & mtsIntuitiveResearchKitConsole::Arm::PIDComponentName(void) const {
    return mPIDComponentName;
}


mtsIntuitiveResearchKitConsole::TeleopECM::TeleopECM(const std::string & name,
                                                     const std::string & masterLeftName,
                                                     const std::string & masterRightName,
                                                     const std::string & slaveName,
                                                     const std::string & consoleName):
    mName(name),
    mMTMLName(masterLeftName),
    mMTMRName(masterRightName),
    mECMName(slaveName),
    mConsoleName(consoleName)
{
}

void mtsIntuitiveResearchKitConsole::TeleopECM::ConfigureTeleop(const TeleopECMType type,
                                                                const vctMatRot3 & orientation,
                                                                const double & periodInSeconds)
{
    mType = type;
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    switch (type) {
    case TELEOP_ECM:
        {
            mtsTeleOperationECM * teleop = new mtsTeleOperationECM(mName, periodInSeconds);
            teleop->Configure();
            teleop->SetRegistrationRotation(orientation);
            componentManager->AddComponent(teleop);
        }
        break;
    case TELEOP_ECM_DERIVED:
        {
            mtsComponent * component;
            component = componentManager->GetComponent(Name());
            if (component) {
                mtsTeleOperationECM * teleop = dynamic_cast<mtsTeleOperationECM *>(component);
                if (teleop) {
                    teleop->Configure();
                } else {
                    CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::Arm::ConfigureTeleop: component \""
                                       << Name() << "\" doesn't seem to be derived from mtsTeleOperationECM."
                                       << std::endl;
                }
            } else {
                CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::Arm::ConfigureTeleop: component \""
                                   << Name() << "\" not found."
                                   << std::endl;
            }
        }
        break;
    default:
        break;
    }
}

bool mtsIntuitiveResearchKitConsole::TeleopECM::Connect(void)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(mName, "MTML", mMTMLName, "Robot");
    componentManager->Connect(mName, "MTMR", mMTMRName, "Robot");
    componentManager->Connect(mName, "ECM", mECMName, "Robot");
    componentManager->Connect(mName, "Clutch", mConsoleName, "Clutch");
    componentManager->Connect(mConsoleName, mName, mName, "Setting");
    return true;
}

const std::string & mtsIntuitiveResearchKitConsole::TeleopECM::Name(void) const {
    return mName;
}


mtsIntuitiveResearchKitConsole::TeleopPSM::TeleopPSM(const std::string & name,
                                                     const std::string & masterName,
                                                     const std::string & slaveName,
                                                     const std::string & consoleName):
    mName(name),
    mMTMName(masterName),
    mPSMName(slaveName),
    mConsoleName(consoleName)
{
}

void mtsIntuitiveResearchKitConsole::TeleopPSM::ConfigureTeleop(const TeleopPSMType type,
                                                                const vctMatRot3 & orientation,
                                                                const double & periodInSeconds)
{
    mType = type;
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    switch (type) {
    case TELEOP_PSM:
        {
            mtsTeleOperationPSM * teleop = new mtsTeleOperationPSM(mName, periodInSeconds);
            teleop->Configure();
            teleop->SetRegistrationRotation(orientation);
            componentManager->AddComponent(teleop);
        }
        break;
    case TELEOP_PSM_DERIVED:
        {
            mtsComponent * component;
            component = componentManager->GetComponent(Name());
            if (component) {
                mtsTeleOperationPSM * teleop = dynamic_cast<mtsTeleOperationPSM *>(component);
                if (teleop) {
                    teleop->Configure();
                    teleop->SetRegistrationRotation(orientation);
                } else {
                    CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::Arm::ConfigureTeleop: component \""
                                       << Name() << "\" doesn't seem to be derived from mtsTeleOperationPSM."
                                       << std::endl;
                }
            } else {
                CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::Arm::ConfigureTeleop: component \""
                                   << Name() << "\" not found."
                                   << std::endl;
            }
        }
        break;
    default:
        break;
    }
}

bool mtsIntuitiveResearchKitConsole::TeleopPSM::Connect(void)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    componentManager->Connect(mName, "MTM", mMTMName, "Robot");
    componentManager->Connect(mName, "PSM", mPSMName, "Robot");
    componentManager->Connect(mName, "Clutch", mConsoleName, "Clutch");
    componentManager->Connect(mConsoleName, mName, mName, "Setting");
    return true;
}

const std::string & mtsIntuitiveResearchKitConsole::TeleopPSM::Name(void) const {
    return mName;
}



mtsIntuitiveResearchKitConsole::mtsIntuitiveResearchKitConsole(const std::string & componentName):
    mtsTaskFromSignal(componentName, 100),
    mConfigured(false),
    mTeleopEnabled(false),
    mTeleopPSMRunning(false),
    mTeleopPSMAligning(false),
    mTeleopECMRunning(false),
    mTeleopECM(0),
    mOperatorPresent(false),
    mCameraPressed(false),
    mSUJECMInterfaceRequired(0),
    mECMBaseFrameInterfaceProvided(0)
{
    mInterface = AddInterfaceProvided("Main");
    if (mInterface) {
        mInterface->AddMessageEvents();
        mInterface->AddCommandVoid(&mtsIntuitiveResearchKitConsole::PowerOff, this,
                                           "PowerOff");
        mInterface->AddCommandVoid(&mtsIntuitiveResearchKitConsole::Home, this,
                                           "Home");
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::TeleopEnable, this,
                                           "TeleopEnable", false);
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::SetScale, this,
                                           "SetScale", 0.5);
        mInterface->AddEventWrite(ConfigurationEvents.Scale,
                                         "Scale", 0.5);
    }

    mIOComponentName = "io";
    mOperatorPresentComponent = mIOComponentName;
    mOperatorPresentInterface = "COAG";
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

    // extract path of main json config file to search other files relative to it
    cmnPath configPath(cmnPath::GetWorkingDirectory());
    std::string fullname = configPath.Find(filename);
    std::string configDir = fullname.substr(0, fullname.find_last_of('/'));
    configPath.Add(configDir, cmnPath::TAIL);

    // add path to source/share directory to find common files.  This
    // will work as long as this component is located in the same
    // parent directory as the "shared" directory.
    configPath.Add(std::string(sawIntuitiveResearchKit_SOURCE_DIR) + "/../share", cmnPath::TAIL);

    // IO default settings
    double periodIO = 0.3 * cmn_ms;
    int firewirePort = 0;
    sawRobotIO1394::ProtocolType protocol = sawRobotIO1394::PROTOCOL_SEQ_R_BC_W;

    // get user preferences
    jsonValue = jsonConfig["io"];
    if (!jsonValue.empty()) {
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: looking for user provided io:period and io:port" << std::endl;
        jsonValue = jsonConfig["io"]["firewire-protocol"];
        if (!jsonValue.empty()) {
            const std::string protocolString = jsonValue.asString();
            if (protocolString == "sequential-read-write") {
                protocol = sawRobotIO1394::PROTOCOL_SEQ_RW;
            } else if (protocolString == "sequential-read-broadcast-write") {
                protocol = sawRobotIO1394::PROTOCOL_SEQ_R_BC_W;
            } else if (protocolString == "broadcast-read-write") {
                protocol = sawRobotIO1394::PROTOCOL_BC_QRW;
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure \"firewire-protocol\", values must be \"sequential-read-write\", \"sequential-read-broadcast-write\" or \"broadcast-read-write\".   Using default instead: \"sequential-read-broadcast-write\"" << std::endl;
            }
        }

        jsonValue = jsonConfig["io"]["period"];
        if (!jsonValue.empty()) {
            periodIO = jsonValue.asDouble();
            if (periodIO > 1.0 * cmn_ms) {
                std::stringstream message;
                message << "Configure:" << std::endl
                        << "------------------------------------------------------------------------------" << std::endl
                        << "WARNING, the period provided is quite high, i.e. " << periodIO << " seconds" << std::endl
                        << "We strongly recommend you change it to a value below 1 ms, i.e. 0.001" << std::endl
                        << "------------------------------------------------------------------------------";
                std::cerr << "mtsIntuitiveResearchKitConsole::" << message.str() << std::endl;
                CMN_LOG_CLASS_INIT_WARNING << message << std::endl;
            }
        }
        jsonValue = jsonConfig["io"]["port"];
        if (!jsonValue.empty()) {
            firewirePort = jsonValue.asInt();
        }
    } else {
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using default io:period and io:port" << std::endl;
    }
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: period IO is " << periodIO << std::endl
                               << "Configure: FireWire port is " << firewirePort << std::endl;

    const Json::Value arms = jsonConfig["arms"];
    for (unsigned int index = 0; index < arms.size(); ++index) {
        if (!ConfigureArmJSON(arms[index], mIOComponentName, configPath)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure arms[" << index << "]" << std::endl;
            return;
        }
    }

    // loop over all arms to check if IO is needed
    mHasIO = false;
    const ArmList::iterator end = mArms.end();
    ArmList::iterator iter;
    for (iter = mArms.begin(); iter != end; ++iter) {
        std::string ioConfig = iter->second->mIOConfigurationFile;
        if (!ioConfig.empty()) {
            mHasIO = true;
        }
    }
    // create IO if needed and configure IO
    if (mHasIO) {
        mtsRobotIO1394 * io = new mtsRobotIO1394(mIOComponentName, periodIO, firewirePort);
        io->SetProtocol(protocol);
        for (iter = mArms.begin(); iter != end; ++iter) {
            std::string ioConfig = iter->second->mIOConfigurationFile;
            if (ioConfig != "") {
                io->Configure(ioConfig);
            }
        }
        mtsComponentManager::GetInstance()->AddComponent(io);
    }

    // now can configure PID and Arms
    for (iter = mArms.begin(); iter != end; ++iter) {
        const std::string pidConfig = iter->second->mPIDConfigurationFile;
        if (!pidConfig.empty()) {
            iter->second->ConfigurePID(pidConfig);
        }
        // for generic arms, see if we can/should dynamically create the component
        mtsComponent * dynamicComponent = 0;
        if (iter->second->mIsGeneric) {
            if (!iter->second->mSharedLibrary.empty()) {
                dynamicComponent =
                    mtsComponentManager::GetInstance()
                    ->CreateComponentDynamicallyJSON(iter->second->mSharedLibrary,
                                                     iter->second->mClassName,
                                                     iter->second->mConstructorArgJSON);
            }
        }
        const std::string armConfig = iter->second->mArmConfigurationFile;
        if (dynamicComponent) {
            iter->second->ConfigureArm(iter->second->mType, armConfig,
                                       iter->second->mArmPeriod, dynamicComponent);
        } else {
            iter->second->ConfigureArm(iter->second->mType, armConfig,
                                       iter->second->mArmPeriod);
        }
    }

    bool hasSUJ = false;
    bool hasECM = false;

    // look for ECM teleop
    const Json::Value ecmTeleop = jsonConfig["ecm-teleop"];
    if (!ecmTeleop.isNull()) {
        if (!ConfigureECMTeleopJSON(ecmTeleop)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure ecm-teleop" << std::endl;
            return;
        }
    }

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
        mOperatorPresentComponent = mIOComponentName;
    }
    if (mOperatorPresentInterface == "") {
        mOperatorPresentInterface = "COAG";
    }

    // look for footpedals in json config
    jsonValue = jsonConfig["io"]["has-footpedals"];
    if (jsonValue.empty()) {
        mHasFootpedals = false;
    } else {
        mHasFootpedals = jsonValue.asBool();
    }
    // if we have any teleoperation component, we need to add the interfaces for the foot pedals
    if ((mTeleopsPSM.size() > 0) || mTeleopECM) {
        mHasFootpedals = true;
    }
    if (mHasFootpedals) {
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
            mSUJECMInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::SUJECMBaseFrameHandler,
                                                           this, "PositionCartesianDesired");
        }
        if (ecmArmInterface) {
            ecmArmInterface->AddFunction("GetPositionCartesianLocal", mGetPositionCartesianLocalFromECM);
        } else {
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: arm interface not yet added for ECM" << std::endl;
        }
        mECMBaseFrameInterfaceProvided = AddInterfaceProvided("ECMBaseFrame");
        if (mECMBaseFrameInterfaceProvided) {
            mECMBaseFrameInterfaceProvided->AddEventWrite(mECMBaseFrameEvent, "PositionCartesianDesired", prmPositionCartesianGet());
        }
    }

    // connect arm SUJ clutch button to SUJ
    if (hasSUJ) {
        for (iter = mArms.begin(); iter != end; ++iter) {
            Arm * arm = iter->second;
            // only for PSM and ECM when not simulated
            if (((arm->mType == Arm::ARM_ECM) || (arm->mType == Arm::ARM_PSM))
                && (arm->mSimulation == Arm::SIMULATION_NONE)) {
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
    std::string message = this->GetName();
    message.append(" started, dVRK ");
    message.append(sawIntuitiveResearchKit_VERSION);
    message.append(" / cisst ");
    message.append(CISST_VERSION);
    mInterface->SendStatus(message);
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
    if ((newArm->mType != Arm::ARM_PSM_SOCKET)
        && (!newArm->mIsGeneric)) {
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

std::string mtsIntuitiveResearchKitConsole::GetArmIOComponentName(const std::string & armName)
{
    ArmList::iterator armIterator = mArms.find(armName);
    if (armIterator != mArms.end()) {
        return armIterator->second->mIOComponentName;
    }
    return "";
}

// to be deprecated
bool mtsIntuitiveResearchKitConsole::AddTeleOperation(const std::string & name,
                                                      const std::string & masterName,
                                                      const std::string & slaveName)
{
    const TeleopPSMList::const_iterator teleopIterator = mTeleopsPSM.find(name);
    if (teleopIterator != mTeleopsPSM.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "AddTeleOperation: there is already a teleop named \""
                                 << name << "\"" << std::endl;
        return false;
    }
    TeleopPSM * teleop = new TeleopPSM(name, masterName, slaveName, this->GetName());
    mTeleopsPSM[name] = teleop;
    if (AddTeleopPSMInterfaces(teleop)) {
        return true;
    }
    return false;
}

bool mtsIntuitiveResearchKitConsole::AddTeleopECMInterfaces(TeleopECM * teleop)
{
    teleop->InterfaceRequired = this->AddInterfaceRequired(teleop->Name());
    if (teleop->InterfaceRequired) {
        teleop->InterfaceRequired->AddFunction("SetDesiredState", teleop->SetDesiredState);
        teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler, this, "Error");
        teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler, this, "Warning");
        teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler, this, "Status");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "AddTeleopECMInterfaces: failed to add Main interface for teleop \""
                                 << teleop->Name() << "\"" << std::endl;
        return false;
    }
    return true;
}

bool mtsIntuitiveResearchKitConsole::AddTeleopPSMInterfaces(TeleopPSM * teleop)
{
    teleop->InterfaceRequired = this->AddInterfaceRequired(teleop->Name());
    if (teleop->InterfaceRequired) {
        teleop->InterfaceRequired->AddFunction("SetDesiredState", teleop->SetDesiredState);
        teleop->InterfaceRequired->AddFunction("SetScale", teleop->SetScale);
        teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler, this, "Error");
        teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler, this, "Warning");
        teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler, this, "Status");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "AddTeleopPSMInterfaces: failed to add Main interface for teleop \""
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
    if (mHasIO) {
        componentManager->Connect(this->GetName(), "Clutch",
                                  mIOComponentName, "CLUTCH");
        componentManager->Connect(this->GetName(), "Camera",
                                  mIOComponentName, "CAMERA");
        componentManager->Connect(this->GetName(), "OperatorPresent",
                                  this->mOperatorPresentComponent, this->mOperatorPresentInterface);
    }
    return true;
}

bool mtsIntuitiveResearchKitConsole::ConfigureArmJSON(const Json::Value & jsonArm,
                                                      const std::string & ioComponentName,
                                                      const cmnPath & configPath)
{
    const std::string armName = jsonArm["name"].asString();
    const ArmList::iterator armIterator = mArms.find(armName);
    Arm * armPointer = 0;
    if (armIterator == mArms.end()) {
        // create a new arm if needed
        armPointer = new Arm(armName, ioComponentName);
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
        } else if (typeString == "MTM_DERIVED") {
            armPointer->mType = Arm::ARM_MTM_DERIVED;
        } else if (typeString == "PSM_DERIVED") {
            armPointer->mType = Arm::ARM_PSM_DERIVED;
        } else if (typeString == "ECM_DERIVED") {
            armPointer->mType = Arm::ARM_ECM_DERIVED;
        } else if (typeString == "MTM_GENERIC") {
            armPointer->mType = Arm::ARM_MTM_GENERIC;
            armPointer->mIsGeneric = true;
        } else if (typeString == "PSM_GENERIC") {
            armPointer->mType = Arm::ARM_PSM_GENERIC;
            armPointer->mIsGeneric = true;
        } else if (typeString == "ECM_GENERIC") {
            armPointer->mType = Arm::ARM_ECM_GENERIC;
            armPointer->mIsGeneric = true;
        } else if (typeString == "PSM_SOCKET") {
            armPointer->mType = Arm::ARM_PSM_SOCKET;
        } else if (typeString == "SUJ") {
            armPointer->mType = Arm::ARM_SUJ;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: arm " << armName << ": invalid type \""
                                     << typeString << "\", needs to be MTM(_DERIVED), PSM(_DERIVED) or ECM(_DERIVED)" << std::endl;
            return false;
        }
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: arm " << armName
                                 << ": doesn't have a \"type\" specified, needs to be MTM(_DERIVED), PSM(_DERIVED) or ECM(_DERIVED)" << std::endl;
        return false;
    }

    jsonValue = jsonArm["simulation"];
    if (!jsonValue.empty()) {
        std::string typeString = jsonValue.asString();
        if (typeString == "KINEMATIC") {
            armPointer->mSimulation = Arm::SIMULATION_KINEMATIC;
        } else if (typeString == "DYNAMIC") {
            armPointer->mSimulation = Arm::SIMULATION_DYNAMIC;
        } else if (typeString == "NONE") {
            armPointer->mSimulation = Arm::SIMULATION_NONE;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: arm " << armName << ": invalid simulation \""
                                     << typeString << "\", needs to be NONE, KINEMATIC or DYNAMIC" << std::endl;
            return false;
        }
    } else {
        armPointer->mSimulation = Arm::SIMULATION_NONE;
    }

    // check if we need to create a socket server attached to this arm
    armPointer->mSocketServer = false;
    jsonValue = jsonArm["socket-server"];
    if (!jsonValue.empty()) {
        armPointer->mSocketServer = jsonValue.asBool();
    }

    // for socket client or server, look for remote IP / port
    if (armPointer->mType == Arm::ARM_PSM_SOCKET || armPointer->mSocketServer) {
        armPointer->mSocketComponentName = armPointer->mName + "-SocketServer";
        jsonValue = jsonArm["remote-ip"];
        if(!jsonValue.empty()){
            armPointer->mIp = jsonValue.asString();
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"server-ip\" for arm \""
                                     << armName << "\"" << std::endl;
            return false;
        }
        jsonValue = jsonArm["port"];
        if (!jsonValue.empty()) {
            armPointer->mPort = jsonValue.asInt();
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"port\" for arm \""
                                     << armName << "\"" << std::endl;
            return false;
        }
    }

    // for generic arm, look for shared library and class name, if not
    // provided we assume the object already exists and has been
    // configured
    if (armPointer->mIsGeneric) {
        jsonValue = jsonArm["shared-library"];
        if (!jsonValue.empty()){
            armPointer->mSharedLibrary = jsonValue.asString();
            jsonValue = jsonArm["class-name"];
            if (!jsonValue.empty()) {
                armPointer->mClassName = jsonValue.asString();
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"class-name\" for arm \""
                                         << armName << "\"" << std::endl;
                return false;
            }
            jsonValue = jsonArm["constructor-arg"];
            if (!jsonValue.empty()) {
                Json::FastWriter fastWriter;
                armPointer->mConstructorArgJSON = fastWriter.write(jsonValue);
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"constructor-arg\" for arm \""
                                         << armName << "\"" << std::endl;
                return false;
            }
        }
    }

    // IO for anything not simulated or socket client
    if ((armPointer->mType != Arm::ARM_PSM_SOCKET)
        && (!armPointer->mIsGeneric)) {
        if (armPointer->mSimulation == Arm::SIMULATION_NONE) {
            jsonValue = jsonArm["io"];
            if (!jsonValue.empty()) {
                armPointer->mIOConfigurationFile = configPath.Find(jsonValue.asString());
                if (armPointer->mIOConfigurationFile == "") {
                    CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find IO file " << jsonValue.asString() << std::endl;
                    return false;
                }
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"io\" setting for arm \""
                                         << armName << "\"" << std::endl;
                return false;
            }
        }
    }

    // PID only required for MTM, PSM and ECM
    if ((armPointer->mType == Arm::ARM_MTM)
        || (armPointer->mType == Arm::ARM_MTM_DERIVED)
        || (armPointer->mType == Arm::ARM_PSM)
        || (armPointer->mType == Arm::ARM_PSM_DERIVED)
        || (armPointer->mType == Arm::ARM_ECM)
        || (armPointer->mType == Arm::ARM_ECM_DERIVED)) {
        jsonValue = jsonArm["pid"];
        if (!jsonValue.empty()) {
            armPointer->mPIDConfigurationFile = configPath.Find(jsonValue.asString());
            if (armPointer->mPIDConfigurationFile == "") {
                CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find PID file " << jsonValue.asString() << std::endl;
                return false;
            }
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"pid\" setting for arm \""
                                     << armName << "\"" << std::endl;
            return false;
        }
    }

    // only configure kinematics if not arm socket client
    if ((armPointer->mType != Arm::ARM_PSM_SOCKET)
        && (!armPointer->mIsGeneric)) {
        jsonValue = jsonArm["kinematic"];
        if (!jsonValue.empty()) {
            armPointer->mArmConfigurationFile = configPath.Find(jsonValue.asString());
            if (armPointer->mArmConfigurationFile == "") {
                CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find Kinematic file " << jsonValue.asString() << std::endl;
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
    }

    // read period if present
    jsonValue = jsonArm["period"];
    if (!jsonValue.empty()) {
        armPointer->mArmPeriod = jsonValue.asFloat();
    }

    // add the arm if it's a new one
    if (armIterator == mArms.end()) {
        AddArm(armPointer);
    }
    return true;
}

bool mtsIntuitiveResearchKitConsole::ConfigureECMTeleopJSON(const Json::Value & jsonTeleop)
{
    const std::string masterLeftName = jsonTeleop["master-left"].asString();
    const std::string masterRightName = jsonTeleop["master-right"].asString();
    const std::string slaveName = jsonTeleop["slave"].asString();
    if ((masterLeftName == "") || (masterRightName == "") || (slaveName == "")) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: \"master-left\", \"master-right\" and \"slave\" must be provided as strings" << std::endl;
        return false;
    }

    if (masterLeftName == masterRightName) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: \"master-left\" and \"master-right\" must be different" << std::endl;
        return false;
    }

    // check that both arms have been defined and have correct type
    Arm * armPointer;
    ArmList::iterator armIterator = mArms.find(masterLeftName);
    if (armIterator == mArms.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: master left\""
                                 << masterLeftName << "\" is not defined in \"arms\"" << std::endl;
        return false;
    } else {
        armPointer = armIterator->second;
        if (!((armPointer->mType == Arm::ARM_MTM_GENERIC) ||
              (armPointer->mType == Arm::ARM_MTM_DERIVED) ||
              (armPointer->mType == Arm::ARM_MTM))) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: master left\""
                                     << masterLeftName << "\" type must be \"MTM\", \"MTM_DERIVED\" or \"MTM_GENERIC\"" << std::endl;
            return false;
        }
    }
    armIterator = mArms.find(masterRightName);
    if (armIterator == mArms.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: master right\""
                                 << masterRightName << "\" is not defined in \"arms\"" << std::endl;
        return false;
    } else {
        armPointer = armIterator->second;
        if (!((armPointer->mType == Arm::ARM_MTM_GENERIC) ||
              (armPointer->mType == Arm::ARM_MTM_DERIVED) ||
              (armPointer->mType == Arm::ARM_MTM))) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: master right\""
                                     << masterRightName << "\" type must be \"MTM\", \"MTM_DERIVED\" or \"MTM_GENERIC\"" << std::endl;
            return false;
        }
    }
    armIterator = mArms.find(slaveName);
    if (armIterator == mArms.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: slave \""
                                 << slaveName << "\" is not defined in \"arms\"" << std::endl;
        return false;
    } else {
        armPointer = armIterator->second;
        if (!((armPointer->mType == Arm::ARM_ECM_GENERIC) ||
              (armPointer->mType == Arm::ARM_ECM_DERIVED) ||
              (armPointer->mType == Arm::ARM_ECM))) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: slave \""
                                     << slaveName << "\" type must be \"ECM\" or \"GENERIC_ECM\"" << std::endl;
            return false;
        }
    }

    // check if pair already exist and then add
    const std::string name = masterLeftName + "-" + masterRightName + "-" + slaveName;
    if (mTeleopECM == 0) {
        // create a new teleop if needed
        mTeleopECM = new TeleopECM(name, masterLeftName, masterRightName,
                                   slaveName, this->GetName());
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: there is already an ECM teleop" << std::endl;
        return false;
    }

    Json::Value jsonValue;
    jsonValue = jsonTeleop["type"];
    if (!jsonValue.empty()) {
        std::string typeString = jsonValue.asString();
        if (typeString == "TELEOP_ECM") {
            mTeleopECM->mType = TeleopECM::TELEOP_ECM;
        } else if (typeString == "TELEOP_ECM_DERIVED") {
            mTeleopECM->mType = TeleopECM::TELEOP_ECM_DERIVED;
        } else if (typeString == "TELEOP_ECM_GENERIC") {
            mTeleopECM->mType = TeleopECM::TELEOP_ECM_GENERIC;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: teleop " << name << ": invalid type \""
                                     << typeString << "\", needs to be TELEOP_ECM, TELEOP_ECM_DERIVED or TELEOP_ECM_GENERIC" << std::endl;
            return false;
        }
    } else {
        // default value
        mTeleopECM->mType = TeleopECM::TELEOP_ECM;
    }

    // read orientation if present
    vctMatRot3 orientation; // identity by default
    jsonValue = jsonTeleop["rotation"];
    if (!jsonValue.empty()) {
        cmnDataJSON<vctMatRot3>::DeSerializeText(orientation, jsonTeleop["rotation"]);
    }

    // read period if present
    double period = 1.0 * cmn_ms;
    jsonValue = jsonTeleop["period"];
    if (!jsonValue.empty()) {
        period = jsonValue.asFloat();
    }
    mTeleopECM->ConfigureTeleop(mTeleopECM->mType, orientation, period);
    AddTeleopECMInterfaces(mTeleopECM);
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
        if (!((armPointer->mType == Arm::ARM_MTM_GENERIC) ||
              (armPointer->mType == Arm::ARM_MTM_DERIVED) ||
              (armPointer->mType == Arm::ARM_MTM))) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: master \""
                                     << masterName << "\" type must be \"MTM\", \"MTM_DERIVED\" or \"MTM_GENERIC\"" << std::endl;
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
        if (!((armPointer->mType == Arm::ARM_PSM_GENERIC) ||
              (armPointer->mType == Arm::ARM_PSM_DERIVED) ||
              (armPointer->mType == Arm::ARM_PSM_SOCKET)  ||
              (armPointer->mType == Arm::ARM_PSM))) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: slave \""
                                     << slaveName << "\" type must be \"PSM\", \"PSM_DERIVED\" or \"PSM_GENERIC\"" << std::endl;
            return false;
        }
    }

    // check if pair already exist and then add
    const std::string name = masterName + "-" + slaveName;
    const TeleopPSMList::iterator teleopIterator = mTeleopsPSM.find(name);
    TeleopPSM * teleopPointer = 0;
    if (teleopIterator == mTeleopsPSM.end()) {
        // create a new teleop if needed
        teleopPointer = new TeleopPSM(name, masterName, slaveName, this->GetName());
        mTeleopsPSM[name] = teleopPointer;
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: there is already a teleop for the pair \""
                                 << name << "\"" << std::endl;
        return false;
    }

    Json::Value jsonValue;
    jsonValue = jsonTeleop["type"];
    if (!jsonValue.empty()) {
        std::string typeString = jsonValue.asString();
        if (typeString == "TELEOP_PSM") {
            teleopPointer->mType = TeleopPSM::TELEOP_PSM;
        } else if (typeString == "TELEOP_PSM_DERIVED") {
            teleopPointer->mType = TeleopPSM::TELEOP_PSM_DERIVED;
        } else if (typeString == "TELEOP_PSM_GENERIC") {
            teleopPointer->mType = TeleopPSM::TELEOP_PSM_GENERIC;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: teleop " << name << ": invalid type \""
                                     << typeString << "\", needs to be TELEOP_PSM, TELEOP_PSM_DERIVED or TELEOP_PSM_GENERIC" << std::endl;
            return false;
        }
    } else {
        // default value
        teleopPointer->mType = TeleopPSM::TELEOP_PSM;
    }

    // read orientation if present
    vctMatRot3 orientation; // identity by default
    jsonValue = jsonTeleop["rotation"];
    if (!jsonValue.empty()) {
        cmnDataJSON<vctMatRot3>::DeSerializeText(orientation, jsonTeleop["rotation"]);
    }

    // read period if present
    double period = 1.0 * cmn_ms;
    jsonValue = jsonTeleop["period"];
    if (!jsonValue.empty()) {
        period = jsonValue.asFloat();
    }
    teleopPointer->ConfigureTeleop(teleopPointer->mType, orientation, period);
    AddTeleopPSMInterfaces(teleopPointer);
    return true;
}

bool mtsIntuitiveResearchKitConsole::AddArmInterfaces(Arm * arm)
{
    // IO
    if (!arm->mIOConfigurationFile.empty()) {
        const std::string interfaceNameIO = "IO-" + arm->Name();
        arm->IOInterfaceRequired = AddInterfaceRequired(interfaceNameIO);
        if (arm->IOInterfaceRequired) {
            arm->IOInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
                                                           this, "Error");
            arm->IOInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                           this, "Warning");
            arm->IOInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                           this, "Status");
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "AddArmInterfaces: failed to add IO interface for arm \""
                                     << arm->Name() << "\"" << std::endl;
            return false;
        }
    }

    // PID
    if (!arm->mPIDConfigurationFile.empty()) {
        const std::string interfaceNamePID = "PID-" + arm->Name();
        arm->PIDInterfaceRequired = AddInterfaceRequired(interfaceNamePID);
        if (arm->PIDInterfaceRequired) {
            arm->PIDInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
                                                            this, "Error");
            arm->PIDInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                            this, "Warning");
            arm->PIDInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                            this, "Status");
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
        if (arm->mType != Arm::ARM_SUJ) {
            arm->ArmInterfaceRequired->AddFunction("Freeze", arm->Freeze);
        }
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
                                                        this, "Error");
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                        this, "Warning");
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                        this, "Status");
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

    // ECM teleop
    if (mTeleopECM) {
        mTeleopECM->Connect();
    }

    // PSM teleops
    const TeleopPSMList::iterator teleopPSMEnd = mTeleopsPSM.end();
    for (TeleopPSMList::iterator teleopPSMIter = mTeleopsPSM.begin();
         teleopPSMIter != teleopPSMEnd;
         ++teleopPSMIter) {
        TeleopPSM * teleop = teleopPSMIter->second;
        teleop->Connect();
    }

    // connect the foot pedals if needed
    if (mHasFootpedals) {
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

void mtsIntuitiveResearchKitConsole::PowerOff(void)
{
    mTeleopEnabled = false;
    UpdateTeleopState();
    const ArmList::iterator end = mArms.end();
    for (ArmList::iterator arm = mArms.begin();
         arm != end;
         ++arm) {
        arm->second->SetRobotControlState(mtsStdString("DVRK_UNINITIALIZED"));
    }
}

void mtsIntuitiveResearchKitConsole::Home(void)
{
    mTeleopEnabled = false;
    UpdateTeleopState();
    const ArmList::iterator end = mArms.end();
    for (ArmList::iterator arm = mArms.begin();
         arm != end;
         ++arm) {
        arm->second->SetRobotControlState(mtsStdString("DVRK_HOMING_BIAS_ENCODER"));
    }
}

void mtsIntuitiveResearchKitConsole::TeleopEnable(const bool & enable)
{
    mtsExecutionResult result;
    mTeleopEnabled = enable;
    UpdateTeleopState();
}

void mtsIntuitiveResearchKitConsole::UpdateTeleopState(void)
{
    // determine is any teleop should be running
    bool teleopPSM = false;
    bool teleopECM = false;

    // all fine
    bool allFine = mOperatorPresent;

    const ArmList::iterator endArms = mArms.end();
    ArmList::iterator iterArms = mArms.begin();
    for (; iterArms != endArms; ++iterArms) {
        if (iterArms->second->mSUJClutched) {
            allFine = false;
        }
    }

    // which one should be running now
    if (mTeleopEnabled && allFine) {
        // if the camera is pressed and there is a teleop ECM component
        if (mCameraPressed && mTeleopECM) {
            teleopECM = true;
        } else {
            teleopPSM = true;
        }
    }

    // shutdown all teleop that should be shut down
    if (!mTeleopEnabled
        || (mTeleopPSMRunning && !teleopPSM)) {
        mTeleopPSMAligning = false;
        const TeleopPSMList::iterator endTeleopPSM = mTeleopsPSM.end();
        for (TeleopPSMList::iterator iterTeleopPSM = mTeleopsPSM.begin();
             iterTeleopPSM != endTeleopPSM;
             ++iterTeleopPSM) {
            iterTeleopPSM->second->SetDesiredState(mtsStdString("DISABLED"));
        }
    }
    if (mTeleopECM
        && (!mTeleopEnabled
            || (mTeleopECMRunning && !teleopECM))) {
        mTeleopECM->SetDesiredState(mtsStdString("DISABLED"));
    }

    // freeze masters if needed
    if (mTeleopEnabled
        && !teleopECM
        && !teleopPSM) {
        iterArms = mArms.begin();
        for (; iterArms != endArms; ++iterArms) {
            if ((iterArms->second->mType == Arm::ARM_MTM) ||
                (iterArms->second->mType == Arm::ARM_MTM_DERIVED) ||
                (iterArms->second->mType == Arm::ARM_MTM_GENERIC)) {
                iterArms->second->Freeze();
            }
        }
    }

    // turn on teleop if needed
    if (!mTeleopPSMRunning && teleopPSM) {
        mTeleopPSMAligning = false;
        const TeleopPSMList::iterator endTeleopPSM = mTeleopsPSM.end();
        for (TeleopPSMList::iterator iterTeleopPSM = mTeleopsPSM.begin();
             iterTeleopPSM != endTeleopPSM;
             ++iterTeleopPSM) {
                 iterTeleopPSM->second->SetDesiredState(mtsStdString("ENABLED"));
        }
    }
    if (mTeleopECM &&
        (!mTeleopECMRunning && teleopECM)) {
        mTeleopECM->SetDesiredState(mtsStdString("ENABLED"));
    }

    // update data members
    mTeleopPSMRunning = teleopPSM;
    mTeleopECMRunning = teleopECM;

    // check if nothing is running but teleop is required, then ask to align MTMS
    if (mTeleopEnabled
        && !mTeleopPSMRunning
        && !mTeleopECMRunning) {
        std::cerr << CMN_LOG_DETAILS
                  << " Teleop PSM should have a way to check if slave orientation has changed, maybe add a run callback on TeleopPSM in align mode to set new goal, would work as soon as we get reflexx to work" << std::endl;
        const TeleopPSMList::iterator endTeleopPSM = mTeleopsPSM.end();
        for (TeleopPSMList::iterator iterTeleopPSM = mTeleopsPSM.begin();
             iterTeleopPSM != endTeleopPSM;
             ++iterTeleopPSM) {
            iterTeleopPSM->second->SetDesiredState(mtsStdString("ALIGNING_MTM"));
        }
    }
}

void mtsIntuitiveResearchKitConsole::SetScale(const double & scale)
{
    const TeleopPSMList::iterator endTeleopPSM = mTeleopsPSM.end();
    for (TeleopPSMList::iterator iterTeleopPSM = mTeleopsPSM.begin();
         iterTeleopPSM != endTeleopPSM;
         ++iterTeleopPSM) {
        iterTeleopPSM->second->SetScale(scale);
    }
    ConfigurationEvents.Scale(scale);
}

void mtsIntuitiveResearchKitConsole::ClutchEventHandler(const prmEventButton & button)
{
    ConsoleEvents.Clutch(button);
    if (button.Type() == prmEventButton::PRESSED) {
        mInterface->SendStatus(this->GetName() + ": clutch pressed");
    } else {
        mInterface->SendStatus(this->GetName() + ": clutch released");
    }
}

void mtsIntuitiveResearchKitConsole::CameraEventHandler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        mCameraPressed = true;
        mInterface->SendStatus(this->GetName() + ": camera pressed");
    } else {
        mCameraPressed = false;
        mInterface->SendStatus(this->GetName() + ": camera released");
    }
    UpdateTeleopState();
    ConsoleEvents.OperatorPresent(button);
}

void mtsIntuitiveResearchKitConsole::OperatorPresentEventHandler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        mOperatorPresent = true;
        mInterface->SendStatus(this->GetName() + ": operator present");
    } else {
        mOperatorPresent = false;
        mInterface->SendStatus(this->GetName() + ": operator not present");
    }
    UpdateTeleopState();
    ConsoleEvents.OperatorPresent(button);
}

void mtsIntuitiveResearchKitConsole::ErrorEventHandler(const mtsMessage & message) {
    TeleopEnable(false);
    mInterface->SendError(message.Message);
}

void mtsIntuitiveResearchKitConsole::WarningEventHandler(const mtsMessage & message) {
    mInterface->SendWarning(message.Message);
}

void mtsIntuitiveResearchKitConsole::StatusEventHandler(const mtsMessage & message) {
    mInterface->SendStatus(message.Message);
}

void mtsIntuitiveResearchKitConsole::ECMManipClutchEventHandler(const prmEventButton & button)
{
    std::cerr << CMN_LOG_DETAILS << " this should be probably be treated as any other clutch event  -- remove this code?" << std::endl;
    /*
    mtsExecutionResult result;
    const TeleopPSMList::iterator end = mTeleopsPSM.end();
    for (TeleopPSMList::iterator teleOp = mTeleopsPSM.begin();
         teleOp != end;
         ++teleOp) {
        result = teleOp->second->ManipClutch(button);
        if (!result) {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": ManipClutch: failed to send \""
                                    << button << "\" for tele-op \"" << teleOp->second->Name()
                                    << "\": " << result << std::endl;
        }
    }
    */
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
