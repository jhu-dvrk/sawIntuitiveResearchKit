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
    mArmPeriod(mtsIntuitiveResearchKit::ArmPeriod),
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
                              (periodInSeconds != 0.0) ? periodInSeconds : mtsIntuitiveResearchKit::IOPeriod);
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
                                                       const double & periodInSeconds)
{
    mType = armType;
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    mArmConfigurationFile = configFile;
    // for research kit arms, create, add to manager and connect to
    // extra IO, PID, etc.  For generic arms, do nothing.
    switch (armType) {
    case ARM_MTM:
        {
            mtsIntuitiveResearchKitMTM * master = new mtsIntuitiveResearchKitMTM(Name(), periodInSeconds);
            if (mSimulation == SIMULATION_KINEMATIC) {
                master->SetSimulated();
            }
            master->Configure(mArmConfigurationFile);
            componentManager->AddComponent(master);
        }
        break;
    case ARM_PSM:
        {
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
        break;
    case ARM_PSM_SOCKET:
        {
            mtsSocketClientPSM * clientPSM = new mtsSocketClientPSM(Name(), periodInSeconds, mIp, mPort);
            clientPSM->Configure();
            componentManager->AddComponent(clientPSM);
        }
        break;
    case ARM_ECM:
        {
            mtsIntuitiveResearchKitECM * ecm = new mtsIntuitiveResearchKitECM(Name(), periodInSeconds);
            if (mSimulation == SIMULATION_KINEMATIC) {
                ecm->SetSimulated();
            }
            ecm->Configure(mArmConfigurationFile);
            componentManager->AddComponent(ecm);
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
            mtsComponent * component;
            component = componentManager->GetComponent(Name());
            if (component) {
                mtsIntuitiveResearchKitMTM * master = dynamic_cast<mtsIntuitiveResearchKitMTM *>(component);
                if (master) {
                    if (mSimulation == SIMULATION_KINEMATIC) {
                        master->SetSimulated();
                    }
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
        break;
    case ARM_PSM_DERIVED:
        {
            mtsComponent * component;
            component = componentManager->GetComponent(Name());
            if (component) {
                mtsIntuitiveResearchKitPSM * slave = dynamic_cast<mtsIntuitiveResearchKitPSM *>(component);
                if (slave) {
                    if (mSimulation == SIMULATION_KINEMATIC) {
                        slave->SetSimulated();
                    }
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
        break;
     case ARM_ECM_DERIVED:
        {
            mtsComponent * component;
            component = componentManager->GetComponent(Name());
            if (component) {
                mtsIntuitiveResearchKitECM * ecm = dynamic_cast<mtsIntuitiveResearchKitECM *>(component);
                if (ecm) {
                    if (mSimulation == SIMULATION_KINEMATIC) {
                        ecm->SetSimulated();
                    }
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
                                      ComponentName(), InterfaceName());
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

const std::string & mtsIntuitiveResearchKitConsole::Arm::ComponentName(void) const {
    return mComponentName;
}

const std::string & mtsIntuitiveResearchKitConsole::Arm::InterfaceName(void) const {
    return mInterfaceName;
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
                                                     const std::string & masterLeftComponentName,
                                                     const std::string & masterLeftInterfaceName,
                                                     const std::string & masterRightComponentName,
                                                     const std::string & masterRightInterfaceName,
                                                     const std::string & slaveComponentName,
                                                     const std::string & slaveInterfaceName,
                                                     const std::string & consoleName):
    mName(name),
    mMTMLComponentName(masterLeftComponentName),
    mMTMLInterfaceName(masterLeftInterfaceName),
    mMTMRComponentName(masterRightComponentName),
    mMTMRInterfaceName(masterRightInterfaceName),
    mECMComponentName(slaveComponentName),
    mECMInterfaceName(slaveInterfaceName),
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
    componentManager->Connect(mName, "MTML", mMTMLComponentName, mMTMLInterfaceName);
    componentManager->Connect(mName, "MTMR", mMTMRComponentName, mMTMRInterfaceName);
    componentManager->Connect(mName, "ECM", mECMComponentName, mECMInterfaceName);
    componentManager->Connect(mName, "Clutch", mConsoleName, "Clutch");
    componentManager->Connect(mConsoleName, mName, mName, "Setting");
    return true;
}

const std::string & mtsIntuitiveResearchKitConsole::TeleopECM::Name(void) const {
    return mName;
}


mtsIntuitiveResearchKitConsole::TeleopPSM::TeleopPSM(const std::string & name,
                                                     const std::string & masterComponentName,
                                                     const std::string & masterInterfaceName,
                                                     const std::string & slaveComponentName,
                                                     const std::string & slaveInterfaceName,
                                                     const std::string & consoleName):
    mName(name),
    mMTMComponentName(masterComponentName),
    mMTMInterfaceName(masterInterfaceName),
    mPSMComponentName(slaveComponentName),
    mPSMInterfaceName(slaveInterfaceName),
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
    componentManager->Connect(mName, "MTM", mMTMComponentName, mMTMInterfaceName);
    componentManager->Connect(mName, "PSM", mPSMComponentName, mPSMInterfaceName);
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
    mIOComponentName("io"),
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
}

void mtsIntuitiveResearchKitConsole::Configure(const std::string & filename)
{
    mConfigured = false;

    std::ifstream jsonStream;
    jsonStream.open(filename.c_str());

    Json::Value jsonConfig, jsonValue;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration" << std::endl
                                 << "File: " << filename << std::endl << "Error(s):" << std::endl
                                 << jsonReader.getFormattedErrorMessages();
        this->mConfigured = false;
        return;
    }

    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << this->GetName()
                               << " using file \"" << filename << "\"" << std::endl
                               << "----> content of configuration file: " << std::endl
                               << jsonConfig << std::endl
                               << "<----" << std::endl;

    // extract path of main json config file to search other files relative to it
    cmnPath configPath(cmnPath::GetWorkingDirectory());
    std::string fullname = configPath.Find(filename);
    std::string configDir = fullname.substr(0, fullname.find_last_of('/'));
    configPath.Add(configDir, cmnPath::TAIL);

    // add path to source/share directory to find common files.  This
    // will work as long as this component is located in the same
    // parent directory as the "shared" directory.
    configPath.Add(std::string(sawIntuitiveResearchKit_SOURCE_DIR) + "/../share", cmnPath::TAIL);

    mtsComponentManager * manager = mtsComponentManager::GetInstance();

    // first, create all custom components and connections, i.e. dynamic loading and creation
    const Json::Value componentManager = jsonConfig["component-manager"];
    if (!componentManager.empty()) {
        if (!manager->ConfigureJSON(componentManager, configPath)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure component-manager" << std::endl;
            return;
        }
    }

    // IO default settings
    double periodIO = mtsIntuitiveResearchKit::IOPeriod;
    int firewirePort = 0;
    sawRobotIO1394::ProtocolType protocol = sawRobotIO1394::PROTOCOL_BC_QRW;

    // get user preferences
    jsonValue = jsonConfig["io"];
    if (!jsonValue.empty()) {
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
                return;
            }
        }

        jsonValue = jsonConfig["io"]["period"];
        if (!jsonValue.empty()) {
            periodIO = jsonValue.asDouble();
            if (periodIO > 1.0 * cmn_ms) {
                std::stringstream message;
                message << "Configure:" << std::endl
                        << "----------------------------------------------------" << std::endl
                        << " Warning:" << std::endl
                        << "   The period provided is quite high, i.e. " << periodIO << std::endl
                        << "   seconds.  We strongly recommend you change it to" << std::endl
                        << "   a value below 1 ms, i.e. 0.001." << std::endl
                        << "----------------------------------------------------";
                std::cerr << "mtsIntuitiveResearchKitConsole::" << message.str() << std::endl;
                CMN_LOG_CLASS_INIT_WARNING << message.str() << std::endl;
            }
        }
        jsonValue = jsonConfig["io"]["port"];
        if (!jsonValue.empty()) {
            firewirePort = jsonValue.asInt();
        }
    } else {
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using default io:period, io:port and io:firewire-protocol" << std::endl;
    }
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure:" << std::endl
                               << "     - Period IO is " << periodIO << std::endl
                               << "     - FireWire port is " << firewirePort << std::endl
                               << "     - Protocol is " << protocol << std::endl;

    if (protocol != sawRobotIO1394::PROTOCOL_BC_QRW) {
        std::stringstream message;
        message << "Configure:" << std::endl
                << "----------------------------------------------------" << std::endl
                << " Warning:" << std::endl
                << "   The firewire-protocol is not full broadcast" << std::endl
                << "   We recommend you set it to \"broadcast-read-write\"." << std::endl
                << "   You'll need firmware rev. 4 or above for this." << std::endl
                << "----------------------------------------------------";
        std::cerr << "mtsIntuitiveResearchKitConsole::" << message.str() << std::endl;
        CMN_LOG_CLASS_INIT_WARNING << message.str() << std::endl;
    }

    const Json::Value arms = jsonConfig["arms"];
    for (unsigned int index = 0; index < arms.size(); ++index) {
        if (!ConfigureArmJSON(arms[index], mIOComponentName, configPath)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure arms[" << index << "]" << std::endl;
            return;
        }
    }

    // loop over all arms to check if IO is needed, also check if some IO configuration files are listed in "io"
    mHasIO = false;
    const ArmList::iterator end = mArms.end();
    ArmList::iterator iter;
    for (iter = mArms.begin(); iter != end; ++iter) {
        std::string ioConfig = iter->second->mIOConfigurationFile;
        if (!ioConfig.empty()) {
            mHasIO = true;
        }
    }
    jsonValue = jsonConfig["io"];
    if (!jsonValue.empty()) {
        // generic files
        Json::Value configFiles = jsonValue["configuration-files"];
        if (!configFiles.empty()) {
            mHasIO = true;
        }
        // footpedals config
        configFiles = jsonValue["footpedals"];
        if (!configFiles.empty()) {
            mHasIO = true;
        }
    }

    // create IO if needed and configure IO
    if (mHasIO) {
        mtsRobotIO1394 * io = new mtsRobotIO1394(mIOComponentName, periodIO, firewirePort);
        io->SetProtocol(protocol);
        // configure for each arm
        for (iter = mArms.begin(); iter != end; ++iter) {
            std::string ioConfig = iter->second->mIOConfigurationFile;
            if (ioConfig != "") {
                CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring IO using \"" << ioConfig << "\"" << std::endl;
                io->Configure(ioConfig);
            }
        }
        // configure using extra configuration files
        jsonValue = jsonConfig["io"];
        if (!jsonValue.empty()) {
            // generic files
            Json::Value configFiles = jsonValue["configuration-files"];
            if (!configFiles.empty()) {
                for (unsigned int index = 0; index < configFiles.size(); ++index) {
                    const std::string configFile = configPath.Find(configFiles[index].asString());
                    if (configFile == "") {
                        CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find configuration file "
                                                 << configFiles[index].asString() << std::endl;
                        return;
                    }
                    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring IO using \"" << configFile << "\"" << std::endl;
                    io->Configure(configFile);
                }
            }
            // footpedals, we assume these are the default one provided along the dVRK
            configFiles = jsonValue["footpedals"];
            if (!configFiles.empty()) {
                const std::string configFile = configPath.Find(configFiles.asString());
                if (configFile == "") {
                    CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find configuration file "
                                             << configFiles.asString() << std::endl;
                    return;
                }
                CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring IO foot pedals using \"" << configFile << "\"" << std::endl;
                // these can be overwritten using console-inputs
                mDInputSources["Clutch"] = InterfaceComponentType(mIOComponentName, "Clutch");
                mDInputSources["OperatorPresent"] = InterfaceComponentType(mIOComponentName, "Coag");
                mDInputSources["BiCoag"] = InterfaceComponentType(mIOComponentName, "BiCoag");
                mDInputSources["Camera"] = InterfaceComponentType(mIOComponentName, "Camera");
                mDInputSources["Cam-"] = InterfaceComponentType(mIOComponentName, "Cam-");
                mDInputSources["Cam+"] = InterfaceComponentType(mIOComponentName, "Cam+");
                mDInputSources["Head"] = InterfaceComponentType(mIOComponentName, "Head");
                io->Configure(configFile);
            }
        }
        // and add the io component!
        mtsComponentManager::GetInstance()->AddComponent(io);
    }

    // now can configure PID and Arms
    for (iter = mArms.begin(); iter != end; ++iter) {
        const std::string pidConfig = iter->second->mPIDConfigurationFile;
        if (!pidConfig.empty()) {
            iter->second->ConfigurePID(pidConfig);
        }
        // for generic arms, nothing to do
        if (!iter->second->mIsGeneric) {
            const std::string armConfig = iter->second->mArmConfigurationFile;
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

    // support for older files
    std::string operatorPresentComponent = jsonConfig["operator-present"]["component"].asString();
    std::string operatorPresentInterface = jsonConfig["operator-present"]["interface"].asString();
    if ((operatorPresentComponent != "")
        || (operatorPresentInterface != "")) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: \"operator-present\" should now be defined within \"console-inputs\" scope" << std::endl;
        return;
    }

    // load from console inputs
    const Json::Value consoleInputs = jsonConfig["console-inputs"];
    if (!consoleInputs.empty()) {
        std::string component, interface;
        component = consoleInputs["operator-present"]["component"].asString();
        interface = consoleInputs["operator-present"]["interface"].asString();
        if ((component != "") && (interface != "")) {
            mDInputSources["OperatorPresent"] = InterfaceComponentType(component, interface);
        }
        component = consoleInputs["clutch"]["component"].asString();
        interface = consoleInputs["clutch"]["interface"].asString();
        if ((component != "") && (interface != "")) {
            mDInputSources["Clutch"] = InterfaceComponentType(component, interface);
        }
        component = consoleInputs["camera"]["component"].asString();
        interface = consoleInputs["camera"]["interface"].asString();
        if ((component != "") && (interface != "")) {
            mDInputSources["Camera"] = InterfaceComponentType(component, interface);
        }
    }

    // if we have any teleoperation component, we need to have the interfaces for the foot pedals
    const DInputSourceType::const_iterator endDInputs = mDInputSources.end();
    const bool foundClutch = (mDInputSources.find("Clutch") != endDInputs);
    const bool foundOperatorPresent = (mDInputSources.find("OperatorPresent") != endDInputs);
    const bool foundCamera = (mDInputSources.find("Camera") != endDInputs);

    if (mTeleopsPSM.size() > 0) {
        if (!foundClutch || !foundOperatorPresent) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: inputs for footpedals \"Clutch\" and \"OperatorPresent\" need to be defined since there's at least one PSM tele-operation component.  Maybe you're missing \"io\":\"footpedals\" in your configuration file." << std::endl;
            return;
        }
    }
    if (mTeleopECM) {
        if (!foundCamera || !foundOperatorPresent) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: inputs for footpedals \"Camera\" and \"OperatorPresent\" need to be defined since there's an ECM tele-operation component.  Maybe you're missing \"io\":\"footpedals\" in your configuration file." << std::endl;
            return;
        }
    }
    this->AddFootpedalInterfaces();

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
    TeleopPSM * teleop = new TeleopPSM(name,
                                       masterName, "Robot",
                                       slaveName, "Robot",
                                       this->GetName());
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

void mtsIntuitiveResearchKitConsole::AddFootpedalInterfaces(void)
{
    const DInputSourceType::const_iterator endDInputs = mDInputSources.end();
    const bool foundClutch = (mDInputSources.find("Clutch") != endDInputs);
    const bool foundOperatorPresent = (mDInputSources.find("OperatorPresent") != endDInputs);
    const bool foundCamera = (mDInputSources.find("Camera") != endDInputs);

    if (foundClutch) {
        mtsInterfaceRequired * clutchRequired = AddInterfaceRequired("Clutch");
        if (clutchRequired) {
            clutchRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ClutchEventHandler, this, "Button");
        }
        mtsInterfaceProvided * clutchProvided = AddInterfaceProvided("Clutch");
        if (clutchProvided) {
            clutchProvided->AddEventWrite(ConsoleEvents.Clutch, "Button", prmEventButton());
        }
    }

    if (foundCamera) {
        mtsInterfaceRequired * cameraRequired = AddInterfaceRequired("Camera");
        if (cameraRequired) {
            cameraRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::CameraEventHandler, this, "Button");
        }
        mtsInterfaceProvided * cameraProvided = AddInterfaceProvided("Camera");
        if (cameraProvided) {
            cameraProvided->AddEventWrite(ConsoleEvents.Camera, "Button", prmEventButton());
        }
    }

    if (foundOperatorPresent) {
        mtsInterfaceRequired * operatorRequired = AddInterfaceRequired("OperatorPresent");
        if (operatorRequired) {
            operatorRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::OperatorPresentEventHandler, this, "Button");
        }
        mtsInterfaceProvided * operatorProvided = AddInterfaceProvided("OperatorPresent");
        if (operatorProvided) {
            operatorProvided->AddEventWrite(ConsoleEvents.OperatorPresent, "Button", prmEventButton());
        }
    }
}

void mtsIntuitiveResearchKitConsole::ConnectFootpedalInterfaces(void)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    const DInputSourceType::const_iterator end = mDInputSources.end();
    DInputSourceType::const_iterator iter;
    iter = mDInputSources.find("Clutch");
    if (iter != end) {
        componentManager->Connect(this->GetName(), "Clutch",
                                  iter->second.first, iter->second.second);
    }
    iter = mDInputSources.find("Camera");
    if (iter != end) {
        componentManager->Connect(this->GetName(), "Camera",
                                  iter->second.first, iter->second.second);
    }
    iter = mDInputSources.find("OperatorPresent");
    if (iter != end) {
        componentManager->Connect(this->GetName(), "OperatorPresent",
                                  iter->second.first, iter->second.second);
    }
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
    armPointer->mIsGeneric = false; // default value
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

    // component and interface, defaults
    armPointer->mComponentName = armName;
    armPointer->mInterfaceName = "Robot";
    jsonValue = jsonArm["component"];
    if (!jsonValue.empty()) {
        armPointer->mComponentName = jsonValue.asString();
    }
    jsonValue = jsonArm["interface"];
    if (!jsonValue.empty()) {
        armPointer->mInterfaceName = jsonValue.asString();
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
    std::string
        masterLeftComponent, masterLeftInterface,
        masterRightComponent, masterRightInterface,
        slaveComponent, slaveInterface;
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
        masterLeftComponent = armPointer->ComponentName();
        masterLeftInterface = armPointer->InterfaceName();
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
        masterRightComponent = armPointer->ComponentName();
        masterRightInterface = armPointer->InterfaceName();
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
        slaveComponent = armPointer->ComponentName();
        slaveInterface = armPointer->InterfaceName();
    }

    // check if pair already exist and then add
    const std::string name = masterLeftName + "-" + masterRightName + "-" + slaveName;
    if (mTeleopECM == 0) {
        // create a new teleop if needed
        mTeleopECM = new TeleopECM(name,
                                   masterLeftComponent, masterLeftInterface,
                                   masterRightComponent, masterRightInterface,
                                   slaveComponent, slaveInterface,
                                   this->GetName());
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
    double period = mtsIntuitiveResearchKit::TeleopPeriod;
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

    std::string masterComponent, masterInterface, slaveComponent, slaveInterface;
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
        masterComponent = armPointer->ComponentName();
        masterInterface = armPointer->InterfaceName();
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
        slaveComponent = armPointer->ComponentName();
        slaveInterface = armPointer->InterfaceName();
    }

    // check if pair already exist and then add
    const std::string name = masterName + "-" + slaveName;
    const TeleopPSMList::iterator teleopIterator = mTeleopsPSM.find(name);
    TeleopPSM * teleopPointer = 0;
    if (teleopIterator == mTeleopsPSM.end()) {
        // create a new teleop if needed
        teleopPointer = new TeleopPSM(name,
                                      masterComponent, masterInterface,
                                      slaveComponent, slaveInterface,
                                      this->GetName());
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
    double period = mtsIntuitiveResearchKit::TeleopPeriod;
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
        arm->ArmInterfaceRequired->AddFunction("SetDesiredState", arm->SetDesiredState);
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
                                      arm->mComponentName, arm->mInterfaceName);
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

    // connect the foot pedals
    this->ConnectFootpedalInterfaces();

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
    TeleopEnable(false);
    const ArmList::iterator end = mArms.end();
    for (ArmList::iterator arm = mArms.begin();
         arm != end;
         ++arm) {
        arm->second->SetDesiredState(std::string("UNINITIALIZED"));
    }
}

void mtsIntuitiveResearchKitConsole::Home(void)
{
    TeleopEnable(false);
    const ArmList::iterator end = mArms.end();
    for (ArmList::iterator arm = mArms.begin();
         arm != end;
         ++arm) {
        arm->second->SetDesiredState(std::string("READY"));
    }
}

void mtsIntuitiveResearchKitConsole::TeleopEnable(const bool & enable)
{
    mTeleopEnabled = enable;
    UpdateTeleopState();
}

void mtsIntuitiveResearchKitConsole::UpdateTeleopState(void)
{
    const ArmList::iterator endArms = mArms.end();
    ArmList::iterator iterArms;

    // Check if teleop is enabled
    if (!mTeleopEnabled) {
        bool freezeNeeded = false;
        const TeleopPSMList::iterator endTeleopPSM = mTeleopsPSM.end();
        for (TeleopPSMList::iterator iterTeleopPSM = mTeleopsPSM.begin();
             iterTeleopPSM != endTeleopPSM;
             ++iterTeleopPSM) {
            iterTeleopPSM->second->SetDesiredState(std::string("DISABLED"));
            if (mTeleopPSMRunning) {
                freezeNeeded = true;
            }
            mTeleopPSMRunning = false;
        }

        if (mTeleopECM) {
            mTeleopECM->SetDesiredState(std::string("DISABLED"));
            if (mTeleopECMRunning) {
                freezeNeeded = true;
            }
            mTeleopECMRunning = false;
        }

        // freeze arms if we stopped any teleop
        if (freezeNeeded) {
            for (iterArms = mArms.begin(); iterArms != endArms; ++iterArms) {
                if ((iterArms->second->mType == Arm::ARM_MTM) ||
                    (iterArms->second->mType == Arm::ARM_MTM_DERIVED) ||
                    (iterArms->second->mType == Arm::ARM_MTM_GENERIC)) {
                    iterArms->second->Freeze();
                }
            }
        }
        return;
    }

    // all fine
    bool readyForTeleop = mOperatorPresent;

    for (iterArms = mArms.begin(); iterArms != endArms; ++iterArms) {
        if (iterArms->second->mSUJClutched) {
            readyForTeleop = false;
        }
    }

    // Check if operator is present
    if (!readyForTeleop) {
        // keep MTMs aligned
        const TeleopPSMList::iterator endTeleopPSM = mTeleopsPSM.end();
        for (TeleopPSMList::iterator iterTeleopPSM = mTeleopsPSM.begin();
             iterTeleopPSM != endTeleopPSM;
             ++iterTeleopPSM) {
            iterTeleopPSM->second->SetDesiredState(std::string("ALIGNING_MTM"));
        }
        mTeleopPSMRunning = false;

        // stop ECM if needed
        if (mTeleopECMRunning) {
            mTeleopECM->SetDesiredState(std::string("DISABLED"));
            mTeleopECMRunning = false;
        }
        return;
    }

    // If camera is pressed for ECM Teleop or not
    if (mCameraPressed) {
        if (!mTeleopECMRunning) {
            // if PSM was running so we need to stop it
            if (mTeleopPSMRunning) {
                const TeleopPSMList::iterator endTeleopPSM = mTeleopsPSM.end();
                for (TeleopPSMList::iterator iterTeleopPSM = mTeleopsPSM.begin();
                     iterTeleopPSM != endTeleopPSM;
                     ++iterTeleopPSM) {
                    iterTeleopPSM->second->SetDesiredState(std::string("DISABLED"));
                }
                mTeleopPSMRunning = false;
            }
            // ECM wasn't running, let's start it
            if (mTeleopECM) {
                mTeleopECM->SetDesiredState(std::string("ENABLED"));
                mTeleopECMRunning = true;
            }
        }
    } else {
        // we must teleop PSM
        if (!mTeleopPSMRunning) {
            // if ECM was running so we need to stop it
            if (mTeleopECMRunning) {
                mTeleopECM->SetDesiredState(std::string("DISABLED"));
                mTeleopECMRunning = false;
            }
            // PSM wasn't running, let's start it
            const TeleopPSMList::iterator endTeleopPSM = mTeleopsPSM.end();
            for (TeleopPSMList::iterator iterTeleopPSM = mTeleopsPSM.begin();
                 iterTeleopPSM != endTeleopPSM;
                 ++iterTeleopPSM) {
                iterTeleopPSM->second->SetDesiredState(std::string("ENABLED"));
            }
            mTeleopPSMRunning = true;
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
    if (button.Type() == prmEventButton::PRESSED) {
        mInterface->SendStatus(this->GetName() + ": clutch pressed");
    } else {
        mInterface->SendStatus(this->GetName() + ": clutch released");
    }
    ConsoleEvents.Clutch(button);
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
    ConsoleEvents.Camera(button);
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
