/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-17

  (C) Copyright 2013-2020 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstCommon/cmnRandomSequence.h>
#include <cisstOSAbstraction/osaDynamicLoader.h>

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawTextToSpeech/mtsTextToSpeech.h>
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
#include <sawIntuitiveResearchKit/mtsDaVinciHeadSensor.h>
#include <sawIntuitiveResearchKit/mtsDaVinciEndoscopeFocus.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationPSM.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationECM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>

#include <json/json.h>

CMN_IMPLEMENT_SERVICES(mtsIntuitiveResearchKitConsole);


mtsIntuitiveResearchKitConsole::Arm::Arm(mtsIntuitiveResearchKitConsole * console,
                                         const std::string & name,
                                         const std::string & ioComponentName):
    mConsole(console),
    mName(name),
    mIOComponentName(ioComponentName),
    mArmPeriod(mtsIntuitiveResearchKit::ArmPeriod),
    IOInterfaceRequired(0),
    PIDInterfaceRequired(0),
    ArmInterfaceRequired(0),
    SUJInterfaceRequiredFromIO(0),
    SUJInterfaceRequiredToSUJ(0),
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
                                                       const std::string & kinematicsConfigFile,
                                                       const double & periodInSeconds)
{
    mType = armType;
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    mArmConfigurationFile = kinematicsConfigFile;
    // for research kit arms, create, add to manager and connect to
    // extra IO, PID, etc.  For generic arms, do nothing.
    switch (armType) {
    case ARM_MTM:
        {
            mtsIntuitiveResearchKitMTM * mtm = new mtsIntuitiveResearchKitMTM(Name(), periodInSeconds);
            if (mSimulation == SIMULATION_KINEMATIC) {
                mtm->SetSimulated();
            }
            mtm->Configure(mArmConfigurationFile);
            SetBaseFrameIfNeeded(mtm);
            componentManager->AddComponent(mtm);
        }
        break;
    case ARM_PSM:
        {
            mtsIntuitiveResearchKitPSM * psm = new mtsIntuitiveResearchKitPSM(Name(), periodInSeconds);
            if (mSimulation == SIMULATION_KINEMATIC) {
                psm->SetSimulated();
            }
            psm->Configure(mArmConfigurationFile);
            SetBaseFrameIfNeeded(psm);
            componentManager->AddComponent(psm);

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
            SetBaseFrameIfNeeded(ecm);
            componentManager->AddComponent(ecm);
        }
        break;
    case ARM_SUJ:
        {
            mtsIntuitiveResearchKitSUJ * suj = new mtsIntuitiveResearchKitSUJ(Name(), periodInSeconds);
            if (mSimulation == SIMULATION_KINEMATIC) {
                suj->SetSimulated();
            }
            suj->Configure(mArmConfigurationFile);
            componentManager->AddComponent(suj);
        }
        break;
    case ARM_MTM_DERIVED:
        {
            mtsComponent * component;
            component = componentManager->GetComponent(Name());
            if (component) {
                mtsIntuitiveResearchKitMTM * mtm = dynamic_cast<mtsIntuitiveResearchKitMTM *>(component);
                if (mtm) {
                    if (mSimulation == SIMULATION_KINEMATIC) {
                        mtm->SetSimulated();
                    }
                    mtm->Configure(mArmConfigurationFile);
                    SetBaseFrameIfNeeded(mtm);
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
                mtsIntuitiveResearchKitPSM * psm = dynamic_cast<mtsIntuitiveResearchKitPSM *>(component);
                if (psm) {
                    if (mSimulation == SIMULATION_KINEMATIC) {
                        psm->SetSimulated();
                    }
                    psm->Configure(mArmConfigurationFile);
                    SetBaseFrameIfNeeded(psm);
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
                    SetBaseFrameIfNeeded(ecm);
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

void mtsIntuitiveResearchKitConsole::Arm::SetBaseFrameIfNeeded(mtsIntuitiveResearchKitArm * armPointer)
{
    if (mBaseFrame.ReferenceFrame() != "") {
        armPointer->SetBaseFrame(mBaseFrame);
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
            componentManager->Connect(Name(), "Dallas",
                                      IOComponentName(), Name() + "-Dallas");
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
        if (mSimulation == SIMULATION_NONE) {
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
        }
        break;
    default:
        break;
    }

    // if the arm is a research kit arm
    if (mIsNativeOrDerived) {
        // Connect arm to IO if not simulated
        if (mSimulation == SIMULATION_NONE) {
            componentManager->Connect(Name(), "RobotIO",
                                      IOComponentName(), Name());
        }
        // connect MTM gripper to IO
        if ((mType == ARM_MTM)
            || (mType == ARM_MTM_DERIVED)) {
            componentManager->Connect(Name(), "GripperIO",
                                      IOComponentName(), Name() + "-Gripper");
        }
        // connect PID
        componentManager->Connect(Name(), "PID",
                                  PIDComponentName(), "Controller");
        // connect m_base_frame if needed
        if ((mBaseFrameComponentName != "") && (mBaseFrameInterfaceName != "")) {
            componentManager->Connect(mBaseFrameComponentName, mBaseFrameInterfaceName,
                                      Name(), "Arm");
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

void mtsIntuitiveResearchKitConsole::Arm::CurrentStateEventHandler(const prmOperatingState & currentState)
{
    mConsole->SetArmCurrentState(mName, currentState);
}

mtsIntuitiveResearchKitConsole::TeleopECM::TeleopECM(const std::string & name):
    mName(name)
{
}

void mtsIntuitiveResearchKitConsole::TeleopECM::ConfigureTeleop(const TeleopECMType type,
                                                                const double & periodInSeconds,
                                                                const Json::Value & jsonConfig)
{
    mType = type;
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    switch (type) {
    case TELEOP_ECM:
        {
            mtsTeleOperationECM * teleop = new mtsTeleOperationECM(mName, periodInSeconds);
            teleop->Configure(jsonConfig);
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
                    teleop->Configure(jsonConfig);
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

const std::string & mtsIntuitiveResearchKitConsole::TeleopECM::Name(void) const {
    return mName;
}


mtsIntuitiveResearchKitConsole::TeleopPSM::TeleopPSM(const std::string & name,
                                                     const std::string & nameMTM,
                                                     const std::string & namePSM):
    mSelected(false),
    mName(name),
    mMTMName(nameMTM),
    mPSMName(namePSM)
{
}

void mtsIntuitiveResearchKitConsole::TeleopPSM::ConfigureTeleop(const TeleopPSMType type,
                                                                const double & periodInSeconds,
                                                                const Json::Value & jsonConfig)
{
    mType = type;
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    switch (type) {
    case TELEOP_PSM:
        {
            mtsTeleOperationPSM * teleop = new mtsTeleOperationPSM(mName, periodInSeconds);
            teleop->Configure(jsonConfig);
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
                    teleop->Configure(jsonConfig);
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

const std::string & mtsIntuitiveResearchKitConsole::TeleopPSM::Name(void) const {
    return mName;
}



mtsIntuitiveResearchKitConsole::mtsIntuitiveResearchKitConsole(const std::string & componentName):
    mtsTaskFromSignal(componentName, 100),
    mConfigured(false),
    mTimeOfLastErrorBeep(0.0),
    mTeleopEnabled(false),
    mTeleopPSMRunning(false),
    mTeleopPSMAligning(false),
    mTeleopECMRunning(false),
    mTeleopMTMToCycle(""),
    mTeleopECM(0),
    mDaVinciHeadSensor(0),
    mDaVinciEndoscopeFocus(0),
    mOperatorPresent(false),
    mCameraPressed(false),
    mIOComponentName("io")
{
    mInterface = AddInterfaceProvided("Main");
    if (mInterface) {
        mInterface->AddMessageEvents();
        mInterface->AddCommandVoid(&mtsIntuitiveResearchKitConsole::PowerOff, this,
                                   "PowerOff");
        mInterface->AddCommandVoid(&mtsIntuitiveResearchKitConsole::PowerOn, this,
                                   "PowerOn");
        mInterface->AddCommandVoid(&mtsIntuitiveResearchKitConsole::Home, this,
                                   "Home");
        mInterface->AddEventWrite(ConfigurationEvents.ArmCurrentState,
                                  "ArmCurrentState", prmKeyValue());
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::TeleopEnable, this,
                                    "TeleopEnable", false);
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::CycleTeleopPSMByMTM, this,
                                    "CycleTeleopPSMByMTM", std::string(""));
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::SelectTeleopPSM, this,
                                    "SelectTeleopPSM", prmKeyValue("mtm", "psm"));
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::SetScale, this,
                                    "SetScale", 0.5);
        mInterface->AddEventWrite(ConfigurationEvents.Scale,
                                  "Scale", 0.5);
        mInterface->AddEventWrite(ConfigurationEvents.TeleopPSMSelected,
                                  "TeleopPSMSelected", prmKeyValue("MTM", "PSM"));
        mInterface->AddEventWrite(ConfigurationEvents.TeleopPSMUnselected,
                                  "TeleopPSMUnselected", prmKeyValue("MTM", "PSM"));
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::SetVolume, this,
                                    "SetVolume", 0.5);
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::StringToSpeech, this,
                                    "StringToSpeech", std::string());
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
        exit(EXIT_FAILURE);
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
            exit(EXIT_FAILURE);
        }
    }

    // add text to speech compoment for the whole system
    mTextToSpeech = new mtsTextToSpeech();
    manager->AddComponent(mTextToSpeech);
    mtsInterfaceRequired * textToSpeechInterface = this->AddInterfaceRequired("TextToSpeech");
    textToSpeechInterface->AddFunction("Beep", mAudio.Beep);
    textToSpeechInterface->AddFunction("StringToSpeech", mAudio.StringToSpeech);
    mAudioVolume = 0.5;

    // IO default settings
    double periodIO = mtsIntuitiveResearchKit::IOPeriod;
    int firewirePort = 0;
    sawRobotIO1394::ProtocolType protocol = sawRobotIO1394::PROTOCOL_SEQ_R_BC_W;
    double watchdogTimeout = mtsIntuitiveResearchKit::WatchdogTimeout;

    jsonValue = jsonConfig["chatty"];
    if (!jsonValue.empty()) {
        mChatty = jsonValue.asBool();
    } else {
        mChatty = false;
    }

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
                exit(EXIT_FAILURE);
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

        jsonValue = jsonConfig["io"]["watchdog-timeout"];
        if (!jsonValue.empty()) {
            watchdogTimeout = jsonValue.asDouble();
            if (watchdogTimeout > 300.0 * cmn_ms) {
                watchdogTimeout = 300.0 * cmn_ms;
                CMN_LOG_CLASS_INIT_WARNING << "Configure: io:watchdog-timeout has to be lower than 300 ms, it has been capped at 300 ms" << std::endl;
            }
            if (watchdogTimeout <= 0.0) {
                watchdogTimeout = 0.0;
                std::stringstream message;
                message << "Configure:" << std::endl
                        << "----------------------------------------------------" << std::endl
                        << " Warning:" << std::endl
                        << "   Setting the watchdog timeout to zero disables the" << std::endl
                        << "   watchdog.   We strongly recommend to no specify" << std::endl
                        << "   io:watchdog-timeout or set it around 10 ms." << std::endl
                        << "----------------------------------------------------";
                std::cerr << "mtsIntuitiveResearchKitConsole::" << message.str() << std::endl;
                CMN_LOG_CLASS_INIT_WARNING << message.str() << std::endl;
            }
        }
    } else {
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using default io:period, io:port, io:firewire-protocol and io:watchdog-timeout" << std::endl;
    }
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure:" << std::endl
                               << "     - Period IO is " << periodIO << std::endl
                               << "     - FireWire port is " << firewirePort << std::endl
                               << "     - Protocol is " << protocol << std::endl
                               << "     - Watchdog timeout is " << watchdogTimeout << std::endl;

    if ((protocol != sawRobotIO1394::PROTOCOL_BC_QRW) && (protocol != sawRobotIO1394::PROTOCOL_SEQ_R_BC_W)) {
        std::stringstream message;
        message << "Configure:" << std::endl
                << "----------------------------------------------------" << std::endl
                << " Warning:" << std::endl
                << "   The firewire-protocol is not using broadcast" << std::endl
                << "   We recommend you set it to \"sequential-read-broadcast-write\"." << std::endl
                << "   You'll need firmware rev. 4 or above for this." << std::endl
                << "----------------------------------------------------";
        std::cerr << "mtsIntuitiveResearchKitConsole::" << message.str() << std::endl;
        CMN_LOG_CLASS_INIT_WARNING << message.str() << std::endl;
    }

    const Json::Value arms = jsonConfig["arms"];
    for (unsigned int index = 0; index < arms.size(); ++index) {
        if (!ConfigureArmJSON(arms[index], mIOComponentName, configPath)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure arms[" << index << "]" << std::endl;
            exit(EXIT_FAILURE);
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
    jsonValue = jsonConfig["operator-present"];
    if (!jsonValue.empty()) {
        // check if operator present uses IO
        Json::Value jsonConfigFile = jsonValue["io"];
        if (!jsonConfigFile.empty()) {
            mHasIO = true;
        }
    }
    // create IO if needed and configure IO
    if (mHasIO) {
        mtsRobotIO1394 * io = new mtsRobotIO1394(mIOComponentName, periodIO, firewirePort);
        io->SetProtocol(protocol);
        io->SetWatchdogPeriod(watchdogTimeout);
        // configure for each arm
        for (iter = mArms.begin(); iter != end; ++iter) {
            std::string ioConfig = iter->second->mIOConfigurationFile;
            if (ioConfig != "") {
                CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring IO using \"" << ioConfig << "\"" << std::endl;
                io->Configure(ioConfig);
            }
            std::string ioGripperConfig = iter->second->mIOGripperConfigurationFile;
            if (ioGripperConfig != "") {
                CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring IO gripper using \"" << ioGripperConfig << "\"" << std::endl;
                io->Configure(ioGripperConfig);
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
                        exit(EXIT_FAILURE);
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
                    exit(EXIT_FAILURE);
                }
                CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring IO foot pedals using \"" << configFile << "\"" << std::endl;
                // these can be overwritten using console-inputs
                mDInputSources["Clutch"] = InterfaceComponentType(mIOComponentName, "Clutch");
                mDInputSources["OperatorPresent"] = InterfaceComponentType(mIOComponentName, "Coag");
                mDInputSources["Coag"] = InterfaceComponentType(mIOComponentName, "Coag");
                mDInputSources["BiCoag"] = InterfaceComponentType(mIOComponentName, "BiCoag");
                mDInputSources["Camera"] = InterfaceComponentType(mIOComponentName, "Camera");
                mDInputSources["Cam-"] = InterfaceComponentType(mIOComponentName, "Cam-");
                mDInputSources["Cam+"] = InterfaceComponentType(mIOComponentName, "Cam+");
                mDInputSources["Head"] = InterfaceComponentType(mIOComponentName, "Head");
                io->Configure(configFile);
            }
        }
        // configure for operator present
        jsonValue = jsonConfig["operator-present"];
        if (!jsonValue.empty()) {
            // check if operator present uses IO
            Json::Value jsonConfigFile = jsonValue["io"];
            if (!jsonConfigFile.empty()) {
                const std::string configFile = configPath.Find(jsonConfigFile.asString());
                if (configFile == "") {
                    CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find configuration file "
                                             << jsonConfigFile.asString() << std::endl;
                    exit(EXIT_FAILURE);
                }
                CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring operator present using \""
                                           << configFile << "\"" << std::endl;
                io->Configure(configFile);
            }
        }
        // configure for endoscope focus
        jsonValue = jsonConfig["endoscope-focus"];
        if (!jsonValue.empty()) {
            // check if operator present uses IO
            Json::Value jsonConfigFile = jsonValue["io"];
            if (!jsonConfigFile.empty()) {
                const std::string configFile = configPath.Find(jsonConfigFile.asString());
                if (configFile == "") {
                    CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find configuration file "
                                             << jsonConfigFile.asString() << std::endl;
                    exit(EXIT_FAILURE);
                }
                CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring endoscope focus using \""
                                           << configFile << "\"" << std::endl;
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

    // look for ECM teleop
    const Json::Value ecmTeleop = jsonConfig["ecm-teleop"];
    if (!ecmTeleop.isNull()) {
        if (!ConfigureECMTeleopJSON(ecmTeleop)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure ecm-teleop" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // now load all PSM teleops
    const Json::Value psmTeleops = jsonConfig["psm-teleops"];
    for (unsigned int index = 0; index < psmTeleops.size(); ++index) {
        if (!ConfigurePSMTeleopJSON(psmTeleops[index])) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure psm-teleops[" << index << "]" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // see which event is used for operator present
    // find name of button event used to detect if operator is present

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

    // load operator-present settings, this will over write older settings
    const Json::Value operatorPresent = jsonConfig["operator-present"];
    if (!operatorPresent.empty()) {
        const std::string headSensorName = "daVinciHeadSensor";
        mDaVinciHeadSensor = new mtsDaVinciHeadSensor(headSensorName);
        mtsComponentManager::GetInstance()->AddComponent(mDaVinciHeadSensor);
        // main DInput is OperatorPresent comming from the newly added component
        mDInputSources["OperatorPresent"] = InterfaceComponentType(headSensorName, "OperatorPresent");
        // also expose the digital inputs from RobotIO (e.g. ROS topics)
        mDInputSources["HeadSensor1"] = InterfaceComponentType(mIOComponentName, "HeadSensor1");
        mDInputSources["HeadSensor2"] = InterfaceComponentType(mIOComponentName, "HeadSensor2");
        mDInputSources["HeadSensor3"] = InterfaceComponentType(mIOComponentName, "HeadSensor3");
        mDInputSources["HeadSensor4"] = InterfaceComponentType(mIOComponentName, "HeadSensor4");
        // schedule connections
        mConnections.Add(headSensorName, "HeadSensorTurnOff",
                         mIOComponentName, "HeadSensorTurnOff");
        mConnections.Add(headSensorName, "HeadSensor1",
                         mIOComponentName, "HeadSensor1");
        mConnections.Add(headSensorName, "HeadSensor2",
                         mIOComponentName, "HeadSensor2");
        mConnections.Add(headSensorName, "HeadSensor3",
                         mIOComponentName, "HeadSensor3");
        mConnections.Add(headSensorName, "HeadSensor4",
                         mIOComponentName, "HeadSensor4");

    }

    // load endoscope-focus settings
    const Json::Value endoscopeFocus = jsonConfig["endoscope-focus"];
    if (!endoscopeFocus.empty()) {
        const std::string endoscopeFocusName = "daVinciEndoscopeFocus";
        mDaVinciEndoscopeFocus = new mtsDaVinciEndoscopeFocus(endoscopeFocusName);
        mtsComponentManager::GetInstance()->AddComponent(mDaVinciEndoscopeFocus);
        // make sure we have cam+ and cam- in digital inputs
        const DInputSourceType::const_iterator endDInputs = mDInputSources.end();
        const bool foundCamMinus = (mDInputSources.find("Cam-") != endDInputs);
        if (!foundCamMinus) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: input for footpedal \"Cam-\" is required for \"endoscope-focus\".  Maybe you're missing \"io\":\"footpedals\" in your configuration file." << std::endl;
            exit(EXIT_FAILURE);
        }
        const bool foundCamPlus = (mDInputSources.find("Cam+") != endDInputs);
        if (!foundCamPlus) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: input for footpedal \"Cam+\" is required for \"endoscope-focus\".  Maybe you're missing \"io\":\"footpedals\" in your configuration file." << std::endl;
            exit(EXIT_FAILURE);
        }
        // schedule connections
        mConnections.Add(endoscopeFocusName, "EndoscopeFocusIn",
                         mIOComponentName, "EndoscopeFocusIn");
        mConnections.Add(endoscopeFocusName, "EndoscopeFocusOut",
                         mIOComponentName, "EndoscopeFocusOut");
        mConnections.Add(endoscopeFocusName, "FocusIn",
                         mIOComponentName, "Cam+");
        mConnections.Add(endoscopeFocusName, "FocusOut",
                         mIOComponentName, "Cam-");
    }

    // if we have any teleoperation component, we need to have the interfaces for the foot pedals
    const DInputSourceType::const_iterator endDInputs = mDInputSources.end();
    const bool foundClutch = (mDInputSources.find("Clutch") != endDInputs);
    const bool foundOperatorPresent = (mDInputSources.find("OperatorPresent") != endDInputs);
    const bool foundCamera = (mDInputSources.find("Camera") != endDInputs);

    if (mTeleopsPSM.size() > 0) {
        if (!foundClutch || !foundOperatorPresent) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: inputs for footpedals \"Clutch\" and \"OperatorPresent\" need to be defined since there's at least one PSM tele-operation component.  Maybe you're missing \"io\":\"footpedals\" in your configuration file." << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    if (mTeleopECM) {
        if (!foundCamera || !foundOperatorPresent) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: inputs for footpedals \"Camera\" and \"OperatorPresent\" need to be defined since there's an ECM tele-operation component.  Maybe you're missing \"io\":\"footpedals\" in your configuration file." << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    this->AddFootpedalInterfaces();

    // search for SUJs
    bool hasSUJ = false;
    for (iter = mArms.begin(); iter != end; ++iter) {
        if (iter->second->mType == Arm::ARM_SUJ) {
            hasSUJ = true;
        }
    }

    if (hasSUJ) {
        for (iter = mArms.begin(); iter != end; ++iter) {
            Arm * arm = iter->second;
            // only for PSM and ECM when not simulated
            if (((arm->mType == Arm::ARM_ECM)
                 || (arm->mType == Arm::ARM_ECM_DERIVED)
                 || (arm->mType == Arm::ARM_PSM)
                 || (arm->mType == Arm::ARM_PSM_DERIVED)
                 )
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

    if (mChatty) {
        // someone is going to hate me for this :-)
        std::vector<std::string> prompts;
        prompts.push_back("Hello");
        prompts.push_back("It will not work!");
        prompts.push_back("It might work");
        prompts.push_back("It will work!");
        prompts.push_back("Are we there yet?");
        prompts.push_back("When is that paper deadline?");
        prompts.push_back("Don't you have something better to do?");
        prompts.push_back("Today is the day!");
        prompts.push_back("It's free software, what did you expect?");
        prompts.push_back("I didn't do it!");
        prompts.push_back("Be careful!");
        prompts.push_back("Peter will fix it");
        prompts.push_back("Ask Google");
        prompts.push_back("Did you forget to re-compile?");
        prompts.push_back("Reboot me");
        prompts.push_back("Coffee break?");
        prompts.push_back("If you can hear this, the code compiles!");
        prompts.push_back("I'm a bit tired");
        prompts.push_back("Commit often, always pull!");
        prompts.push_back("Feel free to fix it");
        prompts.push_back("What's the weather like outside?");
        prompts.push_back("Call your parents");
        int index;
        cmnRandomSequence & randomSequence = cmnRandomSequence::GetInstance();
        cmnRandomSequence::SeedType seed
            = static_cast<cmnRandomSequence::SeedType>(mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime() * 100000.0);
        randomSequence.SetSeed(seed % 1000);
        randomSequence.ExtractRandomValue<int>(0, prompts.size() - 1, index);
        mAudio.StringToSpeech(prompts.at(index));
    }
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
    Arm * newArm = new Arm(this, genericArm->GetName(), "");
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

bool mtsIntuitiveResearchKitConsole::AddTeleopECMInterfaces(TeleopECM * teleop)
{
    teleop->InterfaceRequired = this->AddInterfaceRequired(teleop->Name());
    if (teleop->InterfaceRequired) {
        teleop->InterfaceRequired->AddFunction("state_command", teleop->state_command);
        teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler, this, "error");
        teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler, this, "warning");
        teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler, this, "status");
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
        teleop->InterfaceRequired->AddFunction("state_command", teleop->state_command);
        teleop->InterfaceRequired->AddFunction("SetScale", teleop->SetScale);
        teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler, this, "error");
        teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler, this, "warning");
        teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler, this, "status");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "AddTeleopPSMInterfaces: failed to add Main interface for teleop \""
                                 << teleop->Name() << "\"" << std::endl;
        return false;
    }
    return true;
}

void mtsIntuitiveResearchKitConsole::AddFootpedalInterfaces(void)
{
    const auto endDInputs = mDInputSources.end();

    auto iter = mDInputSources.find("Clutch");
    if (iter != endDInputs) {
        mtsInterfaceRequired * clutchRequired = AddInterfaceRequired("Clutch");
        if (clutchRequired) {
            clutchRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ClutchEventHandler, this, "Button");
        }
        mConnections.Add(this->GetName(), "Clutch",
                         iter->second.first, iter->second.second);
        mtsInterfaceProvided * clutchProvided = AddInterfaceProvided("Clutch");
        if (clutchProvided) {
            clutchProvided->AddEventWrite(ConsoleEvents.Clutch, "Button", prmEventButton());
        }
    }

    iter = mDInputSources.find("Camera");
    if (iter != endDInputs) {
        mtsInterfaceRequired * cameraRequired = AddInterfaceRequired("Camera");
        if (cameraRequired) {
            cameraRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::CameraEventHandler, this, "Button");
        }
        mConnections.Add(this->GetName(), "Camera",
                         iter->second.first, iter->second.second);
        mtsInterfaceProvided * cameraProvided = AddInterfaceProvided("Camera");
        if (cameraProvided) {
            cameraProvided->AddEventWrite(ConsoleEvents.Camera, "Button", prmEventButton());
        }
    }

    iter = mDInputSources.find("OperatorPresent");
    if (iter != endDInputs) {
        mtsInterfaceRequired * operatorRequired = AddInterfaceRequired("OperatorPresent");
        if (operatorRequired) {
            operatorRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::OperatorPresentEventHandler, this, "Button");
        }
        mConnections.Add(this->GetName(), "OperatorPresent",
                         iter->second.first, iter->second.second);
        mtsInterfaceProvided * operatorProvided = AddInterfaceProvided("OperatorPresent");
        if (operatorProvided) {
            operatorProvided->AddEventWrite(ConsoleEvents.OperatorPresent, "Button", prmEventButton());
        }
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
        armPointer = new Arm(this, armName, ioComponentName);
    } else {
        armPointer = armIterator->second;
    }

    // read from JSON and check if configuration files exist
    Json::Value jsonValue;
    jsonValue = jsonArm["type"];
    armPointer->mIsGeneric = false; // default value
    armPointer->mIsNativeOrDerived = false;
    if (!jsonValue.empty()) {
        std::string typeString = jsonValue.asString();
        if (typeString == "MTM") {
            armPointer->mType = Arm::ARM_MTM;
            armPointer->mIsNativeOrDerived = true;
        } else if (typeString == "PSM") {
            armPointer->mType = Arm::ARM_PSM;
            armPointer->mIsNativeOrDerived = true;
        } else if (typeString == "ECM") {
            armPointer->mType = Arm::ARM_ECM;
            armPointer->mIsNativeOrDerived = true;
        } else if (typeString == "MTM_DERIVED") {
            armPointer->mType = Arm::ARM_MTM_DERIVED;
            armPointer->mIsNativeOrDerived = true;
        } else if (typeString == "PSM_DERIVED") {
            armPointer->mType = Arm::ARM_PSM_DERIVED;
            armPointer->mIsNativeOrDerived = true;
        } else if (typeString == "ECM_DERIVED") {
            armPointer->mType = Arm::ARM_ECM_DERIVED;
            armPointer->mIsNativeOrDerived = true;
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
                                     << typeString << "\", needs to be one of {MTM,PSM,ECM}{,_DERIVED,_GENERIC}" << std::endl;
            return false;
        }
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: arm " << armName
                                 << ": doesn't have a \"type\" specified, needs to be one of {MTM,PSM,ECM}{,_DERIVED,_GENERIC}" << std::endl;
        return false;
    }

    jsonValue = jsonArm["serial"];
    if (!jsonValue.empty()) {
        armPointer->mSerial = jsonValue.asString();
    }

    // type of simulation, if any
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

    // should we automatically create ROS bridge for this arm
    armPointer->mSkipROSBridge = false;
    jsonValue = jsonArm["skip-ros-bridge"];
    if (!jsonValue.empty()) {
        armPointer->mSkipROSBridge = jsonValue.asBool();
    }

    // component and interface, defaults
    armPointer->mComponentName = armName;
    armPointer->mInterfaceName = "Arm";
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
                // try to find default if serial number has been provided
                if (armPointer->mSerial != "") {
                    std::string defaultFile = "sawRobotIO1394-" + armName + "-" + armPointer->mSerial + ".xml";
                    armPointer->mIOConfigurationFile = configPath.Find(defaultFile);
                    if (armPointer->mIOConfigurationFile == "") {
                        CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find IO file " << defaultFile << std::endl;
                        return false;
                    }
                } else {
                    // no io nor serial
                    CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"io\" setting for arm \""
                                             << armName << "\" and \"serial\" is not provided so we can't search for it" << std::endl;
                    return false;
                }
            }
            // IO for MTM gripper
            if ((armPointer->mType == Arm::ARM_MTM)
                || (armPointer->mType == Arm::ARM_MTM_DERIVED)) {
                jsonValue = jsonArm["io-gripper"];
                if (!jsonValue.empty()) {
                    armPointer->mIOGripperConfigurationFile = configPath.Find(jsonValue.asString());
                    if (armPointer->mIOGripperConfigurationFile == "") {
                        CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find IO gripper file "
                                                 << jsonValue.asString() << std::endl;
                        return false;
                    }
                } else {
                    // try to find default if serial number has been provided
                    if (armPointer->mSerial != "") {
                        std::string defaultFile = "sawRobotIO1394-" + armName + "-gripper-" + armPointer->mSerial + ".xml";
                        armPointer->mIOGripperConfigurationFile = configPath.Find(defaultFile);
                        if (armPointer->mIOGripperConfigurationFile == "") {
                            CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find IO gripper file " << defaultFile << std::endl;
                            return false;
                        }
                    } else {
                        // no io nor serial
                        CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"io-gripper\" setting for arm \""
                                                 << armName << "\" and \"serial\" is not provided so we can't search for it" << std::endl;
                        return false;
                    }
                }
            }
        }
    }

    // PID only required for MTM, PSM and ECM (and derived)
    if (armPointer->mIsNativeOrDerived) {
        jsonValue = jsonArm["pid"];
        if (!jsonValue.empty()) {
            armPointer->mPIDConfigurationFile = configPath.Find(jsonValue.asString());
            if (armPointer->mPIDConfigurationFile == "") {
                CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find PID file " << jsonValue.asString() << std::endl;
                return false;
            }
        } else {
            // try to find default
            std::string defaultFile;
            if ((armPointer->mType == Arm::ARM_PSM) || (armPointer->mType == Arm::ARM_PSM_DERIVED)) {
                defaultFile = "pid/sawControllersPID-PSM.xml";
            } else if ((armPointer->mType == Arm::ARM_ECM) || (armPointer->mType == Arm::ARM_ECM_DERIVED)) {
                defaultFile = "pid/sawControllersPID-ECM.xml";
            } else {
                defaultFile = "pid/sawControllersPID-" + armName + ".xml";
            }
            CMN_LOG_CLASS_INIT_VERBOSE << "ConfigureArmJSON: can't find \"pid\" setting for arm \""
                                       << armName << "\", using default: \""
                                       << defaultFile << "\"" << std::endl;
            armPointer->mPIDConfigurationFile = configPath.Find(defaultFile);
            if (armPointer->mPIDConfigurationFile == "") {
                CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find PID file " << defaultFile << std::endl;
                return false;
            }
        }
    }

    // only configure kinematics if not arm socket client
    if ((armPointer->mType != Arm::ARM_PSM_SOCKET)
        && (!armPointer->mIsGeneric)) {
        // renamed "kinematic" to "arm" so we can have a more complex configuration file for the arm class
        if (armPointer->mIsNativeOrDerived) {
            jsonValue = jsonArm["arm"];
            if (!jsonValue.empty()) {
                armPointer->mArmConfigurationFile = configPath.Find(jsonValue.asString());
                if (armPointer->mArmConfigurationFile == "") {
                    CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find configuration file " << jsonValue.asString() << std::endl;
                    return false;
                }
            }
        }
        jsonValue = jsonArm["kinematic"];
        if (!jsonValue.empty()) {
            if (armPointer->mArmConfigurationFile != "") {
                CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: arm configuration file is already set using \"arm\", you should remove the deprecated \"kinetic\" field:"
                                         << jsonValue.asString() << std::endl;
                return false;
            } else {
                armPointer->mArmConfigurationFile = configPath.Find(jsonValue.asString());
                if (armPointer->mArmConfigurationFile == "") {
                    CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find Kinematic file " << jsonValue.asString() << std::endl;
                    return false;
                }
            }
        }

        // make sure we have an arm configuration file
        if (armPointer->mArmConfigurationFile == "") {
            if (armPointer->mIsNativeOrDerived) {
                // try to find the arm file using default
                std::string defaultFile = armName + "-" + armPointer->mSerial + ".json";
                armPointer->mArmConfigurationFile = configPath.Find(defaultFile);
                if (armPointer->mArmConfigurationFile == "") {
                    CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"arm\" setting for arm \""
                                             << armName << "\".  \"arm\" is not set and the default file \""
                                             << defaultFile << "\" doesn't seem to exist either." << std::endl;
                    return false;
                }
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"kinematic\" setting for arm \""
                                         << armName << "\"" << std::endl;
                return false;
            }
        }

        jsonValue = jsonArm["base-frame"];
        if (!jsonValue.empty()) {
            Json::Value fixedJson = jsonValue["transform"];
            if (!fixedJson.empty()) {
                std::string reference = jsonValue["reference-frame"].asString();
                if (reference.empty()) {
                    CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: both \"transform\" (4x4) and \"reference-frame\" (name) must be provided with \"base-frame\" for arm \""
                                             << armName << "\"" << std::endl;
                    return false;
                }
                vctFrm4x4 frame;
                cmnDataJSON<vctFrm4x4>::DeSerializeText(frame, fixedJson);
                armPointer->mBaseFrame.Goal().From(frame);
                armPointer->mBaseFrame.ReferenceFrame() = reference;
                armPointer->mBaseFrame.Valid() = true;
            } else {
                armPointer->mBaseFrameComponentName = jsonValue.get("component", "").asString();
                armPointer->mBaseFrameInterfaceName = jsonValue.get("interface", "").asString();
                if ((armPointer->mBaseFrameComponentName == "")
                    || (armPointer->mBaseFrameInterfaceName == "")) {
                    CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: both \"component\" and \"interface\" OR \"transform\" (4x4) and \"reference-frame\" (name) must be provided with \"base-frame\" for arm \""
                                             << armName << "\"" << std::endl;
                    return false;
                }
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
    std::string mtmLeftName = jsonTeleop["mtm-left"].asString();
    // for backward compatibility
    if (mtmLeftName == "") {
        mtmLeftName = jsonTeleop["master-left"].asString();
        CMN_LOG_CLASS_INIT_WARNING << "ConfigureECMTeleopJSON: keyword \"master-left\" is deprecated, use \"mtm-left\" instead" << std::endl;
    }
    std::string mtmRightName = jsonTeleop["mtm-right"].asString();
    // for backward compatibility
    if (mtmRightName == "") {
        mtmRightName = jsonTeleop["master-right"].asString();
        CMN_LOG_CLASS_INIT_WARNING << "ConfigureECMTeleopJSON: keyword \"master-right\" is deprecated, use \"mtm-right\" instead" << std::endl;
    }
    std::string ecmName = jsonTeleop["ecm"].asString();
    // for backward compatibility
    if (ecmName == "") {
        ecmName = jsonTeleop["slave"].asString();
        CMN_LOG_CLASS_INIT_WARNING << "ConfigureECMTeleopJSON: keyword \"slave\" is deprecated, use \"ecm\" instead" << std::endl;
    }
    // all must be provided
    if ((mtmLeftName == "") || (mtmRightName == "") || (ecmName == "")) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: \"mtm-left\", \"mtm-right\" and \"ecm\" must be provided as strings" << std::endl;
        return false;
    }

    if (mtmLeftName == mtmRightName) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: \"mtm-left\" and \"mtm-right\" must be different" << std::endl;
        return false;
    }
    std::string
        mtmLeftComponent, mtmLeftInterface,
        mtmRightComponent, mtmRightInterface,
        ecmComponent, ecmInterface;
    // check that both arms have been defined and have correct type
    Arm * armPointer;
    ArmList::iterator armIterator = mArms.find(mtmLeftName);
    if (armIterator == mArms.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: mtm left\""
                                 << mtmLeftName << "\" is not defined in \"arms\"" << std::endl;
        return false;
    } else {
        armPointer = armIterator->second;
        if (!((armPointer->mType == Arm::ARM_MTM_GENERIC) ||
              (armPointer->mType == Arm::ARM_MTM_DERIVED) ||
              (armPointer->mType == Arm::ARM_MTM))) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: mtm left\""
                                     << mtmLeftName << "\" type must be \"MTM\", \"MTM_DERIVED\" or \"MTM_GENERIC\"" << std::endl;
            return false;
        }
        mtmLeftComponent = armPointer->ComponentName();
        mtmLeftInterface = armPointer->InterfaceName();
    }
    armIterator = mArms.find(mtmRightName);
    if (armIterator == mArms.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: mtm right\""
                                 << mtmRightName << "\" is not defined in \"arms\"" << std::endl;
        return false;
    } else {
        armPointer = armIterator->second;
        if (!((armPointer->mType == Arm::ARM_MTM_GENERIC) ||
              (armPointer->mType == Arm::ARM_MTM_DERIVED) ||
              (armPointer->mType == Arm::ARM_MTM))) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: mtm right\""
                                     << mtmRightName << "\" type must be \"MTM\", \"MTM_DERIVED\" or \"MTM_GENERIC\"" << std::endl;
            return false;
        }
        mtmRightComponent = armPointer->ComponentName();
        mtmRightInterface = armPointer->InterfaceName();
    }
    armIterator = mArms.find(ecmName);
    if (armIterator == mArms.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: ecm \""
                                 << ecmName << "\" is not defined in \"arms\"" << std::endl;
        return false;
    } else {
        armPointer = armIterator->second;
        if (!((armPointer->mType == Arm::ARM_ECM_GENERIC) ||
              (armPointer->mType == Arm::ARM_ECM_DERIVED) ||
              (armPointer->mType == Arm::ARM_ECM))) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: ecm \""
                                     << ecmName << "\" type must be \"ECM\" or \"GENERIC_ECM\"" << std::endl;
            return false;
        }
        ecmComponent = armPointer->ComponentName();
        ecmInterface = armPointer->InterfaceName();
    }

    // check if pair already exist and then add
    const std::string name = mtmLeftName + "-" + mtmRightName + "-" + ecmName;
    if (mTeleopECM == 0) {
        // create a new teleop if needed
        mTeleopECM = new TeleopECM(name);
        // schedule connections
        mConnections.Add(name, "MTML", mtmLeftComponent, mtmLeftInterface);
        mConnections.Add(name, "MTMR", mtmRightComponent, mtmRightInterface);
        mConnections.Add(name, "ECM", ecmComponent, ecmInterface);
        mConnections.Add(name, "Clutch", this->GetName(), "Clutch"); // console clutch
        mConnections.Add(this->GetName(), name, name, "Setting");
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

    // read period if present
    double period = mtsIntuitiveResearchKit::TeleopPeriod;
    jsonValue = jsonTeleop["period"];
    if (!jsonValue.empty()) {
        period = jsonValue.asFloat();
    }
    // for backward compatibility, send warning
    jsonValue = jsonTeleop["rotation"];
    if (!jsonValue.empty()) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: teleop " << name << ": \"rotation\" must now be defined under \"configure-parameter\" or in a separate configuration file" << std::endl;
        return false;
    }
    const Json::Value jsonTeleopConfig = jsonTeleop["configure-parameter"];
    mTeleopECM->ConfigureTeleop(mTeleopECM->mType, period, jsonTeleopConfig);
    AddTeleopECMInterfaces(mTeleopECM);
    return true;
}

bool mtsIntuitiveResearchKitConsole::ConfigurePSMTeleopJSON(const Json::Value & jsonTeleop)
{
    std::string mtmName = jsonTeleop["mtm"].asString();
    // for backward compatibility
    if (mtmName == "") {
        mtmName = jsonTeleop["master"].asString();
        CMN_LOG_CLASS_INIT_WARNING << "ConfigurePSMTeleopJSON: keyword \"master\" is deprecated, use \"mtm\" instead" << std::endl;
    }
    std::string psmName = jsonTeleop["psm"].asString();
    // for backward compatibility
    if (psmName == "") {
        psmName = jsonTeleop["slave"].asString();
        CMN_LOG_CLASS_INIT_WARNING << "ConfigurePSMTeleopJSON: keyword \"slave\" is deprecated, use \"psm\" instead" << std::endl;
    }
    // both are required
    if ((mtmName == "") || (psmName == "")) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: both \"mtm\" and \"psm\" must be provided as strings" << std::endl;
        return false;
    }

    std::string mtmComponent, mtmInterface, psmComponent, psmInterface;
    // check that both arms have been defined and have correct type
    Arm * armPointer;
    ArmList::iterator armIterator = mArms.find(mtmName);
    if (armIterator == mArms.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: mtm \""
                                 << mtmName << "\" is not defined in \"arms\"" << std::endl;
        return false;
    } else {
        armPointer = armIterator->second;
        if (!((armPointer->mType == Arm::ARM_MTM_GENERIC) ||
              (armPointer->mType == Arm::ARM_MTM_DERIVED) ||
              (armPointer->mType == Arm::ARM_MTM))) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: mtm \""
                                     << mtmName << "\" type must be \"MTM\", \"MTM_DERIVED\" or \"MTM_GENERIC\"" << std::endl;
            return false;
        }
        mtmComponent = armPointer->ComponentName();
        mtmInterface = armPointer->InterfaceName();
    }
    armIterator = mArms.find(psmName);
    if (armIterator == mArms.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: psm \""
                                 << psmName << "\" is not defined in \"arms\"" << std::endl;
        return false;
    } else {
        armPointer = armIterator->second;
        if (!((armPointer->mType == Arm::ARM_PSM_GENERIC) ||
              (armPointer->mType == Arm::ARM_PSM_DERIVED) ||
              (armPointer->mType == Arm::ARM_PSM_SOCKET)  ||
              (armPointer->mType == Arm::ARM_PSM))) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: psm \""
                                     << psmName << "\" type must be \"PSM\", \"PSM_DERIVED\" or \"PSM_GENERIC\"" << std::endl;
            return false;
        }
        psmComponent = armPointer->ComponentName();
        psmInterface = armPointer->InterfaceName();
    }

    // see if there is a base frame defined for the PSM
    Json::Value jsonValue = jsonTeleop["psm-base-frame"];
    std::string baseFrameComponent, baseFrameInterface;
    if (!jsonValue.empty()) {
        baseFrameComponent = jsonValue.get("component", "").asString();
        baseFrameInterface = jsonValue.get("interface", "").asString();
        if ((baseFrameComponent == "") || (baseFrameInterface == "")) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: both \"component\" and \"interface\" must be provided with \"psm-base-frame\" for teleop \""
                                     << mtmName << "-" << psmName << "\"" << std::endl;
            return false;
        }
    }

    // check if pair already exist and then add
    const std::string name = mtmName + "-" + psmName;
    const TeleopPSMList::iterator teleopIterator = mTeleopsPSM.find(name);
    TeleopPSM * teleopPointer = 0;
    if (teleopIterator == mTeleopsPSM.end()) {
        // create a new teleop if needed
        teleopPointer = new TeleopPSM(name, mtmName, psmName);
        // schedule connections
        mConnections.Add(name, "MTM", mtmComponent, mtmInterface);
        mConnections.Add(name, "PSM", psmComponent, psmInterface);
        mConnections.Add(name, "Clutch", this->GetName(), "Clutch"); // clutch from console
        mConnections.Add(this->GetName(), name, name, "Setting");
        if ((baseFrameComponent != "")
            && (baseFrameInterface != "")) {
            mConnections.Add(name, "PSM-base-frame",
                             baseFrameComponent, baseFrameInterface);
        }

        mTeleopsPSMByMTM.insert(std::make_pair(mtmName, teleopPointer));
        // first MTM with multiple PSMs is selected for single tap
        if ((mTeleopsPSMByMTM.count(mtmName) > 1)
            && (mTeleopMTMToCycle == "")) {
            mTeleopMTMToCycle = mtmName;
        }
        // check if we already have a teleop for the same PSM
        std::string mtmUsingThatPSM;
        GetMTMSelectedForPSM(psmName, mtmUsingThatPSM);
        if (mtmUsingThatPSM != "") {
            teleopPointer->SetSelected(false);
            CMN_LOG_CLASS_INIT_WARNING << "ConfigurePSMTeleopJSON: psm \""
                                       << psmName << "\" is already selected to be controlled by mtm \""
                                       << mtmUsingThatPSM << "\", component \""
                                       << name << "\" is added but not selected"
                                       << std::endl;
        } else {
            // check if we already have a teleop for the same PSM
            std::string psmUsingThatMTM;
            GetPSMSelectedForMTM(mtmName, psmUsingThatMTM);
            if (psmUsingThatMTM != "") {
                teleopPointer->SetSelected(false);
                CMN_LOG_CLASS_INIT_WARNING << "ConfigurePSMTeleopJSON: mtm \""
                                           << mtmName << "\" is already selected to control psm \""
                                           << psmUsingThatMTM << "\", component \""
                                           << name << "\" is added but not selected"
                                           << std::endl;
            } else {
                // neither the MTM nor PSM are used, let's activate that pair
                teleopPointer->SetSelected(true);
            }
        }
        // finally add the new teleop
        mTeleopsPSM[name] = teleopPointer;
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: there is already a teleop for the pair \""
                                 << name << "\"" << std::endl;
        return false;
    }

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

    // read period if present
    double period = mtsIntuitiveResearchKit::TeleopPeriod;
    jsonValue = jsonTeleop["period"];
    if (!jsonValue.empty()) {
        period = jsonValue.asFloat();
    }
    // for backward compatibility, send warning
    jsonValue = jsonTeleop["rotation"];
    if (!jsonValue.empty()) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: teleop " << name << ": \"rotation\" must now be defined under \"configure-parameter\" or in a separate configuration file" << std::endl;
        return false;
    }
    const Json::Value jsonTeleopConfig = jsonTeleop["configure-parameter"];
    teleopPointer->ConfigureTeleop(teleopPointer->mType, period, jsonTeleopConfig);
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
                                                           this, "error");
            arm->IOInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                           this, "warning");
            arm->IOInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                           this, "status");
            mConnections.Add(this->GetName(), interfaceNameIO,
                             arm->IOComponentName(), arm->Name());
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "AddArmInterfaces: failed to add IO interface for arm \""
                                     << arm->Name() << "\"" << std::endl;
            return false;
        }
        // is the arm is a PSM, since it has an IO, it also has a
        // Dallas chip interface and we want to see the messages
        if ((arm->mType == Arm::ARM_PSM)
            || (arm->mType == Arm::ARM_PSM_DERIVED)) {
            const std::string interfaceNameIODallas = "IO-Dallas-" + arm->Name();
            arm->IODallasInterfaceRequired = AddInterfaceRequired(interfaceNameIODallas);
            if (arm->IODallasInterfaceRequired) {
                arm->IODallasInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
                                                                     this, "error");
                arm->IODallasInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                                     this, "warning");
                arm->IODallasInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                                     this, "status");
                mConnections.Add(this->GetName(), interfaceNameIODallas,
                                 arm->IOComponentName(), arm->Name() + "-Dallas");
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "AddArmInterfaces: failed to add IO Dallase interface for arm \""
                                         << arm->Name() << "\"" << std::endl;
                return false;
            }
        }
    }

    // PID
    if (!arm->mPIDConfigurationFile.empty()) {
        const std::string interfaceNamePID = "PID-" + arm->Name();
        arm->PIDInterfaceRequired = AddInterfaceRequired(interfaceNamePID);
        if (arm->PIDInterfaceRequired) {
            arm->PIDInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
                                                            this, "error");
            arm->PIDInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                            this, "warning");
            arm->PIDInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                            this, "status");
            mConnections.Add(this->GetName(), interfaceNamePID,
                             arm->Name() + "-PID", "Controller");
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
        arm->ArmInterfaceRequired->AddFunction("state_command", arm->state_command);
        if (arm->mType != Arm::ARM_SUJ) {
            arm->ArmInterfaceRequired->AddFunction("Freeze", arm->Freeze);
        }
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
                                                        this, "error");
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                        this, "warning");
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                        this, "status");
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::Arm::CurrentStateEventHandler,
                                                        arm, "operating_state");
        mConnections.Add(this->GetName(), interfaceNameArm,
                         arm->ComponentName(), arm->InterfaceName());
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "AddArmInterfaces: failed to add Main interface for arm \""
                                 << arm->Name() << "\"" << std::endl;
        return false;
    }
    return true;
}

bool mtsIntuitiveResearchKitConsole::Connect(void)
{
    mConnections.Connect();

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // connect console for audio feedback
    componentManager->Connect(this->GetName(), "TextToSpeech",
                              mTextToSpeech->GetName(), "Commands");

    for (auto & armIter : mArms) {
        Arm * arm = armIter.second;
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

    return true;
}

void mtsIntuitiveResearchKitConsole::PowerOff(void)
{
    TeleopEnable(false);
    for (auto & arm : mArms) {
        arm.second->state_command(std::string("disable"));
    }
}

void mtsIntuitiveResearchKitConsole::PowerOn(void)
{
    DisableFaultyArms();
    for (auto & arm : mArms) {
        arm.second->state_command(std::string("enable"));
    }
}

void mtsIntuitiveResearchKitConsole::Home(void)
{
    DisableFaultyArms();
    TeleopEnable(false);
    for (auto & arm : mArms) {
        arm.second->state_command(std::string("home"));
    }
}

void mtsIntuitiveResearchKitConsole::DisableFaultyArms(void)
{
    for (auto & arm : mArms) {
        auto armState = ArmStates.find(arm.first);
        if (// search if we already have a state
            (armState != ArmStates.end())
            // and the arm is faulty
            && (armState->second.State() == prmOperatingState::FAULT) ) {
            arm.second->state_command(std::string("disable"));
        }
    }
}

void mtsIntuitiveResearchKitConsole::TeleopEnable(const bool & enable)
{
    // for convenience, if we start teleop we assume all arms should
    // be homed too
    if (enable) {
        Home();
    }
    mTeleopEnabled = enable;
    UpdateTeleopState();
}

void mtsIntuitiveResearchKitConsole::CycleTeleopPSMByMTM(const std::string & mtmName)
{
    // try to cycle through all the teleopPSMs associated to the MTM
    if (mTeleopsPSMByMTM.count(mtmName) == 0) {
        // we use empty string to query, no need to send warning about bad mtm name
        if (mtmName != "") {
            mInterface->SendWarning(this->GetName()
                                    + ": no PSM teleoperation found for MTM \""
                                    + mtmName
                                    + "\"");
        }
    } else if (mTeleopsPSMByMTM.count(mtmName) == 1) {
        mInterface->SendStatus(this->GetName()
                               + ": only one PSM teleoperation found for MTM \""
                               + mtmName
                               + "\", cycling has no effect");
    } else {
        // find range of teleops
        std::pair<TeleopPSMByMTMIterator, TeleopPSMByMTMIterator> range;
        range = mTeleopsPSMByMTM.equal_range(mtmName);
        for (TeleopPSMByMTMIterator iter = range.first;
             iter != range.second;
             ++iter) {
            // find first teleop currently selected
            if (iter->second->Selected()) {
                // toggle to next one
                TeleopPSMByMTMIterator nextTeleop = iter;
                nextTeleop++;
                // if next one is last one, go back to first
                if (nextTeleop == range.second) {
                    nextTeleop = range.first;
                }
                // now make sure the PSM in next teleop is not used
                std::string mtmUsingThatPSM;
                GetMTMSelectedForPSM(nextTeleop->second->mPSMName, mtmUsingThatPSM);
                if (mtmUsingThatPSM != "") {
                    // message
                    mInterface->SendWarning(this->GetName()
                                            + ": cycling from \""
                                            + iter->second->mName
                                            + "\" to \""
                                            + nextTeleop->second->mName
                                            + "\" failed, PSM is already controlled by \""
                                            + mtmUsingThatPSM
                                            + "\"");
                } else {
                    // mark which one should be active
                    iter->second->SetSelected(false);
                    nextTeleop->second->SetSelected(true);
                    // if teleop PSM is active, enable/disable components now
                    if (mTeleopEnabled) {
                        iter->second->state_command(std::string("disable"));
                        if (mTeleopPSMRunning) {
                            nextTeleop->second->state_command(std::string("enable"));
                        } else {
                            nextTeleop->second->state_command(std::string("align_mtm"));
                        }
                    }
                    // message
                    mInterface->SendStatus(this->GetName()
                                           + ": cycling from \""
                                           + iter->second->mName
                                           + "\" to \""
                                           + nextTeleop->second->mName
                                           + "\"");
                }
                // stop for loop
                break;
            }
        }
    }
    // in all cases, emit events so users can figure out which components are selected
    EventSelectedTeleopPSMs();
}

void mtsIntuitiveResearchKitConsole::SelectTeleopPSM(const prmKeyValue & mtmPsm)
{
    // for readability
    const std::string mtmName = mtmPsm.Key;
    const std::string psmName = mtmPsm.Value;

    // if the psm value is empty, disable any teleop for the mtm -- this can be used to free the mtm
    if (psmName == "") {
        std::pair<TeleopPSMByMTMIterator, TeleopPSMByMTMIterator> range;
        range = mTeleopsPSMByMTM.equal_range(mtmName);
        for (TeleopPSMByMTMIterator iter = range.first;
             iter != range.second;
             ++iter) {
            // look for the teleop that was selected if any
            if (iter->second->Selected()) {
                iter->second->SetSelected(false);
                // if teleop PSM is active, enable/disable components now
                if (mTeleopEnabled) {
                    iter->second->state_command(std::string("disable"));
                }
                // message
                mInterface->SendWarning(this->GetName()
                                        + ": teleop \""
                                        + iter->second->Name()
                                        + "\" has been unselected ");
            }
        }
        EventSelectedTeleopPSMs();
        return;
    }

    // actual teleop to select
    std::string name = mtmName + "-" + psmName;
    const TeleopPSMList::iterator teleopIterator = mTeleopsPSM.find(name);
    if (teleopIterator == mTeleopsPSM.end()) {
        mInterface->SendWarning(this->GetName()
                                + ": unable to select \""
                                + name
                                + "\", this component doesn't exist");
        EventSelectedTeleopPSMs();
        return;
    }
    // there seems to be some redundant information here, let's use it for a safety check
    CMN_ASSERT(mtmName == teleopIterator->second->mMTMName);
    CMN_ASSERT(psmName == teleopIterator->second->mPSMName);
    // check that the PSM is available to be used
    std::string mtmUsingThatPSM;
    GetMTMSelectedForPSM(psmName, mtmUsingThatPSM);
    if (mtmUsingThatPSM != "") {
        mInterface->SendWarning(this->GetName()
                                + ": unable to select \""
                                + name
                                + "\", PSM is already controlled by \""
                                + mtmUsingThatPSM
                                + "\"");
        EventSelectedTeleopPSMs();
        return;
    }

    // make sure the teleop using that MTM is unselected
    SelectTeleopPSM(prmKeyValue(mtmName, ""));

    // now turn on the teleop
    teleopIterator->second->SetSelected(true);
    // if teleop PSM is active, enable/disable components now
    if (mTeleopEnabled) {
        if (mTeleopPSMRunning) {
            teleopIterator->second->state_command(std::string("enable"));
        } else {
            teleopIterator->second->state_command(std::string("align_mtm"));
        }
    }
    // message
    mInterface->SendStatus(this->GetName()
                           + ": \""
                           + teleopIterator->second->mName
                           + "\" has been selected");

    // always send a message to let user know the current status
    EventSelectedTeleopPSMs();
}

bool mtsIntuitiveResearchKitConsole::GetPSMSelectedForMTM(const std::string & mtmName, std::string & psmName) const
{
    bool mtmFound = false;
    psmName = "";
    // find range of teleops
    std::pair<TeleopPSMByMTMConstIterator, TeleopPSMByMTMConstIterator> range;
    range = mTeleopsPSMByMTM.equal_range(mtmName);
    for (TeleopPSMByMTMConstIterator iter = range.first;
         iter != range.second;
         ++iter) {
        mtmFound = true;
        if (iter->second->Selected()) {
            psmName = iter->second->mPSMName;
        }
    }
    return mtmFound;
}

bool mtsIntuitiveResearchKitConsole::GetMTMSelectedForPSM(const std::string & psmName, std::string & mtmName) const
{
    bool psmFound = false;
    mtmName = "";
    for (auto & iter : mTeleopsPSM) {
        if (iter.second->mPSMName == psmName) {
            psmFound = true;
            if (iter.second->Selected()) {
                mtmName = iter.second->mMTMName;
            }
        }
    }
    return psmFound;
}

void mtsIntuitiveResearchKitConsole::EventSelectedTeleopPSMs(void) const
{
    for (auto & iter : mTeleopsPSM) {
        if (iter.second->Selected()) {
            ConfigurationEvents.TeleopPSMSelected(prmKeyValue(iter.second->mMTMName,
                                                              iter.second->mPSMName));
        } else {
            ConfigurationEvents.TeleopPSMUnselected(prmKeyValue(iter.second->mMTMName,
                                                                iter.second->mPSMName));
        }
    }
}

void mtsIntuitiveResearchKitConsole::UpdateTeleopState(void)
{
    // Check if teleop is enabled
    if (!mTeleopEnabled) {
        bool freezeNeeded = false;
        for (auto & iterTeleopPSM : mTeleopsPSM) {
            iterTeleopPSM.second->state_command(std::string("disable"));
            if (mTeleopPSMRunning) {
                freezeNeeded = true;
            }
            mTeleopPSMRunning = false;
        }

        if (mTeleopECM) {
            mTeleopECM->state_command(std::string("disable"));
            if (mTeleopECMRunning) {
                freezeNeeded = true;
            }
            mTeleopECMRunning = false;
        }

        // freeze arms if we stopped any teleop
        if (freezeNeeded) {
            for (auto & iterArms : mArms) {
                if ((iterArms.second->mType == Arm::ARM_MTM) ||
                    (iterArms.second->mType == Arm::ARM_MTM_DERIVED) ||
                    (iterArms.second->mType == Arm::ARM_MTM_GENERIC)) {
                    iterArms.second->Freeze();
                }
            }
        }
        return;
    }

    // if none are running, freeze
    if (!mTeleopECMRunning && !mTeleopPSMRunning) {
        for (auto & iterArms : mArms) {
            if ((iterArms.second->mType == Arm::ARM_MTM) ||
                (iterArms.second->mType == Arm::ARM_MTM_DERIVED) ||
                (iterArms.second->mType == Arm::ARM_MTM_GENERIC)) {
                iterArms.second->Freeze();
            }
        }
    }

    // all fine
    bool readyForTeleop = mOperatorPresent;

    for (auto & iterArms : mArms) {
        if (iterArms.second->mSUJClutched) {
            readyForTeleop = false;
        }
    }

    // Check if operator is present
    if (!readyForTeleop) {
        // keep MTMs aligned
        for (auto & iterTeleopPSM : mTeleopsPSM) {
            if (iterTeleopPSM.second->Selected()) {
                iterTeleopPSM.second->state_command(std::string("align_mtm"));
            } else {
                iterTeleopPSM.second->state_command(std::string("disable"));
            }
        }
        mTeleopPSMRunning = false;

        // stop ECM if needed
        if (mTeleopECMRunning) {
            mTeleopECM->state_command(std::string("disable"));
            mTeleopECMRunning = false;
        }
        return;
    }

    // If camera is pressed for ECM Teleop or not
    if (mCameraPressed) {
        if (!mTeleopECMRunning) {
            // if PSM was running so we need to stop it
            if (mTeleopPSMRunning) {
                for (auto & iterTeleopPSM : mTeleopsPSM) {
                    iterTeleopPSM.second->state_command(std::string("disable"));
                }
                mTeleopPSMRunning = false;
            }
            // ECM wasn't running, let's start it
            if (mTeleopECM) {
                mTeleopECM->state_command(std::string("enable"));
                mTeleopECMRunning = true;
            }
        }
    } else {
        // we must teleop PSM
        if (!mTeleopPSMRunning) {
            // if ECM was running so we need to stop it
            if (mTeleopECMRunning) {
                mTeleopECM->state_command(std::string("disable"));
                mTeleopECMRunning = false;
            }
            // PSM wasn't running, let's start it
            for (auto & iterTeleopPSM : mTeleopsPSM) {
                if (iterTeleopPSM.second->Selected()) {
                    iterTeleopPSM.second->state_command(std::string("enable"));
                } else {
                    iterTeleopPSM.second->state_command(std::string("disable"));
                }
                mTeleopPSMRunning = true;
            }
        }
    }
}

void mtsIntuitiveResearchKitConsole::SetScale(const double & scale)
{
    for (auto & iterTeleopPSM : mTeleopsPSM) {
        iterTeleopPSM.second->SetScale(scale);
    }
    ConfigurationEvents.Scale(scale);
}

void mtsIntuitiveResearchKitConsole::SetVolume(const double & volume)
{
    if (volume > 1.0) {
        mAudioVolume = 1.0;
    } else if (volume < 0.0) {
        mAudioVolume = 0.0;
    } else {
        mAudioVolume = volume;
    }
    std::stringstream message;
    message << this->GetName() << ": volume set to " << static_cast<int>(volume * 100.0);
    mInterface->SendStatus(message.str());
}

void mtsIntuitiveResearchKitConsole::StringToSpeech(const std::string & text)
{
    mAudio.StringToSpeech(text);
}

void mtsIntuitiveResearchKitConsole::ClutchEventHandler(const prmEventButton & button)
{
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        mInterface->SendStatus(this->GetName() + ": clutch pressed");
        mAudio.Beep(vct3(0.1, 700.0, mAudioVolume));
        break;
    case prmEventButton::RELEASED:
        mInterface->SendStatus(this->GetName() + ": clutch released");
        mAudio.Beep(vct3(0.1, 700.0, mAudioVolume));
        break;
    case prmEventButton::CLICKED:
        if (mTeleopMTMToCycle != "") {
            mInterface->SendStatus(this->GetName() + ": clutch quick tap");
            mAudio.Beep(vct3(0.05, 2000.0, mAudioVolume));
            mAudio.Beep(vct3(0.05, 2000.0, mAudioVolume));
            CycleTeleopPSMByMTM(mTeleopMTMToCycle);
        }
        break;
    default:
        break;
    }
    ConsoleEvents.Clutch(button);
}

void mtsIntuitiveResearchKitConsole::CameraEventHandler(const prmEventButton & button)
{
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        mCameraPressed = true;
        mInterface->SendStatus(this->GetName() + ": camera pressed");
        mAudio.Beep(vct3(0.1, 1000.0, mAudioVolume));
        break;
    case prmEventButton::RELEASED:
        mCameraPressed = false;
        mInterface->SendStatus(this->GetName() + ": camera released");
        mAudio.Beep(vct3(0.1, 1000.0, mAudioVolume));
        break;
    default:
        break;
    }
    UpdateTeleopState();
    ConsoleEvents.Camera(button);
}

void mtsIntuitiveResearchKitConsole::OperatorPresentEventHandler(const prmEventButton & button)
{
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        mOperatorPresent = true;
        mInterface->SendStatus(this->GetName() + ": operator present");
        mAudio.Beep(vct3(0.3, 1500.0, mAudioVolume));
        break;
    case prmEventButton::RELEASED:
        mOperatorPresent = false;
        mInterface->SendStatus(this->GetName() + ": operator not present");
        mAudio.Beep(vct3(0.3, 1200.0, mAudioVolume));
        break;
    default:
        break;
    }
    UpdateTeleopState();
    ConsoleEvents.OperatorPresent(button);
}

void mtsIntuitiveResearchKitConsole::ErrorEventHandler(const mtsMessage & message) {
    TeleopEnable(false);
    mInterface->SendError(message.Message);
    // throttle error beeps
    double currentTime = mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime();
    if ((currentTime - mTimeOfLastErrorBeep) > 2.0 * cmn_s) {
        mAudio.Beep(vct3(0.3, 3000.0, mAudioVolume));
        mTimeOfLastErrorBeep = currentTime;
    }
}

void mtsIntuitiveResearchKitConsole::WarningEventHandler(const mtsMessage & message) {
    mInterface->SendWarning(message.Message);
}

void mtsIntuitiveResearchKitConsole::StatusEventHandler(const mtsMessage & message) {
    mInterface->SendStatus(message.Message);
}

void mtsIntuitiveResearchKitConsole::SetArmCurrentState(const std::string & armName,
                                                        const prmOperatingState & currentState)
{
    auto armState = ArmStates.find(armName);
    if (// first time we receive state from this arm
        (armState == ArmStates.end())
        ||
        // only update the state if it was not enabled and the new
        // state is enabled, home and not busy
        ((armState->second.State() != prmOperatingState::ENABLED)
         && currentState.IsEnabledHomedAndNotBusy())) {
        UpdateTeleopState();
    }

    // save state
    ArmStates[armName] = currentState;

    // emit event (for Qt GUI)
    std::string payload = "";
    if (currentState.IsEnabledAndHomed()) {
        payload = "ENABLED";
    } else if (currentState.State() == prmOperatingState::FAULT) {
        payload = "FAULT";
    }
    ConfigurationEvents.ArmCurrentState(prmKeyValue(armName, payload));
}
