/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-17

  (C) Copyright 2013-2021 Johns Hopkins University (JHU), All Rights Reserved.

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
    m_console(console),
    m_name(name),
    m_IO_component_name(ioComponentName),
    m_arm_period(mtsIntuitiveResearchKit::ArmPeriod),
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
    m_PID_configuration_file = configFile;
    m_PID_component_name = m_name + "-PID";

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    mtsPID * pid = new mtsPID(m_PID_component_name,
                              (periodInSeconds != 0.0) ? periodInSeconds : mtsIntuitiveResearchKit::IOPeriod);
    bool hasIO = true;
    pid->Configure(m_PID_configuration_file);
    if (m_simulation == SIMULATION_KINEMATIC) {
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
    m_type = armType;
    bool armPSMOrDerived = false;
    bool armECMOrDerived = false;

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    m_arm_configuration_file = kinematicsConfigFile;
    // for research kit arms, create, add to manager and connect to
    // extra IO, PID, etc.  For generic arms, do nothing.
    switch (armType) {
    case ARM_MTM:
        {
            mtsIntuitiveResearchKitMTM * mtm = new mtsIntuitiveResearchKitMTM(Name(), periodInSeconds);
            if (m_simulation == SIMULATION_KINEMATIC) {
                mtm->set_simulated();
            }
            mtm->set_calibration_mode(m_calibration_mode);
            mtm->Configure(m_arm_configuration_file);
            SetBaseFrameIfNeeded(mtm);
            componentManager->AddComponent(mtm);
        }
        break;
    case ARM_PSM:
        armPSMOrDerived = true;
        {
            mtsIntuitiveResearchKitPSM * psm = new mtsIntuitiveResearchKitPSM(Name(), periodInSeconds);
            if (m_simulation == SIMULATION_KINEMATIC) {
                psm->set_simulated();
            }
            psm->set_calibration_mode(m_calibration_mode);
            psm->Configure(m_arm_configuration_file);
            SetBaseFrameIfNeeded(psm);
            componentManager->AddComponent(psm);

            if (m_socket_server) {
                mtsSocketServerPSM *serverPSM = new mtsSocketServerPSM(SocketComponentName(), periodInSeconds, m_IP, m_port);
                serverPSM->Configure();
                componentManager->AddComponent(serverPSM);
                m_console->mConnections.Add(SocketComponentName(), "PSM",
                                            ComponentName(), InterfaceName());
            }
        }
        break;
    case ARM_PSM_SOCKET:
        {
            mtsSocketClientPSM * clientPSM = new mtsSocketClientPSM(Name(), periodInSeconds, m_IP, m_port);
            clientPSM->Configure();
            componentManager->AddComponent(clientPSM);
        }
        break;
    case ARM_ECM:
        armECMOrDerived = true;
        {
            mtsIntuitiveResearchKitECM * ecm = new mtsIntuitiveResearchKitECM(Name(), periodInSeconds);
            if (m_simulation == SIMULATION_KINEMATIC) {
                ecm->set_simulated();
            }
            ecm->set_calibration_mode(m_calibration_mode);
            ecm->Configure(m_arm_configuration_file);
            SetBaseFrameIfNeeded(ecm);
            componentManager->AddComponent(ecm);
        }
        break;
    case ARM_SUJ:
        {
            mtsIntuitiveResearchKitSUJ * suj = new mtsIntuitiveResearchKitSUJ(Name(), periodInSeconds);
            if (m_simulation == SIMULATION_KINEMATIC) {
                suj->set_simulated();
            } else if (m_simulation == SIMULATION_NONE) {
                m_console->mConnections.Add(Name(), "RobotIO",
                                            IOComponentName(), Name());
                m_console->mConnections.Add(Name(), "NoMuxReset",
                                            IOComponentName(), "NoMuxReset");
                m_console->mConnections.Add(Name(), "MuxIncrement",
                                            IOComponentName(), "MuxIncrement");
                m_console->mConnections.Add(Name(), "ControlPWM",
                                            IOComponentName(), "ControlPWM");
                m_console->mConnections.Add(Name(), "DisablePWM",
                                            IOComponentName(), "DisablePWM");
                m_console->mConnections.Add(Name(), "MotorUp",
                                            IOComponentName(), "MotorUp");
                m_console->mConnections.Add(Name(), "MotorDown",
                                            IOComponentName(), "MotorDown");
                m_console->mConnections.Add(Name(), "SUJ-Clutch-1",
                                            IOComponentName(), "SUJ-Clutch-1");
                m_console->mConnections.Add(Name(), "SUJ-Clutch-2",
                                            IOComponentName(), "SUJ-Clutch-2");
                m_console->mConnections.Add(Name(), "SUJ-Clutch-3",
                                            IOComponentName(), "SUJ-Clutch-3");
                m_console->mConnections.Add(Name(), "SUJ-Clutch-4",
                                            IOComponentName(), "SUJ-Clutch-4");
            }
            suj->Configure(m_arm_configuration_file);
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
                    if (m_simulation == SIMULATION_KINEMATIC) {
                        mtm->set_simulated();
                    }
                    mtm->set_calibration_mode(m_calibration_mode);
                    mtm->Configure(m_arm_configuration_file);
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
        armPSMOrDerived = true;
        {
            mtsComponent * component;
            component = componentManager->GetComponent(Name());
            if (component) {
                mtsIntuitiveResearchKitPSM * psm = dynamic_cast<mtsIntuitiveResearchKitPSM *>(component);
                if (psm) {
                    if (m_simulation == SIMULATION_KINEMATIC) {
                        psm->set_simulated();
                    }
                    psm->set_calibration_mode(m_calibration_mode);
                    psm->Configure(m_arm_configuration_file);
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
        armECMOrDerived = true;
        {
            mtsComponent * component;
            component = componentManager->GetComponent(Name());
            if (component) {
                mtsIntuitiveResearchKitECM * ecm = dynamic_cast<mtsIntuitiveResearchKitECM *>(component);
                if (ecm) {
                    if (m_simulation == SIMULATION_KINEMATIC) {
                        ecm->set_simulated();
                    }
                    ecm->set_calibration_mode(m_calibration_mode);
                    ecm->Configure(m_arm_configuration_file);
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

    if (armPSMOrDerived && (m_simulation == SIMULATION_NONE)) {
        m_console->mConnections.Add(Name(), "Adapter",
                                    IOComponentName(), Name() + "-Adapter");
        m_console->mConnections.Add(Name(), "Tool",
                                    IOComponentName(), Name() + "-Tool");
        m_console->mConnections.Add(Name(), "ManipClutch",
                                    IOComponentName(), Name() + "-ManipClutch");
        m_console->mConnections.Add(Name(), "Dallas",
                                    IOComponentName(), Name() + "-Dallas");
    }

    if (armECMOrDerived && (m_simulation == SIMULATION_NONE)) {
        m_console->mConnections.Add(Name(), "ManipClutch",
                                    IOComponentName(), Name() + "-ManipClutch");
    }
}

void mtsIntuitiveResearchKitConsole::Arm::SetBaseFrameIfNeeded(mtsIntuitiveResearchKitArm * armPointer)
{
    if (m_base_frame.ReferenceFrame() != "") {
        armPointer->set_base_frame(m_base_frame);
    }
}

bool mtsIntuitiveResearchKitConsole::Arm::Connect(void)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    // if the arm is a research kit arm
    if (m_native_or_derived) {
        // Connect arm to IO if not simulated
        if (m_simulation == SIMULATION_NONE) {
            componentManager->Connect(Name(), "RobotIO",
                                      IOComponentName(), Name());
        }
        // connect MTM gripper to IO
        if (((m_type == ARM_MTM)
             || (m_type == ARM_MTM_DERIVED))
            && (m_simulation == SIMULATION_NONE)) {
            componentManager->Connect(Name(), "GripperIO",
                                      IOComponentName(), Name() + "-Gripper");
        }
        // connect PID
        componentManager->Connect(Name(), "PID",
                                  PIDComponentName(), "Controller");
        // connect m_base_frame if needed
        if ((m_base_frame_component_name != "") && (m_base_frame_interface_name != "")) {
            componentManager->Connect(m_base_frame_component_name, m_base_frame_interface_name,
                                      Name(), "Arm");
        }
    }
    return true;
}

const std::string & mtsIntuitiveResearchKitConsole::Arm::Name(void) const {
    return m_name;
}

const std::string & mtsIntuitiveResearchKitConsole::Arm::ComponentName(void) const {
    return m_arm_component_name;
}

const std::string & mtsIntuitiveResearchKitConsole::Arm::InterfaceName(void) const {
    return m_arm_interface_name;
}

const std::string & mtsIntuitiveResearchKitConsole::Arm::SocketComponentName(void) const {
    return m_socket_component_name;
}

const std::string & mtsIntuitiveResearchKitConsole::Arm::IOComponentName(void) const {
    return m_IO_component_name;
}

const std::string & mtsIntuitiveResearchKitConsole::Arm::PIDComponentName(void) const {
    return m_PID_component_name;
}

void mtsIntuitiveResearchKitConsole::Arm::CurrentStateEventHandler(const prmOperatingState & currentState)
{
    m_console->SetArmCurrentState(m_name, currentState);
}

mtsIntuitiveResearchKitConsole::TeleopECM::TeleopECM(const std::string & name):
    m_name(name)
{
}

void mtsIntuitiveResearchKitConsole::TeleopECM::ConfigureTeleop(const TeleopECMType type,
                                                                const double & periodInSeconds,
                                                                const Json::Value & jsonConfig)
{
    m_type = type;
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    switch (type) {
    case TELEOP_ECM:
        {
            mtsTeleOperationECM * teleop = new mtsTeleOperationECM(m_name, periodInSeconds);
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
    return m_name;
}


mtsIntuitiveResearchKitConsole::TeleopPSM::TeleopPSM(const std::string & name,
                                                     const std::string & nameMTM,
                                                     const std::string & namePSM):
    mSelected(false),
    m_name(name),
    mMTMName(nameMTM),
    mPSMName(namePSM)
{
}

void mtsIntuitiveResearchKitConsole::TeleopPSM::ConfigureTeleop(const TeleopPSMType type,
                                                                const double & periodInSeconds,
                                                                const Json::Value & jsonConfig)
{
    m_type = type;
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    switch (type) {
    case TELEOP_PSM:
        {
            mtsTeleOperationPSM * teleop = new mtsTeleOperationPSM(m_name, periodInSeconds);
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
    return m_name;
}



mtsIntuitiveResearchKitConsole::mtsIntuitiveResearchKitConsole(const std::string & componentName):
    mtsTaskFromSignal(componentName, 100),
    mConfigured(false),
    mTimeOfLastErrorBeep(0.0),
    mTeleopMTMToCycle(""),
    mTeleopECM(0),
    mDaVinciHeadSensor(0),
    mDaVinciEndoscopeFocus(0),
    mOperatorPresent(false),
    mCameraPressed(false),
    m_IO_component_name("io")
{
    mInterface = AddInterfaceProvided("Main");
    if (mInterface) {
        mInterface->AddMessageEvents();
        mInterface->AddCommandVoid(&mtsIntuitiveResearchKitConsole::power_off, this,
                                   "power_off");
        mInterface->AddCommandVoid(&mtsIntuitiveResearchKitConsole::power_on, this,
                                   "power_on");
        mInterface->AddCommandVoid(&mtsIntuitiveResearchKitConsole::home, this,
                                   "home");
        mInterface->AddEventWrite(ConfigurationEvents.ArmCurrentState,
                                  "ArmCurrentState", prmKeyValue());
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::teleop_enable, this,
                                    "teleop_enable", false);
        mInterface->AddEventWrite(console_events.teleop_enabled,
                                  "teleop_enabled", false);
        // manage tele-op
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::cycle_teleop_psm_by_mtm, this,
                                    "cycle_teleop_psm_by_mtm", std::string(""));
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::select_teleop_psm, this,
                                    "select_teleop_psm", prmKeyValue("mtm", "psm"));
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::set_scale, this,
                                    "set_scale", 0.5);
        mInterface->AddEventWrite(ConfigurationEvents.scale,
                                  "scale", 0.5);
        mInterface->AddEventWrite(ConfigurationEvents.teleop_psm_selected,
                                  "teleop_psm_selected", prmKeyValue("MTM", "PSM"));
        mInterface->AddEventWrite(ConfigurationEvents.teleop_psm_unselected,
                                  "teleop_psm_unselected", prmKeyValue("MTM", "PSM"));
        // audio
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::set_volume, this,
                                    "set_volume", m_audio_volume);
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::beep, this,
                                    "beep", vctDoubleVec());
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::string_to_speech, this,
                                    "string_to_speech", std::string());
        mInterface->AddEventWrite(audio.volume,
                                  "volume", m_audio_volume);
        // emulate foot pedal events
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::OperatorPresentEventHandler, this,
                                    "emulate_operator_present", prmEventButton());
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::ClutchEventHandler, this,
                                    "emulate_clutch", prmEventButton());
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::CameraEventHandler, this,
                                    "emulate_camera", prmEventButton());
        // misc.
        mInterface->AddCommandRead(&mtsIntuitiveResearchKitConsole::calibration_mode, this,
                                   "calibration_mode", false);
    }
}

void mtsIntuitiveResearchKitConsole::set_calibration_mode(const bool mode)
{
    m_calibration_mode = mode;
    if (m_calibration_mode) {
        std::stringstream message;
        message << "set_calibration_mode:" << std::endl
                << "----------------------------------------------------" << std::endl
                << " Warning:" << std::endl
                << " You're running the dVRK console in calibration mode." << std::endl
                << " You should only do this if you are currently calibrating" << std::endl
                << " potentiometers." << std::endl
                << "----------------------------------------------------";
        std::cerr << "mtsIntuitiveResearchKitConsole::" << message.str() << std::endl;
        CMN_LOG_CLASS_INIT_WARNING << message.str() << std::endl;
    }
}

const bool & mtsIntuitiveResearchKitConsole::calibration_mode(void) const
{
    return m_calibration_mode;
}

void mtsIntuitiveResearchKitConsole::calibration_mode(bool & result) const
{
    result = m_calibration_mode;
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

    // base component configuration
    mtsComponent::ConfigureJSON(jsonConfig);

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

    // add text to speech component for the whole system
    mTextToSpeech = new mtsTextToSpeech();
    manager->AddComponent(mTextToSpeech);
    mtsInterfaceRequired * textToSpeechInterface = this->AddInterfaceRequired("TextToSpeech");
    textToSpeechInterface->AddFunction("Beep", audio.beep);
    textToSpeechInterface->AddFunction("StringToSpeech", audio.string_to_speech);
    m_audio_volume = 0.5;

    // IO default settings
    double periodIO = mtsIntuitiveResearchKit::IOPeriod;
    std::string port = mtsRobotIO1394::DefaultPort();
    std::string protocol = mtsIntuitiveResearchKit::FireWireProtocol;
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
            protocol = jsonValue.asString();
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
            port = jsonValue.asString();
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
                               << "     - Port is " << port << std::endl
                               << "     - Protocol is " << protocol << std::endl
                               << "     - Watchdog timeout is " << watchdogTimeout << std::endl;

    const Json::Value arms = jsonConfig["arms"];
    for (unsigned int index = 0; index < arms.size(); ++index) {
        if (!ConfigureArmJSON(arms[index], m_IO_component_name, configPath)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure arms[" << index << "]" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // loop over all arms to check if IO is needed, also check if some IO configuration files are listed in "io"
    mHasIO = false;
    const auto end = mArms.end();
    for (auto iter = mArms.begin(); iter != end; ++iter) {
        std::string ioConfig = iter->second->m_IO_configuration_file;
        if (!ioConfig.empty()) {
            mHasIO = true;
        }
    }
    bool physicalFootpedalsRequired = true;
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
        // see if user wants to force no foot pedals
        Json::Value footpedalsRequired = jsonValue["physical-footpedals-required"];
        if (!footpedalsRequired.empty()) {
            physicalFootpedalsRequired = footpedalsRequired.asBool();
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
        mtsRobotIO1394 * io = new mtsRobotIO1394(m_IO_component_name, periodIO, port);
        io->SetProtocol(protocol);
        io->SetWatchdogPeriod(watchdogTimeout);
        // configure for each arm
        for (auto iter = mArms.begin(); iter != end; ++iter) {
            std::string ioConfig = iter->second->m_IO_configuration_file;
            if (ioConfig != "") {
                CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring IO using \"" << ioConfig << "\"" << std::endl;
                io->Configure(ioConfig);
            }
            std::string ioGripperConfig = iter->second->m_IO_gripper_configuration_file;
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
                mDInputSources["Clutch"] = InterfaceComponentType(m_IO_component_name, "Clutch");
                mDInputSources["OperatorPresent"] = InterfaceComponentType(m_IO_component_name, "Coag");
                mDInputSources["Coag"] = InterfaceComponentType(m_IO_component_name, "Coag");
                mDInputSources["BiCoag"] = InterfaceComponentType(m_IO_component_name, "BiCoag");
                mDInputSources["Camera"] = InterfaceComponentType(m_IO_component_name, "Camera");
                mDInputSources["Cam-"] = InterfaceComponentType(m_IO_component_name, "Cam-");
                mDInputSources["Cam+"] = InterfaceComponentType(m_IO_component_name, "Cam+");
                mDInputSources["Head"] = InterfaceComponentType(m_IO_component_name, "Head");
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
    for (auto iter = mArms.begin(); iter != end; ++iter) {
        const std::string pidConfig = iter->second->m_PID_configuration_file;
        if (!pidConfig.empty()) {
            iter->second->ConfigurePID(pidConfig);
        }
        // for generic arms, nothing to do
        if (!iter->second->m_generic) {
            const std::string armConfig = iter->second->m_arm_configuration_file;
            iter->second->ConfigureArm(iter->second->m_type, armConfig,
                                       iter->second->m_arm_period);
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
        mDInputSources["HeadSensor1"] = InterfaceComponentType(m_IO_component_name, "HeadSensor1");
        mDInputSources["HeadSensor2"] = InterfaceComponentType(m_IO_component_name, "HeadSensor2");
        mDInputSources["HeadSensor3"] = InterfaceComponentType(m_IO_component_name, "HeadSensor3");
        mDInputSources["HeadSensor4"] = InterfaceComponentType(m_IO_component_name, "HeadSensor4");
        // schedule connections
        mConnections.Add(headSensorName, "HeadSensorTurnOff",
                         m_IO_component_name, "HeadSensorTurnOff");
        mConnections.Add(headSensorName, "HeadSensor1",
                         m_IO_component_name, "HeadSensor1");
        mConnections.Add(headSensorName, "HeadSensor2",
                         m_IO_component_name, "HeadSensor2");
        mConnections.Add(headSensorName, "HeadSensor3",
                         m_IO_component_name, "HeadSensor3");
        mConnections.Add(headSensorName, "HeadSensor4",
                         m_IO_component_name, "HeadSensor4");

    }

    // message re. footpedals are likely missing but user can override this requirement
    const std::string footpedalMessage = "Maybe you're missing \"io\":\"footpedals\" in your configuration file.  If you don't need physical footpedals, set \"physical-footpedals-required\" to false.";

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
            CMN_LOG_CLASS_INIT_ERROR << "Configure: input for footpedal \"Cam-\" is required for \"endoscope-focus\".  "
                                     << footpedalMessage << std::endl;
            exit(EXIT_FAILURE);
        }
        const bool foundCamPlus = (mDInputSources.find("Cam+") != endDInputs);
        if (!foundCamPlus) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: input for footpedal \"Cam+\" is required for \"endoscope-focus\".  "
                                     << footpedalMessage << std::endl;
            exit(EXIT_FAILURE);
        }
        // schedule connections
        mConnections.Add(endoscopeFocusName, "EndoscopeFocusIn",
                         m_IO_component_name, "EndoscopeFocusIn");
        mConnections.Add(endoscopeFocusName, "EndoscopeFocusOut",
                         m_IO_component_name, "EndoscopeFocusOut");
        mConnections.Add(endoscopeFocusName, "focus_in",
                         m_IO_component_name, "Cam+");
        mConnections.Add(endoscopeFocusName, "focus_out",
                         m_IO_component_name, "Cam-");
    }

    // if we have any teleoperation component, we need to have the interfaces for the foot pedals
    // unless user explicitly says we can skip
    if (physicalFootpedalsRequired) {
        const DInputSourceType::const_iterator endDInputs = mDInputSources.end();
        const bool foundClutch = (mDInputSources.find("Clutch") != endDInputs);
        const bool foundOperatorPresent = (mDInputSources.find("OperatorPresent") != endDInputs);
        const bool foundCamera = (mDInputSources.find("Camera") != endDInputs);

        if (mTeleopsPSM.size() > 0) {
            if (!foundClutch || !foundOperatorPresent) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: inputs for footpedals \"Clutch\" and \"OperatorPresent\" need to be defined since there's at least one PSM tele-operation component.  "
                                         << footpedalMessage << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        if (mTeleopECM) {
            if (!foundCamera || !foundOperatorPresent) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: inputs for footpedals \"Camera\" and \"OperatorPresent\" need to be defined since there's an ECM tele-operation component.  "
                                         << footpedalMessage << std::endl;
                exit(EXIT_FAILURE);
            }
        }
    }
    this->AddFootpedalInterfaces();

    // search for SUJs
    bool hasSUJ = false;
    for (auto iter = mArms.begin(); iter != end; ++iter) {
        if (iter->second->m_type == Arm::ARM_SUJ) {
            hasSUJ = true;
        }
    }

    if (hasSUJ) {
        for (auto iter = mArms.begin(); iter != end; ++iter) {
            Arm * arm = iter->second;
            // only for PSM and ECM when not simulated
            if (((arm->m_type == Arm::ARM_ECM)
                 || (arm->m_type == Arm::ARM_ECM_DERIVED)
                 || (arm->m_type == Arm::ARM_PSM)
                 || (arm->m_type == Arm::ARM_PSM_DERIVED)
                 )
                && (arm->m_simulation == Arm::SIMULATION_NONE)) {
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

    // emit events for active PSM teleop pairs
    EventSelectedTeleopPSMs();
    // emit scale event
    ConfigurationEvents.scale(mtsIntuitiveResearchKit::TeleOperationPSM::Scale);
    // emit volume event
    audio.volume(m_audio_volume);

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
        prompts.push_back("Maybe we should use ROS control");
        prompts.push_back("Use with caution");
        prompts.push_back("It's about time");
        prompts.push_back("When did you last commit your changes?");
        prompts.push_back("Some documentation would be nice");
        int index;
        cmnRandomSequence & randomSequence = cmnRandomSequence::GetInstance();
        cmnRandomSequence::SeedType seed
            = static_cast<cmnRandomSequence::SeedType>(mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime() * 100000.0);
        randomSequence.SetSeed(seed % 1000);
        randomSequence.ExtractRandomValue<int>(0, prompts.size() - 1, index);
        audio.string_to_speech(prompts.at(index));
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
    if ((newArm->m_type != Arm::ARM_PSM_SOCKET)
        && (!newArm->m_generic)) {
        if (newArm->m_type != Arm::ARM_SUJ) {
            if (newArm->m_PID_configuration_file.empty()) {
                CMN_LOG_CLASS_INIT_ERROR << GetName() << ": AddArm, "
                                         << newArm->Name() << " must be configured first (PID)." << std::endl;
                return false;
            }
        }
        if (newArm->m_arm_configuration_file.empty()) {
            CMN_LOG_CLASS_INIT_ERROR << GetName() << ": AddArm, "
                                     << newArm->Name() << " must be configured first (Arm config)." << std::endl;
            return false;
        }
    }
    if (AddArmInterfaces(newArm)) {
        auto armIterator = mArms.find(newArm->m_name);
        if (armIterator == mArms.end()) {
            mArms[newArm->m_name] = newArm;
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
        auto armIterator = mArms.find(newArm->m_name);
        if (armIterator != mArms.end()) {
            mArms[newArm->m_name] = newArm;
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
    auto armIterator = mArms.find(armName);
    if (armIterator != mArms.end()) {
        return armIterator->second->m_IO_component_name;
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
        teleop->InterfaceRequired->AddFunction("set_scale", teleop->set_scale);
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
    }
    mtsInterfaceProvided * clutchProvided = AddInterfaceProvided("Clutch");
    if (clutchProvided) {
        clutchProvided->AddEventWrite(console_events.clutch, "Button", prmEventButton());
    }

    iter = mDInputSources.find("Camera");
    if (iter != endDInputs) {
        mtsInterfaceRequired * cameraRequired = AddInterfaceRequired("Camera");
        if (cameraRequired) {
            cameraRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::CameraEventHandler, this, "Button");
        }
        mConnections.Add(this->GetName(), "Camera",
                         iter->second.first, iter->second.second);
    }
    mtsInterfaceProvided * cameraProvided = AddInterfaceProvided("Camera");
    if (cameraProvided) {
        cameraProvided->AddEventWrite(console_events.camera, "Button", prmEventButton());
    }

    iter = mDInputSources.find("OperatorPresent");
    if (iter != endDInputs) {
        mtsInterfaceRequired * operatorRequired = AddInterfaceRequired("OperatorPresent");
        if (operatorRequired) {
            operatorRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::OperatorPresentEventHandler, this, "Button");
        }
        mConnections.Add(this->GetName(), "OperatorPresent",
                         iter->second.first, iter->second.second);
    }
    mtsInterfaceProvided * operatorProvided = AddInterfaceProvided("OperatorPresent");
    if (operatorProvided) {
        operatorProvided->AddEventWrite(console_events.operator_present, "Button", prmEventButton());
    }
}

bool mtsIntuitiveResearchKitConsole::ConfigureArmJSON(const Json::Value & jsonArm,
                                                      const std::string & ioComponentName,
                                                      const cmnPath & configPath)
{
    const std::string armName = jsonArm["name"].asString();
    const auto armIterator = mArms.find(armName);
    Arm * armPointer = 0;
    if (armIterator == mArms.end()) {
        // create a new arm if needed
        armPointer = new Arm(this, armName, ioComponentName);
    } else {
        armPointer = armIterator->second;
    }

    Json::Value jsonValue;

    // create search path based on optional system
    cmnPath armConfigPath = configPath;
    jsonValue = jsonArm["system"];
    if (!jsonValue.empty()) {
        armConfigPath.Add(std::string(sawIntuitiveResearchKit_SOURCE_DIR)
                          + "/../share/"
                          + jsonValue.asString() + "/",
                          cmnPath::TAIL);
    }

    // read from JSON and check if configuration files exist
    jsonValue = jsonArm["type"];
    armPointer->m_generic = false; // default value
    armPointer->m_native_or_derived = false;
    if (!jsonValue.empty()) {
        std::string typeString = jsonValue.asString();
        if (typeString == "MTM") {
            armPointer->m_type = Arm::ARM_MTM;
            armPointer->m_native_or_derived = true;
        } else if (typeString == "PSM") {
            armPointer->m_type = Arm::ARM_PSM;
            armPointer->m_native_or_derived = true;
        } else if (typeString == "ECM") {
            armPointer->m_type = Arm::ARM_ECM;
            armPointer->m_native_or_derived = true;
        } else if (typeString == "MTM_DERIVED") {
            armPointer->m_type = Arm::ARM_MTM_DERIVED;
            armPointer->m_native_or_derived = true;
        } else if (typeString == "PSM_DERIVED") {
            armPointer->m_type = Arm::ARM_PSM_DERIVED;
            armPointer->m_native_or_derived = true;
        } else if (typeString == "ECM_DERIVED") {
            armPointer->m_type = Arm::ARM_ECM_DERIVED;
            armPointer->m_native_or_derived = true;
        } else if (typeString == "MTM_GENERIC") {
            armPointer->m_type = Arm::ARM_MTM_GENERIC;
            armPointer->m_generic = true;
        } else if (typeString == "PSM_GENERIC") {
            armPointer->m_type = Arm::ARM_PSM_GENERIC;
            armPointer->m_generic = true;
        } else if (typeString == "ECM_GENERIC") {
            armPointer->m_type = Arm::ARM_ECM_GENERIC;
            armPointer->m_generic = true;
        } else if (typeString == "PSM_SOCKET") {
            armPointer->m_type = Arm::ARM_PSM_SOCKET;
        } else if (typeString == "FOCUS_CONTROLLER") {
            armPointer->m_type = Arm::FOCUS_CONTROLLER;
            armPointer->m_native_or_derived = true;
        } else if (typeString == "SUJ") {
            armPointer->m_type = Arm::ARM_SUJ;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: arm " << armName << ": invalid type \""
                                     << typeString << "\", needs to be one of {MTM,PSM,ECM}{,_DERIVED,_GENERIC}" << std::endl;
            return false;
        }
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: arm " << armName
                                 << ": doesn't have a \"type\" specified, needs to be one of {MTM,PSM,ECM}{,_DERIVED,_GENERIC} or FOCUS_CONTROLLER" << std::endl;
        return false;
    }

    jsonValue = jsonArm["serial"];
    if (!jsonValue.empty()) {
        armPointer->m_serial = jsonValue.asString();
    }

    // type of simulation, if any
    jsonValue = jsonArm["simulation"];
    if (!jsonValue.empty()) {
        std::string typeString = jsonValue.asString();
        if (typeString == "KINEMATIC") {
            armPointer->m_simulation = Arm::SIMULATION_KINEMATIC;
        } else if (typeString == "DYNAMIC") {
            armPointer->m_simulation = Arm::SIMULATION_DYNAMIC;
        } else if (typeString == "NONE") {
            armPointer->m_simulation = Arm::SIMULATION_NONE;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: arm " << armName << ": invalid simulation \""
                                     << typeString << "\", needs to be NONE, KINEMATIC or DYNAMIC" << std::endl;
            return false;
        }
    } else {
        armPointer->m_simulation = Arm::SIMULATION_NONE;
    }

    // set arm calibration mode based on console calibration mode
    armPointer->m_calibration_mode = m_calibration_mode;

    // should we automatically create ROS bridge for this arm
    armPointer->m_skip_ROS_bridge = false;
    jsonValue = jsonArm["skip-ros-bridge"];
    if (!jsonValue.empty()) {
        armPointer->m_skip_ROS_bridge = jsonValue.asBool();
    }

    // component and interface, defaults
    armPointer->m_arm_component_name = armName;
    armPointer->m_arm_interface_name = "Arm";
    jsonValue = jsonArm["component"];
    if (!jsonValue.empty()) {
        armPointer->m_arm_component_name = jsonValue.asString();
    }
    jsonValue = jsonArm["interface"];
    if (!jsonValue.empty()) {
        armPointer->m_arm_interface_name = jsonValue.asString();
    }

    // check if we need to create a socket server attached to this arm
    armPointer->m_socket_server = false;
    jsonValue = jsonArm["socket-server"];
    if (!jsonValue.empty()) {
        armPointer->m_socket_server = jsonValue.asBool();
    }

    // for socket client or server, look for remote IP / port
    if (armPointer->m_type == Arm::ARM_PSM_SOCKET || armPointer->m_socket_server) {
        armPointer->m_socket_component_name = armPointer->m_name + "-SocketServer";
        jsonValue = jsonArm["remote-ip"];
        if(!jsonValue.empty()){
            armPointer->m_IP = jsonValue.asString();
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"server-ip\" for arm \""
                                     << armName << "\"" << std::endl;
            return false;
        }
        jsonValue = jsonArm["port"];
        if (!jsonValue.empty()) {
            armPointer->m_port = jsonValue.asInt();
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"port\" for arm \""
                                     << armName << "\"" << std::endl;
            return false;
        }
    }

    // IO for anything not simulated or socket client
    if ((armPointer->m_type != Arm::ARM_PSM_SOCKET)
        && (!armPointer->m_generic)) {
        if (armPointer->m_simulation == Arm::SIMULATION_NONE) {
            jsonValue = jsonArm["io"];
            if (!jsonValue.empty()) {
                armPointer->m_IO_configuration_file = armConfigPath.Find(jsonValue.asString());
                if (armPointer->m_IO_configuration_file == "") {
                    CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find IO file " << jsonValue.asString() << std::endl;
                    return false;
                }
            } else {
                // try to find default if serial number has been provided
                if (armPointer->m_serial != "") {
                    std::string defaultFile = "sawRobotIO1394-" + armName + "-" + armPointer->m_serial + ".xml";
                    armPointer->m_IO_configuration_file = armConfigPath.Find(defaultFile);
                    if (armPointer->m_IO_configuration_file == "") {
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
            if ((armPointer->m_type == Arm::ARM_MTM)
                || (armPointer->m_type == Arm::ARM_MTM_DERIVED)) {
                jsonValue = jsonArm["io-gripper"];
                if (!jsonValue.empty()) {
                    armPointer->m_IO_gripper_configuration_file = armConfigPath.Find(jsonValue.asString());
                    if (armPointer->m_IO_gripper_configuration_file == "") {
                        CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find IO gripper file "
                                                 << jsonValue.asString() << std::endl;
                        return false;
                    }
                } else {
                    // try to find default if serial number has been provided
                    if (armPointer->m_serial != "") {
                        std::string defaultFile = "sawRobotIO1394-" + armName + "-gripper-" + armPointer->m_serial + ".xml";
                        armPointer->m_IO_gripper_configuration_file = armConfigPath.Find(defaultFile);
                        if (armPointer->m_IO_gripper_configuration_file == "") {
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
    if (armPointer->m_native_or_derived) {
        jsonValue = jsonArm["pid"];
        if (!jsonValue.empty()) {
            armPointer->m_PID_configuration_file = armConfigPath.Find(jsonValue.asString());
            if (armPointer->m_PID_configuration_file == "") {
                CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find PID file " << jsonValue.asString() << std::endl;
                return false;
            }
        } else {
            // try to find default
            std::string defaultFile;
            if ((armPointer->m_type == Arm::ARM_PSM) || (armPointer->m_type == Arm::ARM_PSM_DERIVED)) {
                defaultFile = "pid/sawControllersPID-PSM.xml";
            } else if ((armPointer->m_type == Arm::ARM_ECM) || (armPointer->m_type == Arm::ARM_ECM_DERIVED)) {
                defaultFile = "pid/sawControllersPID-ECM.xml";
            } else {
                defaultFile = "pid/sawControllersPID-" + armName + ".xml";
            }
            CMN_LOG_CLASS_INIT_VERBOSE << "ConfigureArmJSON: can't find \"pid\" setting for arm \""
                                       << armName << "\", using default: \""
                                       << defaultFile << "\"" << std::endl;
            armPointer->m_PID_configuration_file = armConfigPath.Find(defaultFile);
            if (armPointer->m_PID_configuration_file == "") {
                CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find PID file " << defaultFile << std::endl;
                return false;
            }
        }
    }

    // only configure kinematics if not arm socket client
    if ((armPointer->m_type != Arm::ARM_PSM_SOCKET)
        && (!armPointer->m_generic)) {
        // renamed "kinematic" to "arm" so we can have a more complex configuration file for the arm class
        if (armPointer->m_native_or_derived) {
            jsonValue = jsonArm["arm"];
            if (!jsonValue.empty()) {
                armPointer->m_arm_configuration_file = armConfigPath.Find(jsonValue.asString());
                if (armPointer->m_arm_configuration_file == "") {
                    CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find configuration file " << jsonValue.asString() << std::endl;
                    return false;
                }
            }
        }
        jsonValue = jsonArm["kinematic"];
        if (!jsonValue.empty()) {
            if (armPointer->m_arm_configuration_file != "") {
                CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: arm configuration file is already set using \"arm\", you should remove the deprecated \"kinetic\" field:"
                                         << jsonValue.asString() << std::endl;
                return false;
            } else {
                armPointer->m_arm_configuration_file = armConfigPath.Find(jsonValue.asString());
                if (armPointer->m_arm_configuration_file == "") {
                    CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find Kinematic file " << jsonValue.asString() << std::endl;
                    return false;
                }
            }
        }

        // make sure we have an arm configuration file for all arms except FOCUS_CONTROLLER
        if ((armPointer->m_arm_configuration_file == "")
            && (armPointer->m_type != Arm::FOCUS_CONTROLLER)) {
            if (armPointer->m_native_or_derived) {
                // try to find the arm file using default
                std::string defaultFile = armName + "-" + armPointer->m_serial + ".json";
                armPointer->m_arm_configuration_file = armConfigPath.Find(defaultFile);
                if (armPointer->m_arm_configuration_file == "") {
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
                armPointer->m_base_frame.Goal().From(frame);
                armPointer->m_base_frame.ReferenceFrame() = reference;
                armPointer->m_base_frame.Valid() = true;
            } else {
                armPointer->m_base_frame_component_name = jsonValue.get("component", "").asString();
                armPointer->m_base_frame_interface_name = jsonValue.get("interface", "").asString();
                if ((armPointer->m_base_frame_component_name == "")
                    || (armPointer->m_base_frame_interface_name == "")) {
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
        armPointer->m_arm_period = jsonValue.asFloat();
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
    auto armIterator = mArms.find(mtmLeftName);
    if (armIterator == mArms.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: mtm left\""
                                 << mtmLeftName << "\" is not defined in \"arms\"" << std::endl;
        return false;
    } else {
        armPointer = armIterator->second;
        if (!((armPointer->m_type == Arm::ARM_MTM_GENERIC) ||
              (armPointer->m_type == Arm::ARM_MTM_DERIVED) ||
              (armPointer->m_type == Arm::ARM_MTM))) {
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
        if (!((armPointer->m_type == Arm::ARM_MTM_GENERIC) ||
              (armPointer->m_type == Arm::ARM_MTM_DERIVED) ||
              (armPointer->m_type == Arm::ARM_MTM))) {
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
        if (!((armPointer->m_type == Arm::ARM_ECM_GENERIC) ||
              (armPointer->m_type == Arm::ARM_ECM_DERIVED) ||
              (armPointer->m_type == Arm::ARM_ECM))) {
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
            mTeleopECM->m_type = TeleopECM::TELEOP_ECM;
        } else if (typeString == "TELEOP_ECM_DERIVED") {
            mTeleopECM->m_type = TeleopECM::TELEOP_ECM_DERIVED;
        } else if (typeString == "TELEOP_ECM_GENERIC") {
            mTeleopECM->m_type = TeleopECM::TELEOP_ECM_GENERIC;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: teleop " << name << ": invalid type \""
                                     << typeString << "\", needs to be TELEOP_ECM, TELEOP_ECM_DERIVED or TELEOP_ECM_GENERIC" << std::endl;
            return false;
        }
    } else {
        // default value
        mTeleopECM->m_type = TeleopECM::TELEOP_ECM;
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
    mTeleopECM->ConfigureTeleop(mTeleopECM->m_type, period, jsonTeleopConfig);
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
    auto armIterator = mArms.find(mtmName);
    if (armIterator == mArms.end()) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: mtm \""
                                 << mtmName << "\" is not defined in \"arms\"" << std::endl;
        return false;
    } else {
        armPointer = armIterator->second;
        if (!((armPointer->m_type == Arm::ARM_MTM_GENERIC) ||
              (armPointer->m_type == Arm::ARM_MTM_DERIVED) ||
              (armPointer->m_type == Arm::ARM_MTM))) {
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
        if (!((armPointer->m_type == Arm::ARM_PSM_GENERIC) ||
              (armPointer->m_type == Arm::ARM_PSM_DERIVED) ||
              (armPointer->m_type == Arm::ARM_PSM_SOCKET)  ||
              (armPointer->m_type == Arm::ARM_PSM))) {
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
    const auto teleopIterator = mTeleopsPSM.find(name);
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

        // insert
        mTeleopsPSMByMTM.insert(std::make_pair(mtmName, teleopPointer));
        mTeleopsPSMByPSM.insert(std::make_pair(psmName, teleopPointer));

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
            teleopPointer->m_type = TeleopPSM::TELEOP_PSM;
        } else if (typeString == "TELEOP_PSM_DERIVED") {
            teleopPointer->m_type = TeleopPSM::TELEOP_PSM_DERIVED;
        } else if (typeString == "TELEOP_PSM_GENERIC") {
            teleopPointer->m_type = TeleopPSM::TELEOP_PSM_GENERIC;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: teleop " << name << ": invalid type \""
                                     << typeString << "\", needs to be TELEOP_PSM, TELEOP_PSM_DERIVED or TELEOP_PSM_GENERIC" << std::endl;
            return false;
        }
    } else {
        // default value
        teleopPointer->m_type = TeleopPSM::TELEOP_PSM;
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
    teleopPointer->ConfigureTeleop(teleopPointer->m_type, period, jsonTeleopConfig);
    AddTeleopPSMInterfaces(teleopPointer);
    return true;
}

bool mtsIntuitiveResearchKitConsole::AddArmInterfaces(Arm * arm)
{
    // IO
    if (!arm->m_IO_configuration_file.empty()) {
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
        if ((arm->m_type == Arm::ARM_PSM)
            || (arm->m_type == Arm::ARM_PSM_DERIVED)) {
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
    if (!arm->m_PID_configuration_file.empty()) {
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
        if (arm->m_type != Arm::ARM_SUJ) {
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

void mtsIntuitiveResearchKitConsole::power_off(void)
{
    teleop_enable(false);
    for (auto & arm : mArms) {
        arm.second->state_command(std::string("disable"));
    }
}

void mtsIntuitiveResearchKitConsole::power_on(void)
{
    DisableFaultyArms();
    for (auto & arm : mArms) {
        arm.second->state_command(std::string("enable"));
    }
}

void mtsIntuitiveResearchKitConsole::home(void)
{
    DisableFaultyArms();
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

void mtsIntuitiveResearchKitConsole::teleop_enable(const bool & enable)
{
    mTeleopEnabled = enable;
    mTeleopDesired = enable;
    // event
    console_events.teleop_enabled(mTeleopEnabled);
    UpdateTeleopState();
}

void mtsIntuitiveResearchKitConsole::cycle_teleop_psm_by_mtm(const std::string & mtmName)
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
        auto range = mTeleopsPSMByMTM.equal_range(mtmName);
        for (auto iter = range.first;
             iter != range.second;
             ++iter) {
            // find first teleop currently selected
            if (iter->second->Selected()) {
                // toggle to next one
                auto nextTeleop = iter;
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
                                            + iter->second->m_name
                                            + "\" to \""
                                            + nextTeleop->second->m_name
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
                                           + iter->second->m_name
                                           + "\" to \""
                                           + nextTeleop->second->m_name
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

void mtsIntuitiveResearchKitConsole::select_teleop_psm(const prmKeyValue & mtmPsm)
{
    // for readability
    const std::string mtmName = mtmPsm.Key;
    const std::string psmName = mtmPsm.Value;

    // if the psm value is empty, disable any teleop for the mtm -- this can be used to free the mtm
    if (psmName == "") {
        auto range = mTeleopsPSMByMTM.equal_range(mtmName);
        for (auto iter = range.first;
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
    const auto teleopIterator = mTeleopsPSM.find(name);
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
    select_teleop_psm(prmKeyValue(mtmName, ""));

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
                           + teleopIterator->second->m_name
                           + "\" has been selected");

    // always send a message to let user know the current status
    EventSelectedTeleopPSMs();
}

bool mtsIntuitiveResearchKitConsole::GetPSMSelectedForMTM(const std::string & mtmName, std::string & psmName) const
{
    bool mtmFound = false;
    psmName = "";
    // find range of teleops
    auto range = mTeleopsPSMByMTM.equal_range(mtmName);
    for (auto iter = range.first;
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
            ConfigurationEvents.teleop_psm_selected(prmKeyValue(iter.second->mMTMName,
                                                                iter.second->mPSMName));
        } else {
            ConfigurationEvents.teleop_psm_unselected(prmKeyValue(iter.second->mMTMName,
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
                if ((iterArms.second->m_type == Arm::ARM_MTM) ||
                    (iterArms.second->m_type == Arm::ARM_MTM_DERIVED) ||
                    (iterArms.second->m_type == Arm::ARM_MTM_GENERIC)) {
                    iterArms.second->Freeze();
                }
            }
        }
        return;
    }

    // if none are running, freeze
    if (!mTeleopECMRunning && !mTeleopPSMRunning) {
        for (auto & iterArms : mArms) {
            if ((iterArms.second->m_type == Arm::ARM_MTM) ||
                (iterArms.second->m_type == Arm::ARM_MTM_DERIVED) ||
                (iterArms.second->m_type == Arm::ARM_MTM_GENERIC)) {
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

void mtsIntuitiveResearchKitConsole::set_scale(const double & scale)
{
    for (auto & iterTeleopPSM : mTeleopsPSM) {
        iterTeleopPSM.second->set_scale(scale);
    }
    ConfigurationEvents.scale(scale);
}

void mtsIntuitiveResearchKitConsole::set_volume(const double & volume)
{
    if (volume > 1.0) {
        m_audio_volume = 1.0;
    } else if (volume < 0.0) {
        m_audio_volume = 0.0;
    } else {
        m_audio_volume = volume;
    }
    std::stringstream message;
    message << this->GetName() << ": volume set to " << static_cast<int>(volume * 100.0) << "%";
    mInterface->SendStatus(message.str());
    audio.volume(m_audio_volume);
}

void mtsIntuitiveResearchKitConsole::beep(const vctDoubleVec & values)
{
    const size_t size = values.size();
    if ((size == 0) || (size > 3)) {
        mInterface->SendError(this->GetName() + ": beep expect up to 3 values (duration, frequency, volume)");
        return;
    }
    vctDoubleVec result(3);
    result.Assign(0.3, 3000.0, m_audio_volume);
    result.Ref(size).Assign(values); // overwrite with data sent
    // check duration
    bool durationError = false;
    if (result[0] < 0.1) {
        result[0] = 0.1;
        durationError = true;
    } else if (result[0] > 60.0) {
        result[0] = 60.0;
        durationError = true;
    }
    if (durationError) {
        mInterface->SendWarning(this->GetName() + ": beep, duration must be between 0.1 and 60s");
    }
    // check volume
    bool volumeError = false;
    if (result[2] < 0.0) {
        result[2] = 0.0;
        volumeError = true;
    } else if (result[2] > 1.0) {
        result[2] = 1.0;
        volumeError = true;
    }
    if (volumeError) {
        mInterface->SendWarning(this->GetName() + ": beep, volume must be between 0 and 1");
    }
    // convert to fixed size vector and send
    audio.beep(vct3(result));
}

void mtsIntuitiveResearchKitConsole::string_to_speech(const std::string & text)
{
    audio.string_to_speech(text);
}

void mtsIntuitiveResearchKitConsole::ClutchEventHandler(const prmEventButton & button)
{
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        mInterface->SendStatus(this->GetName() + ": clutch pressed");
        audio.beep(vct3(0.1, 700.0, m_audio_volume));
        break;
    case prmEventButton::RELEASED:
        mInterface->SendStatus(this->GetName() + ": clutch released");
        audio.beep(vct3(0.1, 700.0, m_audio_volume));
        break;
    case prmEventButton::CLICKED:
        mInterface->SendStatus(this->GetName() + ": clutch quick tap");
        audio.beep(vct3(0.05, 2000.0, m_audio_volume));
        audio.beep(vct3(0.05, 2000.0, m_audio_volume));
        if (mTeleopMTMToCycle != "") {
            cycle_teleop_psm_by_mtm(mTeleopMTMToCycle);
        }
        break;
    default:
        break;
    }
    console_events.clutch(button);
}

void mtsIntuitiveResearchKitConsole::CameraEventHandler(const prmEventButton & button)
{
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        mCameraPressed = true;
        mInterface->SendStatus(this->GetName() + ": camera pressed");
        audio.beep(vct3(0.1, 1000.0, m_audio_volume));
        break;
    case prmEventButton::RELEASED:
        mCameraPressed = false;
        mInterface->SendStatus(this->GetName() + ": camera released");
        audio.beep(vct3(0.1, 1000.0, m_audio_volume));
        break;
    case prmEventButton::CLICKED:
        mInterface->SendStatus(this->GetName() + ": camera quick tap");
        audio.beep(vct3(0.05, 2500.0, m_audio_volume));
        audio.beep(vct3(0.05, 2500.0, m_audio_volume));
        break;
    default:
        break;
    }
    UpdateTeleopState();
    console_events.camera(button);
}

void mtsIntuitiveResearchKitConsole::OperatorPresentEventHandler(const prmEventButton & button)
{
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        mOperatorPresent = true;
        mInterface->SendStatus(this->GetName() + ": operator present");
        audio.beep(vct3(0.3, 1500.0, m_audio_volume));
        break;
    case prmEventButton::RELEASED:
        mOperatorPresent = false;
        mInterface->SendStatus(this->GetName() + ": operator not present");
        audio.beep(vct3(0.3, 1200.0, m_audio_volume));
        break;
    default:
        break;
    }
    UpdateTeleopState();
    console_events.operator_present(button);
}

void mtsIntuitiveResearchKitConsole::ErrorEventHandler(const mtsMessage & message)
{
    // similar to teleop_enable(false) except we don't change mTeleopDesired
    mTeleopEnabled = false;
    console_events.teleop_enabled(mTeleopEnabled);
    UpdateTeleopState();

    mInterface->SendError(message.Message);
    // throttle error beeps
    double currentTime = mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime();
    if ((currentTime - mTimeOfLastErrorBeep) > 2.0 * cmn_s) {
        audio.beep(vct3(0.3, 3000.0, m_audio_volume));
        mTimeOfLastErrorBeep = currentTime;
    }
}

void mtsIntuitiveResearchKitConsole::WarningEventHandler(const mtsMessage & message)
{
    mInterface->SendWarning(message.Message);
}

void mtsIntuitiveResearchKitConsole::StatusEventHandler(const mtsMessage & message)
{
    mInterface->SendStatus(message.Message);
}

void mtsIntuitiveResearchKitConsole::SetArmCurrentState(const std::string & armName,
                                                        const prmOperatingState & currentState)
{
    if (mTeleopDesired) {
        auto armState = ArmStates.find(armName);
        bool newArm = (armState == ArmStates.end());
        if (newArm) {
            teleop_enable(true);
        } else {
            // for all arms update the state if it was not enabled and the new
            // state is enabled, home and not busy
            if (((armState->second.State() != prmOperatingState::ENABLED)
                 && currentState.IsEnabledHomedAndNotBusy())) {
                teleop_enable(true);
            } else {
                // special case for PSMs when coming back from busy state
                // (e.g. engaging adapter or instrument)
                const auto count = mTeleopsPSMByPSM.count(armName);
                if (count != 0) {
                    if (!armState->second.IsBusy() && currentState.IsEnabledHomedAndNotBusy()) {
                        teleop_enable(true);
                    }
                }
            }
        }
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
