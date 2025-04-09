/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-17

  (C) Copyright 2013-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>

// cisst
#include <cisstCommon/cmnClassRegister.h>
#include <cisstCommon/cmnRandomSequence.h>
#include <cisstOSAbstraction/osaDynamicLoader.h>
#include <cisstOSAbstraction/osaSleep.h>

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
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitSUJSi.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitSUJFixed.h>
#include <sawIntuitiveResearchKit/mtsDaVinciHeadSensor.h>
#if sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR
#include <sawIntuitiveResearchKit/mtsHIDHeadSensor.h>
#endif
#include <sawIntuitiveResearchKit/mtsDaVinciEndoscopeFocus.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationPSM.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationECM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>

#include <json/json.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitConsole, mtsTaskFromSignal, std::string);


mtsIntuitiveResearchKitConsole::arm_proxy_t::arm_proxy_t(const std::string & name,
                                                         mtsIntuitiveResearchKitConsole * console):
    m_console(console),
    m_arm_component_name(name),
    m_arm_interface_name("Arm")
{
    m_config.name = name;
}


void mtsIntuitiveResearchKitConsole::arm_proxy_t::configure(const Json::Value & json_config)
{
    // Json parse and load values, will complain if a field with no
    // default is not provided (e.g. "name")
    cmnDataJSON<dvrk::arm_proxy_configuration_t>::DeSerializeText(m_config, json_config);
    CMN_LOG_INIT_VERBOSE << "arm_proxy_t::configure, loaded:" << std::endl
                         << "------>" << std::endl
                         << m_config << std::endl
                         << "<------" << std::endl;
    // for generic and derived arms, component name must be provided
    if (m_config.generic_or_derived()) {
        if ((m_config.component != "") && (m_config.interface != "")) {
            m_arm_component_name = m_config.component;
            m_arm_interface_name = m_config.interface;
        } else {
            CMN_LOG_INIT_ERROR << "arm_proxy_t::configure: \"component\" and \"interface\" must be provided for generic or derived arm: "
                               << m_config.name << std::endl;
        }
    }
}


void mtsIntuitiveResearchKitConsole::arm_proxy_t::create_arm(void)
{
    if (!m_config.native_or_derived()) {
        return;
    }

    // infer arm configuration file
    // -1- provided by user
    if (m_config.arm != "") {
        m_arm_configuration_file = m_console->find_file(m_config.arm);
        if (m_arm_configuration_file == "") {
            CMN_LOG_INIT_ERROR << "arm_proxy_t::create_arm: can't find configuration file " << m_config.arm
                               << " for arm " << m_config.name << std::endl;
            exit(EXIT_FAILURE);
        }
    } else {
        // -2- using serial number
        if (m_config.type != dvrk::arm_type_t::FOCUS_CONTROLLER) {
            if (m_config.serial == "") {
                CMN_LOG_INIT_ERROR << "arm_proxy_t::create_arm: serial number required for arm "
                                   << m_config.name << std::endl;
                exit(EXIT_FAILURE);
            }
            const auto default_file = m_config.name + "-" + m_config.serial + ".json";
            m_arm_configuration_file = m_console->find_file(default_file);
            if (m_arm_configuration_file == "") {
                CMN_LOG_INIT_ERROR << "arm_proxy_t::create_arm: can't find \"arm\" setting for arm "
                                   << m_config.name << ". \"arm\" is not set and the default file "
                                   << default_file << " doesn't seem to exist either." << std::endl;
                exit(EXIT_FAILURE);
            } else {
                CMN_LOG_INIT_VERBOSE << "arm_proxy_t::create_arm: using the default file "
                                     << default_file << " for arm " << m_config.name << std::endl;

            }
        }
    }


    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    // for research kit arms, create, add to manager and connect to
    // extra IO, PID, etc.  For generic arms, do nothing.
    switch (m_config.type) {
    case dvrk::arm_type_t::MTM:
        {
            m_arm = std::make_shared<mtsIntuitiveResearchKitMTM>(m_config.name, m_config.period);
        }
        break;
    case dvrk::arm_type_t::PSM:
        {
            m_arm = std::make_shared<mtsIntuitiveResearchKitPSM>(m_config.name, m_config.period);
        }
        break;
    case dvrk::arm_type_t::ECM:
        {
            m_arm = std::make_shared<mtsIntuitiveResearchKitECM>(m_config.name, m_config.period);
        }
        break;
    case dvrk::arm_type_t::SUJ_Classic:
        {
            mtsIntuitiveResearchKitSUJ * suj = new mtsIntuitiveResearchKitSUJ(m_config.name, m_config.period);
            if (m_config.simulation == dvrk::simulation_t::SIMULATION_KINEMATIC) {
                suj->set_simulated();
            } else if (m_config.simulation == dvrk::simulation_t::SIMULATION_NONE) {
                m_console->mConnections.Add(m_config.name, "NoMuxReset",
                                            m_IO_component_name, "NoMuxReset");
                m_console->mConnections.Add(m_config.name, "MuxIncrement",
                                            m_IO_component_name, "MuxIncrement");
                m_console->mConnections.Add(m_config.name, "ControlPWM",
                                            m_IO_component_name, "ControlPWM");
                m_console->mConnections.Add(m_config.name, "DisablePWM",
                                            m_IO_component_name, "DisablePWM");
                m_console->mConnections.Add(m_config.name, "MotorUp",
                                            m_IO_component_name, "MotorUp");
                m_console->mConnections.Add(m_config.name, "MotorDown",
                                            m_IO_component_name, "MotorDown");
                m_console->mConnections.Add(m_config.name, "SUJ-Clutch-1",
                                            m_IO_component_name, "SUJ-Clutch-1");
                m_console->mConnections.Add(m_config.name, "SUJ-Clutch-2",
                                            m_IO_component_name, "SUJ-Clutch-2");
                m_console->mConnections.Add(m_config.name, "SUJ-Clutch-3",
                                            m_IO_component_name, "SUJ-Clutch-3");
                m_console->mConnections.Add(m_config.name, "SUJ-Clutch-4",
                                            m_IO_component_name, "SUJ-Clutch-4");
            }
            suj->Configure(m_arm_configuration_file);
            componentManager->AddComponent(suj);
        }
        break;
    case dvrk::arm_type_t::SUJ_Si:
        {
#if sawIntuitiveResearchKit_HAS_SUJ_Si
            mtsIntuitiveResearchKitSUJSi * suj = new mtsIntuitiveResearchKitSUJSi(m_config.name, m_config.period);
            if (m_config.simulation == dvrk::simulation_t::SIMULATION_KINEMATIC) {
                suj->set_simulated();
            }
            suj->Configure(m_arm_configuration_file);
            componentManager->AddComponent(suj);
#else
            CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::arm_proxy_t::ConfigureArm: can't create an arm of type SUJ_Si because sawIntuitiveResearchKit_HAS_SUJ_Si is set to OFF in CMake"
                               << std::endl;
            exit(EXIT_FAILURE);
#endif
        }
        break;
    case dvrk::arm_type_t::SUJ_Fixed:
        {
            mtsIntuitiveResearchKitSUJFixed * suj = new mtsIntuitiveResearchKitSUJFixed(m_config.name, m_config.period);
            suj->Configure(m_arm_configuration_file);
            componentManager->AddComponent(suj);
        }
        break;
    case dvrk::arm_type_t::MTM_DERIVED:
        {
            mtsComponent * component;
            component = componentManager->GetComponent(m_config.name);
            if (component) {
                mtsIntuitiveResearchKitMTM * mtm = dynamic_cast<mtsIntuitiveResearchKitMTM *>(component);
                if (mtm) {
                    std::cerr << CMN_LOG_DETAILS << " is this risky?" << std::endl;
                    m_arm = std::shared_ptr<mtsIntuitiveResearchKitMTM>(mtm, [](mtsIntuitiveResearchKitMTM * p){ ; });
                } else {
                    CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::arm_proxy_t::ConfigureArm: component \""
                                       << m_config.name << "\" doesn't seem to be derived from mtsIntuitiveResearchKitMTM."
                                       << std::endl;
                    exit(EXIT_FAILURE);
                }
            } else {
                CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::arm_proxy_t::ConfigureArm: component \""
                                   << m_config.name << "\" not found."
                                   << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        break;
    case dvrk::arm_type_t::PSM_DERIVED:
        {
            mtsComponent * component;
            component = componentManager->GetComponent(m_config.name);
            if (component) {
                mtsIntuitiveResearchKitPSM * psm = dynamic_cast<mtsIntuitiveResearchKitPSM *>(component);
                if (psm) {
                    std::cerr << CMN_LOG_DETAILS << " is this risky?" << std::endl;
                    m_arm = std::shared_ptr<mtsIntuitiveResearchKitArm>(psm);
                } else {
                    CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::arm_proxy_t::ConfigureArm: component \""
                                       << m_config.name << "\" doesn't seem to be derived from mtsIntuitiveResearchKitPSM."
                                       << std::endl;
                    exit(EXIT_FAILURE);
                }
            } else {
                CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::arm_proxy_t::ConfigureArm: component \""
                                   << m_config.name << "\" not found."
                                   << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        break;
    case dvrk::arm_type_t::ECM_DERIVED:
        {
            mtsComponent * component;
            component = componentManager->GetComponent(m_config.name);
            if (component) {
                mtsIntuitiveResearchKitECM * ecm = dynamic_cast<mtsIntuitiveResearchKitECM *>(component);
                if (ecm) {
                    std::cerr << CMN_LOG_DETAILS << " is this risky?" << std::endl;
                    m_arm = std::shared_ptr<mtsIntuitiveResearchKitArm>(ecm);
                } else {
                    CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::arm_proxy_t::ConfigureArm: component \""
                                       << m_config.name << "\" doesn't seem to be derived from mtsIntuitiveResearchKitECM."
                                       << std::endl;
                    exit(EXIT_FAILURE);
                }
            } else {
                CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::arm_proxy_t::ConfigureArm: component \""
                                   << m_config.name << "\" not found."
                                   << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        break;

    default:
        break;
    }

    // if native or derived active arms
    if (m_config.native_or_derived()
        && !m_config.suj()) {
        CMN_ASSERT(m_arm != nullptr);
        if (m_config.simulation == dvrk::simulation_t::SIMULATION_KINEMATIC) {
            m_arm->set_simulated();
        }
        m_arm->set_calibration_mode(m_calibration_mode);
        m_arm->Configure(m_arm_configuration_file);
        set_base_frame_if_needed();
        componentManager->AddComponent(m_arm.get());

        // for all native arms not simulated, connect a few IOS
        if (m_config.simulation == dvrk::simulation_t::SIMULATION_NONE) {

            if (m_config.psm()) {
                std::vector<std::string> itfs = {"Adapter", "Tool", "ManipClutch", "Dallas"};
                for (const auto & itf : itfs) {
                    m_console->mConnections.Add(m_config.name, itf,
                                                m_IO_component_name, m_config.name + "-" + itf);
                }
            }

            if (m_config.ecm()) {
                m_console->mConnections.Add(m_config.name, "ManipClutch",
                                            m_IO_component_name, m_config.name + "-ManipClutch");
            }

            // for Si patient side, connect the SUJ brakes to buttons on arm
            if ((m_config.psm() || m_config.ecm())
                && (m_arm->generation() == dvrk::generation_t::Si)) {
                std::vector<std::string> itfs = {"SUJClutch", "SUJClutch2", "SUJBrake"};
                for (const auto & itf : itfs) {
                    m_console->mConnections.Add(m_config.name, itf,
                                                m_IO_component_name, m_config.name + "-" + itf);
                }
            }
        }
    }
}


void mtsIntuitiveResearchKitConsole::arm_proxy_t::create_PID(void)
{
    if (!m_config.expects_PID()) {
        return;
    }

    // infer pid config file name
    std::string pid_config = m_config.PID;
    if (m_config.PID !=  "") {
        m_PID_configuration_file = m_config.PID;
        CMN_LOG_INIT_VERBOSE << "ConfigurePID: using \"PID\" file name for arm \""
                             << m_config.name << "\": \""
                             << m_PID_configuration_file << "\"" << std::endl;
    } else {
        // not user defined, try to find the default
        auto generation = this->generation();
        if (m_config.native_or_derived_mtm()) {
            m_PID_configuration_file = "pid/sawControllersPID-MTM.json";
        } else if (m_config.native_or_derived_psm()) {
            if (generation == dvrk::generation_t::Classic) {
                m_PID_configuration_file = "pid/sawControllersPID-PSM.json";
            } else {
                m_PID_configuration_file = "pid/sawControllersPID-PSM-Si.json";
            }
        } else if (m_config.native_or_derived_ecm()) {
            if (generation == dvrk::generation_t::Classic) {
                m_PID_configuration_file = "pid/sawControllersPID-ECM.json";
            } else  {
                m_PID_configuration_file = "pid/sawControllersPID-ECM-Si.json";
            }
        } else {
            m_PID_configuration_file = "pid/sawControllersPID-" + m_config.name + ".json";
        }
        CMN_LOG_INIT_VERBOSE << "ConfigurePID: no \"PID\" file name for arm \""
                             << m_config.name << "\", using default based on type and generation: \""
                             << m_PID_configuration_file << "\"" << std::endl;
    }

    auto configuration_file = m_console->find_file(m_PID_configuration_file);
    if (configuration_file == "") {
        CMN_LOG_INIT_ERROR << "configure_PID: can't find PID file " << m_PID_configuration_file << std::endl;
        exit(EXIT_FAILURE);
    }

    CMN_LOG_INIT_VERBOSE << "configure_PID: found file " << configuration_file << std::endl;
    m_PID_configuration_file = configuration_file;
    m_PID_component_name = m_config.name + "-PID";

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    mtsPID * pid = new mtsPID(m_PID_component_name,
                              (m_config.PID_period != 0.0) ? m_config.PID_period : mtsIntuitiveResearchKit::IOPeriod);
    bool hasIO = true;
    pid->Configure(m_PID_configuration_file);
    if (m_config.simulation == dvrk::simulation_t::SIMULATION_KINEMATIC) {
        pid->SetSimulated();
        hasIO = false;
    }
    componentManager->AddComponent(pid);
    if (hasIO) {
        m_console->mConnections.Add(m_PID_component_name, "RobotJointTorqueInterface",
                                    m_IO_component_name, m_config.name);
        if (m_config.PID_period == 0.0) {
            m_console->mConnections.Add(m_PID_component_name, "ExecIn",
                                        m_IO_component_name, "ExecOut");
        }
    }
}


void mtsIntuitiveResearchKitConsole::arm_proxy_t::set_base_frame_if_needed(void)
{
    if (m_base_frame.ReferenceFrame() != "") {
        if (m_arm == nullptr) {
            CMN_LOG_INIT_ERROR << "arm_proxy_t::set_base_frame_if_needed failed, arm needs to be configured first" << std::endl;
            exit(EXIT_FAILURE);
        }
        m_arm->set_base_frame(m_base_frame);
    }
}


bool mtsIntuitiveResearchKitConsole::arm_proxy_t::Connect(void)
{
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();
    // if the arm is a research kit arm
    if (m_config.native_or_derived()) {
        // Connect arm to IO if not simulated
        if (m_config.expects_IO()) {
            componentManager->Connect(m_config.name, "RobotIO",
                                      m_IO_component_name, m_config.name);
        }
        // connect MTM gripper to IO
        if (m_config.native_or_derived_mtm()
            && !m_config.simulated()) {
            componentManager->Connect(m_config.name, "GripperIO",
                                      m_IO_component_name, m_config.name + "-Gripper");
        }
        // connect PID
        if (m_config.expects_PID()) {
            componentManager->Connect(m_config.name, "PID",
                                      m_PID_component_name, "Controller");
        }
        // connect m_base_frame if needed
        if ((m_base_frame_component_name != "") && (m_base_frame_interface_name != "")) {
            componentManager->Connect(m_base_frame_component_name, m_base_frame_interface_name,
                                      m_config.name, "Arm");
        }
    }
    return true;
}


dvrk::generation_t mtsIntuitiveResearchKitConsole::arm_proxy_t::generation(void) const
{
    if (m_arm != nullptr) {
        return m_arm->generation();
    } else {
        CMN_LOG_INIT_ERROR << "arm_proxy_t::generation failed, arm needs to be configured first" << std::endl;
        exit(EXIT_FAILURE);
    }
    return dvrk::generation_t::GENERATION_UNDEFINED;
}


void mtsIntuitiveResearchKitConsole::arm_proxy_t::CurrentStateEventHandler(const prmOperatingState & currentState)
{
    m_console->SetArmCurrentState(m_config.name, currentState);
}


mtsIntuitiveResearchKitConsole::TeleopECM::TeleopECM(const std::string & name):
    m_name(name)
{
}


void mtsIntuitiveResearchKitConsole::TeleopECM::ConfigureTeleop(const TeleopECMType type,
                                                                const double & period_in_seconds,
                                                                const Json::Value & jsonConfig)
{

    // m_type = type;
    // mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // switch (type) {
    // case TELEOP_ECM:
    //     {
    //         mtsTeleOperationECM * teleop = new mtsTeleOperationECM(m_name, period_in_seconds);
    //         teleop->Configure(jsonConfig);
    //         componentManager->AddComponent(teleop);
    //     }
    //     break;
    // case TELEOP_ECM_DERIVED:
    //     {
    //         mtsComponent * component;
    //         component = componentManager->GetComponent(m_config.name);
    //         if (component) {
    //             mtsTeleOperationECM * teleop = dynamic_cast<mtsTeleOperationECM *>(component);
    //             if (teleop) {
    //                 teleop->Configure(jsonConfig);
    //             } else {
    //                 CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::ConfigureTeleop: component \""
    //                                    << m_config.name << "\" doesn't seem to be derived from mtsTeleOperationECM."
    //                                    << std::endl;
    //             }
    //         } else {
    //             CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::ConfigureTeleop: component \""
    //                                << m_config.name << "\" not found."
    //                                << std::endl;
    //         }
    //     }
    //     break;
    // default:
    //     break;
    // }
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
                                                                const double & period_in_seconds,
                                                                const Json::Value & jsonConfig)
{
    // m_type = type;
    // mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // switch (type) {
    // case TELEOP_PSM:
    //     {
    //         mtsTeleOperationPSM * teleop = new mtsTeleOperationPSM(m_name, period_in_seconds);
    //         teleop->Configure(jsonConfig);
    //         componentManager->AddComponent(teleop);
    //     }
    //     break;
    // case TELEOP_PSM_DERIVED:
    //     {
    //         mtsComponent * component;
    //         component = componentManager->GetComponent(m_config.name);
    //         if (component) {
    //             mtsTeleOperationPSM * teleop = dynamic_cast<mtsTeleOperationPSM *>(component);
    //             if (teleop) {
    //                 teleop->Configure(jsonConfig);
    //             } else {
    //                 CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::ConfigureTeleop: component \""
    //                                    << m_config.name << "\" doesn't seem to be derived from mtsTeleOperationPSM."
    //                                    << std::endl;
    //             }
    //         } else {
    //             CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::ConfigureTeleop: component \""
    //                                << m_config.name << "\" not found."
    //                                << std::endl;
    //         }
    //     }
    //     break;
    // default:
    //     break;
    // }
}


const std::string & mtsIntuitiveResearchKitConsole::TeleopPSM::Name(void) const {
    return m_name;
}


mtsIntuitiveResearchKitConsole::mtsIntuitiveResearchKitConsole(const std::string & componentName):
    mtsTaskFromSignal(componentName, 100),
    m_configured(false),
    mTimeOfLastErrorBeep(0.0),
    mTeleopMTMToCycle(""),
    mTeleopECM(0),
    mOperatorPresent(false),
    mCameraPressed(false),
    m_IO_component_name("io")
{
    // configure search path
    m_config_path.Add(cmnPath::GetWorkingDirectory());
    // add path to source/share directory to find common files.  This
    // will work as long as this component is located in the same
    // parent directory as the "shared" directory.
    m_config_path.Add(std::string(sawIntuitiveResearchKit_SOURCE_CONFIG_DIR), cmnPath::TAIL);
    // default installation directory
    m_config_path.Add(mtsIntuitiveResearchKit::DefaultInstallationDirectory, cmnPath::TAIL);

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
        // Following is Read instead of VoidReturn because it is called before the component
        // is created (i.e., thread not yet running)
        mInterface->AddCommandRead(&mtsIntuitiveResearchKitConsole::ConnectInternal, this,
                                   "connect", false);
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
    m_configured = false;

    std::ifstream jsonStream;
    jsonStream.open(filename.c_str());

    Json::Value jsonConfig, jsonValue;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration" << std::endl
                                 << "File: " << filename << std::endl << "Error(s):" << std::endl
                                 << jsonReader.getFormattedErrorMessages();
        this->m_configured = false;
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
    std::string fullname = m_config_path.Find(filename);
    std::string configDir = fullname.substr(0, fullname.find_last_of('/'));
    m_config_path.Add(configDir, cmnPath::TAIL);

    mtsComponentManager * manager = mtsComponentManager::GetInstance();

    // first, create all custom components and connections, i.e. dynamic loading and creation
    const Json::Value componentManager = jsonConfig["component-manager"];
    if (!componentManager.empty()) {
        if (!manager->ConfigureJSON(componentManager, m_config_path)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure component-manager" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // add text to speech component for the whole system
    m_text_to_speech = std::make_unique<mtsTextToSpeech>();
    manager->AddComponent(m_text_to_speech.get());
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

    const auto json_arms = jsonConfig["arms"];
    for (unsigned int index = 0; index < json_arms.size(); ++index) {
        const auto json_arm = json_arms[index];
        const auto arm_name = json_arm["name"].asString();
        const auto iter = m_arm_proxies.find(arm_name);
        if (iter == m_arm_proxies.end()) {
            // create a new arm proxy if needed
            auto arm_proxy = std::make_shared<arm_proxy_t>(arm_name, this);
            arm_proxy->configure(json_arm);
            m_arm_proxies[arm_name] = arm_proxy;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure arms["
                                     << index << "], arm already exists" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    //     xxxxx
    //     if (!ConfigureArmJSON(arms[index], m_IO_component_name)) {
    //         CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure arms[" << index << "]" << std::endl;
    //         exit(EXIT_FAILURE);
    //     }
    // }

    // loop over all arms to check if IO is needed, also check if some IO configuration files are listed in "io"
    mHasIO = false;
    const auto end = m_arm_proxies.end();
    for (auto iter = m_arm_proxies.begin(); iter != end; ++iter) {
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
    // just check for IO and make sure we don't have io and hid, will be configured later
    jsonValue = jsonConfig["operator-present"];
    if (!jsonValue.empty()) {
        // check if operator present uses IO
        Json::Value jsonConfigFile = jsonValue["io"];
        if (!jsonConfigFile.empty()) {
            mHasIO = true;
            jsonConfigFile = jsonValue["hid"];
            if (!jsonConfigFile.empty()) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: operator-present can't have both io and hid" << std::endl;
                exit(EXIT_FAILURE);
            }
        }
    }
    // create IO if needed and configure IO
    if (mHasIO) {
        mtsRobotIO1394 * io = new mtsRobotIO1394(m_IO_component_name, periodIO, port);
        io->SetProtocol(protocol);
        io->SetWatchdogPeriod(watchdogTimeout);
        // configure for each arm
        for (auto iter = m_arm_proxies.begin(); iter != end; ++iter) {
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
                    const std::string configFile = find_file(configFiles[index].asString());
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
                const std::string configFile = find_file(configFiles.asString());
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
            // check if user wants to close all relays
            Json::Value close_all_relays = jsonValue["close-all-relays"];
            if (!close_all_relays.empty()) {
                m_close_all_relays_from_config = close_all_relays.asBool();
            }
        }
        // configure IO for operator present
        jsonValue = jsonConfig["operator-present"];
        if (!jsonValue.empty()) {
            // check if operator present uses IO
            Json::Value jsonConfigFile = jsonValue["io"];
            if (!jsonConfigFile.empty()) {
                const std::string configFile = find_file(jsonConfigFile.asString());
                if (configFile == "") {
                    CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find configuration file "
                                             << jsonConfigFile.asString() << std::endl;
                    exit(EXIT_FAILURE);
                }
                CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring operator present using \""
                                           << configFile << "\"" << std::endl;
                io->Configure(configFile);
            } else {
                jsonConfigFile = jsonValue["hid"];
            }
        }
        // configure for endoscope focus
        jsonValue = jsonConfig["endoscope-focus"];
        if (!jsonValue.empty()) {
            // check if operator present uses IO
            Json::Value jsonConfigFile = jsonValue["io"];
            if (!jsonConfigFile.empty()) {
                const std::string configFile = find_file(jsonConfigFile.asString());
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
        m_IO_interface = AddInterfaceRequired("IO");
        if (m_IO_interface) {
            m_IO_interface->AddFunction("close_all_relays", IO.close_all_relays);
            m_IO_interface->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
                                                 this, "error");
            m_IO_interface->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                 this, "warning");
            m_IO_interface->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                 this, "status");
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to create IO required interface" << std::endl;
            exit(EXIT_FAILURE);
        }
        mtsComponentManager::GetInstance()->AddComponent(io);
        if (m_IO_interface) {
            mConnections.Add(this->GetName(), "IO",
                             io->GetName(), "Configuration");
        }
    }

    // now can configure PID and Arms
    for (auto iter = m_arm_proxies.begin(); iter != end; ++iter) {
        auto & arm_proxy = iter->second;
        arm_proxy->create_arm();
        arm_proxy->create_PID();
        add_arm_interfaces(arm_proxy);
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
        // first case, using io to communicate with daVinci original head sensore
        Json::Value operatorPresentConfiguration = operatorPresent["io"];
        if (!operatorPresentConfiguration.empty()) {
            const std::string headSensorName = "daVinciHeadSensor";
            mHeadSensor = new mtsDaVinciHeadSensor(headSensorName);
            mtsComponentManager::GetInstance()->AddComponent(mHeadSensor);
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
        } else {
            // second case, using hid config for goovis head sensor
            operatorPresentConfiguration = operatorPresent["hid"];
            if (!operatorPresentConfiguration.empty()) {
#if sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR
                std::string relativeConfigFile = operatorPresentConfiguration.asString();
                CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring hid head sensor with \""
                                           << relativeConfigFile << "\"" << std::endl;
                const std::string configFile = find_file(relativeConfigFile);
                if (configFile == "") {
                    CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find configuration file "
                                             << relativeConfigFile << std::endl;
                    exit(EXIT_FAILURE);
                }
                const std::string headSensorName = "HIDHeadSensor";
                mHeadSensor = new mtsHIDHeadSensor(headSensorName);
                mHeadSensor->Configure(configFile);
                mtsComponentManager::GetInstance()->AddComponent(mHeadSensor);
                // main DInput is OperatorPresent comming from the newly added component
                mDInputSources["OperatorPresent"] = InterfaceComponentType(headSensorName, "OperatorPresent");
#else
                CMN_LOG_CLASS_INIT_ERROR << "Configure: can't use HID head sensor." << std::endl
                                         << "The code has been compiled with sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR OFF." << std::endl
                                         << "Re-run CMake, re-compile and try again." << std::endl;
                exit(EXIT_FAILURE);
#endif
            }
        }
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

    // search for SUJs, real, not Fixed
    // for (auto iter = m_arm_proxies.begin(); iter != end; ++iter) {
    //     if ((iter->second->m_type == Arm::ARM_SUJ_Classic)
    //         || (iter->second->m_type == Arm::ARM_SUJ_Si)) {
    //         m_SUJ = iter->second;
    //     }
    // }

    // if (m_SUJ) {
    //     for (auto iter = m_arm_proxies.begin(); iter != end; ++iter) {
    //         Arm * arm = iter->second;
    //         // only for PSM and ECM when not simulated
    //         if (((arm->m_type == Arm::ARM_ECM)
    //              || (arm->m_type == Arm::ARM_ECM_DERIVED)
    //              || (arm->m_type == Arm::ARM_PSM)
    //              || (arm->m_type == Arm::ARM_PSM_DERIVED)
    //              )
    //             && (arm->m_simulation == Arm::SIMULATION_NONE)) {
    //             arm->SUJInterfaceRequiredFromIO = this->AddInterfaceRequired("SUJClutch-" + arm->m_config.name + "-IO");
    //             arm->SUJInterfaceRequiredFromIO->AddEventHandlerWrite(&Arm::SUJClutchEventHandlerFromIO, arm, "Button");
    //             if (arm->m_generation == mtsIntuitiveResearchKitArm::GENERATION_Si) {
    //                 arm->SUJInterfaceRequiredFromIO2 = this->AddInterfaceRequired("SUJClutchBack-" + arm->m_config.name + "-IO");
    //                 arm->SUJInterfaceRequiredFromIO2->AddEventHandlerWrite(&Arm::SUJClutchEventHandlerFromIO, arm, "Button");
    //             }
    //             arm->SUJInterfaceRequiredToSUJ = this->AddInterfaceRequired("SUJClutch-" + arm->m_config.name);
    //             arm->SUJInterfaceRequiredToSUJ->AddFunction("Clutch", arm->SUJClutch);
    //         }
    //     }
    // }

    m_configured = true;
}

const bool & mtsIntuitiveResearchKitConsole::Configured(void) const
{
    return m_configured;
}

void mtsIntuitiveResearchKitConsole::Startup(void)
{
    std::string message = this->GetName();
    message.append(" started, dVRK ");
    message.append(sawIntuitiveResearchKit_VERSION);
    message.append(" / cisst ");
    message.append(cisst_VERSION);
    mInterface->SendStatus(message);

    // close all relays if needed
    if (m_close_all_relays_from_config) {
        IO.close_all_relays();
    }

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
        prompts.push_back("ROS 2!");
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

std::string mtsIntuitiveResearchKitConsole::GetArmIOComponentName(const std::string & arm_name)
{
    auto armIterator = m_arm_proxies.find(arm_name);
    if (armIterator != m_arm_proxies.end()) {
        return armIterator->second->m_IO_component_name;
    }
    return "";
}

bool mtsIntuitiveResearchKitConsole::AddTeleopECMInterfaces(TeleopECM * teleop)
{
    // teleop->InterfaceRequired = this->AddInterfaceRequired(teleop->m_config.name);
    // if (teleop->InterfaceRequired) {
    //     teleop->InterfaceRequired->AddFunction("state_command", teleop->state_command);
    //     teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler, this, "error");
    //     teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler, this, "warning");
    //     teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler, this, "status");
    // } else {
    //     CMN_LOG_CLASS_INIT_ERROR << "AddTeleopECMInterfaces: failed to add Main interface for teleop \""
    //                              << teleop->m_config.name << "\"" << std::endl;
    //     return false;
    // }
    return true;
}

bool mtsIntuitiveResearchKitConsole::AddTeleopPSMInterfaces(TeleopPSM * teleop)
{
    // teleop->InterfaceRequired = this->AddInterfaceRequired(teleop->m_config.name);
    // if (teleop->InterfaceRequired) {
    //     teleop->InterfaceRequired->AddFunction("state_command", teleop->state_command);
    //     teleop->InterfaceRequired->AddFunction("set_scale", teleop->set_scale);
    //     teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler, this, "error");
    //     teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler, this, "warning");
    //     teleop->InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler, this, "status");
    // } else {
    //     CMN_LOG_CLASS_INIT_ERROR << "AddTeleopPSMInterfaces: failed to add Main interface for teleop \""
    //                              << teleop->m_config.name << "\"" << std::endl;
    //     return false;
    // }
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

// bool mtsIntuitiveResearchKitConsole::ConfigureArmJSON(const Json::Value & jsonArm,
//                                                       const std::string & ioComponentName)
// {
//     // create search path based on optional system
//     arm_pointer->m_config_path = m_config_path;
//     jsonValue = jsonArm["system"];
//     if (!jsonValue.empty()) {
//         arm_pointer->m_config_path.Add(std::string(sawIntuitiveResearchKit_SOURCE_CONFIG_DIR)
//                                        + "/" + jsonValue.asString() + "/",
//                                        cmnPath::TAIL);
//     }

//     // set arm calibration mode based on console calibration mode
//     arm_pointer->m_calibration_mode = m_calibration_mode;

//     // IO if expected
//     if (arm_pointer->expects_IO()) {
//         jsonValue = jsonArm["io"];
//         if (!jsonValue.empty()) {
//             arm_pointer->m_IO_configuration_file = find_file(jsonValue.asString());
//             if (arm_pointer->m_IO_configuration_file == "") {
//                 CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find IO file " << jsonValue.asString() << std::endl;
//                 return false;
//             }
//         } else {
//             // try to find default if serial number has been provided
//             if (arm_pointer->m_serial != "") {
//                 std::string defaultFile = "sawRobotIO1394-" + arm_name + "-" + arm_pointer->m_serial + ".xml";
//                 arm_pointer->m_IO_configuration_file = find_file(defaultFile);
//                 if (arm_pointer->m_IO_configuration_file == "") {
//                     CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find IO file " << defaultFile << std::endl;
//                     return false;
//                 }
//             } else {
//                 // no io nor serial
//                 CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"io\" setting for arm \""
//                                          << arm_name << "\" and \"serial\" is not provided so we can't search for it" << std::endl;
//                 return false;
//             }
//         }
//         // IO for MTM gripper
//         if ((arm_pointer->m_type == Arm::ARM_MTM)
//             || (arm_pointer->m_type == Arm::ARM_MTM_DERIVED)) {
//             jsonValue = jsonArm["io-gripper"];
//             if (!jsonValue.empty()) {
//                 arm_pointer->m_IO_gripper_configuration_file = find_file(jsonValue.asString());
//                 if (arm_pointer->m_IO_gripper_configuration_file == "") {
//                     CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find IO gripper file "
//                                              << jsonValue.asString() << std::endl;
//                     return false;
//                 }
//             } else {
//                 // try to find default if serial number has been provided
//                 if (arm_pointer->m_serial != "") {
//                     std::string defaultFile = "sawRobotIO1394-" + arm_name + "-gripper-" + arm_pointer->m_serial + ".xml";
//                     arm_pointer->m_IO_gripper_configuration_file = find_file(defaultFile);
//                     if (arm_pointer->m_IO_gripper_configuration_file == "") {
//                         CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find IO gripper file " << defaultFile << std::endl;
//                         return false;
//                     }
//                 } else {
//                     // no io nor serial
//                     CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find \"io-gripper\" setting for arm \""
//                                              << arm_name << "\" and \"serial\" is not provided so we can't search for it" << std::endl;
//                     return false;
//                 }
//             }
//         }
//     }

//     // PID only required for MTM, PSM and ECM (and derived)
//     if (arm_pointer->expects_PID()) {
//         jsonValue = jsonArm["pid"];
//         if (!jsonValue.empty()) {
//             arm_pointer->m_PID_configuration_file = find_file(jsonValue.asString());
//             if (arm_pointer->m_PID_configuration_file == "") {
//                 CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: can't find PID file " << jsonValue.asString() << std::endl;
//                 return false;
//             }
//         }
//     }


//         jsonValue = jsonArm["base-frame"];
//         if (!jsonValue.empty()) {
//             Json::Value fixedJson = jsonValue["transform"];
//             if (!fixedJson.empty()) {
//                 std::string reference = jsonValue["reference-frame"].asString();
//                 if (reference.empty()) {
//                     CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: both \"transform\" (4x4) and \"reference-frame\" (name) must be provided with \"base-frame\" for arm \""
//                                              << arm_name << "\"" << std::endl;
//                     return false;
//                 }
//                 vctFrm4x4 frame;
//                 cmnDataJSON<vctFrm4x4>::DeSerializeText(frame, fixedJson);
//                 arm_pointer->m_base_frame.Goal().From(frame);
//                 arm_pointer->m_base_frame.ReferenceFrame() = reference;
//                 arm_pointer->m_base_frame.Valid() = true;
//             } else {
//                 arm_pointer->m_base_frame_component_name = jsonValue.get("component", "").asString();
//                 arm_pointer->m_base_frame_interface_name = jsonValue.get("interface", "").asString();
//                 if ((arm_pointer->m_base_frame_component_name == "")
//                     || (arm_pointer->m_base_frame_interface_name == "")) {
//                     CMN_LOG_CLASS_INIT_ERROR << "ConfigureArmJSON: both \"component\" and \"interface\" OR \"transform\" (4x4) and \"reference-frame\" (name) must be provided with \"base-frame\" for arm \""
//                                              << arm_name << "\"" << std::endl;
//                     return false;
//                 }
//             }
//         }
//     }

//     // read period if present
//     jsonValue = jsonArm["period"];
//     if (!jsonValue.empty()) {
//         arm_pointer->m_arm_period = jsonValue.asFloat();
//     }

//     // add the arm if it's a new one
//     if (armIterator == m_arm_proxies.end()) {
//         AddArm(arm_pointer);
//     }
//     return true;
// }

bool mtsIntuitiveResearchKitConsole::ConfigureECMTeleopJSON(const Json::Value & jsonTeleop)
{
    // const std::string mtmLeftName = jsonTeleop["mtm-left"].asString();
    // const std::string mtmRightName = jsonTeleop["mtm-right"].asString();
    // const std::string ecmName = jsonTeleop["ecm"].asString();
    // // all must be provided
    // if ((mtmLeftName == "") || (mtmRightName == "") || (ecmName == "")) {
    //     CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: \"mtm-left\", \"mtm-right\" and \"ecm\" must be provided as strings" << std::endl;
    //     return false;
    // }

    // if (mtmLeftName == mtmRightName) {
    //     CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: \"mtm-left\" and \"mtm-right\" must be different" << std::endl;
    //     return false;
    // }
    // std::string
    //     mtmLeftComponent, mtmLeftInterface,
    //     mtmRightComponent, mtmRightInterface,
    //     ecmComponent, ecmInterface;
    // // check that both arms have been defined and have correct type
    // Arm * arm_pointer;
    // auto armIterator = m_arm_proxies.find(mtmLeftName);
    // if (armIterator == m_arm_proxies.end()) {
    //     CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: mtm left\""
    //                              << mtmLeftName << "\" is not defined in \"arms\"" << std::endl;
    //     return false;
    // } else {
    //     arm_pointer = armIterator->second;
    //     if (!arm_pointer->mtm()) {
    //         CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: mtm left\""
    //                                  << mtmLeftName << "\" type must be some kind of MTM" << std::endl;
    //         return false;
    //     }
    //     mtmLeftComponent = arm_pointer->ComponentName();
    //     mtmLeftInterface = arm_pointer->InterfaceName();
    // }
    // armIterator = m_arm_proxies.find(mtmRightName);
    // if (armIterator == m_arm_proxies.end()) {
    //     CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: mtm right\""
    //                              << mtmRightName << "\" is not defined in \"arms\"" << std::endl;
    //     return false;
    // } else {
    //     arm_pointer = armIterator->second;
    //     if (!arm_pointer->mtm()) {
    //         CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: mtm right\""
    //                                  << mtmRightName << "\" type must be some kind of MTM" << std::endl;
    //         return false;
    //     }
    //     mtmRightComponent = arm_pointer->ComponentName();
    //     mtmRightInterface = arm_pointer->InterfaceName();
    // }
    // armIterator = m_arm_proxies.find(ecmName);
    // if (armIterator == m_arm_proxies.end()) {
    //     CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: ecm \""
    //                              << ecmName << "\" is not defined in \"arms\"" << std::endl;
    //     return false;
    // } else {
    //     arm_pointer = armIterator->second;
    //     if (!arm_pointer->ecm()) {
    //         CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: ecm \""
    //                                  << ecmName << "\" type must be some kind of ECM" << std::endl;
    //         return false;
    //     }
    //     ecmComponent = arm_pointer->ComponentName();
    //     ecmInterface = arm_pointer->InterfaceName();
    // }

    // // check if pair already exist and then add
    // const std::string name = mtmLeftName + "-" + mtmRightName + "-" + ecmName;
    // if (mTeleopECM == 0) {
    //     // create a new teleop if needed
    //     mTeleopECM = new TeleopECM(name);
    //     // schedule connections
    //     mConnections.Add(name, "MTML", mtmLeftComponent, mtmLeftInterface);
    //     mConnections.Add(name, "MTMR", mtmRightComponent, mtmRightInterface);
    //     mConnections.Add(name, "ECM", ecmComponent, ecmInterface);
    //     mConnections.Add(name, "Clutch", this->GetName(), "Clutch"); // console clutch
    //     mConnections.Add(this->GetName(), name, name, "Setting");
    // } else {
    //     CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: there is already an ECM teleop" << std::endl;
    //     return false;
    // }

    // Json::Value jsonValue;
    // jsonValue = jsonTeleop["type"];
    // if (!jsonValue.empty()) {
    //     std::string typeString = jsonValue.asString();
    //     if (typeString == "TELEOP_ECM") {
    //         mTeleopECM->m_type = TeleopECM::TELEOP_ECM;
    //     } else if (typeString == "TELEOP_ECM_DERIVED") {
    //         mTeleopECM->m_type = TeleopECM::TELEOP_ECM_DERIVED;
    //     } else if (typeString == "TELEOP_ECM_GENERIC") {
    //         mTeleopECM->m_type = TeleopECM::TELEOP_ECM_GENERIC;
    //     } else {
    //         CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: teleop " << name << ": invalid type \""
    //                                  << typeString << "\", needs to be TELEOP_ECM, TELEOP_ECM_DERIVED or TELEOP_ECM_GENERIC" << std::endl;
    //         return false;
    //     }
    // } else {
    //     // default value
    //     mTeleopECM->m_type = TeleopECM::TELEOP_ECM;
    // }

    // // read period if present
    // double period = mtsIntuitiveResearchKit::TeleopPeriod;
    // jsonValue = jsonTeleop["period"];
    // if (!jsonValue.empty()) {
    //     period = jsonValue.asFloat();
    // }
    // // for backward compatibility, send warning
    // jsonValue = jsonTeleop["rotation"];
    // if (!jsonValue.empty()) {
    //     CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: teleop " << name << ": \"rotation\" must now be defined under \"configure-parameter\" or in a separate configuration file" << std::endl;
    //     return false;
    // }
    // const Json::Value jsonTeleopConfig = jsonTeleop["configure-parameter"];
    // mTeleopECM->ConfigureTeleop(mTeleopECM->m_type, period, jsonTeleopConfig);
    // AddTeleopECMInterfaces(mTeleopECM);
    return true;
}

bool mtsIntuitiveResearchKitConsole::ConfigurePSMTeleopJSON(const Json::Value & jsonTeleop)
{
    // const std::string mtmName = jsonTeleop["mtm"].asString();
    // const std::string psmName = jsonTeleop["psm"].asString();
    // // both are required
    // if ((mtmName == "") || (psmName == "")) {
    //     CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: both \"mtm\" and \"psm\" must be provided as strings" << std::endl;
    //     return false;
    // }

    // std::string mtmComponent, mtmInterface, psmComponent, psmInterface;
    // // check that both arms have been defined and have correct type
    // Arm * arm_pointer;
    // auto armIterator = m_arm_proxies.find(mtmName);
    // if (armIterator == m_arm_proxies.end()) {
    //     CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: mtm \""
    //                              << mtmName << "\" is not defined in \"arms\"" << std::endl;
    //     return false;
    // } else {
    //     arm_pointer = armIterator->second;
    //     if (!arm_pointer->mtm()) {
    //         CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: mtm \""
    //                                  << mtmName << "\" type must be some kind of MTM" << std::endl;
    //         return false;
    //     }
    //     mtmComponent = arm_pointer->ComponentName();
    //     mtmInterface = arm_pointer->InterfaceName();
    // }
    // armIterator = m_arm_proxies.find(psmName);
    // if (armIterator == m_arm_proxies.end()) {
    //     CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: psm \""
    //                              << psmName << "\" is not defined in \"arms\"" << std::endl;
    //     return false;
    // } else {
    //     arm_pointer = armIterator->second;
    //     if (!arm_pointer->psm()) {
    //         CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: psm \""
    //                                  << psmName << "\" type must be some kind of PSM" << std::endl;
    //         return false;
    //     }
    //     psmComponent = arm_pointer->ComponentName();
    //     psmInterface = arm_pointer->InterfaceName();
    // }

    // // see if there is a base frame defined for the PSM
    // Json::Value jsonValue = jsonTeleop["psm-base-frame"];
    // std::string baseFrameComponent, baseFrameInterface;
    // if (!jsonValue.empty()) {
    //     baseFrameComponent = jsonValue.get("component", "").asString();
    //     baseFrameInterface = jsonValue.get("interface", "").asString();
    //     if ((baseFrameComponent == "") || (baseFrameInterface == "")) {
    //         CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: both \"component\" and \"interface\" must be provided with \"psm-base-frame\" for teleop \""
    //                                  << mtmName << "-" << psmName << "\"" << std::endl;
    //         return false;
    //     }
    // }

    // // check if pair already exist and then add
    // const std::string name = mtmName + "-" + psmName;
    // const auto teleopIterator = mTeleopsPSM.find(name);
    // TeleopPSM * teleopPointer = 0;
    // if (teleopIterator == mTeleopsPSM.end()) {
    //     // create a new teleop if needed
    //     teleopPointer = new TeleopPSM(name, mtmName, psmName);
    //     // schedule connections
    //     mConnections.Add(name, "MTM", mtmComponent, mtmInterface);
    //     mConnections.Add(name, "PSM", psmComponent, psmInterface);
    //     mConnections.Add(name, "Clutch", this->GetName(), "Clutch"); // clutch from console
    //     mConnections.Add(this->GetName(), name, name, "Setting");
    //     if ((baseFrameComponent != "")
    //         && (baseFrameInterface != "")) {
    //         mConnections.Add(name, "PSM-base-frame",
    //                          baseFrameComponent, baseFrameInterface);
    //     }

    //     // insert
    //     mTeleopsPSMByMTM.insert(std::make_pair(mtmName, teleopPointer));
    //     mTeleopsPSMByPSM.insert(std::make_pair(psmName, teleopPointer));

    //     // first MTM with multiple PSMs is selected for single tap
    //     if ((mTeleopsPSMByMTM.count(mtmName) > 1)
    //         && (mTeleopMTMToCycle == "")) {
    //         mTeleopMTMToCycle = mtmName;
    //     }
    //     // check if we already have a teleop for the same PSM
    //     std::string mtmUsingThatPSM;
    //     GetMTMSelectedForPSM(psmName, mtmUsingThatPSM);
    //     if (mtmUsingThatPSM != "") {
    //         teleopPointer->SetSelected(false);
    //         CMN_LOG_CLASS_INIT_WARNING << "ConfigurePSMTeleopJSON: psm \""
    //                                    << psmName << "\" is already selected to be controlled by mtm \""
    //                                    << mtmUsingThatPSM << "\", component \""
    //                                    << name << "\" is added but not selected"
    //                                    << std::endl;
    //     } else {
    //         // check if we already have a teleop for the same PSM
    //         std::string psmUsingThatMTM;
    //         GetPSMSelectedForMTM(mtmName, psmUsingThatMTM);
    //         if (psmUsingThatMTM != "") {
    //             teleopPointer->SetSelected(false);
    //             CMN_LOG_CLASS_INIT_WARNING << "ConfigurePSMTeleopJSON: mtm \""
    //                                        << mtmName << "\" is already selected to control psm \""
    //                                        << psmUsingThatMTM << "\", component \""
    //                                        << name << "\" is added but not selected"
    //                                        << std::endl;
    //         } else {
    //             // neither the MTM nor PSM are used, let's activate that pair
    //             teleopPointer->SetSelected(true);
    //         }
    //     }
    //     // finally add the new teleop
    //     mTeleopsPSM[name] = teleopPointer;
    // } else {
    //     CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: there is already a teleop for the pair \""
    //                              << name << "\"" << std::endl;
    //     return false;
    // }

    // jsonValue = jsonTeleop["type"];
    // if (!jsonValue.empty()) {
    //     std::string typeString = jsonValue.asString();
    //     if (typeString == "TELEOP_PSM") {
    //         teleopPointer->m_type = TeleopPSM::TELEOP_PSM;
    //     } else if (typeString == "TELEOP_PSM_DERIVED") {
    //         teleopPointer->m_type = TeleopPSM::TELEOP_PSM_DERIVED;
    //     } else if (typeString == "TELEOP_PSM_GENERIC") {
    //         teleopPointer->m_type = TeleopPSM::TELEOP_PSM_GENERIC;
    //     } else {
    //         CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: teleop " << name << ": invalid type \""
    //                                  << typeString << "\", needs to be TELEOP_PSM, TELEOP_PSM_DERIVED or TELEOP_PSM_GENERIC" << std::endl;
    //         return false;
    //     }
    // } else {
    //     // default value
    //     teleopPointer->m_type = TeleopPSM::TELEOP_PSM;
    // }

    // // read period if present
    // double period = mtsIntuitiveResearchKit::TeleopPeriod;
    // jsonValue = jsonTeleop["period"];
    // if (!jsonValue.empty()) {
    //     period = jsonValue.asFloat();
    // }
    // const Json::Value jsonTeleopConfig = jsonTeleop["configure-parameter"];
    // teleopPointer->ConfigureTeleop(teleopPointer->m_type, period, jsonTeleopConfig);
    // AddTeleopPSMInterfaces(teleopPointer);
    return true;
}

bool mtsIntuitiveResearchKitConsole::add_arm_interfaces(std::shared_ptr<arm_proxy_t> & arm)
{
    // IO
    if (arm->m_config.expects_IO()) {
        const std::string interfaceNameIO = "IO-" + arm->m_config.name;
        arm->IOInterfaceRequired = AddInterfaceRequired(interfaceNameIO);
        if (arm->IOInterfaceRequired) {
            arm->IOInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
                                                           this, "error");
            arm->IOInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                           this, "warning");
            arm->IOInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                           this, "status");
            mConnections.Add(this->GetName(), interfaceNameIO,
                             arm->m_IO_component_name, arm->m_config.name);
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "AddArmInterfaces: failed to add IO interface for arm \""
                                     << arm->m_config.name << "\"" << std::endl;
            return false;
        }
        // is the arm is a PSM, since it has an IO, it also has a
        // Dallas chip interface and we want to see the messages
        if (arm->m_config.native_or_derived_psm()) {
            const std::string interfaceNameIODallas = "IO-Dallas-" + arm->m_config.name;
            arm->IODallasInterfaceRequired = AddInterfaceRequired(interfaceNameIODallas);
            if (arm->IODallasInterfaceRequired) {
                arm->IODallasInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
                                                                     this, "error");
                arm->IODallasInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                                     this, "warning");
                arm->IODallasInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                                     this, "status");
                mConnections.Add(this->GetName(), interfaceNameIODallas,
                                 arm->m_IO_component_name, arm->m_config.name + "-Dallas");
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "AddArmInterfaces: failed to add IO Dallas interface for arm \""
                                         << arm->m_config.name << "\"" << std::endl;
                return false;
            }
        }
    }

    // PID
    if (arm->m_config.expects_PID()) {
        const std::string interfaceNamePID = "PID-" + arm->m_config.name;
        arm->PIDInterfaceRequired = AddInterfaceRequired(interfaceNamePID);
        if (arm->PIDInterfaceRequired) {
            arm->PIDInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
                                                            this, "error");
            arm->PIDInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                            this, "warning");
            arm->PIDInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                            this, "status");
            mConnections.Add(this->GetName(), interfaceNamePID,
                             arm->m_config.name + "-PID", "Controller");
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "AddArmInterfaces: failed to add PID interface for arm \""
                                     << arm->m_config.name << "\"" << std::endl;
            return false;
        }
    }

    // arm interface
    const std::string interfaceNameArm = arm->m_config.name;
    arm->ArmInterfaceRequired = AddInterfaceRequired(interfaceNameArm);
    if (arm->ArmInterfaceRequired) {
        arm->ArmInterfaceRequired->AddFunction("state_command", arm->state_command);
        if (!arm->m_config.suj()) {
            arm->ArmInterfaceRequired->AddFunction("hold", arm->hold, MTS_OPTIONAL);
        }
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
                                                        this, "error");
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                        this, "warning");
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                        this, "status");
        arm->ArmInterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::arm_proxy_t::CurrentStateEventHandler,
                                                        arm.get(), "operating_state");
        mConnections.Add(this->GetName(), interfaceNameArm,
                         arm->m_arm_component_name, arm->m_arm_interface_name);
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "AddArmInterfaces: failed to add Main interface for arm \""
                                 << arm->m_config.name << "\"" << std::endl;
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
                              m_text_to_speech->GetName(), "Commands");

    for (auto & iter : m_arm_proxies) {
        std::shared_ptr<arm_proxy_t> arm = iter.second;
        // arm specific interfaces
        arm->Connect();
        // connect to SUJ if needed
        if (arm->SUJInterfaceRequiredFromIO) {
            componentManager->Connect(this->GetName(), arm->SUJInterfaceRequiredFromIO->GetName(),
                                      arm->m_IO_component_name, arm->m_config.name + "-SUJClutch");
        }
        if (arm->SUJInterfaceRequiredFromIO2) {
            componentManager->Connect(this->GetName(), arm->SUJInterfaceRequiredFromIO2->GetName(),
                                      arm->m_IO_component_name, arm->m_config.name + "-SUJClutch2");
        }
        if (arm->SUJInterfaceRequiredToSUJ) {
            componentManager->Connect(this->GetName(), arm->SUJInterfaceRequiredToSUJ->GetName(),
                                      "SUJ", arm->m_config.name);
        }
    }

    return true;
}

// ConnectInternal is called before the thread is running, and therefore we force it (by using const_cast)
// to have the signature of a CommandRead, so that it can be executed in the caller's thread.
void mtsIntuitiveResearchKitConsole::ConnectInternal(bool &ret) const
{
    ret = const_cast<mtsIntuitiveResearchKitConsole *>(this)->Connect();
}

std::string mtsIntuitiveResearchKitConsole::find_file(const std::string & filename) const
{
    return m_config_path.Find(filename);
}

void mtsIntuitiveResearchKitConsole::power_off(void)
{
    teleop_enable(false);
    for (auto & arm : m_arm_proxies) {
        arm.second->state_command(std::string("disable"));
    }
}

void mtsIntuitiveResearchKitConsole::power_on(void)
{
    DisableFaultyArms();
    for (auto & arm : m_arm_proxies) {
        arm.second->state_command(std::string("enable"));
    }
}

void mtsIntuitiveResearchKitConsole::home(void)
{
    DisableFaultyArms();
    bool allArmsEnabled = true;
    // enable all arms that need it
    for (auto & arm : m_arm_proxies) {
        auto armState = ArmStates.find(arm.first);
        if (// search if we already have a state
            (armState != ArmStates.end())
            // and the arm is disabled
            && (armState->second.State() == prmOperatingState::DISABLED) ) {
            arm.second->state_command(std::string("enable"));
            allArmsEnabled = false;
        }
    }
    if (!allArmsEnabled) {
        osaSleep(1.0 * cmn_s);
    }
    for (auto & arm : m_arm_proxies) {
        arm.second->state_command(std::string("home"));
    }
}

void mtsIntuitiveResearchKitConsole::DisableFaultyArms(void)
{
    for (auto & arm : m_arm_proxies) {
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
    // if we have an SUJ, make sure it's ready
    if (enable && m_SUJ) {
        const auto sujState = ArmStates.find("SUJ");
        if ((sujState == ArmStates.end())
            || (sujState->second.State() != prmOperatingState::ENABLED)) {
            mTeleopEnabled = false;
        }
    }
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
        bool holdNeeded = false;
        for (auto & iterTeleopPSM : mTeleopsPSM) {
            iterTeleopPSM.second->state_command(std::string("disable"));
            if (mTeleopPSMRunning) {
                holdNeeded = true;
            }
            mTeleopPSMRunning = false;
        }

        if (mTeleopECM) {
            mTeleopECM->state_command(std::string("disable"));
            if (mTeleopECMRunning) {
                holdNeeded = true;
            }
            mTeleopECMRunning = false;
        }

        // hold arms if we stopped any teleop
        if (holdNeeded) {
            for (auto & arm_proxy : m_arm_proxies) {
                if (arm_proxy.second->m_config.mtm()
                    && arm_proxy.second->hold.IsValid()) {
                    arm_proxy.second->hold();
                }
            }
        }
        return;
    }

    // if none are running, hold
    if (!mTeleopECMRunning && !mTeleopPSMRunning) {
        for (auto & arm_proxy : m_arm_proxies) {
            if (arm_proxy.second->m_config.mtm()
                && arm_proxy.second->hold.IsValid()) {
                arm_proxy.second->hold();
            }
        }
    }

    // all fine
    bool readyForTeleop = mOperatorPresent;

    for (auto & arm_proxy : m_arm_proxies) {
        if (arm_proxy.second->m_SUJ_clutched) {
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

void mtsIntuitiveResearchKitConsole::SetArmCurrentState(const std::string & arm_name,
                                                        const prmOperatingState & currentState)
{
    if (mTeleopDesired && !mTeleopEnabled) {
        teleop_enable(true);
    }

    // save state
    ArmStates[arm_name] = currentState;

    // emit event (for Qt GUI)
    std::string payload = "";
    if (currentState.IsEnabledAndHomed()) {
        payload = "ENABLED";
    } else if (currentState.State() == prmOperatingState::FAULT) {
        payload = "FAULT";
    }
    ConfigurationEvents.ArmCurrentState(prmKeyValue(arm_name, payload));
}
