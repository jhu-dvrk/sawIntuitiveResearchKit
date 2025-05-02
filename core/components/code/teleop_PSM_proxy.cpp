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


mtsIntuitiveResearchKitConsole::IO_proxy_t::IO_proxy_t(const std::string & name,
                                                       mtsIntuitiveResearchKitConsole * console,
                                                       dvrk::IO_proxy_configuration_t * config):
    m_name(name),
    m_console(console),
    m_config(config)
{
}


void mtsIntuitiveResearchKitConsole::IO_proxy_t::post_configure(void)
{
    if (m_config->period > 1.0 * cmn_ms) {
        std::stringstream message;
        message << "post_configure for IO " << m_config->name << std::endl
                << "----------------------------------------------------" << std::endl
                << " Warning:" << std::endl
                << "   The period provided is quite high, i.e. " << m_config->period << std::endl
                << "   seconds.  We strongly recommend you change it to" << std::endl
                << "   a value below 1 ms, i.e. 0.001." << std::endl
                << "----------------------------------------------------";
        std::cerr << "mtsIntuitiveResearchKitConsole::" << message.str() << std::endl;
        CMN_LOG_INIT_WARNING << message.str() << std::endl;
    }

    if (m_config->watchdog_timeout > 300.0 * cmn_ms) {
        m_config->watchdog_timeout = 300.0 * cmn_ms;
        CMN_LOG_INIT_WARNING << "post_configure for IO "
                             << m_config->name
                             << ": watchdog_timeout has to be lower than 300 ms, it has been capped at 300 ms" << std::endl;
    }
    if (m_config->watchdog_timeout <= 0.0) {
        m_config->watchdog_timeout = 0.0;
        std::stringstream message;
        message << "post_configure for IO " << m_config->name << std::endl
                << "----------------------------------------------------" << std::endl
                << " Warning:" << std::endl
                << "   Setting the watchdog timeout to zero disables the" << std::endl
                << "   watchdog.   We strongly recommend to no specify" << std::endl
                << "   io:watchdog_timeout or set it around 30 ms." << std::endl
                << "----------------------------------------------------";
        std::cerr << "mtsIntuitiveResearchKitConsole::" << message.str() << std::endl;
        CMN_LOG_INIT_WARNING << message.str() << std::endl;
    }
}


void mtsIntuitiveResearchKitConsole::IO_proxy_t::create_IO(void)
{
    m_IO = std::make_unique<mtsRobotIO1394>(m_name, m_config->period, m_config->port);
    m_IO->SetProtocol(m_config->protocol);
    m_IO->SetWatchdogPeriod(m_config->watchdog_timeout);
    for (const auto & config_file : m_config->configuration_files) {
        std::string file = m_console->find_file(config_file);
        if (file == "") {
            CMN_LOG_INIT_ERROR << "IO_proxy_t::create_IO: can't find IO file " << config_file << std::endl;
            exit(EXIT_FAILURE);
        }
        m_IO->Configure(file);
    }
    mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();
    component_manager->AddComponent(m_IO.get());
}


mtsIntuitiveResearchKitConsole::arm_proxy_t::arm_proxy_t(const std::string & name,
                                                         mtsIntuitiveResearchKitConsole * console,
                                                         dvrk::arm_proxy_configuration_t * config):
    m_name(name),
    m_console(console),
    m_config(config),
    m_arm_component_name(name),
    m_arm_interface_name("Arm")
{
}


void mtsIntuitiveResearchKitConsole::arm_proxy_t::post_configure(void)
{
    // make sure IO component is provided
    if (m_config->expects_IO()) {
        // valid string
        if (m_config->IO == "") {
            CMN_LOG_INIT_ERROR << "arm_proxy_t::post_configure: \"IO\" must be provided for native or derived arm: "
                               << m_name << std::endl;
            exit(EXIT_FAILURE);
        }
        // registered in component manager?
        auto iter = m_console->m_IO_proxies.find(m_config->IO);
        if (iter == m_console->m_IO_proxies.end()) {
            CMN_LOG_INIT_ERROR << "arm_proxy_t::post_configure: IO \""
                               << m_config->IO << "\" is not defined in \"IOs\"" << std::endl;
            exit(EXIT_FAILURE);
        }
        m_IO_component_name = m_config->IO;
        m_IO_interface_name = m_config->name;


    }

    // for generic and derived arms, component name must be provided
    if (m_config->generic_or_derived()) {
        if ((m_config->component != "") && (m_config->interface != "")) {
            m_arm_component_name = m_config->component;
            m_arm_interface_name = m_config->interface;
        } else {
            CMN_LOG_INIT_ERROR << "arm_proxy_t::post_configure: \"component\" and \"interface\" must be provided for generic or derived arm: "
                               << m_name << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // check base frame settings
    if (!m_config->base_frame.valid()) {
        CMN_LOG_INIT_ERROR << "arm_proxy_t::post_configure, base_frame must be defined by either \"reference_frame\" name and \"transform\" OR \"component\" and \"interface\" for "
                           << m_name << std::endl;
        exit(EXIT_FAILURE);
    } else {
        if ((m_config->base_frame.component != "") && (m_config->base_frame.interface != "")) {
            m_base_frame_component_name = m_config->base_frame.component;
            m_base_frame_interface_name = m_config->base_frame.interface;
        }
    }
}


void mtsIntuitiveResearchKitConsole::arm_proxy_t::create_arm(void)
{
    if (!m_config->native_or_derived()) {
        return;
    }

    // infer arm configuration file
    // -1- provided by user
    if (m_config->arm_file != "") {
        m_arm_configuration_file = m_console->find_file(m_config->arm_file);
        if (m_arm_configuration_file == "") {
            CMN_LOG_INIT_ERROR << "arm_proxy_t::create_arm: can't find configuration file " << m_config->arm_file
                               << " for arm " << m_name << std::endl;
            exit(EXIT_FAILURE);
        }
    } else {
        // -2- using serial number
        if (m_config->type != dvrk::arm_type_t::FOCUS_CONTROLLER) {
            if (m_config->serial == "") {
                CMN_LOG_INIT_ERROR << "arm_proxy_t::create_arm: serial number required for arm "
                                   << m_name << std::endl;
                exit(EXIT_FAILURE);
            }
            const auto default_file = m_name + "-" + m_config->serial + ".json";
            m_arm_configuration_file = m_console->find_file(default_file);
            if (m_arm_configuration_file == "") {
                CMN_LOG_INIT_ERROR << "arm_proxy_t::create_arm: can't find \"arm\" setting for arm "
                                   << m_name << ". \"arm\" is not set and the default file "
                                   << default_file << " doesn't seem to exist either." << std::endl;
                exit(EXIT_FAILURE);
            } else {
                CMN_LOG_INIT_VERBOSE << "arm_proxy_t::create_arm: using the default file "
                                     << default_file << " for arm " << m_name << std::endl;

            }
        }
    }


    mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();
    // for research kit arms, create, add to manager and connect to
    // extra IO, PID, etc.  For generic arms, do nothing.
    switch (m_config->type) {
    case dvrk::arm_type_t::MTM:
        {
            m_arm = std::make_shared<mtsIntuitiveResearchKitMTM>(m_name, m_config->period);
        }
        break;
    case dvrk::arm_type_t::PSM:
        {
            m_arm = std::make_shared<mtsIntuitiveResearchKitPSM>(m_name, m_config->period);
        }
        break;
    case dvrk::arm_type_t::ECM:
        {
            m_arm = std::make_shared<mtsIntuitiveResearchKitECM>(m_name, m_config->period);
        }
        break;
    case dvrk::arm_type_t::SUJ_Classic:
        {
            mtsIntuitiveResearchKitSUJ * suj = new mtsIntuitiveResearchKitSUJ(m_name, m_config->period);
            if (m_config->simulation == dvrk::simulation_t::SIMULATION_KINEMATIC) {
                suj->set_simulated();
            } else if (m_config->simulation == dvrk::simulation_t::SIMULATION_NONE) {
                m_console->m_connections.Add(m_name, "NoMuxReset",
                                             m_IO_component_name, "NoMuxReset");
                m_console->m_connections.Add(m_name, "MuxIncrement",
                                             m_IO_component_name, "MuxIncrement");
                m_console->m_connections.Add(m_name, "ControlPWM",
                                             m_IO_component_name, "ControlPWM");
                m_console->m_connections.Add(m_name, "DisablePWM",
                                             m_IO_component_name, "DisablePWM");
                m_console->m_connections.Add(m_name, "MotorUp",
                                             m_IO_component_name, "MotorUp");
                m_console->m_connections.Add(m_name, "MotorDown",
                                             m_IO_component_name, "MotorDown");
                m_console->m_connections.Add(m_name, "SUJ-Clutch-1",
                                             m_IO_component_name, "SUJ-Clutch-1");
                m_console->m_connections.Add(m_name, "SUJ-Clutch-2",
                                             m_IO_component_name, "SUJ-Clutch-2");
                m_console->m_connections.Add(m_name, "SUJ-Clutch-3",
                                             m_IO_component_name, "SUJ-Clutch-3");
                m_console->m_connections.Add(m_name, "SUJ-Clutch-4",
                                             m_IO_component_name, "SUJ-Clutch-4");
            }
            suj->Configure(m_arm_configuration_file);
            component_manager->AddComponent(suj);
        }
        break;
    case dvrk::arm_type_t::SUJ_Si:
        {
#if sawIntuitiveResearchKit_HAS_SUJ_Si
            mtsIntuitiveResearchKitSUJSi * suj = new mtsIntuitiveResearchKitSUJSi(m_name, m_config->period);
            if (m_config->simulation == dvrk::simulation_t::SIMULATION_KINEMATIC) {
                suj->set_simulated();
            }
            suj->Configure(m_arm_configuration_file);
            component_manager->AddComponent(suj);
#else
            CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::arm_proxy_t::ConfigureArm: can't create an arm of type SUJ_Si because sawIntuitiveResearchKit_HAS_SUJ_Si is set to OFF in CMake"
                               << std::endl;
            exit(EXIT_FAILURE);
#endif
        }
        break;
    case dvrk::arm_type_t::SUJ_Fixed:
        {
            mtsIntuitiveResearchKitSUJFixed * suj = new mtsIntuitiveResearchKitSUJFixed(m_name, m_config->period);
            suj->Configure(m_arm_configuration_file);
            component_manager->AddComponent(suj);
        }
        break;
    case dvrk::arm_type_t::MTM_DERIVED:
        {
            mtsComponent * component;
            component = component_manager->GetComponent(m_name);
            if (component) {
                mtsIntuitiveResearchKitMTM * mtm = dynamic_cast<mtsIntuitiveResearchKitMTM *>(component);
                if (mtm) {
                    std::cerr << CMN_LOG_DETAILS << " is this risky?" << std::endl;
                    m_arm = std::shared_ptr<mtsIntuitiveResearchKitMTM>(mtm, [](mtsIntuitiveResearchKitMTM * p){ ; });
                } else {
                    CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::arm_proxy_t::ConfigureArm: component \""
                                       << m_name << "\" doesn't seem to be derived from mtsIntuitiveResearchKitMTM."
                                       << std::endl;
                    exit(EXIT_FAILURE);
                }
            } else {
                CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::arm_proxy_t::ConfigureArm: component \""
                                   << m_name << "\" not found."
                                   << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        break;
    case dvrk::arm_type_t::PSM_DERIVED:
        {
            mtsComponent * component;
            component = component_manager->GetComponent(m_name);
            if (component) {
                mtsIntuitiveResearchKitPSM * psm = dynamic_cast<mtsIntuitiveResearchKitPSM *>(component);
                if (psm) {
                    std::cerr << CMN_LOG_DETAILS << " is this risky?" << std::endl;
                    m_arm = std::shared_ptr<mtsIntuitiveResearchKitArm>(psm);
                } else {
                    CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::arm_proxy_t::ConfigureArm: component \""
                                       << m_name << "\" doesn't seem to be derived from mtsIntuitiveResearchKitPSM."
                                       << std::endl;
                    exit(EXIT_FAILURE);
                }
            } else {
                CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::arm_proxy_t::ConfigureArm: component \""
                                   << m_name << "\" not found."
                                   << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        break;
    case dvrk::arm_type_t::ECM_DERIVED:
        {
            mtsComponent * component;
            component = component_manager->GetComponent(m_name);
            if (component) {
                mtsIntuitiveResearchKitECM * ecm = dynamic_cast<mtsIntuitiveResearchKitECM *>(component);
                if (ecm) {
                    std::cerr << CMN_LOG_DETAILS << " is this risky?" << std::endl;
                    m_arm = std::shared_ptr<mtsIntuitiveResearchKitArm>(ecm);
                } else {
                    CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::arm_proxy_t::ConfigureArm: component \""
                                       << m_name << "\" doesn't seem to be derived from mtsIntuitiveResearchKitECM."
                                       << std::endl;
                    exit(EXIT_FAILURE);
                }
            } else {
                CMN_LOG_INIT_ERROR << "mtsIntuitiveResearchKitConsole::arm_proxy_t::ConfigureArm: component \""
                                   << m_name << "\" not found."
                                   << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        break;

    default:
        break;
    }

    // if native or derived active arms
    if (m_config->native_or_derived()
        && !m_config->SUJ()) {
        CMN_ASSERT(m_arm != nullptr);
        if (m_config->simulation == dvrk::simulation_t::SIMULATION_KINEMATIC) {
            m_arm->set_simulated();
        }
        m_arm->set_calibration_mode(m_calibration_mode);
        m_arm->Configure(m_arm_configuration_file);
        set_base_frame_if_needed();
        component_manager->AddComponent(m_arm.get());

        // for all native arms not simulated, connect a few IOS
        if (m_config->simulation == dvrk::simulation_t::SIMULATION_NONE) {

            if (m_config->PSM()) {
                std::vector<std::string> itfs = {"adapter", "tool", "arm_clutch", "dallas"};
                for (const auto & itf : itfs) {
                    m_console->m_connections.Add(m_name, itf,
                                                 m_IO_component_name, m_name + "_" + itf);
                }
            }

            if (m_config->ECM()) {
                m_console->m_connections.Add(m_name, "arm_clutch",
                                             m_IO_component_name, m_name + "_arm_clutch");
            }

            // for Si patient side, connect the SUJ brakes to buttons on arm
            if ((m_config->PSM() || m_config->ECM())
                && (m_arm->generation() == dvrk::generation_t::Si)) {
                std::vector<std::string> itfs = {"SUJ_clutch", "SUJ_clutch2", "SUJ_brake"};
                for (const auto & itf : itfs) {
                    m_console->m_connections.Add(m_name, itf,
                                                 m_IO_component_name, m_name + "_" + itf);
                }
            }
        }
    }
}


void mtsIntuitiveResearchKitConsole::arm_proxy_t::configure_IO(void)
{
    if (!m_config->expects_IO()) {
        return;
    }

    // search for the config file
    std::string config_file;
    if (m_config->IO_file != "") {
        config_file = m_config->IO_file;
    } else {
        if (m_config->serial != "") {
            config_file = "sawRobotIO1394-" + m_name + "-" + m_config->serial + ".xml";
        } else {
            CMN_LOG_INIT_ERROR << "arm_proxy_t::configure_IO: can't find \"serial\" nor \"IO_file\" setting for arm \""
                               << m_name << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    m_IO_configuration_file = m_console->find_file(config_file);
    if (m_IO_configuration_file == "") {
        CMN_LOG_INIT_ERROR << "arm_proxy_t::configure_IO: can't find IO file " << config_file << std::endl;
        exit(EXIT_FAILURE);
    }

    // search for the IO component
    auto iter_IO = m_console->m_IO_proxies.find(m_config->IO);
    CMN_LOG_INIT_VERBOSE << "arm_proxy_t::configure_IO: configuring IO \""
                         << m_config->IO << "\" for arm " << m_name
                         << " using configuration file: " << m_IO_configuration_file << std::endl;
    // configure_IO should only happen if IO is valid
    CMN_ASSERT(iter_IO != m_console->m_IO_proxies.end());
    iter_IO->second->m_IO->Configure(m_IO_configuration_file);


    // search for the gripper config file
    if (m_config->MTM()) {
        std::string config_file;
        if (m_config->IO_gripper_file != "") {
            config_file = m_config->IO_gripper_file;
        } else {
            if (m_config->serial != "") {
                config_file = "sawRobotIO1394-" + m_name + "-gripper-" + m_config->serial + ".xml";
            } else {
                CMN_LOG_INIT_ERROR << "arm_proxy_t::configure_IO: can't find \"serial\" nor \"IO_gripper_file\" setting for arm \""
                                   << m_name << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        m_IO_gripper_configuration_file = m_console->find_file(config_file);
        if (m_IO_gripper_configuration_file == "") {
            CMN_LOG_INIT_ERROR << "arm_proxy_t::configure_IO: can't find IO gripper file " << config_file << std::endl;
            exit(EXIT_FAILURE);
        }

        // re-use IO component
        iter_IO->second->m_IO->Configure(m_IO_gripper_configuration_file);
    }
}


void mtsIntuitiveResearchKitConsole::arm_proxy_t::create_PID(void)
{
    if (!m_config->expects_PID()) {
        return;
    }

    // infer pid config file name
    std::string pid_config;
    if (m_config->PID_file !=  "") {
        m_PID_configuration_file = m_config->PID_file;
        CMN_LOG_INIT_VERBOSE << "ConfigurePID: using \"PID\" file name for arm \""
                             << m_name << "\": \""
                             << m_PID_configuration_file << "\"" << std::endl;
    } else {
        // not user defined, try to find the default
        auto generation = this->generation();
        if (m_config->native_or_derived_MTM()) {
            m_PID_configuration_file = "pid/sawControllersPID-MTM.json";
        } else if (m_config->native_or_derived_PSM()) {
            if (generation == dvrk::generation_t::Classic) {
                m_PID_configuration_file = "pid/sawControllersPID-PSM.json";
            } else {
                m_PID_configuration_file = "pid/sawControllersPID-PSM-Si.json";
            }
        } else if (m_config->native_or_derived_ECM()) {
            if (generation == dvrk::generation_t::Classic) {
                m_PID_configuration_file = "pid/sawControllersPID-ECM.json";
            } else  {
                m_PID_configuration_file = "pid/sawControllersPID-ECM-Si.json";
            }
        } else {
            m_PID_configuration_file = "pid/sawControllersPID-" + m_name + ".json";
        }
        CMN_LOG_INIT_VERBOSE << "ConfigurePID: no \"PID\" file name for arm \""
                             << m_name << "\", using default based on type and generation: \""
                             << m_PID_configuration_file << "\"" << std::endl;
    }

    auto configuration_file = m_console->find_file(m_PID_configuration_file);
    if (configuration_file == "") {
        CMN_LOG_INIT_ERROR << "configure_PID: can't find PID file " << m_PID_configuration_file << std::endl;
        exit(EXIT_FAILURE);
    }

    CMN_LOG_INIT_VERBOSE << "configure_PID: found file " << configuration_file << std::endl;
    m_PID_configuration_file = configuration_file;
    m_PID_component_name = m_name + "_PID";

    mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();
    mtsPID * pid = new mtsPID(m_PID_component_name,
                              (m_config->PID_period != 0.0) ? m_config->PID_period : mtsIntuitiveResearchKit::IOPeriod);
    bool hasIO = true;
    pid->Configure(m_PID_configuration_file);
    if (m_config->simulation == dvrk::simulation_t::SIMULATION_KINEMATIC) {
        pid->SetSimulated();
        hasIO = false;
    }
    component_manager->AddComponent(pid);
    if (hasIO) {
        m_console->m_connections.Add(m_PID_component_name, "RobotJointTorqueInterface",
                                     m_IO_component_name, m_IO_interface_name);
        if (m_config->PID_period == 0.0) {
            m_console->m_connections.Add(m_PID_component_name, "ExecIn",
                                         m_IO_component_name, "ExecOut");
        }
    }
}


void mtsIntuitiveResearchKitConsole::arm_proxy_t::set_base_frame_if_needed(void)
{
    // use reference_name to determine if we're using a fixed base frame
    if (m_config->base_frame.reference_frame != "") {
        m_base_frame.Goal().From(m_config->base_frame.transform);
        m_base_frame.ReferenceFrame() = m_config->base_frame.reference_frame;
        m_base_frame.Valid() = true;

        if (m_arm == nullptr) {
            CMN_LOG_INIT_ERROR << "arm_proxy_t::set_base_frame_if_needed failed, arm needs to be configured first" << std::endl;
            exit(EXIT_FAILURE);
        }
        m_arm->set_base_frame(m_base_frame);
    }
}


bool mtsIntuitiveResearchKitConsole::arm_proxy_t::connect(void)
{
    mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();
    // if the arm is a research kit arm
    if (m_config->native_or_derived()) {
        // Connect arm to IO if not simulated
        if (m_config->expects_IO()) {
            component_manager->Connect(m_name, "RobotIO",
                                       m_IO_component_name, m_name);
        }
        // connect MTM gripper to IO
        if (m_config->native_or_derived_MTM()
            && !m_config->simulated()) {
            component_manager->Connect(m_name, "GripperIO",
                                       m_IO_component_name, m_name + "_gripper");
        }
        // connect PID
        if (m_config->expects_PID()) {
            component_manager->Connect(m_name, "PID",
                                       m_PID_component_name, "Controller");
        }
        // connect m_base_frame if needed
        if ((m_base_frame_component_name != "") && (m_base_frame_interface_name != "")) {
            component_manager->Connect(m_base_frame_component_name, m_base_frame_interface_name,
                                       m_name, "Arm");
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
    m_console->SetArmCurrentState(m_name, currentState);
}



mtsIntuitiveResearchKitConsole::teleop_PSM_proxy_t::teleop_PSM_proxy_t(const std::string & name,
                                                                       mtsIntuitiveResearchKitConsole * console,
                                                                       dvrk::teleop_PSM_proxy_configuration_t * config):
    m_name(name),
    m_console(console),
    m_config(config)
{
}


void mtsIntuitiveResearchKitConsole::teleop_PSM_proxy_t::post_configure(void)
{
    auto iter = m_console->m_arm_proxies.find(m_config->MTM);
    if (iter == m_console->m_arm_proxies.end()) {
        CMN_LOG_INIT_ERROR << "teleop_PSM_proxy_t::post_configure: MTM \""
                           << m_config->MTM << "\" is not defined in \"arms\"" << std::endl;
        exit(EXIT_FAILURE);
    } else {
        auto arm_proxy = iter->second;
        if (!arm_proxy->m_config->MTM()) {
            CMN_LOG_INIT_ERROR << "teleop_PSM_proxy_t::post_configure: MTM \""
                               << m_config->MTM << "\" type must be some type of MTM" << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    iter = m_console->m_arm_proxies.find(m_config->PSM);
    if (iter == m_console->m_arm_proxies.end()) {
        CMN_LOG_INIT_ERROR << "teleop_PSM_proxy_t::post_configure: PSM \""
                           << m_config->PSM << "\" is not defined in \"arms\"" << std::endl;
        exit(EXIT_FAILURE);
    } else {
        auto arm_proxy = iter->second;
        if (!arm_proxy->m_config->PSM()) {
            CMN_LOG_INIT_ERROR << "teleop_PSM_proxy_t::post_configure: PSM \""
                               << m_config->PSM << "\" type must be some type of PSM" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

}


void mtsIntuitiveResearchKitConsole::teleop_PSM_proxy_t::create_teleop(void)
{
    mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();
    switch (m_config->type) {
    case dvrk::teleop_PSM_type_t::TELEOP_PSM:
        {
            mtsTeleOperationPSM * teleop = new mtsTeleOperationPSM(m_name, m_config->period);
            teleop->Configure(m_config->configure_parameter);
            component_manager->AddComponent(teleop);
        }
        break;
    case dvrk::teleop_PSM_type_t::TELEOP_PSM_DERIVED:
        {
            mtsComponent * component;
            component = component_manager->GetComponent(m_name);
            if (component) {
                mtsTeleOperationPSM * teleop = dynamic_cast<mtsTeleOperationPSM *>(component);
                if (teleop) {
                    teleop->Configure(m_config->configure_parameter);
                } else {
                    CMN_LOG_INIT_ERROR << "teleop_PSM_proxy_t::create_teleop: component \""
                                       << m_name << "\" doesn't seem to be derived from mtsTeleOperationPSM."
                                       << std::endl;
                    exit(EXIT_FAILURE);
                }
            } else {
                CMN_LOG_INIT_ERROR << "teleop_PSM_proxy_t::create_teleop: component \""
                                   << m_name << "\" not found."
                                   << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        break;
    default:
        break;
    }

    // schedule connections
    auto iter_MTM = m_console->m_arm_proxies.find(m_config->MTM);
    CMN_ASSERT(iter_MTM != m_console->m_arm_proxies.end());
    m_console->m_connections.Add(m_name, "MTM",
                                 iter_MTM->second->m_arm_component_name,
                                 iter_MTM->second->m_arm_interface_name);
    auto iter_PSM = m_console->m_arm_proxies.find(m_config->PSM);
    CMN_ASSERT(iter_PSM != m_console->m_arm_proxies.end());
    m_console->m_connections.Add(m_name, "PSM",
                                 iter_PSM->second->m_arm_component_name,
                                 iter_PSM->second->m_arm_interface_name);
    m_console->m_connections.Add(m_name, "Clutch",
                                 m_console->GetName(), "Clutch"); // clutch from console
    m_console->m_connections.Add(m_console->GetName(), m_name,
                                 m_name, "Setting");
    // if ((baseFrameComponent != "")
    //     && (baseFrameInterface != "")) {
    //     m_connections.Add(name, "PSM_base_frame",
    //                       baseFrameComponent, baseFrameInterface);
    // }
}



mtsIntuitiveResearchKitConsole::teleop_ECM_proxy_t::teleop_ECM_proxy_t(const std::string & name,
                                                                       mtsIntuitiveResearchKitConsole * console,
                                                                       dvrk::teleop_ECM_proxy_configuration_t * config):
    m_name(name),
    m_console(console),
    m_config(config)
{
}


void mtsIntuitiveResearchKitConsole::teleop_ECM_proxy_t::post_configure(void)
{
    std::cerr << CMN_LOG_DETAILS << "add checks re. scale is valid, different MTMs" << std::endl;
}


void mtsIntuitiveResearchKitConsole::teleop_ECM_proxy_t::create_teleop(void)
{
    mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();
    switch (m_config->type) {
    case dvrk::teleop_ECM_type_t::TELEOP_ECM:
        {
            mtsTeleOperationECM * teleop = new mtsTeleOperationECM(m_name, m_config->period);
            teleop->Configure(m_config->configure_parameter);
            component_manager->AddComponent(teleop);
        }
        break;
    case dvrk::teleop_ECM_type_t::TELEOP_ECM_DERIVED:
        {
            mtsComponent * component;
            component = component_manager->GetComponent(m_name);
            if (component) {
                mtsTeleOperationECM * teleop = dynamic_cast<mtsTeleOperationECM *>(component);
                if (teleop) {
                    teleop->Configure(m_config->configure_parameter);
                } else {
                    CMN_LOG_INIT_ERROR << "teleop_ECM_proxy_t::create_teleop: component \""
                                       << m_name << "\" doesn't seem to be derived from mtsTeleOperationECM."
                                       << std::endl;
                    exit(EXIT_FAILURE);
                }
            } else {
                CMN_LOG_INIT_ERROR << "teleop_ECM_proxy_t::create_teleop: component \""
                                   << m_name << "\" not found."
                                   << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        break;
    default:
        break;
    }

    std::cerr << CMN_LOG_DETAILS << " move this check to post_configure and save component/interface names in proxy " << std::endl;
    // find the names for the MTM and ECM components as well as their provided interfaces
    std::string
        mtml_component, mtml_interface,
        mtmr_component, mtmr_interface,
        ecm_component, ecm_interface;
    // check that all arms have been defined and have correct type
    // MTML
    auto iter = m_console->m_arm_proxies.find(m_config->MTML);
    if (iter == m_console->m_arm_proxies.end()) {
        CMN_LOG_INIT_ERROR << "teleop_ECM_proxy_t::create_teleop: MTML \""
                           << m_config->MTML << "\" is not defined in \"arms\"" << std::endl;
        exit(EXIT_FAILURE);
    } else {
        auto arm_proxy = iter->second;
        if (!arm_proxy->m_config->MTM()) {
            CMN_LOG_INIT_ERROR << "teleop_ECM_proxy_t::create_teleop: MTML \""
                               << m_config->MTML << "\" type must be some type of MTM" << std::endl;
            exit(EXIT_FAILURE);
        }
        mtml_component = arm_proxy->m_arm_component_name;
        mtml_interface = arm_proxy->m_arm_interface_name;
    }
    // MTMR
    iter = m_console->m_arm_proxies.find(m_config->MTMR);
    if (iter == m_console->m_arm_proxies.end()) {
        CMN_LOG_INIT_ERROR << "teleop_ECM_proxy_t::create_teleop: MTMR \""
                           << m_config->MTMR << "\" is not defined in \"arms\"" << std::endl;
        exit(EXIT_FAILURE);
    } else {
        auto arm_proxy = iter->second;
        if (!arm_proxy->m_config->MTM()) {
            CMN_LOG_INIT_ERROR << "teleop_ECM_proxy_t::create_teleop: MTMR \""
                               << m_config->MTMR << "\" type must be some type of MTM" << std::endl;
            exit(EXIT_FAILURE);
        }
        mtmr_component = arm_proxy->m_arm_component_name;
        mtmr_interface = arm_proxy->m_arm_interface_name;
    }

    // ECM
    iter = m_console->m_arm_proxies.find(m_config->ECM);
    if (iter == m_console->m_arm_proxies.end()) {
        CMN_LOG_INIT_ERROR << "teleop_ECM_proxy_t::create_teleop: ECM \""
                           << m_config->ECM << "\" is not defined in \"arms\"" << std::endl;
        exit(EXIT_FAILURE);
    } else {
        auto arm_proxy = iter->second;
        if (!arm_proxy->m_config->ECM()) {
            CMN_LOG_INIT_ERROR << "teleop_ECM_proxy_t::create_teleop: ECM \""
                               << m_config->ECM << "\" type must be some type of ECM" << std::endl;
            exit(EXIT_FAILURE);
        }
        ecm_component = arm_proxy->m_arm_component_name;
        ecm_interface = arm_proxy->m_arm_interface_name;
    }

    // schedule connections
    m_console->m_connections.Add(m_name, "MTML",
                                 mtml_component, mtml_interface);
    m_console->m_connections.Add(m_name, "MTMR",
                                 mtmr_component, mtmr_interface);
    m_console->m_connections.Add(m_name, "ECM",
                                 ecm_component, ecm_interface);
    m_console->m_connections.Add(m_name, "Clutch",
                                 m_console->GetName(), "Clutch"); // clutch from console
    m_console->m_connections.Add(m_console->GetName(), m_name,
                                 m_name, "Setting");
    // if ((baseFrameComponent != "")
    //     && (baseFrameInterface != "")) {
    //     m_connections.Add(name, "ECM_base_frame",
    //                       baseFrameComponent, baseFrameInterface);
    // }
}





mtsIntuitiveResearchKitConsole::mtsIntuitiveResearchKitConsole(const std::string & componentName):
    mtsTaskFromSignal(componentName, 100),
    m_configured(false),
    mTimeOfLastErrorBeep(0.0)
    // mTeleopMTMToCycle(""),
    // mOperatorPresent(false),
    // mCameraPressed(false)
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
        // mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::teleop_enable, this,
        //                             "teleop_enable", false);
        // mInterface->AddEventWrite(console_events.teleop_enabled,
        //                           "teleop_enabled", false);
        // // manage tele-op
        // mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::cycle_teleop_PSM_by_MTM, this,
        //                             "cycle_teleop_PSM_by_MTM", std::string(""));
        // mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::select_teleop_PSM, this,
        //                             "select_teleop_PSM", prmKeyValue("MTM", "PSM"));
        // mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::set_scale, this,
        //                             "set_scale", 0.5);
        // mInterface->AddEventWrite(ConfigurationEvents.scale,
        //                           "scale", 0.5);
        // mInterface->AddEventWrite(ConfigurationEvents.teleop_PSM_selected,
        //                           "teleop_PSM_selected", prmKeyValue("MTM", "PSM"));
        // mInterface->AddEventWrite(ConfigurationEvents.teleop_PSM_unselected,
        //                           "teleop_PSM_unselected", prmKeyValue("MTM", "PSM"));
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
        // mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::OperatorPresentEventHandler, this,
        //                             "emulate_operator_present", prmEventButton());
        // mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::ClutchEventHandler, this,
        //                             "emulate_clutch", prmEventButton());
        // mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::CameraEventHandler, this,
        //                             "emulate_camera", prmEventButton());
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

    std::ifstream json_stream;
    json_stream.open(filename.c_str());

    Json::Value json_config, json_value;
    Json::Reader json_reader;
    if (!json_reader.parse(json_stream, json_config)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration" << std::endl
                                 << "File: " << filename << std::endl << "Error(s):" << std::endl
                                 << json_reader.getFormattedErrorMessages();
        this->m_configured = false;
        exit(EXIT_FAILURE);
    }

    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << this->GetName()
                               << " using file \"" << filename << "\"" << std::endl
                               << "----> content of configuration file: " << std::endl
                               << json_config << std::endl
                               << "<----" << std::endl;

    // base component configuration
    mtsComponent::ConfigureJSON(json_config);

    // extract path of main json config file to search other files relative to it
    std::string fullname = m_config_path.Find(filename);
    std::string config_dir = fullname.substr(0, fullname.find_last_of('/'));
    m_config_path.Add(config_dir, cmnPath::TAIL);

    mtsComponentManager * manager = mtsComponentManager::GetInstance();

    // Json parse and load values, will complain if a field with no
    // default is not provided (e.g. "name")
    cmnDataDeSerializeTextJSON(m_config, json_config);
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsIntuitiveResearchKitConsole::Configure, loaded:" << std::endl
                               << "------>" << std::endl
                               << cmnDataSerializeTextJSON(m_config) << std::endl
                               << "<------" << std::endl;

    // first, create all custom components and connections, i.e. dynamic loading and creation
    if (!m_config.component_manager.empty()) {
        if (!manager->ConfigureJSON(m_config.component_manager, m_config_path)) {
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

    // find all IOs
    for (auto & IO : m_config.IOs) {
        const auto iter = m_IO_proxies.find(IO.name);
        if (iter == m_IO_proxies.end()) {
            // create a new IO proxy if needed
            auto IO_proxy = std::make_shared<IO_proxy_t>(IO.name, this, &IO);
            IO_proxy->post_configure();
            m_IO_proxies[IO.name] = IO_proxy;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure IO "
                                     << IO.name << ", IO already exists" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    for (auto & arm : m_config.arms) {
        const auto iter = m_arm_proxies.find(arm.name);
        if (iter == m_arm_proxies.end()) {
            // create a new arm proxy if needed
            auto arm_proxy = std::make_shared<arm_proxy_t>(arm.name, this, &arm);
            arm_proxy->post_configure();
            m_arm_proxies[arm.name] = arm_proxy;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure arm "
                                     << arm.name << ", arm already exists" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // loop over all arms to check if IO is needed, also check if some IO configuration files are listed in "io"
    // mHasIO = false;
    // for (auto iter = m_arm_proxies.begin(); iter != end; ++iter) {
    //     std::string ioConfig = iter->second->m_IO_configuration_file;
    //     if (!ioConfig.empty()) {
    //         mHasIO = true;
    //     }
    // }
    // bool physicalFootpedalsRequired = true;
    // jsonValue = jsonConfig["io"];
    // if (!jsonValue.empty()) {
    //     // generic files
    //     Json::Value configFiles = jsonValue["configuration-files"];
    //     if (!configFiles.empty()) {
    //         mHasIO = true;
    //     }
    //     // footpedals config
    //     configFiles = jsonValue["footpedals"];
    //     if (!configFiles.empty()) {
    //         mHasIO = true;
    //     }
    //     // see if user wants to force no foot pedals
    //     Json::Value footpedalsRequired = jsonValue["physical_footpedals_required"];
    //     if (!footpedalsRequired.empty()) {
    //         physicalFootpedalsRequired = footpedalsRequired.asBool();
    //     }
    // }
    // // just check for IO and make sure we don't have io and hid, will be configured later
    // jsonValue = jsonConfig["operator-present"];
    // if (!jsonValue.empty()) {
    //     // check if operator present uses IO
    //     Json::Value jsonConfigFile = jsonValue["io"];
    //     if (!jsonConfigFile.empty()) {
    //         mHasIO = true;
    //         jsonConfigFile = jsonValue["hid"];
    //         if (!jsonConfigFile.empty()) {
    //             CMN_LOG_CLASS_INIT_ERROR << "Configure: operator-present can't have both io and hid" << std::endl;
    //             exit(EXIT_FAILURE);
    //         }
    //     }
    // }
    // create IO if needed and configure IO
    // if (mHasIO) {
    //     mtsRobotIO1394 * io = new mtsRobotIO1394(m_IO_component_name, periodIO, port);
    //     io->SetProtocol(protocol);
    //     io->SetWatchdogPeriod(watchdogTimeout);
    //     // configure for each arm
    //     for (auto iter = m_arm_proxies.begin(); iter != end; ++iter) {
    //         std::string ioConfig = iter->second->m_IO_configuration_file;
    //         if (ioConfig != "") {
    //             CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring IO using \"" << ioConfig << "\"" << std::endl;
    //             io->Configure(ioConfig);
    //         }
    //         std::string ioGripperConfig = iter->second->m_IO_gripper_configuration_file;
    //         if (ioGripperConfig != "") {
    //             CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring IO gripper using \"" << ioGripperConfig << "\"" << std::endl;
    //             io->Configure(ioGripperConfig);
    //         }
    //     }
    //     // configure using extra configuration files
    //     jsonValue = jsonConfig["io"];
    //     if (!jsonValue.empty()) {
    //         // generic files
    //         Json::Value configFiles = jsonValue["configuration-files"];
    //         if (!configFiles.empty()) {
    //             for (unsigned int index = 0; index < configFiles.size(); ++index) {
    //                 const std::string configFile = find_file(configFiles[index].asString());
    //                 if (configFile == "") {
    //                     CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find configuration file "
    //                                              << configFiles[index].asString() << std::endl;
    //                     exit(EXIT_FAILURE);
    //                 }
    //                 CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring IO using \"" << configFile << "\"" << std::endl;
    //                 io->Configure(configFile);
    //             }
    //         }
    //         // footpedals, we assume these are the default one provided along the dVRK
    //         configFiles = jsonValue["footpedals"];
    //         if (!configFiles.empty()) {
    //             const std::string configFile = find_file(configFiles.asString());
    //             if (configFile == "") {
    //                 CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find configuration file "
    //                                          << configFiles.asString() << std::endl;
    //                 exit(EXIT_FAILURE);
    //             }
    //             CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring IO foot pedals using \"" << configFile << "\"" << std::endl;
    //             // these can be overwritten using console-inputs
    //             mDInputSources["Clutch"] = InterfaceComponentType(m_IO_component_name, "Clutch");
    //             mDInputSources["OperatorPresent"] = InterfaceComponentType(m_IO_component_name, "Coag");
    //             mDInputSources["Coag"] = InterfaceComponentType(m_IO_component_name, "Coag");
    //             mDInputSources["BiCoag"] = InterfaceComponentType(m_IO_component_name, "BiCoag");
    //             mDInputSources["Camera"] = InterfaceComponentType(m_IO_component_name, "Camera");
    //             mDInputSources["Cam-"] = InterfaceComponentType(m_IO_component_name, "Cam-");
    //             mDInputSources["Cam+"] = InterfaceComponentType(m_IO_component_name, "Cam+");
    //             mDInputSources["Head"] = InterfaceComponentType(m_IO_component_name, "Head");
    //             io->Configure(configFile);
    //         }
    //     }
    //     // configure IO for operator present
    //     jsonValue = jsonConfig["operator-present"];
    //     if (!jsonValue.empty()) {
    //         // check if operator present uses IO
    //         Json::Value jsonConfigFile = jsonValue["io"];
    //         if (!jsonConfigFile.empty()) {
    //             const std::string configFile = find_file(jsonConfigFile.asString());
    //             if (configFile == "") {
    //                 CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find configuration file "
    //                                          << jsonConfigFile.asString() << std::endl;
    //                 exit(EXIT_FAILURE);
    //             }
    //             CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring operator present using \""
    //                                        << configFile << "\"" << std::endl;
    //             io->Configure(configFile);
    //         } else {
    //             jsonConfigFile = jsonValue["hid"];
    //         }
    //     }
    //     // configure for endoscope focus
    //     jsonValue = jsonConfig["endoscope-focus"];
    //     if (!jsonValue.empty()) {
    //         // check if operator present uses IO
    //         Json::Value jsonConfigFile = jsonValue["io"];
    //         if (!jsonConfigFile.empty()) {
    //             const std::string configFile = find_file(jsonConfigFile.asString());
    //             if (configFile == "") {
    //                 CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find configuration file "
    //                                          << jsonConfigFile.asString() << std::endl;
    //                 exit(EXIT_FAILURE);
    //             }
    //             CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring endoscope focus using \""
    //                                        << configFile << "\"" << std::endl;
    //             io->Configure(configFile);
    //         }
    //     }
    //     // and add the io component!
    //     m_IO_interface = AddInterfaceRequired("IO");
    //     if (m_IO_interface) {
    //         m_IO_interface->AddFunction("close_all_relays", IO.close_all_relays);
    //         m_IO_interface->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
    //                                              this, "error");
    //         m_IO_interface->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
    //                                              this, "warning");
    //         m_IO_interface->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
    //                                              this, "status");
    //     } else {
    //         CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to create IO required interface" << std::endl;
    //         exit(EXIT_FAILURE);
    //     }
    //     mtsComponentManager::GetInstance()->AddComponent(io);
    //     if (m_IO_interface) {
    //         m_connections.Add(this->GetName(), "IO",
    //                          io->GetName(), "Configuration");
    //     }
    // }

    // first, create the IO components
    for (auto iter : m_IO_proxies) {
        auto & IO_proxy = iter.second;
        IO_proxy->create_IO();
        add_IO_interfaces(IO_proxy);
    }

    // now we can configure PID and Arms
    for (auto iter : m_arm_proxies) {
        auto & arm_proxy = iter.second;
        // create arm first to initialize generation
        arm_proxy->create_arm();
        arm_proxy->configure_IO();
        arm_proxy->create_PID();
        add_arm_interfaces(arm_proxy);
    }

    // ECM teleops
    // const auto json_teleop_ecms = jsonConfig["teleop_ECMs"];
    // for (unsigned int index = 0; index < json_teleop_ecms.size(); ++index) {
    //     const auto json_teleop_ecm = json_teleop_ecms[index];
    //     std::string teleop_ecm_name;
    //     if (!json_teleop_ecm["name"].empty()) {
    //         teleop_ecm_name = json_teleop_ecm["name"].asString();
    //     } else {
    //         const auto teleop_ecm_mtml_name = json_teleop_ecm["MTML"].asString();
    //         const auto teleop_ecm_mtmr_name = json_teleop_ecm["MTMR"].asString();
    //         const auto teleop_ecm_ecm_name = json_teleop_ecm["ECM"].asString();
    //         teleop_ecm_name = teleop_ecm_mtml_name + "_" + teleop_ecm_mtmr_name + "_" + teleop_ecm_ecm_name;
    //     }
    //     CMN_LOG_CLASS_INIT_VERBOSE << "Configure: name for teleop_ECMs["
    //                                << index << "] is: " << teleop_ecm_name << std::endl;
    //     const auto iter = m_teleop_ECM_proxies.find(teleop_ecm_name);
    //     if (iter == m_teleop_ECM_proxies.end()) {
    //         // create a new teleop_ecm proxy if needed
    //         auto teleop_ECM_proxy = std::make_shared<teleop_ECM_proxy_t>(teleop_ecm_name, this);
    //         teleop_ECM_proxy->configure(json_teleop_ecm);
    //         m_teleop_ECM_proxies[teleop_ecm_name] = teleop_ECM_proxy;
    //         teleop_ECM_proxy->create_teleop();
    //         add_teleop_ECM_interfaces(teleop_ECM_proxy);
    //     } else {
    //         CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure teleop_ECMs["
    //                                  << index << "], teleop_ECM "
    //                                  << teleop_ecm_name << " already exists" << std::endl;
    //         exit(EXIT_FAILURE);
    //     }
    // }

    // // PSM teleops
    // const auto json_teleop_psms = jsonConfig["teleop_PSMs"];
    // for (unsigned int index = 0; index < json_teleop_psms.size(); ++index) {
    //     const auto json_teleop_psm = json_teleop_psms[index];
    //     std::string teleop_psm_name;
    //     if (!json_teleop_psm["name"].empty()) {
    //         teleop_psm_name = json_teleop_psm["name"].asString();
    //     } else {
    //         const auto teleop_psm_mtm_name = json_teleop_psm["MTM"].asString();
    //         const auto teleop_psm_psm_name = json_teleop_psm["PSM"].asString();
    //         teleop_psm_name = teleop_psm_mtm_name + "_" + teleop_psm_psm_name;
    //     }
    //     CMN_LOG_CLASS_INIT_VERBOSE << "Configure: name for teleop_PSMs["
    //                                << index << "] is: " << teleop_psm_name << std::endl;
    //     const auto iter = m_teleop_PSM_proxies.find(teleop_psm_name);
    //     if (iter == m_teleop_PSM_proxies.end()) {
    //         // create a new teleop_psm proxy if needed
    //         auto teleop_PSM_proxy = std::make_shared<teleop_PSM_proxy_t>(teleop_psm_name, this);
    //         teleop_PSM_proxy->configure(json_teleop_psm);
    //         m_teleop_PSM_proxies[teleop_psm_name] = teleop_PSM_proxy;
    //         teleop_PSM_proxy->create_teleop();
    //         add_teleop_PSM_interfaces(teleop_PSM_proxy);
    //     } else {
    //         CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure teleop_PSMs["
    //                                  << index << "], teleop_PSM "
    //                                  << teleop_psm_name << " already exists" << std::endl;
    //         exit(EXIT_FAILURE);
    //     }
    // }

    // // see which event is used for operator present
    // // find name of button event used to detect if operator is present

    // // load from console inputs
    // const Json::Value consoleInputs = jsonConfig["console-inputs"];
    // if (!consoleInputs.empty()) {
    //     std::string component, interface;
    //     component = consoleInputs["operator-present"]["component"].asString();
    //     interface = consoleInputs["operator-present"]["interface"].asString();
    //     if ((component != "") && (interface != "")) {
    //         mDInputSources["OperatorPresent"] = InterfaceComponentType(component, interface);
    //     }
    //     component = consoleInputs["clutch"]["component"].asString();
    //     interface = consoleInputs["clutch"]["interface"].asString();
    //     if ((component != "") && (interface != "")) {
    //         mDInputSources["Clutch"] = InterfaceComponentType(component, interface);
    //     }
    //     component = consoleInputs["camera"]["component"].asString();
    //     interface = consoleInputs["camera"]["interface"].asString();
    //     if ((component != "") && (interface != "")) {
    //         mDInputSources["Camera"] = InterfaceComponentType(component, interface);
    //     }
    // }

    // load operator-present settings, this will over write older settings
    //     const Json::Value operatorPresent = jsonConfig["operator-present"];
    //     if (!operatorPresent.empty()) {
    //         // first case, using io to communicate with daVinci original head sensore
    //         Json::Value operatorPresentConfiguration = operatorPresent["io"];
    //         if (!operatorPresentConfiguration.empty()) {
    //             const std::string headSensorName = "daVinciHeadSensor";
    //             mHeadSensor = new mtsDaVinciHeadSensor(headSensorName);
    //             mtsComponentManager::GetInstance()->AddComponent(mHeadSensor);
    //             // main DInput is OperatorPresent comming from the newly added component
    //             mDInputSources["OperatorPresent"] = InterfaceComponentType(headSensorName, "OperatorPresent");
    //             // also expose the digital inputs from RobotIO (e.g. ROS topics)
    //             mDInputSources["HeadSensor1"] = InterfaceComponentType(m_IO_component_name, "HeadSensor1");
    //             mDInputSources["HeadSensor2"] = InterfaceComponentType(m_IO_component_name, "HeadSensor2");
    //             mDInputSources["HeadSensor3"] = InterfaceComponentType(m_IO_component_name, "HeadSensor3");
    //             mDInputSources["HeadSensor4"] = InterfaceComponentType(m_IO_component_name, "HeadSensor4");
    //             // schedule connections
    //             m_connections.Add(headSensorName, "HeadSensorTurnOff",
    //                              m_IO_component_name, "HeadSensorTurnOff");
    //             m_connections.Add(headSensorName, "HeadSensor1",
    //                              m_IO_component_name, "HeadSensor1");
    //             m_connections.Add(headSensorName, "HeadSensor2",
    //                              m_IO_component_name, "HeadSensor2");
    //             m_connections.Add(headSensorName, "HeadSensor3",
    //                              m_IO_component_name, "HeadSensor3");
    //             m_connections.Add(headSensorName, "HeadSensor4",
    //                              m_IO_component_name, "HeadSensor4");
    //         } else {
    //             // second case, using hid config for goovis head sensor
    //             operatorPresentConfiguration = operatorPresent["hid"];
    //             if (!operatorPresentConfiguration.empty()) {
    // #if sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR
    //                 std::string relativeConfigFile = operatorPresentConfiguration.asString();
    //                 CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring hid head sensor with \""
    //                                            << relativeConfigFile << "\"" << std::endl;
    //                 const std::string configFile = find_file(relativeConfigFile);
    //                 if (configFile == "") {
    //                     CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find configuration file "
    //                                              << relativeConfigFile << std::endl;
    //                     exit(EXIT_FAILURE);
    //                 }
    //                 const std::string headSensorName = "HIDHeadSensor";
    //                 mHeadSensor = new mtsHIDHeadSensor(headSensorName);
    //                 mHeadSensor->Configure(configFile);
    //                 mtsComponentManager::GetInstance()->AddComponent(mHeadSensor);
    //                 // main DInput is OperatorPresent comming from the newly added component
    //                 mDInputSources["OperatorPresent"] = InterfaceComponentType(headSensorName, "OperatorPresent");
    // #else
    //                 CMN_LOG_CLASS_INIT_ERROR << "Configure: can't use HID head sensor." << std::endl
    //                                          << "The code has been compiled with sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR OFF." << std::endl
    //                                          << "Re-run CMake, re-compile and try again." << std::endl;
    //                 exit(EXIT_FAILURE);
    // #endif
    //             }
    //         }
    //     }

    // message re. footpedals are likely missing but user can override this requirement
    // const std::string footpedalMessage = "Maybe you're missing \"io\":\"footpedals\" in your configuration file.  If you don't need physical footpedals, set \"physical_footpedals_required\" to false.";

    // load endoscope-focus settings
    // const Json::Value endoscopeFocus = jsonConfig["endoscope-focus"];
    // if (!endoscopeFocus.empty()) {
    //     const std::string endoscopeFocusName = "daVinciEndoscopeFocus";
    //     mDaVinciEndoscopeFocus = new mtsDaVinciEndoscopeFocus(endoscopeFocusName);
    //     mtsComponentManager::GetInstance()->AddComponent(mDaVinciEndoscopeFocus);
    //     // make sure we have cam+ and cam- in digital inputs
    //     const DInputSourceType::const_iterator endDInputs = mDInputSources.end();
    //     const bool foundCamMinus = (mDInputSources.find("Cam-") != endDInputs);
    //     if (!foundCamMinus) {
    //         CMN_LOG_CLASS_INIT_ERROR << "Configure: input for footpedal \"Cam-\" is required for \"endoscope-focus\".  "
    //                                  << footpedalMessage << std::endl;
    //         exit(EXIT_FAILURE);
    //     }
    //     const bool foundCamPlus = (mDInputSources.find("Cam+") != endDInputs);
    //     if (!foundCamPlus) {
    //         CMN_LOG_CLASS_INIT_ERROR << "Configure: input for footpedal \"Cam+\" is required for \"endoscope-focus\".  "
    //                                  << footpedalMessage << std::endl;
    //         exit(EXIT_FAILURE);
    //     }
    //     // schedule connections
    //     // m_connections.Add(endoscopeFocusName, "EndoscopeFocusIn",
    //     //                  m_IO_component_name, "EndoscopeFocusIn");
    //     // m_connections.Add(endoscopeFocusName, "EndoscopeFocusOut",
    //     //                  m_IO_component_name, "EndoscopeFocusOut");
    //     // m_connections.Add(endoscopeFocusName, "focus_in",
    //     //                  m_IO_component_name, "Cam+");
    //     // m_connections.Add(endoscopeFocusName, "focus_out",
    //     //                  m_IO_component_name, "Cam-");
    // }

    // if we have any teleoperation component, we need to have the interfaces for the foot pedals
    // unless user explicitly says we can skip
    // if (physicalFootpedalsRequired) {
    //     const DInputSourceType::const_iterator endDInputs = mDInputSources.end();
    //     const bool foundClutch = (mDInputSources.find("Clutch") != endDInputs);
    //     const bool foundOperatorPresent = (mDInputSources.find("OperatorPresent") != endDInputs);
    //     const bool foundCamera = (mDInputSources.find("Camera") != endDInputs);

    //     if (m_teleop_PSM_proxies.size() > 0) {
    //         if (!foundClutch || !foundOperatorPresent) {
    //             CMN_LOG_CLASS_INIT_ERROR << "Configure: inputs for footpedals \"Clutch\" and \"OperatorPresent\" need to be defined since there's at least one PSM tele-operation component.  "
    //                                      << footpedalMessage << std::endl;
    //             exit(EXIT_FAILURE);
    //         }
    //     }
    // if (mTeleopECM) {
    //     if (!foundCamera || !foundOperatorPresent) {
    //         CMN_LOG_CLASS_INIT_ERROR << "Configure: inputs for footpedals \"Camera\" and \"OperatorPresent\" need to be defined since there's an ECM tele-operation component.  "
    //                                  << footpedalMessage << std::endl;
    //         exit(EXIT_FAILURE);
    //     }
    // }
    // }
    // this->AddFootpedalInterfaces();

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
    //             arm->SUJInterfaceRequiredFromIO = this->AddInterfaceRequired("SUJClutch-" + arm->m_name + "_IO");
    //             arm->SUJInterfaceRequiredFromIO->AddEventHandlerWrite(&Arm::SUJClutchEventHandlerFromIO, arm, "Button");
    //             if (arm->m_generation == mtsIntuitiveResearchKitArm::GENERATION_Si) {
    //                 arm->SUJInterfaceRequiredFromIO2 = this->AddInterfaceRequired("SUJClutchBack-" + arm->m_name + "_IO");
    //                 arm->SUJInterfaceRequiredFromIO2->AddEventHandlerWrite(&Arm::SUJClutchEventHandlerFromIO, arm, "Button");
    //             }
    //             arm->SUJInterfaceRequiredToSUJ = this->AddInterfaceRequired("SUJClutch-" + arm->m_name);
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
    for (auto & iter : m_IO_proxies) {
        if (iter.second->m_config->close_all_relays) {
            iter.second->close_all_relays();
        }
    }

    // emit events for active PSM teleop pairs
    // EventSelectedTeleopPSMs();
    // emit scale event
    ConfigurationEvents.scale(mtsIntuitiveResearchKit::TeleOperationPSM::Scale);
    // emit volume event
    audio.volume(m_audio_volume);

    if (m_config.chatty) {
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

bool mtsIntuitiveResearchKitConsole::add_teleop_ECM_interfaces(std::shared_ptr<teleop_ECM_proxy_t> teleop_proxy)
{
    teleop_proxy->m_interface_required = this->AddInterfaceRequired(teleop_proxy->m_name);
    if (teleop_proxy->m_interface_required) {
        teleop_proxy->m_interface_required->AddFunction("state_command", teleop_proxy->state_command);
        teleop_proxy->m_interface_required->AddFunction("set_scale", teleop_proxy->set_scale);
        teleop_proxy->m_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler, this, "error");
        teleop_proxy->m_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler, this, "warning");
        teleop_proxy->m_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler, this, "status");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "add_teleop_ECM_interfaces: failed to add main interface for teleop \""
                                 << teleop_proxy->m_name << "\"" << std::endl;
        return false;
    }
    return true;
}


bool mtsIntuitiveResearchKitConsole::add_teleop_PSM_interfaces(std::shared_ptr<teleop_PSM_proxy_t> teleop_proxy)
{
    teleop_proxy->m_interface_required = this->AddInterfaceRequired(teleop_proxy->m_name);
    if (teleop_proxy->m_interface_required) {
        teleop_proxy->m_interface_required->AddFunction("state_command", teleop_proxy->state_command);
        teleop_proxy->m_interface_required->AddFunction("set_scale", teleop_proxy->set_scale);
        teleop_proxy->m_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler, this, "error");
        teleop_proxy->m_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler, this, "warning");
        teleop_proxy->m_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler, this, "status");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "add_teleop_PSM_interfaces: failed to add main interface for teleop \""
                                 << teleop_proxy->m_name << "\"" << std::endl;
        return false;
    }
    return true;
}

// void mtsIntuitiveResearchKitConsole::AddFootpedalInterfaces(void)
// {
//     const auto endDInputs = mDInputSources.end();

//     auto iter = mDInputSources.find("Clutch");
//     if (iter != endDInputs) {
//         mtsInterfaceRequired * clutchRequired = AddInterfaceRequired("Clutch");
//         if (clutchRequired) {
//             clutchRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ClutchEventHandler, this, "Button");
//         }
//         m_connections.Add(this->GetName(), "Clutch",
//                           iter->second.first, iter->second.second);
//     }
//     mtsInterfaceProvided * clutchProvided = AddInterfaceProvided("Clutch");
//     if (clutchProvided) {
//         clutchProvided->AddEventWrite(console_events.clutch, "Button", prmEventButton());
//     }

//     iter = mDInputSources.find("Camera");
//     if (iter != endDInputs) {
//         mtsInterfaceRequired * cameraRequired = AddInterfaceRequired("Camera");
//         if (cameraRequired) {
//             cameraRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::CameraEventHandler, this, "Button");
//         }
//         m_connections.Add(this->GetName(), "Camera",
//                           iter->second.first, iter->second.second);
//     }
//     mtsInterfaceProvided * cameraProvided = AddInterfaceProvided("Camera");
//     if (cameraProvided) {
//         cameraProvided->AddEventWrite(console_events.camera, "Button", prmEventButton());
//     }

//     iter = mDInputSources.find("OperatorPresent");
//     if (iter != endDInputs) {
//         mtsInterfaceRequired * operatorRequired = AddInterfaceRequired("OperatorPresent");
//         if (operatorRequired) {
//             operatorRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::OperatorPresentEventHandler, this, "Button");
//         }
//         m_connections.Add(this->GetName(), "OperatorPresent",
//                           iter->second.first, iter->second.second);
//     }
//     mtsInterfaceProvided * operatorProvided = AddInterfaceProvided("OperatorPresent");
//     if (operatorProvided) {
//         operatorProvided->AddEventWrite(console_events.operator_present, "Button", prmEventButton());
//     }
// }

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



// }

// bool mtsIntuitiveResearchKitConsole::ConfigureECMTeleopJSON(const Json::Value & jsonTeleop)
// {
// if (mtmLeftName == mtmRightName) {
//     CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: \"mtm-left\" and \"mtm-right\" must be different" << std::endl;
//     return false;
// }
// }
//     // default value
//     mTeleopECM->m_type = TeleopECM::TELEOP_ECM;
// }
//     return true;
// }

// bool mtsIntuitiveResearchKitConsole::ConfigurePSMTeleopJSON(const Json::Value & jsonTeleop)
// {

// Json::Value jsonValue = jsonTeleop["PSM_base_frame"];
// std::string baseFrameComponent, baseFrameInterface;
// if (!jsonValue.empty()) {
//     baseFrameComponent = jsonValue.get("component", "").asString();
//     baseFrameInterface = jsonValue.get("interface", "").asString();
//     if ((baseFrameComponent == "") || (baseFrameInterface == "")) {
//         CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: both \"component\" and \"interface\" must be provided with \"PSM_base_frame\" for teleop \""
//                                  << mtmName << "_" << psmName << "\"" << std::endl;
//         return false;
//     }
// }

// // check if pair already exist and then add
// const auto teleopIterator = m_teleop_PSM_proxies.find(name);
// TeleopPSM * teleopPointer = 0;
// if (teleopIterator == m_teleop_PSM_proxies.end()) {

//     // insert
//     m_teleop_PSM_proxiesByMTM.insert(std::make_pair(mtmName, teleopPointer));
//     m_teleop_PSM_proxiesByPSM.insert(std::make_pair(psmName, teleopPointer));

//     // first MTM with multiple PSMs is selected for single tap
//     if ((m_teleop_PSM_proxiesByMTM.count(mtmName) > 1)
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
//     m_teleop_PSM_proxies[name] = teleopPointer;
// } else {
//     CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: there is already a teleop for the pair \""
//                              << name << "\"" << std::endl;
//     return false;
// }

// const Json::Value jsonTeleopConfig = jsonTeleop["configure-parameter"];
// teleopPointer->ConfigureTeleop(teleopPointer->m_type, period, jsonTeleopConfig);
//     return true;
// }


bool mtsIntuitiveResearchKitConsole::add_IO_interfaces(std::shared_ptr<IO_proxy_t> IO)
{
    IO->m_interface_required = AddInterfaceRequired(IO->m_name);
    if (IO->m_interface_required) {
        IO->m_interface_required->AddFunction("close_all_relays", IO->close_all_relays);
        IO->m_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
                                                       this, "error");
        IO->m_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                       this, "warning");
        IO->m_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                       this, "status");
        m_connections.Add(this->GetName(), IO->m_name,
                          IO->m_name, "Configuration");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "add_IO_interfaces: failed to create IO required interface" << std::endl;
        return false;
    }
    return true;
}


bool mtsIntuitiveResearchKitConsole::add_arm_interfaces(std::shared_ptr<arm_proxy_t> arm)
{
    // IO
    if (arm->m_config->expects_IO()) {
        const std::string interfaceNameIO = "IO-" + arm->m_name;
        arm->m_IO_interface_required = AddInterfaceRequired(interfaceNameIO);
        if (arm->m_IO_interface_required) {
            arm->m_IO_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
                                                               this, "error");
            arm->m_IO_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                               this, "warning");
            arm->m_IO_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                               this, "status");
            m_connections.Add(this->GetName(), interfaceNameIO,
                              arm->m_IO_component_name, arm->m_name);
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "add_arm_interfaces: failed to add IO interface for arm \""
                                     << arm->m_name << "\"" << std::endl;
            return false;
        }
        // is the arm is a PSM, since it has an IO, it also has a
        // Dallas chip interface and we want to see the messages
        if (arm->m_config->native_or_derived_PSM()) {
            const std::string interfaceNameIODallas = "IO_dallas-" + arm->m_name;
            arm->m_IO_dallas_interface_required = AddInterfaceRequired(interfaceNameIODallas);
            if (arm->m_IO_dallas_interface_required) {
                arm->m_IO_dallas_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
                                                                          this, "error");
                arm->m_IO_dallas_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                                          this, "warning");
                arm->m_IO_dallas_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                                          this, "status");
                m_connections.Add(this->GetName(), interfaceNameIODallas,
                                  arm->m_IO_component_name, arm->m_name + "_dallas");
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "AddArmInterfaces: failed to add IO Dallas interface for arm \""
                                         << arm->m_name << "\"" << std::endl;
                return false;
            }
        }
    }

    // PID
    if (arm->m_config->expects_PID()) {
        const std::string interfaceNamePID = "PID_" + arm->m_name;
        arm->m_PID_interface_required = AddInterfaceRequired(interfaceNamePID);
        if (arm->m_PID_interface_required) {
            arm->m_PID_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
                                                                this, "error");
            arm->m_PID_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                                this, "warning");
            arm->m_PID_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                                this, "status");
            m_connections.Add(this->GetName(), interfaceNamePID,
                              arm->m_name + "_PID", "Controller");
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "add_arm_interfaces: failed to add PID interface for arm \""
                                     << arm->m_name << "\"" << std::endl;
            return false;
        }
    }

    // arm interface
    const std::string interfaceNameArm = arm->m_name;
    arm->m_arm_interface_required = AddInterfaceRequired(interfaceNameArm);
    if (arm->m_arm_interface_required) {
        arm->m_arm_interface_required->AddFunction("state_command", arm->state_command);
        if (!arm->m_config->SUJ()) {
            arm->m_arm_interface_required->AddFunction("hold", arm->hold, MTS_OPTIONAL);
        }
        arm->m_arm_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
                                                            this, "error");
        arm->m_arm_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
                                                            this, "warning");
        arm->m_arm_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
                                                            this, "status");
        arm->m_arm_interface_required->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::arm_proxy_t::CurrentStateEventHandler,
                                                            arm.get(), "operating_state");
        m_connections.Add(this->GetName(), interfaceNameArm,
                          arm->m_arm_component_name, arm->m_arm_interface_name);
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "add_arm_interfaces: failed to add Main interface for arm \""
                                 << arm->m_name << "\"" << std::endl;
        return false;
    }
    return true;
}

bool mtsIntuitiveResearchKitConsole::Connect(void)
{
    mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();

    // connect console for audio feedback
    component_manager->Connect(this->GetName(), "TextToSpeech",
                               m_text_to_speech->GetName(), "Commands");

    // arms
    for (const auto & iter : m_arm_proxies) {
        std::shared_ptr<arm_proxy_t> arm = iter.second;
        // arm specific interfaces
        arm->connect();
        // connect to SUJ if needed
        if (arm->SUJInterfaceRequiredFromIO) {
            component_manager->Connect(this->GetName(), arm->SUJInterfaceRequiredFromIO->GetName(),
                                       arm->m_IO_component_name, arm->m_name + "_SUJ_clutch");
        }
        if (arm->SUJInterfaceRequiredFromIO2) {
            component_manager->Connect(this->GetName(), arm->SUJInterfaceRequiredFromIO2->GetName(),
                                       arm->m_IO_component_name, arm->m_name + "_SUJ_clutch2");
        }
        if (arm->SUJInterfaceRequiredToSUJ) {
            component_manager->Connect(this->GetName(), arm->SUJInterfaceRequiredToSUJ->GetName(),
                                       "SUJ", arm->m_name);
        }
    }

    m_connections.Connect();

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
    // teleop_enable(false);
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

// void mtsIntuitiveResearchKitConsole::teleop_enable(const bool & enable)
// {
//     mTeleopEnabled = enable;
//     // if we have an SUJ, make sure it's ready
//     if (enable && m_SUJ) {
//         const auto sujState = ArmStates.find("SUJ");
//         if ((sujState == ArmStates.end())
//             || (sujState->second.State() != prmOperatingState::ENABLED)) {
//             mTeleopEnabled = false;
//         }
//     }
//     mTeleopDesired = enable;
//     // event
//     console_events.teleop_enabled(mTeleopEnabled);
//     UpdateTeleopState();
// }

// void mtsIntuitiveResearchKitConsole::cycle_teleop_PSM_by_MTM(const std::string & mtmName)
// {
//     // try to cycle through all the teleopPSMs associated to the MTM
//     if (m_teleop_PSM_proxies_by_MTM.count(mtmName) == 0) {
//         // we use empty string to query, no need to send warning about bad mtm name
//         if (mtmName != "") {
//             mInterface->SendWarning(this->GetName()
//                                     + ": no PSM teleoperation found for MTM \""
//                                     + mtmName
//                                     + "\"");
//         }
//     } else if (m_teleop_PSM_proxies_by_MTM.count(mtmName) == 1) {
//         mInterface->SendStatus(this->GetName()
//                                + ": only one PSM teleoperation found for MTM \""
//                                + mtmName
//                                + "\", cycling has no effect");
//     } else {
//         // find range of teleops
//         auto range = m_teleop_PSM_proxies_by_MTM.equal_range(mtmName);
//         for (auto iter = range.first;
//              iter != range.second;
//              ++iter) {
//             // find first teleop currently selected
//             if (iter->second->selected()) {
//                 // toggle to next one
//                 auto nextTeleop = iter;
//                 nextTeleop++;
//                 // if next one is last one, go back to first
//                 if (nextTeleop == range.second) {
//                     nextTeleop = range.first;
//                 }
//                 // now make sure the PSM in next teleop is not used
//                 std::string mtmUsingThatPSM;
//                 GetMTMSelectedForPSM(nextTeleop->second->m_config->PSM, mtmUsingThatPSM);
//                 if (mtmUsingThatPSM != "") {
//                     // message
//                     mInterface->SendWarning(this->GetName()
//                                             + ": cycling from \""
//                                             + iter->second->m_name
//                                             + "\" to \""
//                                             + nextTeleop->second->m_name
//                                             + "\" failed, PSM is already controlled by \""
//                                             + mtmUsingThatPSM
//                                             + "\"");
//                 } else {
//                     // mark which one should be active
//                     iter->second->set_selected(false);
//                     nextTeleop->second->set_selected(true);
//                     // if teleop PSM is active, enable/disable components now
//                     if (mTeleopEnabled) {
//                         iter->second->state_command(std::string("disable"));
//                         if (mTeleopPSMRunning) {
//                             nextTeleop->second->state_command(std::string("enable"));
//                         } else {
//                             nextTeleop->second->state_command(std::string("align_MTM"));
//                         }
//                     }
//                     // message
//                     mInterface->SendStatus(this->GetName()
//                                            + ": cycling from \""
//                                            + iter->second->m_name
//                                            + "\" to \""
//                                            + nextTeleop->second->m_name
//                                            + "\"");
//                 }
//                 // stop for loop
//                 break;
//             }
//         }
//     }
//     // in all cases, emit events so users can figure out which components are selected
//     EventSelectedTeleopPSMs();
// }

// void mtsIntuitiveResearchKitConsole::select_teleop_PSM(const prmKeyValue & mtmPsm)
// {
//     // for readability
//     const std::string mtmName = mtmPsm.Key;
//     const std::string psmName = mtmPsm.Value;

//     // if the psm value is empty, disable any teleop for the mtm -- this can be used to free the mtm
//     if (psmName == "") {
//         auto range = m_teleop_PSM_proxies_by_MTM.equal_range(mtmName);
//         for (auto iter = range.first;
//              iter != range.second;
//              ++iter) {
//             // look for the teleop that was selected if any
//             if (iter->second->selected()) {
//                 iter->second->set_selected(false);
//                 // if teleop PSM is active, enable/disable components now
//                 if (mTeleopEnabled) {
//                     iter->second->state_command(std::string("disable"));
//                 }
//                 // message
//                 mInterface->SendWarning(this->GetName()
//                                         + ": teleop \""
//                                         + iter->second->m_name
//                                         + "\" has been unselected ");
//             }
//         }
//         EventSelectedTeleopPSMs();
//         return;
//     }

//     // actual teleop to select
//     std::string name = mtmName + "_" + psmName;
//     const auto teleopIterator = m_teleop_PSM_proxies.find(name);
//     if (teleopIterator == m_teleop_PSM_proxies.end()) {
//         mInterface->SendWarning(this->GetName()
//                                 + ": unable to select \""
//                                 + name
//                                 + "\", this component doesn't exist");
//         EventSelectedTeleopPSMs();
//         return;
//     }
//     // there seems to be some redundant information here, let's use it for a safety check
//     CMN_ASSERT(mtmName == teleopIterator->second->m_config->MTM);
//     CMN_ASSERT(psmName == teleopIterator->second->m_config->PSM);
//     // check that the PSM is available to be used
//     std::string mtmUsingThatPSM;
//     GetMTMSelectedForPSM(psmName, mtmUsingThatPSM);
//     if (mtmUsingThatPSM != "") {
//         mInterface->SendWarning(this->GetName()
//                                 + ": unable to select \""
//                                 + name
//                                 + "\", PSM is already controlled by \""
//                                 + mtmUsingThatPSM
//                                 + "\"");
//         EventSelectedTeleopPSMs();
//         return;
//     }

//     // make sure the teleop using that MTM is unselected
//     select_teleop_PSM(prmKeyValue(mtmName, ""));

//     // now turn on the teleop
//     teleopIterator->second->set_selected(true);
//     // if teleop PSM is active, enable/disable components now
//     if (mTeleopEnabled) {
//         if (mTeleopPSMRunning) {
//             teleopIterator->second->state_command(std::string("enable"));
//         } else {
//             teleopIterator->second->state_command(std::string("align_MTM"));
//         }
//     }
//     // message
//     mInterface->SendStatus(this->GetName()
//                            + ": \""
//                            + teleopIterator->second->m_name
//                            + "\" has been selected");

//     // always send a message to let user know the current status
//     EventSelectedTeleopPSMs();
// }

// bool mtsIntuitiveResearchKitConsole::GetPSMSelectedForMTM(const std::string & mtmName, std::string & psmName) const
// {
//     bool mtmFound = false;
//     psmName = "";
//     // find range of teleops
//     auto range = m_teleop_PSM_proxies_by_MTM.equal_range(mtmName);
//     for (auto iter = range.first;
//          iter != range.second;
//          ++iter) {
//         mtmFound = true;
//         if (iter->second->selected()) {
//             psmName = iter->second->m_config->PSM;
//         }
//     }
//     return mtmFound;
// }

// bool mtsIntuitiveResearchKitConsole::GetMTMSelectedForPSM(const std::string & psmName, std::string & mtmName) const
// {
//     bool psmFound = false;
//     mtmName = "";
//     for (auto & iter : m_teleop_PSM_proxies) {
//         if (iter.second->m_config->PSM == psmName) {
//             psmFound = true;
//             if (iter.second->selected()) {
//                 mtmName = iter.second->m_config->MTM;
//             }
//         }
//     }
//     return psmFound;
// }

// void mtsIntuitiveResearchKitConsole::EventSelectedTeleopPSMs(void) const
// {
//     for (auto & iter : m_teleop_PSM_proxies) {
//         if (iter.second->selected()) {
//             ConfigurationEvents.teleop_PSM_selected(prmKeyValue(iter.second->m_config->MTM,
//                                                                 iter.second->m_config->PSM));
//         } else {
//             ConfigurationEvents.teleop_PSM_unselected(prmKeyValue(iter.second->m_config->MTM,
//                                                                   iter.second->m_config->PSM));
//         }
//     }
// }

// void mtsIntuitiveResearchKitConsole::UpdateTeleopState(void)
// {
//     // Check if teleop is enabled
//     if (!mTeleopEnabled) {
//         bool holdNeeded = false;
//         for (auto & iterTeleopPSM : m_teleop_PSM_proxies) {
//             iterTeleopPSM.second->state_command(std::string("disable"));
//             if (mTeleopPSMRunning) {
//                 holdNeeded = true;
//             }
//             mTeleopPSMRunning = false;
//         }

//         // if (mTeleopECM) {
//         //     mTeleopECM->state_command(std::string("disable"));
//         //     if (mTeleopECMRunning) {
//         //         holdNeeded = true;
//         //     }
//         //     mTeleopECMRunning = false;
//         // }

//         // hold arms if we stopped any teleop
//         if (holdNeeded) {
//             for (auto & arm_proxy : m_arm_proxies) {
//                 if (arm_proxy.second->m_config->MTM()
//                     && arm_proxy.second->hold.IsValid()) {
//                     arm_proxy.second->hold();
//                 }
//             }
//         }
//         return;
//     }

//     // if none are running, hold
//     if (!mTeleopECMRunning && !mTeleopPSMRunning) {
//         for (auto & arm_proxy : m_arm_proxies) {
//             if (arm_proxy.second->m_config->MTM()
//                 && arm_proxy.second->hold.IsValid()) {
//                 arm_proxy.second->hold();
//             }
//         }
//     }

//     // all fine
//     bool readyForTeleop = mOperatorPresent;

//     for (auto & arm_proxy : m_arm_proxies) {
//         if (arm_proxy.second->m_SUJ_clutched) {
//             readyForTeleop = false;
//         }
//     }

//     // Check if operator is present
//     if (!readyForTeleop) {
//         // keep MTMs aligned
//         for (auto & iterTeleopPSM : m_teleop_PSM_proxies) {
//             if (iterTeleopPSM.second->selected()) {
//                 iterTeleopPSM.second->state_command(std::string("align_MTM"));
//             } else {
//                 iterTeleopPSM.second->state_command(std::string("disable"));
//             }
//         }
//         mTeleopPSMRunning = false;

//         // // stop ECM if needed
//         // if (mTeleopECMRunning) {
//         //     mTeleopECM->state_command(std::string("disable"));
//         //     mTeleopECMRunning = false;
//         // }
//         return;
//     }

//     // If camera is pressed for ECM Teleop or not
//     if (mCameraPressed) {
//         if (!mTeleopECMRunning) {
//             // if PSM was running so we need to stop it
//             if (mTeleopPSMRunning) {
//                 for (auto & iterTeleopPSM : m_teleop_PSM_proxies) {
//                     iterTeleopPSM.second->state_command(std::string("disable"));
//                 }
//                 mTeleopPSMRunning = false;
//             }
//             // ECM wasn't running, let's start it
//             // if (mTeleopECM) {
//             //     mTeleopECM->state_command(std::string("enable"));
//             //     mTeleopECMRunning = true;
//             // }
//         }
//     } else {
//         // we must teleop PSM
//         if (!mTeleopPSMRunning) {
//             // if ECM was running so we need to stop it
//             // if (mTeleopECMRunning) {
//             //     mTeleopECM->state_command(std::string("disable"));
//             //     mTeleopECMRunning = false;
//             // }
//             // PSM wasn't running, let's start it
//             for (auto & iterTeleopPSM : m_teleop_PSM_proxies) {
//                 if (iterTeleopPSM.second->selected()) {
//                     iterTeleopPSM.second->state_command(std::string("enable"));
//                 } else {
//                     iterTeleopPSM.second->state_command(std::string("disable"));
//                 }
//                 mTeleopPSMRunning = true;
//             }
//         }
//     }
// }

// void mtsIntuitiveResearchKitConsole::set_scale(const double & scale)
// {
//     for (auto & iter : m_teleop_PSM_proxies) {
//         iter.second->set_scale(scale);
//     }
//     ConfigurationEvents.scale(scale);
// }

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

// void mtsIntuitiveResearchKitConsole::ClutchEventHandler(const prmEventButton & button)
// {
//     switch (button.Type()) {
//     case prmEventButton::PRESSED:
//         mInterface->SendStatus(this->GetName() + ": clutch pressed");
//         audio.beep(vct3(0.1, 700.0, m_audio_volume));
//         break;
//     case prmEventButton::RELEASED:
//         mInterface->SendStatus(this->GetName() + ": clutch released");
//         audio.beep(vct3(0.1, 700.0, m_audio_volume));
//         break;
//     case prmEventButton::CLICKED:
//         mInterface->SendStatus(this->GetName() + ": clutch quick tap");
//         audio.beep(vct3(0.05, 2000.0, m_audio_volume));
//         audio.beep(vct3(0.05, 2000.0, m_audio_volume));
//         // if (mTeleopMTMToCycle != "") {
//         //     cycle_teleop_PSM_by_MTM(mTeleopMTMToCycle);
//         // }
//         break;
//     default:
//         break;
//     }
//     console_events.clutch(button);
// }

// void mtsIntuitiveResearchKitConsole::CameraEventHandler(const prmEventButton & button)
// {
//     switch (button.Type()) {
//     case prmEventButton::PRESSED:
//         // mCameraPressed = true;
//         mInterface->SendStatus(this->GetName() + ": camera pressed");
//         audio.beep(vct3(0.1, 1000.0, m_audio_volume));
//         break;
//     case prmEventButton::RELEASED:
//         mCameraPressed = false;
//         mInterface->SendStatus(this->GetName() + ": camera released");
//         audio.beep(vct3(0.1, 1000.0, m_audio_volume));
//         break;
//     case prmEventButton::CLICKED:
//         mInterface->SendStatus(this->GetName() + ": camera quick tap");
//         audio.beep(vct3(0.05, 2500.0, m_audio_volume));
//         audio.beep(vct3(0.05, 2500.0, m_audio_volume));
//         break;
//     default:
//         break;
//     }
//     UpdateTeleopState();
//     console_events.camera(button);
// }

// void mtsIntuitiveResearchKitConsole::OperatorPresentEventHandler(const prmEventButton & button)
// {
//     switch (button.Type()) {
//     case prmEventButton::PRESSED:
//         mOperatorPresent = true;
//         mInterface->SendStatus(this->GetName() + ": operator present");
//         audio.beep(vct3(0.3, 1500.0, m_audio_volume));
//         break;
//     case prmEventButton::RELEASED:
//         mOperatorPresent = false;
//         mInterface->SendStatus(this->GetName() + ": operator not present");
//         audio.beep(vct3(0.3, 1200.0, m_audio_volume));
//         break;
//     default:
//         break;
//     }
//     UpdateTeleopState();
//     console_events.operator_present(button);
// }

// void mtsIntuitiveResearchKitConsole::ErrorEventHandler(const mtsMessage & message)
// {
//     // similar to teleop_enable(false) except we don't change mTeleopDesired
//     mTeleopEnabled = false;
//     console_events.teleop_enabled(mTeleopEnabled);
//     UpdateTeleopState();

//     mInterface->SendError(message.Message);
//     // throttle error beeps
//     double currentTime = mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime();
//     if ((currentTime - mTimeOfLastErrorBeep) > 2.0 * cmn_s) {
//         audio.beep(vct3(0.3, 3000.0, m_audio_volume));
//         mTimeOfLastErrorBeep = currentTime;
//     }
// }

void mtsIntuitiveResearchKitConsole::ErrorEventHandler(const mtsMessage & message)
{
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
    // if (mTeleopDesired && !mTeleopEnabled) {
    //     teleop_enable(true);
    // }

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
