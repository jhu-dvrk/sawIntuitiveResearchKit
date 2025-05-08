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

#include <sawIntuitiveResearchKit/arm_proxy.h>

#include <cisstMultiTask/mtsManagerLocal.h>

#include <sawControllers/mtsPID.h>

#include <sawRobotIO1394/mtsRobotIO1394.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitMTM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitECM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitSUJ.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitSUJFixed.h>
#include <sawIntuitiveResearchKit/system.h>

#include <sawIntuitiveResearchKit/IO_proxy.h>

dvrk::arm_proxy::arm_proxy(const std::string & name,
                           dvrk::system * system,
                           dvrk::arm_proxy_configuration * config):
    m_name(name),
    m_system(system),
    m_config(config),
    m_arm_component_name(name),
    m_arm_interface_name("Arm")
{
}


void dvrk::arm_proxy::post_configure(void)
{
    // make sure IO component is provided
    if (m_config->expects_IO()) {
        // valid string
        if (m_config->IO == "") {
            CMN_LOG_INIT_ERROR << "arm_proxy::post_configure: \"IO\" must be provided for native or derived arm: "
                               << m_name << std::endl;
            exit(EXIT_FAILURE);
        }
        // registered in component manager?
        auto iter = m_system->m_IO_proxies.find(m_config->IO);
        if (iter == m_system->m_IO_proxies.end()) {
            CMN_LOG_INIT_ERROR << "arm_proxy::post_configure: IO \""
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
            CMN_LOG_INIT_ERROR << "arm_proxy::post_configure: \"component\" and \"interface\" must be provided for generic or derived arm: "
                               << m_name << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // check base frame settings
    if (!m_config->base_frame.valid()) {
        CMN_LOG_INIT_ERROR << "arm_proxy::post_configure, base_frame must be defined by either \"reference_frame\" name and \"transform\" OR \"component\" and \"interface\" for "
                           << m_name << std::endl;
        exit(EXIT_FAILURE);
    } else {
        if ((m_config->base_frame.component != "") && (m_config->base_frame.interface != "")) {
            m_base_frame_component_name = m_config->base_frame.component;
            m_base_frame_interface_name = m_config->base_frame.interface;
        }
    }
}


void dvrk::arm_proxy::create_arm(void)
{
    if (!m_config->native_or_derived()) {
        return;
    }

    // infer arm configuration file
    // -1- provided by user
    if (m_config->arm_file != "") {
        m_arm_configuration_file = m_system->find_file(m_config->arm_file);
        if (m_arm_configuration_file == "") {
            CMN_LOG_INIT_ERROR << "arm_proxy::create_arm: can't find configuration file " << m_config->arm_file
                               << " for arm " << m_name << std::endl;
            exit(EXIT_FAILURE);
        }
    } else {
        // -2- using serial number
        if (m_config->type != dvrk::arm_type::FOCUS_CONTROLLER) {
            if (m_config->serial == "") {
                CMN_LOG_INIT_ERROR << "arm_proxy::create_arm: serial number required for arm "
                                   << m_name << std::endl;
                exit(EXIT_FAILURE);
            }
            const auto default_file = m_name + "-" + m_config->serial + ".json";
            m_arm_configuration_file = m_system->find_file(default_file);
            if (m_arm_configuration_file == "") {
                CMN_LOG_INIT_ERROR << "arm_proxy::create_arm: can't find \"arm\" setting for arm "
                                   << m_name << ". \"arm\" is not set and the default file "
                                   << default_file << " doesn't seem to exist either." << std::endl;
                exit(EXIT_FAILURE);
            } else {
                CMN_LOG_INIT_VERBOSE << "arm_proxy::create_arm: using the default file "
                                     << default_file << " for arm " << m_name << std::endl;

            }
        }
    }


    mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();
    // for research kit arms, create, add to manager and connect to
    // extra IO, PID, etc.  For generic arms, do nothing.
    switch (m_config->type) {
    case dvrk::arm_type::MTM:
        {
            m_arm = std::make_shared<mtsIntuitiveResearchKitMTM>(m_name, m_config->period);
        }
        break;
    case dvrk::arm_type::PSM:
        {
            m_arm = std::make_shared<mtsIntuitiveResearchKitPSM>(m_name, m_config->period);
        }
        break;
    case dvrk::arm_type::ECM:
        {
            m_arm = std::make_shared<mtsIntuitiveResearchKitECM>(m_name, m_config->period);
        }
        break;
    case dvrk::arm_type::SUJ_Classic:
        {
            mtsIntuitiveResearchKitSUJ * suj = new mtsIntuitiveResearchKitSUJ(m_name, m_config->period);
            if (m_config->simulation == dvrk::simulation::SIMULATION_KINEMATIC) {
                suj->set_simulated();
            } else if (m_config->simulation == dvrk::simulation::SIMULATION_NONE) {
                m_system->m_connections.Add(m_name, "NoMuxReset",
                                            m_IO_component_name, "NoMuxReset");
                m_system->m_connections.Add(m_name, "MuxIncrement",
                                            m_IO_component_name, "MuxIncrement");
                m_system->m_connections.Add(m_name, "ControlPWM",
                                            m_IO_component_name, "ControlPWM");
                m_system->m_connections.Add(m_name, "DisablePWM",
                                            m_IO_component_name, "DisablePWM");
                m_system->m_connections.Add(m_name, "MotorUp",
                                            m_IO_component_name, "MotorUp");
                m_system->m_connections.Add(m_name, "MotorDown",
                                            m_IO_component_name, "MotorDown");
                m_system->m_connections.Add(m_name, "SUJ-Clutch-1",
                                            m_IO_component_name, "SUJ-Clutch-1");
                m_system->m_connections.Add(m_name, "SUJ-Clutch-2",
                                            m_IO_component_name, "SUJ-Clutch-2");
                m_system->m_connections.Add(m_name, "SUJ-Clutch-3",
                                            m_IO_component_name, "SUJ-Clutch-3");
                m_system->m_connections.Add(m_name, "SUJ-Clutch-4",
                                            m_IO_component_name, "SUJ-Clutch-4");
            }
            suj->Configure(m_arm_configuration_file);
            component_manager->AddComponent(suj);
        }
        break;
    case dvrk::arm_type::SUJ_Si:
        {
#if sawIntuitiveResearchKit_HAS_SUJ_Si
            mtsIntuitiveResearchKitSUJSi * suj = new mtsIntuitiveResearchKitSUJSi(m_name, m_config->period);
            if (m_config->simulation == dvrk::simulation::SIMULATION_KINEMATIC) {
                suj->set_simulated();
            }
            suj->Configure(m_arm_configuration_file);
            component_manager->AddComponent(suj);
#else
            CMN_LOG_INIT_ERROR << "dvrk::arm_proxy::ConfigureArm: can't create an arm of type SUJ_Si because sawIntuitiveResearchKit_HAS_SUJ_Si is set to OFF in CMake"
                               << std::endl;
            exit(EXIT_FAILURE);
#endif
        }
        break;
    case dvrk::arm_type::SUJ_Fixed:
        {
            mtsIntuitiveResearchKitSUJFixed * suj = new mtsIntuitiveResearchKitSUJFixed(m_name, m_config->period);
            suj->Configure(m_arm_configuration_file);
            component_manager->AddComponent(suj);
        }
        break;
    case dvrk::arm_type::MTM_DERIVED:
        {
            mtsComponent * component;
            component = component_manager->GetComponent(m_name);
            if (component) {
                mtsIntuitiveResearchKitMTM * mtm = dynamic_cast<mtsIntuitiveResearchKitMTM *>(component);
                if (mtm) {
                    std::cerr << CMN_LOG_DETAILS << " is this risky?" << std::endl;
                    m_arm = std::shared_ptr<mtsIntuitiveResearchKitMTM>(mtm, [](mtsIntuitiveResearchKitMTM * p){ ; });
                } else {
                    CMN_LOG_INIT_ERROR << "dvrk::arm_proxy::ConfigureArm: component \""
                                       << m_name << "\" doesn't seem to be derived from mtsIntuitiveResearchKitMTM."
                                       << std::endl;
                    exit(EXIT_FAILURE);
                }
            } else {
                CMN_LOG_INIT_ERROR << "dvrk::arm_proxy::ConfigureArm: component \""
                                   << m_name << "\" not found."
                                   << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        break;
    case dvrk::arm_type::PSM_DERIVED:
        {
            mtsComponent * component;
            component = component_manager->GetComponent(m_name);
            if (component) {
                mtsIntuitiveResearchKitPSM * psm = dynamic_cast<mtsIntuitiveResearchKitPSM *>(component);
                if (psm) {
                    std::cerr << CMN_LOG_DETAILS << " is this risky?" << std::endl;
                    m_arm = std::shared_ptr<mtsIntuitiveResearchKitArm>(psm);
                } else {
                    CMN_LOG_INIT_ERROR << "dvrk::arm_proxy::ConfigureArm: component \""
                                       << m_name << "\" doesn't seem to be derived from mtsIntuitiveResearchKitPSM."
                                       << std::endl;
                    exit(EXIT_FAILURE);
                }
            } else {
                CMN_LOG_INIT_ERROR << "dvrk::arm_proxy::ConfigureArm: component \""
                                   << m_name << "\" not found."
                                   << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        break;
    case dvrk::arm_type::ECM_DERIVED:
        {
            mtsComponent * component;
            component = component_manager->GetComponent(m_name);
            if (component) {
                mtsIntuitiveResearchKitECM * ecm = dynamic_cast<mtsIntuitiveResearchKitECM *>(component);
                if (ecm) {
                    std::cerr << CMN_LOG_DETAILS << " is this risky?" << std::endl;
                    m_arm = std::shared_ptr<mtsIntuitiveResearchKitArm>(ecm);
                } else {
                    CMN_LOG_INIT_ERROR << "dvrk::arm_proxy::ConfigureArm: component \""
                                       << m_name << "\" doesn't seem to be derived from mtsIntuitiveResearchKitECM."
                                       << std::endl;
                    exit(EXIT_FAILURE);
                }
            } else {
                CMN_LOG_INIT_ERROR << "dvrk::arm_proxy::ConfigureArm: component \""
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
        if (m_config->simulation == dvrk::simulation::SIMULATION_KINEMATIC) {
            m_arm->set_simulated();
        }
        m_arm->set_calibration_mode(m_calibration_mode);
        m_arm->Configure(m_arm_configuration_file);
        set_base_frame_if_needed();
        component_manager->AddComponent(m_arm.get());

        // for all native arms not simulated, connect a few IOS
        if (m_config->simulation == dvrk::simulation::SIMULATION_NONE) {

            if (m_config->PSM()) {
                std::vector<std::string> itfs = {"adapter", "tool", "arm_clutch", "dallas"};
                for (const auto & itf : itfs) {
                    m_system->m_connections.Add(m_name, itf,
                                                m_IO_component_name, m_name + "_" + itf);
                }
            }

            if (m_config->ECM()) {
                m_system->m_connections.Add(m_name, "arm_clutch",
                                            m_IO_component_name, m_name + "_arm_clutch");
            }

            // for Si patient side, connect the SUJ brakes to buttons on arm
            if ((m_config->PSM() || m_config->ECM())
                && (m_arm->generation() == dvrk::generation::Si)) {
                std::vector<std::string> itfs = {"SUJ_clutch", "SUJ_clutch2", "SUJ_brake"};
                for (const auto & itf : itfs) {
                    m_system->m_connections.Add(m_name, itf,
                                                m_IO_component_name, m_name + "_" + itf);
                }
            }
        }
    }
}


void dvrk::arm_proxy::configure_IO(void)
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
            CMN_LOG_INIT_ERROR << "arm_proxy::configure_IO: can't find \"serial\" nor \"IO_file\" setting for arm \""
                               << m_name << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    m_IO_configuration_file = m_system->find_file(config_file);
    if (m_IO_configuration_file == "") {
        CMN_LOG_INIT_ERROR << "arm_proxy::configure_IO: can't find IO file " << config_file << std::endl;
        exit(EXIT_FAILURE);
    }

    // search for the IO component
    auto iter_IO = m_system->m_IO_proxies.find(m_config->IO);
    CMN_LOG_INIT_VERBOSE << "arm_proxy::configure_IO: configuring IO \""
                         << m_config->IO << "\" for arm " << m_name
                         << " using configuration file: " << m_IO_configuration_file << std::endl;
    // configure_IO should only happen if IO is valid
    CMN_ASSERT(iter_IO != m_system->m_IO_proxies.end());
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
                CMN_LOG_INIT_ERROR << "arm_proxy::configure_IO: can't find \"serial\" nor \"IO_gripper_file\" setting for arm \""
                                   << m_name << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        m_IO_gripper_configuration_file = m_system->find_file(config_file);
        if (m_IO_gripper_configuration_file == "") {
            CMN_LOG_INIT_ERROR << "arm_proxy::configure_IO: can't find IO gripper file " << config_file << std::endl;
            exit(EXIT_FAILURE);
        }

        // re-use IO component
        iter_IO->second->m_IO->Configure(m_IO_gripper_configuration_file);
    }
}


void dvrk::arm_proxy::create_PID(void)
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
            if (generation == dvrk::generation::Classic) {
                m_PID_configuration_file = "pid/sawControllersPID-PSM.json";
            } else {
                m_PID_configuration_file = "pid/sawControllersPID-PSM-Si.json";
            }
        } else if (m_config->native_or_derived_ECM()) {
            if (generation == dvrk::generation::Classic) {
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

    auto configuration_file = m_system->find_file(m_PID_configuration_file);
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
    if (m_config->simulation == dvrk::simulation::SIMULATION_KINEMATIC) {
        pid->SetSimulated();
        hasIO = false;
    }
    component_manager->AddComponent(pid);
    if (hasIO) {
        m_system->m_connections.Add(m_PID_component_name, "RobotJointTorqueInterface",
                                    m_IO_component_name, m_IO_interface_name);
        if (m_config->PID_period == 0.0) {
            m_system->m_connections.Add(m_PID_component_name, "ExecIn",
                                        m_IO_component_name, "ExecOut");
        }
    }
}


void dvrk::arm_proxy::set_base_frame_if_needed(void)
{
    // use reference_name to determine if we're using a fixed base frame
    if (m_config->base_frame.reference_frame != "") {
        m_base_frame.Goal().From(m_config->base_frame.transform);
        m_base_frame.ReferenceFrame() = m_config->base_frame.reference_frame;
        m_base_frame.Valid() = true;

        if (m_arm == nullptr) {
            CMN_LOG_INIT_ERROR << "arm_proxy::set_base_frame_if_needed failed, arm needs to be configured first" << std::endl;
            exit(EXIT_FAILURE);
        }
        m_arm->set_base_frame(m_base_frame);
    }
}


bool dvrk::arm_proxy::connect(void)
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


dvrk::generation dvrk::arm_proxy::generation(void) const
{
    if (m_arm != nullptr) {
        return m_arm->generation();
    } else {
        CMN_LOG_INIT_ERROR << "arm_proxy::generation failed, arm needs to be configured first" << std::endl;
        exit(EXIT_FAILURE);
    }
    return dvrk::generation::GENERATION_UNDEFINED;
}


void dvrk::arm_proxy::CurrentStateEventHandler(const prmOperatingState & currentState)
{
    m_system->set_arm_current_state(m_name, currentState);
}
