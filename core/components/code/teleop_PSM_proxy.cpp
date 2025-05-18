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

#include <sawIntuitiveResearchKit/teleop_PSM_proxy.h>

#include <cisstMultiTask/mtsManagerLocal.h>

#include <sawIntuitiveResearchKit/system.h>
#include <sawIntuitiveResearchKit/arm_proxy.h>
#include <sawIntuitiveResearchKit/console.h>
#include <sawIntuitiveResearchKit/teleop_proxy_configuration.h>

#include <sawIntuitiveResearchKit/mtsTeleOperationPSM.h>

dvrk::teleop_PSM_proxy::teleop_PSM_proxy(const std::string & name,
                                         dvrk::system * system,
                                         dvrk::console * console,
                                         dvrk::teleop_PSM_proxy_configuration * config):
    teleop_proxy(name, system, console),
    m_config(config)
{
}


void dvrk::teleop_PSM_proxy::post_configure(void)
{
    auto iter = m_system->m_arm_proxies.find(m_config->MTM);
    if (iter == m_system->m_arm_proxies.end()) {
        CMN_LOG_INIT_ERROR << "teleop_PSM_proxy::post_configure: MTM \""
                           << m_config->MTM << "\" is not defined in \"arms\"" << std::endl;
        exit(EXIT_FAILURE);
    } else {
        auto arm_proxy = iter->second;
        if (!arm_proxy->m_config->MTM()) {
            CMN_LOG_INIT_ERROR << "teleop_PSM_proxy::post_configure: MTM \""
                               << m_config->MTM << "\" type must be some type of MTM" << std::endl;
            exit(EXIT_FAILURE);
        }
        m_MTM_component_name = arm_proxy->m_arm_component_name;
        m_MTM_interface_name = arm_proxy->m_arm_interface_name;
    }
    iter = m_system->m_arm_proxies.find(m_config->PSM);
    if (iter == m_system->m_arm_proxies.end()) {
        CMN_LOG_INIT_ERROR << "teleop_PSM_proxy::post_configure: PSM \""
                           << m_config->PSM << "\" is not defined in \"arms\"" << std::endl;
        exit(EXIT_FAILURE);
    } else {
        auto arm_proxy = iter->second;
        if (!arm_proxy->m_config->PSM()) {
            CMN_LOG_INIT_ERROR << "teleop_PSM_proxy::post_configure: PSM \""
                               << m_config->PSM << "\" type must be some type of PSM" << std::endl;
            exit(EXIT_FAILURE);
        }
        m_PSM_component_name = arm_proxy->m_arm_component_name;
        m_PSM_interface_name = arm_proxy->m_arm_interface_name;
    }
    m_arms_used.insert(m_config->MTM);
    m_arms_used.insert(m_config->PSM);
}


void dvrk::teleop_PSM_proxy::create_teleop(void)
{
    mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();
    switch (m_config->type) {
    case dvrk::teleop_PSM_type::TELEOP_PSM:
        {
            mtsTeleOperationPSM * teleop = new mtsTeleOperationPSM(m_name, m_config->period);
            teleop->Configure(m_config->configure_parameter);
            component_manager->AddComponent(teleop);
        }
        break;
    case dvrk::teleop_PSM_type::TELEOP_PSM_DERIVED:
        {
            mtsComponent * component;
            component = component_manager->GetComponent(m_name);
            if (component) {
                mtsTeleOperationPSM * teleop = dynamic_cast<mtsTeleOperationPSM *>(component);
                if (teleop) {
                    teleop->Configure(m_config->configure_parameter);
                } else {
                    CMN_LOG_INIT_ERROR << "teleop_PSM_proxy::create_teleop: component \""
                                       << m_name << "\" doesn't seem to be derived from mtsTeleOperationPSM."
                                       << std::endl;
                    exit(EXIT_FAILURE);
                }
            } else {
                CMN_LOG_INIT_ERROR << "teleop_PSM_proxy::create_teleop: component \""
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
    m_system->m_connections.Add(m_name, "MTM",
                                m_MTM_component_name, m_MTM_interface_name);
    m_system->m_connections.Add(m_name, "PSM",
                                m_PSM_component_name, m_PSM_interface_name);
    m_system->m_connections.Add(m_name, "clutch",
                                m_system->GetName(), m_console->m_name + "/clutch"); // clutch from console
    m_system->m_connections.Add(m_system->GetName(), m_name,
                                m_name, "Setting");
    // if ((baseFrameComponent != "")
    //     && (baseFrameInterface != "")) {
    //     m_connections.Add(name, "PSM_base_frame",
    //                       baseFrameComponent, baseFrameInterface);
    // }
}
