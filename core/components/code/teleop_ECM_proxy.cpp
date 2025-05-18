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

#include <sawIntuitiveResearchKit/teleop_ECM_proxy.h>

#include <cisstMultiTask/mtsManagerLocal.h>

#include <sawIntuitiveResearchKit/system.h>
#include <sawIntuitiveResearchKit/arm_proxy.h>
#include <sawIntuitiveResearchKit/console.h>
#include <sawIntuitiveResearchKit/teleop_proxy_configuration.h>

#include <sawIntuitiveResearchKit/mtsTeleOperationECM.h>

dvrk::teleop_ECM_proxy::teleop_ECM_proxy(const std::string & name,
                                         dvrk::system * system,
                                         dvrk::console * console,
                                         dvrk::teleop_ECM_proxy_configuration * config):
    teleop_proxy(name, system, console),
    m_config(config)
{
}


void dvrk::teleop_ECM_proxy::post_configure(void)
{
    // check that all arms have been defined and have correct type
    // MTML
    auto iter = m_system->m_arm_proxies.find(m_config->MTML);
    if (iter == m_system->m_arm_proxies.end()) {
        CMN_LOG_INIT_ERROR << "teleop_ECM_proxy::post_configure: MTML \""
                           << m_config->MTML << "\" is not defined in \"arms\"" << std::endl;
        exit(EXIT_FAILURE);
    } else {
        auto arm_proxy = iter->second;
        if (!arm_proxy->m_config->MTM()) {
            CMN_LOG_INIT_ERROR << "teleop_ECM_proxy::post_configure: MTML \""
                               << m_config->MTML << "\" type must be some type of MTM" << std::endl;
            exit(EXIT_FAILURE);
        }
        m_MTML_component_name = arm_proxy->m_arm_component_name;
        m_MTML_interface_name = arm_proxy->m_arm_interface_name;
    }
    // MTMR
    iter = m_system->m_arm_proxies.find(m_config->MTMR);
    if (iter == m_system->m_arm_proxies.end()) {
        CMN_LOG_INIT_ERROR << "teleop_ECM_proxy::post_configure: MTMR \""
                           << m_config->MTMR << "\" is not defined in \"arms\"" << std::endl;
        exit(EXIT_FAILURE);
    } else {
        auto arm_proxy = iter->second;
        if (!arm_proxy->m_config->MTM()) {
            CMN_LOG_INIT_ERROR << "teleop_ECM_proxy::post_configure: MTMR \""
                               << m_config->MTMR << "\" type must be some type of MTM" << std::endl;
            exit(EXIT_FAILURE);
        }
        m_MTMR_component_name = arm_proxy->m_arm_component_name;
        m_MTMR_interface_name = arm_proxy->m_arm_interface_name;
    }

    // ECM
    iter = m_system->m_arm_proxies.find(m_config->ECM);
    if (iter == m_system->m_arm_proxies.end()) {
        CMN_LOG_INIT_ERROR << "teleop_ECM_proxy::post_configure: ECM \""
                           << m_config->ECM << "\" is not defined in \"arms\"" << std::endl;
        exit(EXIT_FAILURE);
    } else {
        auto arm_proxy = iter->second;
        if (!arm_proxy->m_config->ECM()) {
            CMN_LOG_INIT_ERROR << "teleop_ECM_proxy::post_configure: ECM \""
                               << m_config->ECM << "\" type must be some type of ECM" << std::endl;
            exit(EXIT_FAILURE);
        }
        m_ECM_component_name = arm_proxy->m_arm_component_name;
        m_ECM_interface_name = arm_proxy->m_arm_interface_name;
    }

    // make sure both MTMs are different
    if ((m_MTML_component_name == m_MTMR_component_name)
        && (m_MTML_interface_name == m_MTMR_interface_name)) {
        CMN_LOG_INIT_ERROR << "teleop_ECM_proxy::post_configure: MTMs must be different" << std::endl;
        exit(EXIT_FAILURE);
    }

    m_arms_used.insert(m_config->MTML);
    m_arms_used.insert(m_config->MTMR);
    m_arms_used.insert(m_config->ECM);
}


void dvrk::teleop_ECM_proxy::create_teleop(void)
{
    mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();
    switch (m_config->type) {
    case dvrk::teleop_ECM_type::TELEOP_ECM:
        {
            mtsTeleOperationECM * teleop = new mtsTeleOperationECM(m_name, m_config->period);
            teleop->Configure(m_config->configure_parameter);
            component_manager->AddComponent(teleop);
        }
        break;
    case dvrk::teleop_ECM_type::TELEOP_ECM_DERIVED:
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

    // schedule connections
    m_system->m_connections.Add(m_name, "MTML",
                                m_MTML_component_name, m_MTML_interface_name);
    m_system->m_connections.Add(m_name, "MTMR",
                                m_MTMR_component_name, m_MTMR_interface_name);
    m_system->m_connections.Add(m_name, "ECM",
                                m_ECM_component_name, m_ECM_interface_name);
    m_system->m_connections.Add(m_name, "clutch",
                                 m_system->GetName(), m_console->m_name + "/clutch"); // clutch from console
    m_system->m_connections.Add(m_system->GetName(), m_name,
                                 m_name, "Setting");
    // if ((baseFrameComponent != "")
    //     && (baseFrameInterface != "")) {
    //     m_connections.Add(name, "ECM_base_frame",
    //                       baseFrameComponent, baseFrameInterface);
    // }
}
