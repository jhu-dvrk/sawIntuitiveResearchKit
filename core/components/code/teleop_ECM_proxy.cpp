/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-17

  (C) Copyright 2013-2026 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <sawIntuitiveResearchKit/teleop_ECM_proxy_configuration.h>

#include <sawIntuitiveResearchKit/mtsTeleOperationECMTwoHands.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationECMOneHand.h>

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
    // check that ECM has been defined and has correct type
    auto iter = m_system->m_arm_proxies.find(m_config->ECM);
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
    m_arms_used.insert(m_config->ECM);

    if (m_config->one_hand()) {
        if (m_config->type == dvrk::teleop_ECM_type::TELEOP_ECM_TWO_HANDS) {
            m_config->type = dvrk::teleop_ECM_type::TELEOP_ECM_ONE_HAND;
        }
        std::string mtm = m_config->MTM;
        if (mtm.empty()) {
            mtm = m_config->MTMR.empty() ? m_config->MTML : m_config->MTMR;
        }
        iter = m_system->m_arm_proxies.find(mtm);
        if (iter == m_system->m_arm_proxies.end()) {
            CMN_LOG_INIT_ERROR << "teleop_ECM_proxy::post_configure: MTM \""
                               << mtm << "\" is not defined in \"arms\"" << std::endl;
            exit(EXIT_FAILURE);
        } else {
            auto arm_proxy = iter->second;
            if (!arm_proxy->m_config->MTM()) {
                CMN_LOG_INIT_ERROR << "teleop_ECM_proxy::post_configure: MTM \""
                                   << mtm << "\" type must be some type of MTM" << std::endl;
                exit(EXIT_FAILURE);
            }
            m_MTM_component_name = arm_proxy->m_arm_component_name;
            m_MTM_interface_name = arm_proxy->m_arm_interface_name;
        }
        m_arms_used.insert(mtm);
    } else {
        // check that both MTMs have been defined and have correct type
        // MTML
        iter = m_system->m_arm_proxies.find(m_config->MTML);
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

        // make sure both MTMs are different
        if ((m_MTML_component_name == m_MTMR_component_name)
            && (m_MTML_interface_name == m_MTMR_interface_name)) {
            CMN_LOG_INIT_ERROR << "teleop_ECM_proxy::post_configure: MTMs must be different" << std::endl;
            exit(EXIT_FAILURE);
        }

        m_arms_used.insert(m_config->MTML);
        m_arms_used.insert(m_config->MTMR);
    }
}


void dvrk::teleop_ECM_proxy::create_teleop(void)
{
    mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();
    auto type = m_config->type;
    if (m_config->one_hand() && type == dvrk::teleop_ECM_type::TELEOP_ECM_TWO_HANDS) {
        type = dvrk::teleop_ECM_type::TELEOP_ECM_ONE_HAND;
    }
    switch (type) {
    case dvrk::teleop_ECM_type::TELEOP_ECM_TWO_HANDS:
        {
            mtsTeleOperationECMTwoHands * teleop = new mtsTeleOperationECMTwoHands(m_name, m_config->period);
            teleop->Configure(m_config->configure_parameter);
            component_manager->AddComponent(teleop);
        }
        break;
    case dvrk::teleop_ECM_type::TELEOP_ECM_ONE_HAND:
        {
            mtsTeleOperationECMOneHand * teleop = new mtsTeleOperationECMOneHand(m_name, m_config->period);
            teleop->Configure(m_config->configure_parameter);
            component_manager->AddComponent(teleop);
        }
        break;
    case dvrk::teleop_ECM_type::TELEOP_ECM_TWO_HANDS_DERIVED:
        {
            mtsComponent * component;
            component = component_manager->GetComponent(m_name);
            if (component) {
                mtsTeleOperationECMTwoHands * teleop = dynamic_cast<mtsTeleOperationECMTwoHands *>(component);
                if (teleop) {
                    teleop->Configure(m_config->configure_parameter);
                } else {
                    CMN_LOG_INIT_ERROR << "teleop_ECM_proxy_t::create_teleop: component \""
                                       << m_name << "\" doesn't seem to be derived from mtsTeleOperationECMTwoHands."
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
    case dvrk::teleop_ECM_type::TELEOP_ECM_ONE_HAND_DERIVED:
        {
            mtsComponent * component;
            component = component_manager->GetComponent(m_name);
            if (component) {
                mtsTeleOperationECMOneHand * teleop = dynamic_cast<mtsTeleOperationECMOneHand *>(component);
                if (teleop) {
                    teleop->Configure(m_config->configure_parameter);
                } else {
                    CMN_LOG_INIT_ERROR << "teleop_ECM_proxy_t::create_teleop: component \""
                                       << m_name << "\" doesn't seem to be derived from mtsTeleOperationECMOneHand."
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
    case dvrk::teleop_ECM_type::TELEOP_ECM_TWO_HANDS_GENERIC:
    case dvrk::teleop_ECM_type::TELEOP_ECM_ONE_HAND_GENERIC:
        break;
    default:
        break;
    }

    // schedule connections
    if (m_config->one_hand()) {
        m_system->m_connections.Add(m_name, "MTM",
                                    m_MTM_component_name, m_MTM_interface_name);
    } else {
        m_system->m_connections.Add(m_name, "MTML",
                                    m_MTML_component_name, m_MTML_interface_name);
        m_system->m_connections.Add(m_name, "MTMR",
                                    m_MTMR_component_name, m_MTMR_interface_name);
    }
    m_system->m_connections.Add(m_name, "ECM",
                                m_ECM_component_name, m_ECM_interface_name);
    m_system->m_connections.Add(m_name, "clutch",
                                 m_system->GetName(), m_console->m_name + "/clutch"); // clutch from console
    m_system->m_connections.Add(m_system->GetName(), m_name,
                                 m_name, "Setting");
}
