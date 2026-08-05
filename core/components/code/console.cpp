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

#include <sawIntuitiveResearchKit/console.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsManagerLocal.h>

#include <sawIntuitiveResearchKit/console_configuration.h>
#include <sawIntuitiveResearchKit/system.h>
#include <sawIntuitiveResearchKit/IO_proxy.h>
#include <sawIntuitiveResearchKit/teleop_ECM_proxy.h>
#include <sawIntuitiveResearchKit/teleop_PSM_proxy.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitConfig.h>
#include <sawIntuitiveResearchKit/mtsDaVinciHeadSensor.h>
#if sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR
#include <sawIntuitiveResearchKit/mtsHIDHeadSensor.h>
#endif

dvrk::console::console(const std::string & name,
                           dvrk::system * system,
                           dvrk::console_configuration * config):
    m_name(name),
    m_system(system),
    m_config(config)
{
}


void dvrk::console::post_configure(void)
{
    // ECM teleops
    for (auto & proxy_config : m_config->teleop_ECMs) {
        const std::string name = proxy_config.MTML + "_" + proxy_config.MTMR + "_" + proxy_config.ECM;
        auto iter = m_teleop_proxies.find(name);
        if (iter == m_teleop_proxies.end()) {
            // create a new teleop_ecm proxy if needed
            auto teleop_proxy = std::make_shared<teleop_ECM_proxy>(name, this->m_system,
                                                                   this, &proxy_config);
            teleop_proxy->post_configure();
            if (m_system->find_conflicting_teleops(teleop_proxy)) {
                teleop_proxy->m_selected = false;
            } else {
                teleop_proxy->m_selected = true;
            }
            m_teleop_proxies[name] = teleop_proxy;
        } else {
            CMN_LOG_INIT_ERROR << "post_configure: failed to configure teleop_ECMs, "
                               << name << " already exists" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // PSM teleops
    std::map <std::string, std::vector<std::shared_ptr<teleop_PSM_proxy>>> teleops_per_MTM;
    for (auto & proxy_config : m_config->teleop_PSMs) {
        const std::string name = proxy_config.MTM + "_" + proxy_config.PSM;
        auto iter = m_teleop_proxies.find(name);
        if (iter == m_teleop_proxies.end()) {
            // create a new teleop_ecm proxy if needed
            auto teleop_proxy = std::make_shared<teleop_PSM_proxy>(name, this->m_system,
                                                                   this, &proxy_config);
            teleop_proxy->post_configure();
            // make sure no conflicting teleops are selected
            if (m_system->find_conflicting_teleops(teleop_proxy)) {
                teleop_proxy->m_selected = false;
            } else {
                teleop_proxy->m_selected = true;
            }
            // add to list of teleops using each MTM
            teleops_per_MTM[proxy_config.MTM].push_back(teleop_proxy);
            m_teleop_proxies[name] = teleop_proxy;
        } else {
            CMN_LOG_INIT_ERROR << "post_configure: failed to configure teleop_PSMs, "
                               << name << " already exists" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // find with PSM teleops can be cycled using quick-tap
    bool found_cycle = false;
    for (auto & iter : teleops_per_MTM) {
        if (iter.second.size() == 2) {
            if (!found_cycle) {
                found_cycle = true;
                m_teleop_PSM_cycle_1 = iter.second.at(0);
                m_teleop_PSM_cycle_2 = iter.second.at(1);
            }
        }
    }

    // clutch, camera pedals
    if ((m_config->input_type == console_input_type::PEDALS_ONLY)
        || (m_config->input_type == console_input_type::PEDALS_ISI_HEAD_SENSOR)
        || (m_config->input_type == console_input_type::PEDALS_DVRK_HEAD_SENSOR)
        || (m_config->input_type == console_input_type::PEDALS_GOOVIS_HEAD_SENSOR)) {
        if (m_config->IO_pedals.IO == "") {
            CMN_LOG_INIT_ERROR << "console::post_configure: \"IO_pedals\" must be provided for inputs PEDALS_{}: "
                               << m_name << std::endl;
            exit(EXIT_FAILURE);
        }
        m_config->clutch.component = m_config->IO_pedals.IO;
        m_config->clutch.interface = "clutch";
        m_config->camera.component = m_config->IO_pedals.IO;
        m_config->camera.interface = "camera";
        // if only pedals, use coag for operator present
        if (m_config->input_type == console_input_type::PEDALS_ONLY) {
            m_config->operator_present.component = m_config->IO_pedals.IO;
            m_config->operator_present.interface = "coag";
        }
    } else {
        m_config->IO_pedals.IO = "";
    }

    if (m_config->input_type == console_input_type::PEDALS_ISI_HEAD_SENSOR) {
        if (m_config->IO_head_sensor.IO == "") {
            CMN_LOG_INIT_ERROR << "console::post_configure: \"IO_head_sensor\" must be provided for inputs PEDALS_ISI_HEAD_SENSOR: "
                               << m_name << std::endl;
            exit(EXIT_FAILURE);
        }
        m_config->operator_present.component = m_name + "_daVinci_head_sensor";
        m_config->operator_present.interface = "operator_present";
    }

    if (m_config->input_type == console_input_type::PEDALS_GOOVIS_HEAD_SENSOR) {
#if sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR
        if (m_config->HID_file == "") {
            CMN_LOG_INIT_ERROR << "console::post_configure: \"HID_file\" must be provided for inputs PEDALS_GOOVIS_HEAD_SENSOR: "
                               << m_name << std::endl;
            exit(EXIT_FAILURE);
        }
        m_config->operator_present.component = m_name + "_goovis_head_sensor";
        m_config->operator_present.interface = "operator_present";
#else
        CMN_LOG_INIT_ERROR << "console::post_configure: can't use HID head sensor." << std::endl
                           << "The code has been compiled with sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR OFF." << std::endl
                           << "Re-run CMake, re-compile and try again." << std::endl;
        exit(EXIT_FAILURE);
#endif
    } else {
        m_config->HID_file = "";
    }

}


void dvrk::console::create_components(void)
{
    mtsComponentManager * manager = mtsComponentManager::GetInstance();

    // teleops
    for (auto & proxy : m_teleop_proxies) {
        m_system->add_teleop_interfaces(proxy.second);
        proxy.second->create_teleop();
    }

    // inputs
    if (m_config->IO_pedals.IO != "") {
        m_system->configure_IO(m_config->IO_pedals);
    }

    if (m_config->input_type == console_input_type::PEDALS_ISI_HEAD_SENSOR) {
        m_system->configure_IO(m_config->IO_head_sensor);
        m_head_sensor = new mtsDaVinciHeadSensor(m_config->operator_present.component);
        manager->AddComponent(m_head_sensor);
        // schedule connections between IO and head sensor component
        const std::string IO =  m_config->IO_head_sensor.IO;
        m_system->m_connections.Add(m_config->operator_present.component, "head_sensor_turn_off",
                                    IO, "head_sensor_turn_off");
        m_system->m_connections.Add(m_config->operator_present.component, "dv_head_sensor_1",
                                    IO, "dv_head_sensor_1");
        m_system->m_connections.Add(m_config->operator_present.component, "dv_head_sensor_2",
                                    IO, "dv_head_sensor_2");
        m_system->m_connections.Add(m_config->operator_present.component, "dv_head_sensor_3",
                                    IO, "dv_head_sensor_3");
        m_system->m_connections.Add(m_config->operator_present.component, "dv_head_sensor_4",
                                    IO, "dv_head_sensor_4");
    }

    if (m_config->input_type == console_input_type::PEDALS_DVRK_HEAD_SENSOR) {
        m_system->configure_IO(m_config->IO_head_sensor);
        m_config->operator_present.component = m_config->IO_head_sensor.IO;
        m_config->operator_present.interface = "operator_present";
    }

    if (m_config->input_type == console_input_type::PEDALS_GOOVIS_HEAD_SENSOR) {
        CMN_LOG_INIT_VERBOSE << "console::create_component: configuring hid head sensor with \""
                             << m_config->HID_file << "\"" << std::endl;
        const std::string config_file = m_system->find_file(m_config->HID_file);
        if (config_file == "") {
            CMN_LOG_INIT_ERROR << "console::create_component: can't find configuration file "
                               << m_config->HID_file << std::endl;
            exit(EXIT_FAILURE);
        }
#if sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR
        m_head_sensor = new mtsHIDHeadSensor(m_config->operator_present.component);
        m_head_sensor->Configure(config_file);
        manager->AddComponent(m_head_sensor);
#endif
    }
}


void dvrk::console::Startup(void)
{
    // emit all events to initialize GUI and ROs
    emit_teleop_state_events();
    events.scale(mtsIntuitiveResearchKit::TeleOperationPSM::Scale);
}


void dvrk::console::emit_teleop_state_events(void)
{
    for (auto & iter : m_teleop_proxies) {
        if (iter.second->m_selected) {
            events.teleop_selected(iter.first);
        } else {
            events.teleop_unselected(iter.first);
        }
    }
}


void dvrk::console::teleop_enable(const bool & _enable)
{
    // m_teleop_enabled = _enable;
    // //     // if we have an SUJ, make sure it's ready
    // //     if (enable && m_SUJ) {
    // //         const auto sujState = ArmStates.find("SUJ");
    // //         if ((sujState == ArmStates.end())
    // //             || (sujState->second.State() != prmOperatingState::ENABLED)) {
    // //             m_teleop_enabled = false;
    // //         }
    // //     }
    m_teleop_wanted = _enable; // for recovery after errors
    // event
    events.teleop_enabled(m_teleop_wanted);
    m_system->m_interface->SendStatus(m_name + ": tele operation "
                                      + (m_teleop_enabled ? "enabled" : "disabled"));
    // event
    update_teleop_state();
}


void dvrk::console::set_scale(const double & _scale)
{
    // PSM teleops
    for (auto & proxy : m_teleop_proxies) {
        proxy.second->set_scale(_scale);
    }
    events.scale(_scale);
}


void dvrk::console::select_teleop(const std::string & _teleop)
{
    auto iter = m_teleop_proxies.find(_teleop);
    if (iter != m_teleop_proxies.end()) {
        m_system->find_conflicting_teleops(iter->second, true /* to unselect */);
        iter->second->m_selected = true;
        events.teleop_selected(_teleop);
        update_teleop_state();
        return;
    }
    m_interface_provided->SendStatus(m_name + ": select_teleop can't find " + _teleop);
}


void dvrk::console::unselect_teleop(const std::string & _teleop)
{
    auto iter = m_teleop_proxies.find(_teleop);
    if (iter != m_teleop_proxies.end()) {
        iter->second->m_selected = false;
        events.teleop_unselected(_teleop);
        update_teleop_state();
        return;
    }
    m_interface_provided->SendStatus(m_name + ": unselect_teleop can't find " + _teleop);
}


bool dvrk::console::find_conflicting_teleops(std::shared_ptr<teleop_proxy> _teleop, const bool _unselect)
{
    bool result = false;
    for (auto & teleop : m_teleop_proxies) {
        if ((teleop.first != _teleop->m_name)
            && (teleop.second->type() == _teleop->type())
            && (teleop.second->m_selected)) {
            std::list<std::string> conflicting_arms;
            std::set_intersection(_teleop->m_arms_used.begin(), _teleop->m_arms_used.end(),
                                  teleop.second->m_arms_used.begin(), teleop.second->m_arms_used.end(),
                                  std::back_inserter(conflicting_arms));
            for (auto & arm : conflicting_arms) {
                result |= true;
                if (_unselect) {
                    unselect_teleop(teleop.first);
                    m_interface_provided->SendStatus(m_name + ": " + teleop.first + " unselected to free " + arm);
                }
            }
        }
    }
    return result;
}


void dvrk::console::clutch_event_handler(const prmEventButton & _button)
{
    switch (_button.Type()) {
    case prmEventButton::PRESSED:
        m_interface_provided->SendStatus(m_name + ": clutch pressed");
        m_system->audio.beep(vct3(0.1, 700.0, m_system->m_audio_volume));
        break;
    case prmEventButton::RELEASED:
        m_interface_provided->SendStatus(m_name + ": clutch released");
        m_system->audio.beep(vct3(0.1, 700.0, m_system->m_audio_volume));
        break;
    case prmEventButton::CLICKED:
        m_interface_provided->SendStatus(m_name + ": clutch quick tap");
        m_system->audio.beep(vct3(0.05, 2000.0, m_system->m_audio_volume));
        m_system->audio.beep(vct3(0.05, 2000.0, m_system->m_audio_volume));
        if (m_teleop_PSM_cycle_1 != nullptr) {
            if (m_teleop_PSM_cycle_1->m_selected) {
                select_teleop(m_teleop_PSM_cycle_2->m_name);
            } else {
                select_teleop(m_teleop_PSM_cycle_1->m_name);
            }
        }
        break;
    default:
        break;
    }
    events.clutch(_button);
}


void dvrk::console::camera_event_handler(const prmEventButton & _button)
{
    switch (_button.Type()) {
    case prmEventButton::PRESSED:
        m_camera = true;
        m_interface_provided->SendStatus(m_name + ": camera pressed");
        m_system->audio.beep(vct3(0.1, 1000.0, m_system->m_audio_volume));
        break;
    case prmEventButton::RELEASED:
        m_camera = false;
        m_interface_provided->SendStatus(m_name + ": camera released");
        m_system->audio.beep(vct3(0.1, 1000.0, m_system->m_audio_volume));
        break;
    case prmEventButton::CLICKED:
        m_interface_provided->SendStatus(m_name + ": camera quick tap");
        m_system->audio.beep(vct3(0.05, 2500.0, m_system->m_audio_volume));
        m_system->audio.beep(vct3(0.05, 2500.0, m_system->m_audio_volume));
        break;
    default:
        break;
    }
    update_teleop_state();
    events.camera(_button);
}


void dvrk::console::operator_present_event_handler(const prmEventButton & _button)
{
    switch (_button.Type()) {
    case prmEventButton::PRESSED:
        m_operator_present = true;
        m_interface_provided->SendStatus(m_name + ": operator present");
        m_system->audio.beep(vct3(0.3, 1500.0, m_system->m_audio_volume));
        break;
    case prmEventButton::RELEASED:
        m_operator_present = false;
        m_interface_provided->SendStatus(m_name + ": operator not present");
        m_system->audio.beep(vct3(0.3, 1200.0, m_system->m_audio_volume));
        break;
    default:
        break;
    }
    update_teleop_state();
    events.operator_present(_button);
}


void dvrk::console::update_teleop_state(void)
{
    // overall teleop
    if (m_teleop_wanted != m_teleop_enabled) {
        m_teleop_enabled = m_teleop_wanted;
    }

    // per teleop pair
    for (auto & iter : m_teleop_proxies) {
        auto & teleop_proxy = iter.second;
        if (m_teleop_enabled
            && teleop_proxy->m_selected
            && (((teleop_proxy->type() == teleop_proxy::ECM)
                 && m_camera)
                || ((teleop_proxy->type() == teleop_proxy::PSM)
                    && !m_camera)
                )
            ) {
            if (m_operator_present
                || (teleop_proxy->type() == dvrk::teleop_proxy::ECM)
                ) {
                teleop_proxy->m_enabled = true;
                teleop_proxy->state_command(std::string("enable"));
            } else {
                teleop_proxy->state_command(std::string("align_MTM"));
            }
        } else if (!m_teleop_enabled
                   || !m_operator_present
                   || teleop_proxy->m_enabled
                   ) {
            teleop_proxy->m_enabled = false;
            teleop_proxy->state_command(std::string("disable"));
        }
    }
}
