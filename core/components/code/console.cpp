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

#include <sawIntuitiveResearchKit/console_configuration.h>
#include <sawIntuitiveResearchKit/system.h>
#include <sawIntuitiveResearchKit/teleop_ECM_proxy.h>
#include <sawIntuitiveResearchKit/teleop_PSM_proxy.h>

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
            m_teleop_proxies[name] = teleop_proxy;
        } else {
            CMN_LOG_INIT_ERROR << "post: failed to configure teleop_ECMs, "
                               << name << " already exists" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // PSM teleops
    for (auto & proxy_config : m_config->teleop_PSMs) {
        const std::string name = proxy_config.MTM + "_" + proxy_config.PSM;
        auto iter = m_teleop_proxies.find(name);
        if (iter == m_teleop_proxies.end()) {
            // create a new teleop_ecm proxy if needed
            auto teleop_proxy = std::make_shared<teleop_PSM_proxy>(name, this->m_system,
                                                                   this, &proxy_config);
            teleop_proxy->post_configure();
            m_teleop_proxies[name] = teleop_proxy;
        } else {
            CMN_LOG_INIT_ERROR << "post: failed to configure teleop_PSMs, "
                               << name << " already exists" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // update map of arms used by teleop
    for (auto & teleop : m_teleop_proxies) {
        for (auto & arm : teleop.second->m_arms_used) {
            m_system->m_teleops_using_arm[arm].push_back(teleop.first);
        }
    }
    
    // operator present, clutch, camera pedals
    switch (m_config->input_type) {
    case console_input_type::PEDALS_ONLY:
        {
            if (m_config->IO == "") {
                CMN_LOG_INIT_ERROR << "console::post_configure: \"IO\" must be provided for native or derived arm: "
                                   << m_name << std::endl;
                exit(EXIT_FAILURE);
            }
            // registered in component manager?
            auto iter = m_system->m_IO_proxies.find(m_config->IO);
            if (iter == m_system->m_IO_proxies.end()) {
                CMN_LOG_INIT_ERROR << "console::post_configure: IO \""
                                   << m_config->IO << "\" is not defined in \"IOs\"" << std::endl;
                exit(EXIT_FAILURE);
            }
            m_clutch_component_name = m_config->IO;
            m_clutch_interface_name = "clutch";
            m_camera_component_name = m_config->IO;
            m_camera_interface_name = "camera";
            m_operator_present_component_name = m_config->IO;
            m_operator_present_interface_name = "coag";
            break;
        }
    default:    
        break;
    }
}


void dvrk::console::create_components(void)
{
    // teleops
    for (auto & proxy : m_teleop_proxies) {
        proxy.second->create_teleop();
        m_system->add_teleop_interfaces(proxy.second);
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

// dvrk::console
//     // mTeleopMTMToCycle(""),
//     // mOperatorPresent(false),
//     // mCameraPressed(false)

//     if (mInterface) {
//         // // manage tele-op
//         // mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::cycle_teleop_PSM_by_MTM, this,
//         //                             "cycle_teleop_PSM_by_MTM", std::string(""));
//         // mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::select_teleop_PSM, this,
//         //                             "select_teleop_PSM", prmKeyValue("MTM", "PSM"));
//         // mInterface->AddEventWrite(ConfigurationEvents.teleop_PSM_selected,
//         //                           "teleop_PSM_selected", prmKeyValue("MTM", "PSM"));
//         // mInterface->AddEventWrite(ConfigurationEvents.teleop_PSM_unselected,
//         //                           "teleop_PSM_unselected", prmKeyValue("MTM", "PSM"));

//         // misc.
//         mInterface->AddCommandRead(&mtsIntuitiveResearchKitConsole::calibration_mode, this,
//                                    "calibration_mode", false);
//         // Following is Read instead of VoidReturn because it is called before the component
//         // is created (i.e., thread not yet running)
//         mInterface->AddCommandRead(&mtsIntuitiveResearchKitConsole::ConnectInternal, this,
//                                    "connect", false);
//     }
// }


//     //     const Json::Value operatorPresent = jsonConfig["operator-present"];
//     //     if (!operatorPresent.empty()) {
//     //         // first case, using io to communicate with daVinci original head sensore
//     //         Json::Value operatorPresentConfiguration = operatorPresent["io"];
//     //         if (!operatorPresentConfiguration.empty()) {
//     //             const std::string headSensorName = "daVinciHeadSensor";
//     //             mHeadSensor = new mtsDaVinciHeadSensor(headSensorName);
//     //             mtsComponentManager::GetInstance()->AddComponent(mHeadSensor);
//     //             // main DInput is OperatorPresent comming from the newly added component
//     //             mDInputSources["OperatorPresent"] = InterfaceComponentType(headSensorName, "OperatorPresent");
//     //             // also expose the digital inputs from RobotIO (e.g. ROS topics)
//     //             mDInputSources["HeadSensor1"] = InterfaceComponentType(m_IO_component_name, "HeadSensor1");
//     //             mDInputSources["HeadSensor2"] = InterfaceComponentType(m_IO_component_name, "HeadSensor2");
//     //             mDInputSources["HeadSensor3"] = InterfaceComponentType(m_IO_component_name, "HeadSensor3");
//     //             mDInputSources["HeadSensor4"] = InterfaceComponentType(m_IO_component_name, "HeadSensor4");
//     //             // schedule connections
//     //             m_connections.Add(headSensorName, "HeadSensorTurnOff",
//     //                              m_IO_component_name, "HeadSensorTurnOff");
//     //             m_connections.Add(headSensorName, "HeadSensor1",
//     //                              m_IO_component_name, "HeadSensor1");
//     //             m_connections.Add(headSensorName, "HeadSensor2",
//     //                              m_IO_component_name, "HeadSensor2");
//     //             m_connections.Add(headSensorName, "HeadSensor3",
//     //                              m_IO_component_name, "HeadSensor3");
//     //             m_connections.Add(headSensorName, "HeadSensor4",
//     //                              m_IO_component_name, "HeadSensor4");
//     //         } else {
//     //             // second case, using hid config for goovis head sensor
//     //             operatorPresentConfiguration = operatorPresent["hid"];
//     //             if (!operatorPresentConfiguration.empty()) {
//     // #if sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR
//     //                 std::string relativeConfigFile = operatorPresentConfiguration.asString();
//     //                 CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring hid head sensor with \""
//     //                                            << relativeConfigFile << "\"" << std::endl;
//     //                 const std::string configFile = find_file(relativeConfigFile);
//     //                 if (configFile == "") {
//     //                     CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find configuration file "
//     //                                              << relativeConfigFile << std::endl;
//     //                     exit(EXIT_FAILURE);
//     //                 }
//     //                 const std::string headSensorName = "HIDHeadSensor";
//     //                 mHeadSensor = new mtsHIDHeadSensor(headSensorName);
//     //                 mHeadSensor->Configure(configFile);
//     //                 mtsComponentManager::GetInstance()->AddComponent(mHeadSensor);
//     //                 // main DInput is OperatorPresent comming from the newly added component
//     //                 mDInputSources["OperatorPresent"] = InterfaceComponentType(headSensorName, "OperatorPresent");
//     // #else
//     //                 CMN_LOG_CLASS_INIT_ERROR << "Configure: can't use HID head sensor." << std::endl
//     //                                          << "The code has been compiled with sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR OFF." << std::endl
//     //                                          << "Re-run CMake, re-compile and try again." << std::endl;
//     //                 exit(EXIT_FAILURE);
//     // #endif
//     //             }
//     //         }
//     //     }

//     // load endoscope-focus settings
//     // const Json::Value endoscopeFocus = jsonConfig["endoscope-focus"];
//     // if (!endoscopeFocus.empty()) {
//     //     const std::string endoscopeFocusName = "daVinciEndoscopeFocus";
//     //     mDaVinciEndoscopeFocus = new mtsDaVinciEndoscopeFocus(endoscopeFocusName);
//     //     mtsComponentManager::GetInstance()->AddComponent(mDaVinciEndoscopeFocus);
//     //     // make sure we have cam+ and cam- in digital inputs
//     //     const DInputSourceType::const_iterator endDInputs = mDInputSources.end();
//     //     const bool foundCamMinus = (mDInputSources.find("Cam-") != endDInputs);
//     //     if (!foundCamMinus) {
//     //         CMN_LOG_CLASS_INIT_ERROR << "Configure: input for footpedal \"Cam-\" is required for \"endoscope-focus\".  "
//     //                                  << footpedalMessage << std::endl;
//     //         exit(EXIT_FAILURE);
//     //     }
//     //     const bool foundCamPlus = (mDInputSources.find("Cam+") != endDInputs);
//     //     if (!foundCamPlus) {
//     //         CMN_LOG_CLASS_INIT_ERROR << "Configure: input for footpedal \"Cam+\" is required for \"endoscope-focus\".  "
//     //                                  << footpedalMessage << std::endl;
//     //         exit(EXIT_FAILURE);
//     //     }
//     //     // schedule connections
//     //     // m_connections.Add(endoscopeFocusName, "EndoscopeFocusIn",
//     //     //                  m_IO_component_name, "EndoscopeFocusIn");
//     //     // m_connections.Add(endoscopeFocusName, "EndoscopeFocusOut",
//     //     //                  m_IO_component_name, "EndoscopeFocusOut");
//     //     // m_connections.Add(endoscopeFocusName, "focus_in",
//     //     //                  m_IO_component_name, "Cam+");
//     //     // m_connections.Add(endoscopeFocusName, "focus_out",
//     //     //                  m_IO_component_name, "Cam-");
//     // }

// }

// const bool & mtsIntuitiveResearchKitConsole::Configured(void) const
// {
//     return m_configured;
// }


// //     mtsInterfaceProvided * operatorProvided = AddInterfaceProvided("OperatorPresent");
// //     if (operatorProvided) {
// //         operatorProvided->AddEventWrite(console_events.operator_present, "Button", prmEventButton());
// //     }
// // }


// // bool mtsIntuitiveResearchKitConsole::ConfigureECMTeleopJSON(const Json::Value & jsonTeleop)
// // {
// // if (mtmLeftName == mtmRightName) {
// //     CMN_LOG_CLASS_INIT_ERROR << "ConfigureECMTeleopJSON: \"mtm-left\" and \"mtm-right\" must be different" << std::endl;
// //     return false;
// // }
// // }
// //     // default value
// //     mTeleopECM->m_type = TeleopECM::TELEOP_ECM;
// // }
// //     return true;
// // }

// // bool mtsIntuitiveResearchKitConsole::ConfigurePSMTeleopJSON(const Json::Value & jsonTeleop)
// // {

// // Json::Value jsonValue = jsonTeleop["PSM_base_frame"];
// // std::string baseFrameComponent, baseFrameInterface;
// // if (!jsonValue.empty()) {
// //     baseFrameComponent = jsonValue.get("component", "").asString();
// //     baseFrameInterface = jsonValue.get("interface", "").asString();
// //     if ((baseFrameComponent == "") || (baseFrameInterface == "")) {
// //         CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: both \"component\" and \"interface\" must be provided with \"PSM_base_frame\" for teleop \""
// //                                  << mtmName << "_" << psmName << "\"" << std::endl;
// //         return false;
// //     }
// // }

// // // check if pair already exist and then add
// // const auto teleopIterator = m_teleop_PSM_proxies.find(name);
// // TeleopPSM * teleopPointer = 0;
// // if (teleopIterator == m_teleop_PSM_proxies.end()) {

// //     // insert
// //     m_teleop_PSM_proxiesByMTM.insert(std::make_pair(mtmName, teleopPointer));
// //     m_teleop_PSM_proxiesByPSM.insert(std::make_pair(psmName, teleopPointer));

// //     // first MTM with multiple PSMs is selected for single tap
// //     if ((m_teleop_PSM_proxiesByMTM.count(mtmName) > 1)
// //         && (mTeleopMTMToCycle == "")) {
// //         mTeleopMTMToCycle = mtmName;
// //     }
// //     // check if we already have a teleop for the same PSM
// //     std::string mtmUsingThatPSM;
// //     GetMTMSelectedForPSM(psmName, mtmUsingThatPSM);
// //     if (mtmUsingThatPSM != "") {
// //         teleopPointer->SetSelected(false);
// //         CMN_LOG_CLASS_INIT_WARNING << "ConfigurePSMTeleopJSON: psm \""
// //                                    << psmName << "\" is already selected to be controlled by mtm \""
// //                                    << mtmUsingThatPSM << "\", component \""
// //                                    << name << "\" is added but not selected"
// //                                    << std::endl;
// //     } else {
// //         // check if we already have a teleop for the same PSM
// //         std::string psmUsingThatMTM;
// //         GetPSMSelectedForMTM(mtmName, psmUsingThatMTM);
// //         if (psmUsingThatMTM != "") {
// //             teleopPointer->SetSelected(false);
// //             CMN_LOG_CLASS_INIT_WARNING << "ConfigurePSMTeleopJSON: mtm \""
// //                                        << mtmName << "\" is already selected to control psm \""
// //                                        << psmUsingThatMTM << "\", component \""
// //                                        << name << "\" is added but not selected"
// //                                        << std::endl;
// //         } else {
// //             // neither the MTM nor PSM are used, let's activate that pair
// //             teleopPointer->SetSelected(true);
// //         }
// //     }
// //     // finally add the new teleop
// //     m_teleop_PSM_proxies[name] = teleopPointer;
// // } else {
// //     CMN_LOG_CLASS_INIT_ERROR << "ConfigurePSMTeleopJSON: there is already a teleop for the pair \""
// //                              << name << "\"" << std::endl;
// //     return false;
// // }

// // const Json::Value jsonTeleopConfig = jsonTeleop["configure-parameter"];
// // teleopPointer->ConfigureTeleop(teleopPointer->m_type, period, jsonTeleopConfig);
// //     return true;
// // }

// bool mtsIntuitiveResearchKitConsole::Connect(void)
// {
//     mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();

//     return true;
// }


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
        iter->second->m_wanted = true;
        return;
    }
    m_interface_provided->SendStatus("console " + m_name + "::select_teleop can't find " + _teleop);
    update_teleop_state();
}


void dvrk::console::unselect_teleop(const std::string & _teleop)
{
    auto iter = m_teleop_proxies.find(_teleop);
    if (iter != m_teleop_proxies.end()) {
        iter->second->m_wanted = false;
        return;
    }
    m_interface_provided->SendStatus("console " + m_name + "::unselect_teleop can't find " + _teleop);
    update_teleop_state();
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
        // if (mTeleopMTMToCycle != "") {
        //     cycle_teleop_PSM_by_MTM(mTeleopMTMToCycle);
        // }
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


// // void mtsIntuitiveResearchKitConsole::cycle_teleop_PSM_by_MTM(const std::string & mtmName)
// // {
// //     // try to cycle through all the teleopPSMs associated to the MTM
// //     if (m_teleop_PSM_proxies_by_MTM.count(mtmName) == 0) {
// //         // we use empty string to query, no need to send warning about bad mtm name
// //         if (mtmName != "") {
// //             mInterface->SendWarning(this->GetName()
// //                                     + ": no PSM teleoperation found for MTM \""
// //                                     + mtmName
// //                                     + "\"");
// //         }
// //     } else if (m_teleop_PSM_proxies_by_MTM.count(mtmName) == 1) {
// //         mInterface->SendStatus(this->GetName()
// //                                + ": only one PSM teleoperation found for MTM \""
// //                                + mtmName
// //                                + "\", cycling has no effect");
// //     } else {
// //         // find range of teleops
// //         auto range = m_teleop_PSM_proxies_by_MTM.equal_range(mtmName);
// //         for (auto iter = range.first;
// //              iter != range.second;
// //              ++iter) {
// //             // find first teleop currently selected
// //             if (iter->second->selected()) {
// //                 // toggle to next one
// //                 auto nextTeleop = iter;
// //                 nextTeleop++;
// //                 // if next one is last one, go back to first
// //                 if (nextTeleop == range.second) {
// //                     nextTeleop = range.first;
// //                 }
// //                 // now make sure the PSM in next teleop is not used
// //                 std::string mtmUsingThatPSM;
// //                 GetMTMSelectedForPSM(nextTeleop->second->m_config->PSM, mtmUsingThatPSM);
// //                 if (mtmUsingThatPSM != "") {
// //                     // message
// //                     mInterface->SendWarning(this->GetName()
// //                                             + ": cycling from \""
// //                                             + iter->second->m_name
// //                                             + "\" to \""
// //                                             + nextTeleop->second->m_name
// //                                             + "\" failed, PSM is already controlled by \""
// //                                             + mtmUsingThatPSM
// //                                             + "\"");
// //                 } else {
// //                     // mark which one should be active
// //                     iter->second->set_selected(false);
// //                     nextTeleop->second->set_selected(true);
// //                     // if teleop PSM is active, enable/disable components now
// //                     if (m_teleop_enabled) {
// //                         iter->second->state_command(std::string("disable"));
// //                         if (mTeleopPSMRunning) {
// //                             nextTeleop->second->state_command(std::string("enable"));
// //                         } else {
// //                             nextTeleop->second->state_command(std::string("align_MTM"));
// //                         }
// //                     }
// //                     // message
// //                     mInterface->SendStatus(this->GetName()
// //                                            + ": cycling from \""
// //                                            + iter->second->m_name
// //                                            + "\" to \""
// //                                            + nextTeleop->second->m_name
// //                                            + "\"");
// //                 }
// //                 // stop for loop
// //                 break;
// //             }
// //         }
// //     }
// //     // in all cases, emit events so users can figure out which components are selected
// //     EventSelectedTeleopPSMs();
// // }

// // void mtsIntuitiveResearchKitConsole::select_teleop_PSM(const prmKeyValue & mtmPsm)
// // {
// //     // for readability
// //     const std::string mtmName = mtmPsm.Key;
// //     const std::string psmName = mtmPsm.Value;

// //     // if the psm value is empty, disable any teleop for the mtm -- this can be used to free the mtm
// //     if (psmName == "") {
// //         auto range = m_teleop_PSM_proxies_by_MTM.equal_range(mtmName);
// //         for (auto iter = range.first;
// //              iter != range.second;
// //              ++iter) {
// //             // look for the teleop that was selected if any
// //             if (iter->second->selected()) {
// //                 iter->second->set_selected(false);
// //                 // if teleop PSM is active, enable/disable components now
// //                 if (m_teleop_enabled) {
// //                     iter->second->state_command(std::string("disable"));
// //                 }
// //                 // message
// //                 mInterface->SendWarning(this->GetName()
// //                                         + ": teleop \""
// //                                         + iter->second->m_name
// //                                         + "\" has been unselected ");
// //             }
// //         }
// //         EventSelectedTeleopPSMs();
// //         return;
// //     }

// //     // actual teleop to select
// //     std::string name = mtmName + "_" + psmName;
// //     const auto teleopIterator = m_teleop_PSM_proxies.find(name);
// //     if (teleopIterator == m_teleop_PSM_proxies.end()) {
// //         mInterface->SendWarning(this->GetName()
// //                                 + ": unable to select \""
// //                                 + name
// //                                 + "\", this component doesn't exist");
// //         EventSelectedTeleopPSMs();
// //         return;
// //     }
// //     // there seems to be some redundant information here, let's use it for a safety check
// //     CMN_ASSERT(mtmName == teleopIterator->second->m_config->MTM);
// //     CMN_ASSERT(psmName == teleopIterator->second->m_config->PSM);
// //     // check that the PSM is available to be used
// //     std::string mtmUsingThatPSM;
// //     GetMTMSelectedForPSM(psmName, mtmUsingThatPSM);
// //     if (mtmUsingThatPSM != "") {
// //         mInterface->SendWarning(this->GetName()
// //                                 + ": unable to select \""
// //                                 + name
// //                                 + "\", PSM is already controlled by \""
// //                                 + mtmUsingThatPSM
// //                                 + "\"");
// //         EventSelectedTeleopPSMs();
// //         return;
// //     }

// //     // make sure the teleop using that MTM is unselected
// //     select_teleop_PSM(prmKeyValue(mtmName, ""));

// //     // now turn on the teleop
// //     teleopIterator->second->set_selected(true);
// //     // if teleop PSM is active, enable/disable components now
// //     if (m_teleop_enabled) {
// //         if (mTeleopPSMRunning) {
// //             teleopIterator->second->state_command(std::string("enable"));
// //         } else {
// //             teleopIterator->second->state_command(std::string("align_MTM"));
// //         }
// //     }
// //     // message
// //     mInterface->SendStatus(this->GetName()
// //                            + ": \""
// //                            + teleopIterator->second->m_name
// //                            + "\" has been selected");

// //     // always send a message to let user know the current status
// //     EventSelectedTeleopPSMs();
// // }

// // bool mtsIntuitiveResearchKitConsole::GetPSMSelectedForMTM(const std::string & mtmName, std::string & psmName) const
// // {
// //     bool mtmFound = false;
// //     psmName = "";
// //     // find range of teleops
// //     auto range = m_teleop_PSM_proxies_by_MTM.equal_range(mtmName);
// //     for (auto iter = range.first;
// //          iter != range.second;
// //          ++iter) {
// //         mtmFound = true;
// //         if (iter->second->selected()) {
// //             psmName = iter->second->m_config->PSM;
// //         }
// //     }
// //     return mtmFound;
// // }

// // bool mtsIntuitiveResearchKitConsole::GetMTMSelectedForPSM(const std::string & psmName, std::string & mtmName) const
// // {
// //     bool psmFound = false;
// //     mtmName = "";
// //     for (auto & iter : m_teleop_PSM_proxies) {
// //         if (iter.second->m_config->PSM == psmName) {
// //             psmFound = true;
// //             if (iter.second->selected()) {
// //                 mtmName = iter.second->m_config->MTM;
// //             }
// //         }
// //     }
// //     return psmFound;
// // }


void dvrk::console::update_teleop_state(void)
{
    if (m_teleop_wanted != m_teleop_enabled) {
        m_teleop_enabled = m_teleop_wanted;
        // event
        events.teleop_enabled(m_teleop_enabled);
        m_system->m_interface->SendStatus(m_name + ": tele operation "
                                          + (m_teleop_enabled ? "enabled" : "disabled"));
    }
    for (auto & iter : m_teleop_proxies) {
        auto & teleop_name = iter.first;
        auto & teleop_proxy = iter.second;

        if (teleop_proxy->m_wanted != teleop_proxy->m_selected) {
            const std::string command = teleop_proxy->m_wanted ? "enable" : "disable";
            teleop_proxy->state_command(command);
            teleop_proxy->m_selected = teleop_proxy->m_wanted;
        }
    }
    emit_teleop_state_events();
}

// //     // Check if teleop is enabled
// //     if (!m_teleop_enabled) {
// //         bool holdNeeded = false;
// //         for (auto & iterTeleopPSM : m_teleop_PSM_proxies) {
// //             iterTeleopPSM.second->state_command(std::string("disable"));
// //             if (mTeleopPSMRunning) {
// //                 holdNeeded = true;
// //             }
// //             mTeleopPSMRunning = false;
// //         }

// //         // if (mTeleopECM) {
// //         //     mTeleopECM->state_command(std::string("disable"));
// //         //     if (mTeleopECMRunning) {
// //         //         holdNeeded = true;
// //         //     }
// //         //     mTeleopECMRunning = false;
// //         // }

// //         // hold arms if we stopped any teleop
// //         if (holdNeeded) {
// //             for (auto & arm_proxy : m_arm_proxies) {
// //                 if (arm_proxy.second->m_config->MTM()
// //                     && arm_proxy.second->hold.IsValid()) {
// //                     arm_proxy.second->hold();
// //                 }
// //             }
// //         }
// //         return;
// //     }

// //     // if none are running, hold
// //     if (!mTeleopECMRunning && !mTeleopPSMRunning) {
// //         for (auto & arm_proxy : m_arm_proxies) {
// //             if (arm_proxy.second->m_config->MTM()
// //                 && arm_proxy.second->hold.IsValid()) {
// //                 arm_proxy.second->hold();
// //             }
// //         }
// //     }

// //     // all fine
// //     bool readyForTeleop = mOperatorPresent;

// //     for (auto & arm_proxy : m_arm_proxies) {
// //         if (arm_proxy.second->m_SUJ_clutched) {
// //             readyForTeleop = false;
// //         }
// //     }

// //     // Check if operator is present
// //     if (!readyForTeleop) {
// //         // keep MTMs aligned
// //         for (auto & iterTeleopPSM : m_teleop_PSM_proxies) {
// //             if (iterTeleopPSM.second->selected()) {
// //                 iterTeleopPSM.second->state_command(std::string("align_MTM"));
// //             } else {
// //                 iterTeleopPSM.second->state_command(std::string("disable"));
// //             }
// //         }
// //         mTeleopPSMRunning = false;

// //         // // stop ECM if needed
// //         // if (mTeleopECMRunning) {
// //         //     mTeleopECM->state_command(std::string("disable"));
// //         //     mTeleopECMRunning = false;
// //         // }
// //         return;
// //     }

// //     // If camera is pressed for ECM Teleop or not
// //     if (mCameraPressed) {
// //         if (!mTeleopECMRunning) {
// //             // if PSM was running so we need to stop it
// //             if (mTeleopPSMRunning) {
// //                 for (auto & iterTeleopPSM : m_teleop_PSM_proxies) {
// //                     iterTeleopPSM.second->state_command(std::string("disable"));
// //                 }
// //                 mTeleopPSMRunning = false;
// //             }
// //             // ECM wasn't running, let's start it
// //             // if (mTeleopECM) {
// //             //     mTeleopECM->state_command(std::string("enable"));
// //             //     mTeleopECMRunning = true;
// //             // }
// //         }
// //     } else {
// //         // we must teleop PSM
// //         if (!mTeleopPSMRunning) {
// //             // if ECM was running so we need to stop it
// //             // if (mTeleopECMRunning) {
// //             //     mTeleopECM->state_command(std::string("disable"));
// //             //     mTeleopECMRunning = false;
// //             // }
// //             // PSM wasn't running, let's start it
// //             for (auto & iterTeleopPSM : m_teleop_PSM_proxies) {
// //                 if (iterTeleopPSM.second->selected()) {
// //                     iterTeleopPSM.second->state_command(std::string("enable"));
// //                 } else {
// //                     iterTeleopPSM.second->state_command(std::string("disable"));
// //                 }
// //                 mTeleopPSMRunning = true;
// //             }
// //         }
// //     }

// void mtsIntuitiveResearchKitConsole::ErrorEventHandler(const mtsMessage & message)
// {
//     // similar to teleop_enable(false) except we don't change mTeleopDesired
//     m_teleop_enabled = false;
//     console_events.teleop_enabled(m_teleop_enabled);
//     UpdateTeleopState();
// }
