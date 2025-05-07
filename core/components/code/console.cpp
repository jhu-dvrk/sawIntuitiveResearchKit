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

#include <sawIntuitiveResearchKit/mtsDaVinciHeadSensor.h>
#if sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR
#include <sawIntuitiveResearchKit/mtsHIDHeadSensor.h>
#endif



// dvrk::console
//     // mTeleopMTMToCycle(""),
//     // mOperatorPresent(false),
//     // mCameraPressed(false)
// {
//     // configure search path
//     m_config_path.Add(cmnPath::GetWorkingDirectory());
//     // add path to source/share directory to find common files.  This
//     // will work as long as this component is located in the same
//     // parent directory as the "shared" directory.
//     m_config_path.Add(std::string(sawIntuitiveResearchKit_SOURCE_CONFIG_DIR), cmnPath::TAIL);
//     // default installation directory
//     m_config_path.Add(mtsIntuitiveResearchKit::DefaultInstallationDirectory, cmnPath::TAIL);

//     mInterface = AddInterfaceProvided("Main");
//     if (mInterface) {
//         mInterface->AddMessageEvents();
//         mInterface->AddCommandVoid(&mtsIntuitiveResearchKitConsole::power_off, this,
//                                    "power_off");
//         mInterface->AddCommandVoid(&mtsIntuitiveResearchKitConsole::power_on, this,
//                                    "power_on");
//         mInterface->AddCommandVoid(&mtsIntuitiveResearchKitConsole::home, this,
//                                    "home");
//         mInterface->AddEventWrite(ConfigurationEvents.ArmCurrentState,
//                                   "ArmCurrentState", prmKeyValue());
//         // mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::teleop_enable, this,
//         //                             "teleop_enable", false);
//         // mInterface->AddEventWrite(console_events.teleop_enabled,
//         //                           "teleop_enabled", false);
//         // // manage tele-op
//         // mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::cycle_teleop_PSM_by_MTM, this,
//         //                             "cycle_teleop_PSM_by_MTM", std::string(""));
//         // mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::select_teleop_PSM, this,
//         //                             "select_teleop_PSM", prmKeyValue("MTM", "PSM"));
//         // mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::set_scale, this,
//         //                             "set_scale", 0.5);
//         // mInterface->AddEventWrite(ConfigurationEvents.scale,
//         //                           "scale", 0.5);
//         // mInterface->AddEventWrite(ConfigurationEvents.teleop_PSM_selected,
//         //                           "teleop_PSM_selected", prmKeyValue("MTM", "PSM"));
//         // mInterface->AddEventWrite(ConfigurationEvents.teleop_PSM_unselected,
//         //                           "teleop_PSM_unselected", prmKeyValue("MTM", "PSM"));
//         // audio
//         mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::set_volume, this,
//                                     "set_volume", m_audio_volume);
//         mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::beep, this,
//                                     "beep", vctDoubleVec());
//         mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::string_to_speech, this,
//                                     "string_to_speech", std::string());
//         mInterface->AddEventWrite(audio.volume,
//                                   "volume", m_audio_volume);
//         // emulate foot pedal events
//         // mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::OperatorPresentEventHandler, this,
//         //                             "emulate_operator_present", prmEventButton());
//         // mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::ClutchEventHandler, this,
//         //                             "emulate_clutch", prmEventButton());
//         // mInterface->AddCommandWrite(&mtsIntuitiveResearchKitConsole::CameraEventHandler, this,
//         //                             "emulate_camera", prmEventButton());
//         // misc.
//         mInterface->AddCommandRead(&mtsIntuitiveResearchKitConsole::calibration_mode, this,
//                                    "calibration_mode", false);
//         // Following is Read instead of VoidReturn because it is called before the component
//         // is created (i.e., thread not yet running)
//         mInterface->AddCommandRead(&mtsIntuitiveResearchKitConsole::ConnectInternal, this,
//                                    "connect", false);
//     }
// }

// void mtsIntuitiveResearchKitConsole::Configure(const std::string & filename)
// {

//     // loop over all arms to check if IO is needed, also check if some IO configuration files are listed in "io"
//     // mHasIO = false;
//     // for (auto iter = m_arm_proxies.begin(); iter != end; ++iter) {
//     //     std::string ioConfig = iter->second->m_IO_configuration_file;
//     //     if (!ioConfig.empty()) {
//     //         mHasIO = true;
//     //     }
//     // }
//     // bool physicalFootpedalsRequired = true;
//     // jsonValue = jsonConfig["io"];
//     // if (!jsonValue.empty()) {
//     //     // generic files
//     //     Json::Value configFiles = jsonValue["configuration-files"];
//     //     if (!configFiles.empty()) {
//     //         mHasIO = true;
//     //     }
//     //     // footpedals config
//     //     configFiles = jsonValue["footpedals"];
//     //     if (!configFiles.empty()) {
//     //         mHasIO = true;
//     //     }
//     //     // see if user wants to force no foot pedals
//     //     Json::Value footpedalsRequired = jsonValue["physical_footpedals_required"];
//     //     if (!footpedalsRequired.empty()) {
//     //         physicalFootpedalsRequired = footpedalsRequired.asBool();
//     //     }
//     // }
//     // // just check for IO and make sure we don't have io and hid, will be configured later
//     // jsonValue = jsonConfig["operator-present"];
//     // if (!jsonValue.empty()) {
//     //     // check if operator present uses IO
//     //     Json::Value jsonConfigFile = jsonValue["io"];
//     //     if (!jsonConfigFile.empty()) {
//     //         mHasIO = true;
//     //         jsonConfigFile = jsonValue["hid"];
//     //         if (!jsonConfigFile.empty()) {
//     //             CMN_LOG_CLASS_INIT_ERROR << "Configure: operator-present can't have both io and hid" << std::endl;
//     //             exit(EXIT_FAILURE);
//     //         }
//     //     }
//     // }
//     // create IO if needed and configure IO
//     // if (mHasIO) {
//     //     mtsRobotIO1394 * io = new mtsRobotIO1394(m_IO_component_name, periodIO, port);
//     //     io->SetProtocol(protocol);
//     //     io->SetWatchdogPeriod(watchdogTimeout);
//     //     // configure for each arm
//     //     for (auto iter = m_arm_proxies.begin(); iter != end; ++iter) {
//     //         std::string ioConfig = iter->second->m_IO_configuration_file;
//     //         if (ioConfig != "") {
//     //             CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring IO using \"" << ioConfig << "\"" << std::endl;
//     //             io->Configure(ioConfig);
//     //         }
//     //         std::string ioGripperConfig = iter->second->m_IO_gripper_configuration_file;
//     //         if (ioGripperConfig != "") {
//     //             CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring IO gripper using \"" << ioGripperConfig << "\"" << std::endl;
//     //             io->Configure(ioGripperConfig);
//     //         }
//     //     }
//     //         }
//     //         // footpedals, we assume these are the default one provided along the dVRK
//     //         configFiles = jsonValue["footpedals"];
//     //         if (!configFiles.empty()) {
//     //             const std::string configFile = find_file(configFiles.asString());
//     //             if (configFile == "") {
//     //                 CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find configuration file "
//     //                                          << configFiles.asString() << std::endl;
//     //                 exit(EXIT_FAILURE);
//     //             }
//     //             CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring IO foot pedals using \"" << configFile << "\"" << std::endl;
//     //             // these can be overwritten using console-inputs
//     //             mDInputSources["Clutch"] = InterfaceComponentType(m_IO_component_name, "Clutch");
//     //             mDInputSources["OperatorPresent"] = InterfaceComponentType(m_IO_component_name, "Coag");
//     //             mDInputSources["Coag"] = InterfaceComponentType(m_IO_component_name, "Coag");
//     //             mDInputSources["BiCoag"] = InterfaceComponentType(m_IO_component_name, "BiCoag");
//     //             mDInputSources["Camera"] = InterfaceComponentType(m_IO_component_name, "Camera");
//     //             mDInputSources["Cam-"] = InterfaceComponentType(m_IO_component_name, "Cam-");
//     //             mDInputSources["Cam+"] = InterfaceComponentType(m_IO_component_name, "Cam+");
//     //             mDInputSources["Head"] = InterfaceComponentType(m_IO_component_name, "Head");
//     //             io->Configure(configFile);
//     //         }
//     //     }
//     //     // configure IO for operator present
//     //     jsonValue = jsonConfig["operator-present"];
//     //     if (!jsonValue.empty()) {
//     //         // check if operator present uses IO
//     //         Json::Value jsonConfigFile = jsonValue["io"];
//     //         if (!jsonConfigFile.empty()) {
//     //             const std::string configFile = find_file(jsonConfigFile.asString());
//     //             if (configFile == "") {
//     //                 CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find configuration file "
//     //                                          << jsonConfigFile.asString() << std::endl;
//     //                 exit(EXIT_FAILURE);
//     //             }
//     //             CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring operator present using \""
//     //                                        << configFile << "\"" << std::endl;
//     //             io->Configure(configFile);
//     //         } else {
//     //             jsonConfigFile = jsonValue["hid"];
//     //         }
//     //     }
//     //     // configure for endoscope focus
//     //     jsonValue = jsonConfig["endoscope-focus"];
//     //     if (!jsonValue.empty()) {
//     //         // check if operator present uses IO
//     //         Json::Value jsonConfigFile = jsonValue["io"];
//     //         if (!jsonConfigFile.empty()) {
//     //             const std::string configFile = find_file(jsonConfigFile.asString());
//     //             if (configFile == "") {
//     //                 CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find configuration file "
//     //                                          << jsonConfigFile.asString() << std::endl;
//     //                 exit(EXIT_FAILURE);
//     //             }
//     //             CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring endoscope focus using \""
//     //                                        << configFile << "\"" << std::endl;
//     //             io->Configure(configFile);
//     //         }
//     //     }
//     //     // and add the io component!
//     //     m_IO_interface = AddInterfaceRequired("IO");
//     //     if (m_IO_interface) {
//     //         m_IO_interface->AddFunction("close_all_relays", IO.close_all_relays);
//     //         m_IO_interface->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ErrorEventHandler,
//     //                                              this, "error");
//     //         m_IO_interface->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::WarningEventHandler,
//     //                                              this, "warning");
//     //         m_IO_interface->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::StatusEventHandler,
//     //                                              this, "status");
//     //     } else {
//     //         CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to create IO required interface" << std::endl;
//     //         exit(EXIT_FAILURE);
//     //     }
//     //     mtsComponentManager::GetInstance()->AddComponent(io);
//     //     if (m_IO_interface) {
//     //         m_connections.Add(this->GetName(), "IO",
//     //                          io->GetName(), "Configuration");
//     //     }
//     // }


//     // ECM teleops
//     // const auto json_teleop_ecms = jsonConfig["teleop_ECMs"];
//     // for (unsigned int index = 0; index < json_teleop_ecms.size(); ++index) {
//     //     const auto json_teleop_ecm = json_teleop_ecms[index];
//     //     std::string teleop_ecm_name;
//     //     if (!json_teleop_ecm["name"].empty()) {
//     //         teleop_ecm_name = json_teleop_ecm["name"].asString();
//     //     } else {
//     //         const auto teleop_ecm_mtml_name = json_teleop_ecm["MTML"].asString();
//     //         const auto teleop_ecm_mtmr_name = json_teleop_ecm["MTMR"].asString();
//     //         const auto teleop_ecm_ecm_name = json_teleop_ecm["ECM"].asString();
//     //         teleop_ecm_name = teleop_ecm_mtml_name + "_" + teleop_ecm_mtmr_name + "_" + teleop_ecm_ecm_name;
//     //     }
//     //     CMN_LOG_CLASS_INIT_VERBOSE << "Configure: name for teleop_ECMs["
//     //                                << index << "] is: " << teleop_ecm_name << std::endl;
//     //     const auto iter = m_teleop_ECM_proxies.find(teleop_ecm_name);
//     //     if (iter == m_teleop_ECM_proxies.end()) {
//     //         // create a new teleop_ecm proxy if needed
//     //         auto teleop_ECM_proxy = std::make_shared<teleop_ECM_proxy_t>(teleop_ecm_name, this);
//     //         teleop_ECM_proxy->configure(json_teleop_ecm);
//     //         m_teleop_ECM_proxies[teleop_ecm_name] = teleop_ECM_proxy;
//     //         teleop_ECM_proxy->create_teleop();
//     //         add_teleop_ECM_interfaces(teleop_ECM_proxy);
//     //     } else {
//     //         CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure teleop_ECMs["
//     //                                  << index << "], teleop_ECM "
//     //                                  << teleop_ecm_name << " already exists" << std::endl;
//     //         exit(EXIT_FAILURE);
//     //     }
//     // }

//     // // PSM teleops
//     // const auto json_teleop_psms = jsonConfig["teleop_PSMs"];
//     // for (unsigned int index = 0; index < json_teleop_psms.size(); ++index) {
//     //     const auto json_teleop_psm = json_teleop_psms[index];
//     //     std::string teleop_psm_name;
//     //     if (!json_teleop_psm["name"].empty()) {
//     //         teleop_psm_name = json_teleop_psm["name"].asString();
//     //     } else {
//     //         const auto teleop_psm_mtm_name = json_teleop_psm["MTM"].asString();
//     //         const auto teleop_psm_psm_name = json_teleop_psm["PSM"].asString();
//     //         teleop_psm_name = teleop_psm_mtm_name + "_" + teleop_psm_psm_name;
//     //     }
//     //     CMN_LOG_CLASS_INIT_VERBOSE << "Configure: name for teleop_PSMs["
//     //                                << index << "] is: " << teleop_psm_name << std::endl;
//     //     const auto iter = m_teleop_PSM_proxies.find(teleop_psm_name);
//     //     if (iter == m_teleop_PSM_proxies.end()) {
//     //         // create a new teleop_psm proxy if needed
//     //         auto teleop_PSM_proxy = std::make_shared<teleop_PSM_proxy_t>(teleop_psm_name, this);
//     //         teleop_PSM_proxy->configure(json_teleop_psm);
//     //         m_teleop_PSM_proxies[teleop_psm_name] = teleop_PSM_proxy;
//     //         teleop_PSM_proxy->create_teleop();
//     //         add_teleop_PSM_interfaces(teleop_PSM_proxy);
//     //     } else {
//     //         CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure teleop_PSMs["
//     //                                  << index << "], teleop_PSM "
//     //                                  << teleop_psm_name << " already exists" << std::endl;
//     //         exit(EXIT_FAILURE);
//     //     }
//     // }

//     // // see which event is used for operator present
//     // // find name of button event used to detect if operator is present

//     // // load from console inputs
//     // const Json::Value consoleInputs = jsonConfig["console-inputs"];
//     // if (!consoleInputs.empty()) {
//     //     std::string component, interface;
//     //     component = consoleInputs["operator-present"]["component"].asString();
//     //     interface = consoleInputs["operator-present"]["interface"].asString();
//     //     if ((component != "") && (interface != "")) {
//     //         mDInputSources["OperatorPresent"] = InterfaceComponentType(component, interface);
//     //     }
//     //     component = consoleInputs["clutch"]["component"].asString();
//     //     interface = consoleInputs["clutch"]["interface"].asString();
//     //     if ((component != "") && (interface != "")) {
//     //         mDInputSources["Clutch"] = InterfaceComponentType(component, interface);
//     //     }
//     //     component = consoleInputs["camera"]["component"].asString();
//     //     interface = consoleInputs["camera"]["interface"].asString();
//     //     if ((component != "") && (interface != "")) {
//     //         mDInputSources["Camera"] = InterfaceComponentType(component, interface);
//     //     }
//     // }

//     // load operator-present settings, this will over write older settings
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

//     // message re. footpedals are likely missing but user can override this requirement
//     // const std::string footpedalMessage = "Maybe you're missing \"io\":\"footpedals\" in your configuration file.  If you don't need physical footpedals, set \"physical_footpedals_required\" to false.";

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

//     // if we have any teleoperation component, we need to have the interfaces for the foot pedals
//     // unless user explicitly says we can skip
//     // if (physicalFootpedalsRequired) {
//     //     const DInputSourceType::const_iterator endDInputs = mDInputSources.end();
//     //     const bool foundClutch = (mDInputSources.find("Clutch") != endDInputs);
//     //     const bool foundOperatorPresent = (mDInputSources.find("OperatorPresent") != endDInputs);
//     //     const bool foundCamera = (mDInputSources.find("Camera") != endDInputs);

//     //     if (m_teleop_PSM_proxies.size() > 0) {
//     //         if (!foundClutch || !foundOperatorPresent) {
//     //             CMN_LOG_CLASS_INIT_ERROR << "Configure: inputs for footpedals \"Clutch\" and \"OperatorPresent\" need to be defined since there's at least one PSM tele-operation component.  "
//     //                                      << footpedalMessage << std::endl;
//     //             exit(EXIT_FAILURE);
//     //         }
//     //     }
//     // if (mTeleopECM) {
//     //     if (!foundCamera || !foundOperatorPresent) {
//     //         CMN_LOG_CLASS_INIT_ERROR << "Configure: inputs for footpedals \"Camera\" and \"OperatorPresent\" need to be defined since there's an ECM tele-operation component.  "
//     //                                  << footpedalMessage << std::endl;
//     //         exit(EXIT_FAILURE);
//     //     }
//     // }
//     // }
//     // this->AddFootpedalInterfaces();

//     m_configured = true;
// }

// const bool & mtsIntuitiveResearchKitConsole::Configured(void) const
// {
//     return m_configured;
// }

// void mtsIntuitiveResearchKitConsole::Startup(void)
// {
//     // EventSelectedTeleopPSMs();
//     // emit scale event
//     ConfigurationEvents.scale(mtsIntuitiveResearchKit::TeleOperationPSM::Scale);
// }

// // void mtsIntuitiveResearchKitConsole::AddFootpedalInterfaces(void)
// // {
// //     const auto endDInputs = mDInputSources.end();

// //     auto iter = mDInputSources.find("Clutch");
// //     if (iter != endDInputs) {
// //         mtsInterfaceRequired * clutchRequired = AddInterfaceRequired("Clutch");
// //         if (clutchRequired) {
// //             clutchRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::ClutchEventHandler, this, "Button");
// //         }
// //         m_connections.Add(this->GetName(), "Clutch",
// //                           iter->second.first, iter->second.second);
// //     }
// //     mtsInterfaceProvided * clutchProvided = AddInterfaceProvided("Clutch");
// //     if (clutchProvided) {
// //         clutchProvided->AddEventWrite(console_events.clutch, "Button", prmEventButton());
// //     }

// //     iter = mDInputSources.find("Camera");
// //     if (iter != endDInputs) {
// //         mtsInterfaceRequired * cameraRequired = AddInterfaceRequired("Camera");
// //         if (cameraRequired) {
// //             cameraRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::CameraEventHandler, this, "Button");
// //         }
// //         m_connections.Add(this->GetName(), "Camera",
// //                           iter->second.first, iter->second.second);
// //     }
// //     mtsInterfaceProvided * cameraProvided = AddInterfaceProvided("Camera");
// //     if (cameraProvided) {
// //         cameraProvided->AddEventWrite(console_events.camera, "Button", prmEventButton());
// //     }

// //     iter = mDInputSources.find("OperatorPresent");
// //     if (iter != endDInputs) {
// //         mtsInterfaceRequired * operatorRequired = AddInterfaceRequired("OperatorPresent");
// //         if (operatorRequired) {
// //             operatorRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsole::OperatorPresentEventHandler, this, "Button");
// //         }
// //         m_connections.Add(this->GetName(), "OperatorPresent",
// //                           iter->second.first, iter->second.second);
// //     }
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


// // void mtsIntuitiveResearchKitConsole::teleop_enable(const bool & enable)
// // {
// //     mTeleopEnabled = enable;
// //     // if we have an SUJ, make sure it's ready
// //     if (enable && m_SUJ) {
// //         const auto sujState = ArmStates.find("SUJ");
// //         if ((sujState == ArmStates.end())
// //             || (sujState->second.State() != prmOperatingState::ENABLED)) {
// //             mTeleopEnabled = false;
// //         }
// //     }
// //     mTeleopDesired = enable;
// //     // event
// //     console_events.teleop_enabled(mTeleopEnabled);
// //     UpdateTeleopState();
// // }

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
// //                     if (mTeleopEnabled) {
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
// //                 if (mTeleopEnabled) {
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
// //     if (mTeleopEnabled) {
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

// // void mtsIntuitiveResearchKitConsole::EventSelectedTeleopPSMs(void) const
// // {
// //     for (auto & iter : m_teleop_PSM_proxies) {
// //         if (iter.second->selected()) {
// //             ConfigurationEvents.teleop_PSM_selected(prmKeyValue(iter.second->m_config->MTM,
// //                                                                 iter.second->m_config->PSM));
// //         } else {
// //             ConfigurationEvents.teleop_PSM_unselected(prmKeyValue(iter.second->m_config->MTM,
// //                                                                   iter.second->m_config->PSM));
// //         }
// //     }
// // }

// // void mtsIntuitiveResearchKitConsole::UpdateTeleopState(void)
// // {
// //     // Check if teleop is enabled
// //     if (!mTeleopEnabled) {
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
// // }

// // void mtsIntuitiveResearchKitConsole::set_scale(const double & scale)
// // {
// //     for (auto & iter : m_teleop_PSM_proxies) {
// //         iter.second->set_scale(scale);
// //     }
// //     ConfigurationEvents.scale(scale);
// // }

// // void mtsIntuitiveResearchKitConsole::ClutchEventHandler(const prmEventButton & button)
// // {
// //     switch (button.Type()) {
// //     case prmEventButton::PRESSED:
// //         mInterface->SendStatus(this->GetName() + ": clutch pressed");
// //         audio.beep(vct3(0.1, 700.0, m_audio_volume));
// //         break;
// //     case prmEventButton::RELEASED:
// //         mInterface->SendStatus(this->GetName() + ": clutch released");
// //         audio.beep(vct3(0.1, 700.0, m_audio_volume));
// //         break;
// //     case prmEventButton::CLICKED:
// //         mInterface->SendStatus(this->GetName() + ": clutch quick tap");
// //         audio.beep(vct3(0.05, 2000.0, m_audio_volume));
// //         audio.beep(vct3(0.05, 2000.0, m_audio_volume));
// //         // if (mTeleopMTMToCycle != "") {
// //         //     cycle_teleop_PSM_by_MTM(mTeleopMTMToCycle);
// //         // }
// //         break;
// //     default:
// //         break;
// //     }
// //     console_events.clutch(button);
// // }

// // void mtsIntuitiveResearchKitConsole::CameraEventHandler(const prmEventButton & button)
// // {
// //     switch (button.Type()) {
// //     case prmEventButton::PRESSED:
// //         // mCameraPressed = true;
// //         mInterface->SendStatus(this->GetName() + ": camera pressed");
// //         audio.beep(vct3(0.1, 1000.0, m_audio_volume));
// //         break;
// //     case prmEventButton::RELEASED:
// //         mCameraPressed = false;
// //         mInterface->SendStatus(this->GetName() + ": camera released");
// //         audio.beep(vct3(0.1, 1000.0, m_audio_volume));
// //         break;
// //     case prmEventButton::CLICKED:
// //         mInterface->SendStatus(this->GetName() + ": camera quick tap");
// //         audio.beep(vct3(0.05, 2500.0, m_audio_volume));
// //         audio.beep(vct3(0.05, 2500.0, m_audio_volume));
// //         break;
// //     default:
// //         break;
// //     }
// //     UpdateTeleopState();
// //     console_events.camera(button);
// // }

// // void mtsIntuitiveResearchKitConsole::OperatorPresentEventHandler(const prmEventButton & button)
// // {
// //     switch (button.Type()) {
// //     case prmEventButton::PRESSED:
// //         mOperatorPresent = true;
// //         mInterface->SendStatus(this->GetName() + ": operator present");
// //         audio.beep(vct3(0.3, 1500.0, m_audio_volume));
// //         break;
// //     case prmEventButton::RELEASED:
// //         mOperatorPresent = false;
// //         mInterface->SendStatus(this->GetName() + ": operator not present");
// //         audio.beep(vct3(0.3, 1200.0, m_audio_volume));
// //         break;
// //     default:
// //         break;
// //     }
// //     UpdateTeleopState();
// //     console_events.operator_present(button);
// // }

// // void mtsIntuitiveResearchKitConsole::ErrorEventHandler(const mtsMessage & message)
// // {
// //     // similar to teleop_enable(false) except we don't change mTeleopDesired
// //     mTeleopEnabled = false;
// //     console_events.teleop_enabled(mTeleopEnabled);
// //     UpdateTeleopState();

// //     mInterface->SendError(message.Message);
// //     // throttle error beeps
// //     double currentTime = mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime();
// //     if ((currentTime - mTimeOfLastErrorBeep) > 2.0 * cmn_s) {
// //         audio.beep(vct3(0.3, 3000.0, m_audio_volume));
// //         mTimeOfLastErrorBeep = currentTime;
// //     }
// // }
