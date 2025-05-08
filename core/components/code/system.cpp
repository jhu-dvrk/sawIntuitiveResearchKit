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

#include <sawIntuitiveResearchKit/system.h>

#include <cisstCommon/cmnRandomSequence.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstParameterTypes/prmKeyValue.h>

#include <sawTextToSpeech/mtsTextToSpeech.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitConfig.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitRevision.h>
#include <sawIntuitiveResearchKit/IO_proxy.h>
#include <sawIntuitiveResearchKit/arm_proxy.h>
#include <sawIntuitiveResearchKit/console.h>
#include <sawIntuitiveResearchKit/teleop_PSM_proxy.h>
#include <sawIntuitiveResearchKit/teleop_ECM_proxy.h>

#if sawIntuitiveResearchKit_HAS_HID_HEAD_SENSOR
#include <sawIntuitiveResearchKit/mtsHIDHeadSensor.h>
#endif

#include <json/json.h>

typedef dvrk::system dvrk_system;

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(dvrk_system, mtsTaskFromSignal, std::string);

dvrk::system::system(const std::string & componentName):
    mtsTaskFromSignal(componentName, 100),
    m_configured(false),
    mTimeOfLastErrorBeep(0.0)
{
    // configure search path
    m_config_path.Add(cmnPath::GetWorkingDirectory());
    // add path to source/share directory to find common files.  This
    // will work as long as this component is located in the same
    // parent directory as the "shared" directory.
    m_config_path.Add(std::string(sawIntuitiveResearchKit_SOURCE_CONFIG_DIR), cmnPath::TAIL);
    // default installation directory
    m_config_path.Add(mtsIntuitiveResearchKit::DefaultInstallationDirectory, cmnPath::TAIL);

    m_interface = AddInterfaceProvided("Main");
    if (m_interface) {
        m_interface->AddMessageEvents();
        m_interface->AddCommandVoid(&system::power_off, this,
                                    "power_off");
        m_interface->AddCommandVoid(&system::power_on, this,
                                    "power_on");
        m_interface->AddCommandVoid(&system::home, this,
                                    "home");
        m_interface->AddEventWrite(ConfigurationEvents.ArmCurrentState,
                                   "ArmCurrentState", prmKeyValue());
        // audio
        m_interface->AddCommandWrite(&system::set_volume, this,
                                     "set_volume", m_audio_volume);
        m_interface->AddCommandWrite(&system::beep, this,
                                     "beep", vctDoubleVec());
        m_interface->AddCommandWrite(&system::string_to_speech, this,
                                     "string_to_speech", std::string());
        m_interface->AddEventWrite(audio.volume,
                                   "volume", m_audio_volume);
        // misc.
        m_interface->AddCommandRead(&system::calibration_mode, this,
                                   "calibration_mode", false);
        // Following is Read instead of VoidReturn because it is called before the component
        // is created (i.e., thread not yet running)
        m_interface->AddCommandRead(&system::ConnectInternal, this,
                                    "connect", false);
    }
}

void dvrk::system::set_calibration_mode(const bool mode)
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
        std::cerr << "system::" << message.str() << std::endl;
        CMN_LOG_CLASS_INIT_WARNING << message.str() << std::endl;
    }
}

const bool & dvrk::system::calibration_mode(void) const
{
    return m_calibration_mode;
}

void dvrk::system::calibration_mode(bool & result) const
{
    result = m_calibration_mode;
}

void dvrk::system::Configure(const std::string & filename)
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
    CMN_LOG_CLASS_INIT_VERBOSE << "system::Configure, loaded:" << std::endl
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
    for (auto & IO_config : m_config.IOs) {
        const auto iter = m_IO_proxies.find(IO_config.name);
        if (iter == m_IO_proxies.end()) {
            // create a new IO proxy if needed
            auto IO_proxy = std::make_shared<dvrk::IO_proxy>(IO_config.name, this, &IO_config);
            IO_proxy->post_configure();
            m_IO_proxies[IO_config.name] = IO_proxy;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure IO "
                                     << IO_config.name << ", IO already exists" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // find all arms
    for (auto & arm_config : m_config.arms) {
        const auto iter = m_arm_proxies.find(arm_config.name);
        if (iter == m_arm_proxies.end()) {
            // create a new arm proxy if needed
            auto arm_proxy = std::make_shared<dvrk::arm_proxy>(arm_config.name, this, &arm_config);
            arm_proxy->post_configure();
            m_arm_proxies[arm_config.name] = arm_proxy;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure arm "
                                     << arm_config.name << ", arm already exists" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // find all consoles
    for (auto & console_config : m_config.consoles) {
        const auto iter = m_consoles.find(console_config.name);
        if (iter == m_consoles.end()) {
            // create a new console if needed
            auto console = std::make_shared<dvrk::console>(console_config.name, this, &console_config);
            console->post_configure();
            m_consoles[console_config.name] = console;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to configure console "
                                     << console_config.name << ", console already exists" << std::endl;
            exit(EXIT_FAILURE);
        }
    }

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

    // load consoles if any
    for (auto iter : m_consoles) {
        auto & console = iter.second;
        console->create_components();
        add_console_interfaces(console);


        // arm_proxy->configure_IO();
        // arm_proxy->create_PID();
        // add_arm_interfaces(arm_proxy);
    }
    

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


const bool & dvrk::system::Configured(void) const
{
    return m_configured;
}


void dvrk::system::Startup(void)
{
    std::string message = this->GetName();
    message.append(" started, dVRK ");
    message.append(sawIntuitiveResearchKit_VERSION);
    message.append(" / cisst ");
    message.append(cisst_VERSION);
    m_interface->SendStatus(message);

    // close all relays if needed
    for (auto & iter : m_IO_proxies) {
        if (iter.second->m_config->close_all_relays) {
            iter.second->close_all_relays();
        }
    }

    // emit events for active PSM teleop pairs
    // EventSelectedTeleopPSMs();
    // emit scale event
    // ConfigurationEvents.scale(mtsIntuitiveResearchKit::TeleOperationPSM::Scale);
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


void dvrk::system::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();
}


void dvrk::system::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}


bool dvrk::system::add_teleop_ECM_interfaces(std::shared_ptr<dvrk::teleop_ECM_proxy> teleop_proxy)
{
    teleop_proxy->m_interface_required = this->AddInterfaceRequired(teleop_proxy->m_name);
    if (teleop_proxy->m_interface_required) {
        teleop_proxy->m_interface_required->AddFunction("state_command", teleop_proxy->state_command);
        teleop_proxy->m_interface_required->AddFunction("set_scale", teleop_proxy->set_scale);
        teleop_proxy->m_interface_required->AddEventHandlerWrite(&system::ErrorEventHandler, this, "error");
        teleop_proxy->m_interface_required->AddEventHandlerWrite(&system::WarningEventHandler, this, "warning");
        teleop_proxy->m_interface_required->AddEventHandlerWrite(&system::StatusEventHandler, this, "status");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "add_teleop_ECM_interfaces: failed to add main interface for teleop \""
                                 << teleop_proxy->m_name << "\"" << std::endl;
        return false;
    }
    return true;
}


bool dvrk::system::add_teleop_PSM_interfaces(std::shared_ptr<dvrk::teleop_PSM_proxy> teleop_proxy)
{
    teleop_proxy->m_interface_required = this->AddInterfaceRequired(teleop_proxy->m_name);
    if (teleop_proxy->m_interface_required) {
        teleop_proxy->m_interface_required->AddFunction("state_command", teleop_proxy->state_command);
        teleop_proxy->m_interface_required->AddFunction("set_scale", teleop_proxy->set_scale);
        teleop_proxy->m_interface_required->AddEventHandlerWrite(&system::ErrorEventHandler, this, "error");
        teleop_proxy->m_interface_required->AddEventHandlerWrite(&system::WarningEventHandler, this, "warning");
        teleop_proxy->m_interface_required->AddEventHandlerWrite(&system::StatusEventHandler, this, "status");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "add_teleop_PSM_interfaces: failed to add main interface for teleop \""
                                 << teleop_proxy->m_name << "\"" << std::endl;
        return false;
    }
    return true;
}


bool dvrk::system::add_IO_interfaces(std::shared_ptr<dvrk::IO_proxy> IO)
{
    IO->m_interface_required = AddInterfaceRequired(IO->m_name);
    if (IO->m_interface_required) {
        IO->m_interface_required->AddFunction("close_all_relays", IO->close_all_relays);
        IO->m_interface_required->AddEventHandlerWrite(&system::ErrorEventHandler,
                                                       this, "error");
        IO->m_interface_required->AddEventHandlerWrite(&system::WarningEventHandler,
                                                       this, "warning");
        IO->m_interface_required->AddEventHandlerWrite(&system::StatusEventHandler,
                                                       this, "status");
        m_connections.Add(this->GetName(), IO->m_name,
                          IO->m_name, "Configuration");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "add_IO_interfaces: failed to create IO required interface" << std::endl;
        return false;
    }
    return true;
}


bool dvrk::system::add_arm_interfaces(std::shared_ptr<dvrk::arm_proxy> arm)
{
    // IO
    if (arm->m_config->expects_IO()) {
        const std::string interfaceNameIO = "IO-" + arm->m_name;
        arm->m_IO_interface_required = AddInterfaceRequired(interfaceNameIO);
        if (arm->m_IO_interface_required) {
            arm->m_IO_interface_required->AddEventHandlerWrite(&system::ErrorEventHandler,
                                                               this, "error");
            arm->m_IO_interface_required->AddEventHandlerWrite(&system::WarningEventHandler,
                                                               this, "warning");
            arm->m_IO_interface_required->AddEventHandlerWrite(&system::StatusEventHandler,
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
                arm->m_IO_dallas_interface_required->AddEventHandlerWrite(&system::ErrorEventHandler,
                                                                          this, "error");
                arm->m_IO_dallas_interface_required->AddEventHandlerWrite(&system::WarningEventHandler,
                                                                          this, "warning");
                arm->m_IO_dallas_interface_required->AddEventHandlerWrite(&system::StatusEventHandler,
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
            arm->m_PID_interface_required->AddEventHandlerWrite(&system::ErrorEventHandler,
                                                                this, "error");
            arm->m_PID_interface_required->AddEventHandlerWrite(&system::WarningEventHandler,
                                                                this, "warning");
            arm->m_PID_interface_required->AddEventHandlerWrite(&system::StatusEventHandler,
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
        arm->m_arm_interface_required->AddEventHandlerWrite(&system::ErrorEventHandler,
                                                            this, "error");
        arm->m_arm_interface_required->AddEventHandlerWrite(&system::WarningEventHandler,
                                                            this, "warning");
        arm->m_arm_interface_required->AddEventHandlerWrite(&system::StatusEventHandler,
                                                            this, "status");
        arm->m_arm_interface_required->AddEventHandlerWrite(&dvrk::arm_proxy::CurrentStateEventHandler,
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


bool dvrk::system::add_console_interfaces(std::shared_ptr<dvrk::console> console)
{
    // main interface
    console->m_interface_provided = this->AddInterfaceProvided(console->m_name);
    if (console->m_interface_provided) {
        console->m_interface_provided->AddCommandWrite(&console::teleop_enable, console.get(),
                                                       "teleop_enable", false);
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "add_console_interfaces: failed to add interface for console \""
                                 << console->m_name << "\"" << std::endl;
        return false;
    }
    // clutch
    console->m_clutch_interface_provided = this->AddInterfaceProvided(console->m_name + "/clutch");
    if (console->m_clutch_interface_provided) {
        console->m_clutch_propagate =
            console->m_clutch_interface_provided->AddEventWrite("Button", prmEventButton());
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "add_console_interfaces: failed to add clutch interface for console \""
                                 << console->m_name << "\"" << std::endl;
        return false;
    }
    return true;
}


bool dvrk::system::Connect(void)
{
    mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();

    // connect console for audio feedback
    component_manager->Connect(this->GetName(), "TextToSpeech",
                               m_text_to_speech->GetName(), "Commands");

    // arms
    for (const auto & iter : m_arm_proxies) {
        std::shared_ptr<dvrk::arm_proxy> arm = iter.second;
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
void dvrk::system::ConnectInternal(bool &ret) const
{
    ret = const_cast<system *>(this)->Connect();
}


std::string dvrk::system::find_file(const std::string & filename) const
{
    return m_config_path.Find(filename);
}


void dvrk::system::power_off(void)
{
    for (auto & console : m_consoles) {
        console.second->teleop_enable(false);
    }
    for (auto & arm : m_arm_proxies) {
        arm.second->state_command(std::string("disable"));
    }
}


void dvrk::system::power_on(void)
{
    DisableFaultyArms();
    for (auto & arm : m_arm_proxies) {
        arm.second->state_command(std::string("enable"));
    }
}


void dvrk::system::home(void)
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


void dvrk::system::DisableFaultyArms(void)
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


void dvrk::system::set_volume(const double & volume)
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
    m_interface->SendStatus(message.str());
    audio.volume(m_audio_volume);
}


void dvrk::system::beep(const vctDoubleVec & values)
{
    const size_t size = values.size();
    if ((size == 0) || (size > 3)) {
        m_interface->SendError(this->GetName() + ": beep expect up to 3 values (duration, frequency, volume)");
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
        m_interface->SendWarning(this->GetName() + ": beep, duration must be between 0.1 and 60s");
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
        m_interface->SendWarning(this->GetName() + ": beep, volume must be between 0 and 1");
    }
    // convert to fixed size vector and send
    audio.beep(vct3(result));
}


void dvrk::system::string_to_speech(const std::string & text)
{
    audio.string_to_speech(text);
}


void dvrk::system::ErrorEventHandler(const mtsMessage & message)
{
    m_interface->SendError(message.Message);
    // throttle error beeps
    double currentTime = mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime();
    if ((currentTime - mTimeOfLastErrorBeep) > 2.0 * cmn_s) {
        audio.beep(vct3(0.3, 3000.0, m_audio_volume));
        mTimeOfLastErrorBeep = currentTime;
    }
}


void dvrk::system::WarningEventHandler(const mtsMessage & message)
{
    m_interface->SendWarning(message.Message);
}


void dvrk::system::StatusEventHandler(const mtsMessage & message)
{
    m_interface->SendStatus(message.Message);
}


void dvrk::system::SetArmCurrentState(const std::string & arm_name,
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
