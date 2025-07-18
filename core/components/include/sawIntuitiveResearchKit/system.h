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


#ifndef _dvrk_system_h
#define _dvrk_system_h

#include <memory>
#include <set>

#include <cisstCommon/cmnPath.h>
#include <cisstMultiTask/mtsTaskFromSignal.h>
#include <cisstMultiTask/mtsDelayedConnections.h>
#include <cisstParameterTypes/prmOperatingState.h>

#include <sawIntuitiveResearchKit/system_configuration.h>

// Always include last!
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class mtsRobotIO1394;
class mtsTextToSpeech;
class mtsDaVinciEndoscopeFocus;
class mtsIntuitiveResearchKitArm;

namespace dvrk {
    class IO_proxy;
    class arm_proxy;
    class console;
    class teleop_proxy;
    class teleop_ECM_proxy;
    class teleop_PSM_proxy;
    class system_Qt;
    class system_ROS;

    class CISST_EXPORT system: public mtsTaskFromSignal
    {
        CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

    public:
        friend class dvrk::console;
        friend class dvrk::arm_proxy;
        friend class dvrk::teleop_proxy;
        friend class dvrk::teleop_ECM_proxy;
        friend class dvrk::teleop_PSM_proxy;
        friend class dvrk::system_Qt;
        friend class dvrk::system_ROS;
        system(const std::string & componentName);
        inline virtual ~system() {}

        /*! Tells the application to run in calibration mode, i.e. turn
          off all checks using potentiometers and force encoder re-bias
          from potentiometers when the application starts.  This method
          must be called before Configure. */
        void set_calibration_mode(const bool mode);
        const bool & calibration_mode(void) const;
        void calibration_mode(bool & result) const;

        /*! Configure console using JSON file. To test is the configuration
          succeeded, used method Configured().
        */
        void Configure(const std::string & filename);

        /*! Method to check if the configuration was successful, ideally called
          after a call to Configure.
        */
        const bool & Configured(void) const;

        void Startup(void);
        void Run(void);
        void Cleanup(void);

        void AddFootpedalInterfaces(void);

        bool Connect(void);

        std::string find_file(const std::string & filename) const;

    protected:
        dvrk::system_configuration m_config;
        bool m_configured;
        cmnPath m_config_path;
        mtsDelayedConnections m_connections;

        double mTimeOfLastErrorBeep;

        /*! Pointer to mtsTextToSpeech component */
        std::shared_ptr<mtsTextToSpeech> m_text_to_speech;

        typedef std::map<std::string, std::shared_ptr<dvrk::IO_proxy>> IO_proxies;
        IO_proxies m_IO_proxies;

        typedef std::map<std::string, std::shared_ptr<dvrk::arm_proxy>> arm_proxies;
        arm_proxies m_arm_proxies;

        typedef std::map<std::string, std::shared_ptr<dvrk::console>> consoles;
        consoles m_consoles;

        /*! ISI endoscope focus controller through dVRK controller */
        std::shared_ptr<mtsDaVinciEndoscopeFocus> m_ISI_focus_controller;

        bool add_IO_interfaces(std::shared_ptr<dvrk::IO_proxy> IO_proxy);
        bool add_arm_interfaces(std::shared_ptr<dvrk::arm_proxy> arm_proxy);
        bool add_console_interfaces(std::shared_ptr<dvrk::console> console);

        bool add_teleop_interfaces(std::shared_ptr<dvrk::teleop_proxy> teleop_proxy);

        void configure_IO(const dvrk::IO_configuration & _IO_config);

        void power_off(void);
        void power_on(void);
        void home(void);
        void disable_faulty_arms(void);
        // void set_scale(const double & scale);
        void set_volume(const double & volume);
        void beep(const vctDoubleVec & values); // duration, frequency, volume
        void string_to_speech(const std::string & text);

        std::shared_ptr<dvrk::arm_proxy> m_SUJ = nullptr;

        // force console to emit initial events for Startup
        void emit_events_console(void);

        void ConnectInternal(bool &ret) const;

        bool m_calibration_mode = false;

        struct {
            mtsFunctionWrite beep;
            mtsFunctionWrite string_to_speech;
            mtsFunctionWrite volume; // event
        } audio;
        double m_audio_volume;

        // components used for events (digital inputs)
        // typedef std::pair<std::string, std::string> InterfaceComponentType;
        // typedef std::map<std::string, InterfaceComponentType> DInputSourceType;
        // DInputSourceType mDInputSources;

        mtsInterfaceProvided * m_interface = nullptr;
        struct {
            mtsFunctionWrite arm_current_state;
        } events;

        void error_event_handler(const mtsMessage & message);
        void warning_event_handler(const mtsMessage & message);
        void status_event_handler(const mtsMessage & message);

        std::map<std::string, prmOperatingState> m_arm_states;
        void set_arm_current_state(const std::string & armName,
                                   const prmOperatingState & currentState);

        bool find_conflicting_teleops(const std::shared_ptr<dvrk::teleop_proxy> _teleop, const bool _unselect = false);
    };
}

CMN_DECLARE_SERVICES_INSTANTIATION(dvrk::system);

#endif // _dvrk_system
