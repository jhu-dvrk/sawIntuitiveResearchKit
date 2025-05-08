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


#ifndef _dvrk_console_h
#define _dvrk_console_h

#include <iostream>
#include <map>
#include <memory>

#include <cisstMultiTask/mtsFunctionWrite.h>
#include <cisstParameterTypes/prmEventButton.h>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class mtsInterfaceProvided;

namespace dvrk {

    class system;
    class console_configuration;
    class teleop_PSM_proxy;
    class teleop_ECM_proxy;

    class CISST_EXPORT console: std::enable_shared_from_this<dvrk::console>  {

    public:

        friend class system;

        std::string m_name;
        dvrk::system * m_system = nullptr;
        dvrk::console_configuration * m_config = nullptr;

        typedef std::map<std::string, std::shared_ptr<teleop_PSM_proxy>> teleop_PSM_proxies;
        teleop_PSM_proxies m_teleop_PSM_proxies;

        typedef std::map<std::string, std::shared_ptr<teleop_ECM_proxy>> teleop_ECM_proxies;
        teleop_ECM_proxies m_teleop_ECM_proxies;

        console(const std::string & name,
                dvrk::system * system,
                dvrk::console_configuration * config);

        // NOT_COPYABLE(IO_proxy_t);
        // NOT_MOVEABLE(IO_proxy_t);

        /*! Configure, i.e. load json and validate. */
        void post_configure(void);

        /*! Create and configure IO component. */
        void create_components(void);

    protected:
        // main interface for ROS and Qt
        mtsInterfaceProvided * m_interface_provided;

        // to send clutch to teleop components
        mtsInterfaceProvided * m_clutch_interface_provided;
        mtsFunctionWrite m_clutch_propagate;

        // commands
        void teleop_enable(const bool &);
        void set_scale(const double & scale);

        void clutch_event_handler(const prmEventButton & button);
        void camera_event_handler(const prmEventButton & button);
        void operator_present_event_handler(const prmEventButton & button);

        // events
        struct {
            mtsFunctionWrite clutch;
            mtsFunctionWrite camera;
            mtsFunctionWrite operator_present;
            mtsFunctionWrite teleop_enabled;
            mtsFunctionWrite scale;
        } events;

        // internal methods to manage active teleops
        void update_teleop_state(void);

        // status
        bool m_operator_present = false;
        bool m_camera = false;
    };
}

#endif // _dvrk_console_h


        // void cycle_teleop_PSM_by_MTM(const std::string & mtmName);
        // void select_teleop_PSM(const prmKeyValue & mtmPsm);
        // bool GetPSMSelectedForMTM(const std::string & mtmName, std::string & psmName) const;
        // bool GetMTMSelectedForPSM(const std::string & psmName, std::string & mtmName) const;
        // void EventSelectedTeleopPSMs(void) const;

//         typedef std::multimap<std::string, std::shared_ptr<teleop_PSM_proxy_t>> teleop_PSM_proxies_by_arm_t;
//         teleop_PSM_proxies_by_arm_t m_teleop_PSM_proxies_by_psm;
//         teleop_PSM_proxies_by_arm_t m_teleop_PSM_proxies_by_MTM;

//         /*! Name of default MTM to cycle teleops if no name is provided */
//         std::string mTeleopMTMToCycle;

//         bool mTeleopEnabled = false;
//         bool mTeleopDesired = false;
//         bool mTeleopPSMRunning = false;
//         bool mTeleopECMRunning = false;
//         bool mTeleopEnabledBeforeCamera;

//     };



//     void AddFootpedalInterfaces(void);
//     double mTimeOfLastErrorBeep;

//     /*! Head sensor */
//     mtsComponent * mHeadSensor = nullptr;

//     /*! daVinci Endoscope Focus */
//     mtsDaVinciEndoscopeFocus * mDaVinciEndoscopeFocus = nullptr;



//     // components used for events (digital inputs)
//     typedef std::pair<std::string, std::string> InterfaceComponentType;
//     typedef std::map<std::string, InterfaceComponentType> DInputSourceType;
//     DInputSourceType mDInputSources;

//     mtsInterfaceProvided * mInterface = nullptr;
//     struct {
//         mtsFunctionWrite ArmCurrentState;
//         mtsFunctionWrite scale;
//         mtsFunctionWrite teleop_PSM_selected;
//         mtsFunctionWrite teleop_PSM_unselected;
//     } ConfigurationEvents;

//     void ErrorEventHandler(const mtsMessage & message);
//     void WarningEventHandler(const mtsMessage & message);
//     void StatusEventHandler(const mtsMessage & message);

//     std::map<std::string, prmOperatingState> ArmStates;
//     void SetArmCurrentState(const std::string & armName,
//                             const prmOperatingState & currentState);
// };
