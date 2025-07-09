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
#include <set>

#include <cisstMultiTask/mtsFunctionWrite.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawIntuitiveResearchKit/IO_configuration.h>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class mtsInterfaceProvided;

namespace dvrk {

    class system;
    class console_configuration;
    class teleop_proxy;
    class teleop_PSM_proxy;

    class CISST_EXPORT console: std::enable_shared_from_this<dvrk::console>  {

    public:

        friend class system;

        std::string m_name;
        dvrk::system * m_system = nullptr;
        dvrk::console_configuration * m_config = nullptr;

        typedef std::map<std::string, std::shared_ptr<teleop_proxy>> teleop_proxies;
        teleop_proxies m_teleop_proxies;

        console(const std::string & _name,
                dvrk::system * _system,
                dvrk::console_configuration * _config);

        // NOT_COPYABLE(IO_proxy_t);
        // NOT_MOVEABLE(IO_proxy_t);

        /*! Configure, i.e. load json and validate. */
        void post_configure(void);

        /*! Create and configure IO component. */
        void create_components(void);

        /*! Emulate Startup for mtsComponent. */
        void Startup(void);

    protected:
        // main interface for ROS and Qt
        mtsInterfaceProvided * m_interface_provided;

        // commands
        void teleop_enable(const bool &);
        void set_scale(const double & _scale);
        void select_teleop(const std::string & _selected);
        void unselect_teleop(const std::string & _unselected);
        bool find_conflicting_teleops(const std::shared_ptr<dvrk::teleop_proxy> _teleop, const bool _unselect);

        void clutch_event_handler(const prmEventButton & _button);
        void camera_event_handler(const prmEventButton & _button);
        void operator_present_event_handler(const prmEventButton & _button);

        // optional component used for the head sensor (based on input type)
        mtsComponent * m_head_sensor = nullptr;

        // events
        struct {
            mtsFunctionWrite clutch;
            mtsFunctionWrite camera;
            mtsFunctionWrite operator_present;
            mtsFunctionWrite teleop_enabled;
            mtsFunctionWrite scale;
            mtsFunctionWrite teleop_selected;
            mtsFunctionWrite teleop_unselected;
        } events;

        // internal methods to manage active teleops
        void update_teleop_state(void);
        void emit_teleop_state_events(void);

        // status
        bool m_operator_present = false;
        bool m_camera = false;
        bool m_teleop_enabled = false;
        bool m_teleop_wanted = false;

        // teleops PSM used for quick-tap
        std::shared_ptr<teleop_PSM_proxy> m_teleop_PSM_cycle_1 = nullptr;
        std::shared_ptr<teleop_PSM_proxy> m_teleop_PSM_cycle_2 = nullptr;
    };
}

#endif // _dvrk_console_h
