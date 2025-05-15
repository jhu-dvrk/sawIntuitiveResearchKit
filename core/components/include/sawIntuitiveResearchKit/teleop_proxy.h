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

#ifndef _dvrk_teleop_proxy_h
#define _dvrk_teleop_proxy_h

#include <cisstMultiTask/mtsForwardDeclarations.h>
#include <cisstMultiTask/mtsFunctionWrite.h>

// Always include last!
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

namespace dvrk {

    class system;
    class console;

    class CISST_EXPORT teleop_proxy {
    public:

        friend class dvrk::system;
        friend class dvrk::console;

        typedef enum {UNDEFINED, PSM, ECM} teleop_type;

        std::string m_name;
        dvrk::system * m_system = nullptr;
        dvrk::console * m_console = nullptr;

        teleop_proxy(const std::string & name,
                     dvrk::system * system,
                     dvrk::console * console);

        // NOT_COPYABLE(teleop_PSM_proxy);
        // NOT_MOVEABLE(teleop_PSM_proxy);

        virtual teleop_type type(void) const = 0;

        virtual void post_configure(void) = 0;

        /*! Create and configure the teleoperation component. */
        virtual void create_teleop(void) = 0;

     protected:
        mtsFunctionWrite state_command;
        mtsFunctionWrite set_scale;
        mtsInterfaceRequired * m_interface_required;

        bool m_selected = false; // actually selected and in use
        bool m_wanted = false; // user wants this

        std::list<std::string> m_arms_used;
    };
}

#endif // _dvrk_teleop_proxy_h
