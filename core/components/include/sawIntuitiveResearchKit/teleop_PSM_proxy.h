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

#ifndef _dvrk_teleop_PSM_proxy_h
#define _dvrk_teleop_PSM_proxy_h

#include <cisstMultiTask/mtsForwardDeclarations.h>
#include <cisstMultiTask/mtsFunctionWrite.h>

// Always include last!
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

namespace dvrk {

    class system;
    class console;
    class teleop_PSM_proxy_configuration;

    class CISST_EXPORT teleop_PSM_proxy {
    public:
    
        friend class dvrk::system;
    
        std::string m_name;
        dvrk::system * m_system = nullptr;
        dvrk::console * m_console = nullptr;
        dvrk::teleop_PSM_proxy_configuration * m_config = nullptr;
        
        teleop_PSM_proxy(const std::string & name,
                         dvrk::system * system,
                         dvrk::console * console,
                         dvrk::teleop_PSM_proxy_configuration * config);
        
        // NOT_COPYABLE(teleop_PSM_proxy);
        // NOT_MOVEABLE(teleop_PSM_proxy);
        
        void post_configure(void);
        
        /*! Create and configure the teleoperation component. */
        void create_teleop(void);

     protected:
        mtsFunctionWrite state_command;
        mtsFunctionWrite set_scale;
        mtsInterfaceRequired * m_interface_required;
    };

}

#endif // _dvrk_teleop_PSM_proxy_h
