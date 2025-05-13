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

#ifndef _dvrk_teleop_ECM_proxy_h
#define _dvrk_teleop_ECM_proxy_h

#include <cisstMultiTask/mtsForwardDeclarations.h>
#include <cisstMultiTask/mtsFunctionWrite.h>

#include <sawIntuitiveResearchKit/teleop_proxy.h>

// Always include last!
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

namespace dvrk {

    class teleop_ECM_proxy_configuration;

    class CISST_EXPORT teleop_ECM_proxy: public teleop_proxy {
    public:

        friend class dvrk::system;
        friend class dvrk::console;

        teleop_ECM_proxy_configuration * m_config = nullptr;

        teleop_ECM_proxy(const std::string & name,
                         dvrk::system * system,
                         dvrk::console * console,
                         dvrk::teleop_ECM_proxy_configuration * config);

        // NOT_COPYABLE(teleop_ECM_proxy);
        // NOT_MOVEABLE(teleop_ECM_proxy);

        inline teleop_proxy::teleop_type type(void) const override {
            return teleop_proxy::ECM;
        }

        void post_configure(void) override;

        /*! Create and configure the teleoperation component. */
        void create_teleop(void) override;

     protected:
        std::string m_MTML_component_name, m_MTML_interface_name,
        m_MTMR_component_name, m_MTMR_interface_name,
        m_ECM_component_name, m_ECM_interface_name;

    };

}

#endif // _dvrk_teleop_ECM_proxy_h
