/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-07-13

  (C) Copyright 2015-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _dvrk_system_Qt_h
#define _dvrk_system_Qt_h

#include <cisstCommon/cmnGenericObject.h>
#include <cisstMultiTask/mtsDelayedConnections.h>

// Always include last!
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitQtExport.h>

class QTabWidget;
class QWidget;

namespace dvrk {
    class system;

    class CISST_EXPORT system_Qt: public cmnGenericObject
    {
        CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

    public:
        system_Qt(void);

        void configure(dvrk::system * _system);

        inline void connect(void) {
            m_connections.Connect();
        }

        void add_tab(QWidget * _widget, const std::string & _name);

    protected:
        mtsDelayedConnections m_connections;
        QTabWidget * m_tab_widget;
    };
}

CMN_DECLARE_SERVICES_INSTANTIATION(dvrk::system_Qt);

#endif // _dvrk_system_Qt_h
