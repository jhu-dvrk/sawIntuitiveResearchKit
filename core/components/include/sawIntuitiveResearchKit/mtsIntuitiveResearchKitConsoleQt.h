/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-07-13

  (C) Copyright 2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitConsoleQt_h
#define _mtsIntuitiveResearchKitConsoleQt_h

#include <cisstCommon/cmnGenericObject.h>
#include <cisstMultiTask/mtsDelayedConnections.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitQtExport.h>

class QTabWidget;
class QWidget;

class CISST_EXPORT mtsIntuitiveResearchKitConsoleQt: public cmnGenericObject
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitConsoleQt(void);

    void Configure(mtsIntuitiveResearchKitConsole * console);

    inline void Connect(void) {
        Connections.Connect();
    }

    void addTab(QWidget * widget, const std::string & name);

protected:
    mtsDelayedConnections Connections;
    QTabWidget * TabWidget;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitConsoleQt);

#endif // _mtsIntuitiveResearchKitConsoleQt_h
