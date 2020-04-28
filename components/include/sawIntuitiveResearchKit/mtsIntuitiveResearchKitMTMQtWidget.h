/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2020-04-28

  (C) Copyright 2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsIntuitiveResearchKitMTMQtWidget_h
#define _mtsIntuitiveResearchKitMTMQtWidget_h

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmQtWidget.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitQtExport.h>

class CISST_EXPORT mtsIntuitiveResearchKitMTMQtWidget: public mtsIntuitiveResearchKitArmQtWidget
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitMTMQtWidget(const std::string & componentName, double periodInSeconds = 50.0 * cmn_ms);
    ~mtsIntuitiveResearchKitMTMQtWidget() {}

protected:
    void setupUiDerived(void);
    void timerEventDerived(void);

    mtsFunctionRead GetStateGripper;
    QLineEdit * QLEGripperPosition;
    prmStateJoint StateGripper;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitMTMQtWidget);

#endif // _mtsIntuitiveResearchKitMTMQtWidget_h
