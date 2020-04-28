/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2019-08-07

  (C) Copyright 2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsIntuitiveResearchKitPSMQtWidget_h
#define _mtsIntuitiveResearchKitPSMQtWidget_h

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmQtWidget.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitQtExport.h>


class CISST_EXPORT mtsIntuitiveResearchKitPSMQtWidget: public mtsIntuitiveResearchKitArmQtWidget
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitPSMQtWidget(const std::string & componentName, double periodInSeconds = 50.0 * cmn_ms);
    ~mtsIntuitiveResearchKitPSMQtWidget() {}

protected:
    void setupUiDerived(void);
    void timerEventDerived(void);

signals:
    void SignalToolType(QString toolType);
    void SignalToolTypeRequest(void);

private slots:
    void SlotToolTypeEventHandler(QString toolType);
    void SlotToolTypeRequestEventHandler(void);
    void SlotToolTypeSelected(QString toolType);

private:
    void ToolTypeEventHandler(const std::string & toolType);
    void ToolTypeRequestEventHandler(void);

    QLineEdit * QLEToolType;
    QComboBox * QCBToolOptions;
    mtsFunctionWrite SetToolType;

    mtsFunctionRead jaw_measured_js;
    QLineEdit * QLEJawPosition;
    QLineEdit * QLEJawVelocity;
    QLineEdit * QLEJawEffort;
    prmStateJoint m_jaw_measured_js;

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitPSMQtWidget);

#endif // _mtsIntuitiveResearchKitPSMQtWidget_h
