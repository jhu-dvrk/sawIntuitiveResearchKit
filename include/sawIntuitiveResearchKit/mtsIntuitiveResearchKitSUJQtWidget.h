/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-08-24

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsIntuitiveResearchKitSUJQtWidget_h
#define _mtsIntuitiveResearchKitSUJQtWidget_h

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmQtWidget.h>
#include <cisstParameterTypes/prmPositionJointGet.h>

class mtsIntuitiveResearchKitSUJQtWidget: public mtsIntuitiveResearchKitArmQtWidget
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitSUJQtWidget(const std::string & componentName, double periodInSeconds = 50.0 * cmn_ms);
    ~mtsIntuitiveResearchKitSUJQtWidget() {}

protected:
    void setupUiDerived(void);
    void timerEventDerived(void);

    vctQtWidgetDynamicVectorDoubleRead * QVJointWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVPrimaryJointOffsetWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVSecondaryJointOffsetWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVBrakeCurrentWidget;

    mtsFunctionVoid RecalibrateOffsets;

    prmPositionJointGet PositionJointParam;
    vctDoubleVec PrimaryJointOffset;
    vctDoubleVec SecondaryJointOffset;
    double BrakeCurrent;

    mtsFunctionRead GetPositionJoint;
    mtsFunctionRead GetPrimaryJointOffset;
    mtsFunctionRead GetSecondaryJointOffset;
    mtsFunctionRead GetBrakeCurrent;

protected slots:
        void SlotRecalibrateOffsets(void);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitSUJQtWidget);

#endif // _mtsIntuitiveResearchKitSUJQtWidget_h
