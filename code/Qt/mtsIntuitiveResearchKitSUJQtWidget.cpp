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


// system include
#include <iostream>

// Qt include
#include <QString>
#include <QtGui>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmPositionJointGet.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitSUJQtWidget.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitSUJQtWidget, mtsComponent, std::string);

mtsIntuitiveResearchKitSUJQtWidget::mtsIntuitiveResearchKitSUJQtWidget(const std::string & componentName, double periodInSeconds):
    mtsIntuitiveResearchKitArmQtWidget(componentName, periodInSeconds)
{
    CMN_ASSERT(InterfaceRequired);
    InterfaceRequired->AddFunction("GetPositionJoint", GetPositionJoint);
}


void mtsIntuitiveResearchKitSUJQtWidget::setupUiDerived(void)
{
    QLabel * label = new QLabel("Joints");
    MainLayout->addWidget(label);
    QVJointWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVJointWidget->SetPrecision(5);
    MainLayout->addWidget(QVJointWidget);
}

void mtsIntuitiveResearchKitSUJQtWidget::timerEventDerived(void)
{
    GetPositionJoint(PositionJointParam);
    vctDoubleVec position(PositionJointParam.Position());
    // first axis is a translation, convert to mm
    position.Element(0) *= 1000.0;
    // all others are angles
    for (size_t index = 1; index < position.size(); ++index) {
        position.Element(index) *= (180.0 / cmnPI);
    }
    // display
    QVJointWidget->SetValue(position);
}
