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
    InterfaceRequired->AddFunction("GetPrimaryJointOffset", GetPrimaryJointOffset);
    InterfaceRequired->AddFunction("GetSecondaryJointOffset", GetSecondaryJointOffset);
    InterfaceRequired->AddFunction("GetBrakeCurrent", GetBrakeCurrent);
    InterfaceRequired->AddFunction("RecalibrateOffsets", RecalibrateOffsets);
}

void mtsIntuitiveResearchKitSUJQtWidget::setupUiDerived(void)
{
    QLabel * labelJoints = new QLabel("Joints");
    MainLayout->addWidget(labelJoints);
    QVJointWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVJointWidget->SetPrecision(5);
    MainLayout->addWidget(QVJointWidget);

    QLabel * labelPrimaryOffsets = new QLabel("Primary Joint Offsets");
    MainLayout->addWidget(labelPrimaryOffsets);
    QVPrimaryJointOffsetWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVPrimaryJointOffsetWidget->SetPrecision(5);
    MainLayout->addWidget(QVPrimaryJointOffsetWidget);

    QLabel * labelSecondaryOffsets = new QLabel("Secondary Joint Offsets");
    MainLayout->addWidget(labelSecondaryOffsets);
    QVSecondaryJointOffsetWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVSecondaryJointOffsetWidget->SetPrecision(5);
    MainLayout->addWidget(QVSecondaryJointOffsetWidget);

    QHBoxLayout * offsetLayout = new QHBoxLayout;
    MainLayout->addLayout(offsetLayout);
    QPushButton * recalibrateOffsetsButton = new QPushButton("Recalibrate offsets");
    offsetLayout->addWidget(recalibrateOffsetsButton);

    connect(recalibrateOffsetsButton, SIGNAL(clicked()),
            this, SLOT(SlotRecalibrateOffsets()));

    QLabel * labelBrakeCurrent = new QLabel("Break Current (mA)");
    MainLayout->addWidget(labelBrakeCurrent);
    QVBrakeCurrentWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVBrakeCurrentWidget->SetPrecision(5);
    MainLayout->addWidget(QVBrakeCurrentWidget);

}

void mtsIntuitiveResearchKitSUJQtWidget::timerEventDerived(void)
{
    // get data
    GetPositionJoint(PositionJointParam);
    vctDoubleVec position(PositionJointParam.Position());
    GetPrimaryJointOffset(PrimaryJointOffset);
    GetSecondaryJointOffset(SecondaryJointOffset);
    GetBrakeCurrent(BrakeCurrent);
    // first axis is a translation, convert to mm
    position.Element(0) *= 1000.0;
    PrimaryJointOffset.Element(0) *= 1000.0;
    SecondaryJointOffset.Element(0) *= 1000.0;
    // all others are angles, convert to degrees
    for (size_t index = 1; index < position.size(); ++index) {
        position.Element(index) *= (180.0 / cmnPI);
        PrimaryJointOffset.Element(index) *= (180.0 / cmnPI);
        SecondaryJointOffset.Element(index) *= (180.0 / cmnPI);
    }
    // display
    QVJointWidget->SetValue(position);
    QVPrimaryJointOffsetWidget->SetValue(PrimaryJointOffset);
    QVSecondaryJointOffsetWidget->SetValue(SecondaryJointOffset);
    QVBrakeCurrentWidget->SetValue(vctDoubleVec(1, BrakeCurrent * 1000.0));
}

void mtsIntuitiveResearchKitSUJQtWidget::SlotRecalibrateOffsets(void)
{
    RecalibrateOffsets();
}
