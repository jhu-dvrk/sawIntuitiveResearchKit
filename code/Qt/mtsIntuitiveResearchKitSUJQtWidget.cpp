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
    QGridLayout * jointLayout = new QGridLayout;
    MainLayout->addLayout(jointLayout);


    QLabel * labelJoints = new QLabel("Joints");
    QVJointWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVJointWidget->SetPrecision(5);

    QLabel * labelPrimaryOffsets = new QLabel("Primary Joint Offsets");
    QVPrimaryJointOffsetWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVPrimaryJointOffsetWidget->SetPrecision(5);

    QLabel * labelSecondaryOffsets = new QLabel("Secondary Joint Offsets");
    QVSecondaryJointOffsetWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVSecondaryJointOffsetWidget->SetPrecision(5);


    QPushButton * recalibrateOffsetsButton = new QPushButton("Recalibrate offsets");

    QLabel * labelBrakeCurrent = new QLabel("Brake Current (mA)");
    QVBrakeCurrentWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVBrakeCurrentWidget->SetPrecision(5);

    jointLayout->addWidget(labelJoints,1,0);
    jointLayout->addWidget(QVJointWidget,1,1);
    jointLayout->addItem(new QSpacerItem(250,0),2,0);
    jointLayout->addWidget(recalibrateOffsetsButton,3,1);
    jointLayout->addWidget(labelPrimaryOffsets,5,0);
    jointLayout->addWidget(QVPrimaryJointOffsetWidget,5,1);
    jointLayout->addWidget(labelSecondaryOffsets,6,0);
    jointLayout->addWidget(QVSecondaryJointOffsetWidget,6,1);
    jointLayout->addWidget(labelBrakeCurrent,2,0);
    jointLayout->addWidget(QVBrakeCurrentWidget,2,1);

    connect(recalibrateOffsetsButton, SIGNAL(clicked()),
            this, SLOT(SlotRecalibrateOffsets()));
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
