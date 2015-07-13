/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Youri Tan
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
    InterfaceRequired->AddFunction("GetBrakeCurrent", GetBrakeCurrent);
    InterfaceRequired->AddFunction("Clutch", Clutch);
    InterfaceRequired->AddFunction("GetVoltagesPrimary", GetPrimaryVoltages);
    InterfaceRequired->AddFunction("GetVoltagesSecondary", GetSecondaryVoltages);
    InterfaceRequired->AddFunction("SetRecalibrationMatrix", SetRecalibratioMatrix);

    const double negativeInfinity = cmnTypeTraits<double>::MinusInfinity();

    JointVoltageStart.SetSize(6);
    JointVoltageStart.SetAll(negativeInfinity);
    JointVoltageFinish.SetSize(6);
    JointVoltageFinish.SetAll(negativeInfinity);

    JointPositionStart.SetSize(6);
    JointPositionStart.SetAll(negativeInfinity);
    JointPositionFinish.SetSize(6);
    JointPositionFinish.SetAll(negativeInfinity);

    mVoltages[0].SetSize(6);
    mVoltages[0].Zeros();
    mVoltages[1].SetSize(6);
    mVoltages[1].Zeros();

    mJointsRecalibrationMatrix.SetSize(6,6);
    mJointsRecalibrationMatrix.SetAll(negativeInfinity);
}

void mtsIntuitiveResearchKitSUJQtWidget::setupUiDerived(void)
{
    QGridLayout * jointLayout = new QGridLayout;
    MainLayout->addLayout(jointLayout);

    QLabel * labelJoints = new QLabel("Joints");
    QVJointWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVJointWidget->SetPrecision(5);

    QPushButton * clutchButton = new QPushButton("Clutch");
    QPushButton * ManualRecalibrationButton = new QPushButton("Manual Recalibration");

    QLabel * labelBrakeCurrent = new QLabel("Brake (mA)");
    QVBrakeCurrentWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVBrakeCurrentWidget->SetPrecision(5);

    QLabel * labelRecalibrationInputStart = new QLabel("Joint Start");
    QLabel * labelRecalibrationInputFinish = new QLabel("Joint Finish");
    QVPotentiometerRecalibrationStartWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::TEXT_WIDGET);
    QVPotentiometerRecalibrationFinishWidget = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::TEXT_WIDGET);

    jointLayout->addWidget(labelJoints,1,0);
    jointLayout->addWidget(QVJointWidget,1,1);
    jointLayout->addWidget(labelBrakeCurrent,2,0);
    jointLayout->addWidget(QVBrakeCurrentWidget,2,1);
    jointLayout->addWidget(clutchButton,3,1);
    jointLayout->addWidget(labelRecalibrationInputStart,4,0);
    jointLayout->addWidget(QVPotentiometerRecalibrationStartWidget,4,1);
    jointLayout->addWidget(labelRecalibrationInputFinish,5,0);
    jointLayout->addWidget(QVPotentiometerRecalibrationFinishWidget,5,1);
    jointLayout->addWidget(ManualRecalibrationButton,6,1);

    connect(clutchButton, SIGNAL(pressed()),
            this, SLOT(SlotClutchPressed()));
    connect(clutchButton, SIGNAL(released()),
            this, SLOT(SlotClutchReleased()));
    connect(QVPotentiometerRecalibrationStartWidget, SIGNAL(valueChanged()),
            this, SLOT(SlotRecalibrationStartChanged()));
    connect(QVPotentiometerRecalibrationFinishWidget, SIGNAL(valueChanged()),
            this, SLOT(SlotRecalibrationFinishChanged()));
    connect(ManualRecalibrationButton, SIGNAL(clicked()),
            this, SLOT(SlotManualRecalibration()));
}

void mtsIntuitiveResearchKitSUJQtWidget::timerEventDerived(void)
{
    // get data
    GetPositionJoint(PositionJointParam);
    vctDoubleVec position(PositionJointParam.Position());
    GetBrakeCurrent(BrakeCurrent);
    GetPrimaryVoltages(mVoltages[0]);
    GetSecondaryVoltages(mVoltages[1]);

    // first axis is a translation, convert to mm
    position.Element(0) *= 1000.0;
    // all others are angles, convert to degrees
    for (size_t index = 1; index < position.size(); ++index) {
        position.Element(index) *= (180.0 / cmnPI);
    }

    // display
    QVPotentiometerRecalibrationStartWidget->SetValue(JointPositionStart);
    QVPotentiometerRecalibrationFinishWidget->SetValue(JointPositionFinish);
    QVJointWidget->SetValue(position);
    QVBrakeCurrentWidget->SetValue(vctDoubleVec(1, BrakeCurrent * 1000.0));
}

void mtsIntuitiveResearchKitSUJQtWidget::SlotClutchPressed(void)
{
    Clutch(true);
}

void mtsIntuitiveResearchKitSUJQtWidget::SlotClutchReleased(void)
{
    Clutch(false);
}

void mtsIntuitiveResearchKitSUJQtWidget::SlotRecalibrationStartChanged(void)
{
    const unsigned int currentColumn = QVPotentiometerRecalibrationStartWidget->currentColumn();
    QVPotentiometerRecalibrationStartWidget->GetValue(JointPositionStart);
    if (currentColumn == 0) {
        // convert mm to m
        mJointsRecalibrationMatrix.Element(0, currentColumn) = JointPositionStart[currentColumn] / 1000.0;
    } else {
        // convert deg to rad
        mJointsRecalibrationMatrix.Element(0, currentColumn) = JointPositionStart[currentColumn] * (cmnPI / 180.0);
    }
    mJointsRecalibrationMatrix.Element(1, currentColumn) = mVoltages[0][currentColumn];
    mJointsRecalibrationMatrix.Element(2, currentColumn) = mVoltages[1][currentColumn];
}

void mtsIntuitiveResearchKitSUJQtWidget::SlotRecalibrationFinishChanged(void)
{
    const unsigned int currentColumn = QVPotentiometerRecalibrationFinishWidget->currentColumn();
    QVPotentiometerRecalibrationFinishWidget->GetValue(JointPositionFinish);
    if (currentColumn == 0) {
        // convert mm to m
        mJointsRecalibrationMatrix.Element(3, currentColumn) = JointPositionFinish[currentColumn] / 1000.0;
    } else {
        // convert deg to rad
        mJointsRecalibrationMatrix.Element(3, currentColumn) = JointPositionFinish[currentColumn] * (cmnPI / 180.0);
    }
    mJointsRecalibrationMatrix.Element(4, currentColumn) = mVoltages[0][currentColumn];
    mJointsRecalibrationMatrix.Element(5, currentColumn) = mVoltages[1][currentColumn];
}

void mtsIntuitiveResearchKitSUJQtWidget::SlotManualRecalibration(void)
{
    if (mJointsRecalibrationMatrix.IsFinite()) {
        SetRecalibratioMatrix(mJointsRecalibrationMatrix);
    } else {
        QMessageBox::information(this, tr("mtsIntuitiveResearchKitSUJQtWidget"),
                                 tr("You need to provide all joint values"));
    }
}
