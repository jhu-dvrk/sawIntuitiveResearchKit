/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Youri Tan
  Created on: 2013-08-24

  (C) Copyright 2013-2016 Johns Hopkins University (JHU), All Rights Reserved.

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
    mtsIntuitiveResearchKitArmQtWidget(componentName, periodInSeconds),
    mShowMore(false)
{
    CMN_ASSERT(InterfaceRequired);
    InterfaceRequired->AddFunction("GetPositionJoint", GetPositionJoint);
    InterfaceRequired->AddFunction("GetBrakeCurrent", GetBrakeCurrent);
    InterfaceRequired->AddFunction("Clutch", Clutch);
    InterfaceRequired->AddFunction("SetLiftVelocity", SetLiftVelocity, MTS_OPTIONAL);
    InterfaceRequired->AddFunction("GetVoltagesPrimary", GetPrimaryVoltages);
    InterfaceRequired->AddFunction("GetVoltagesSecondary", GetSecondaryVoltages);
    InterfaceRequired->AddFunction("GetVoltagesExtra", GetExtraVoltages);
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
    mVoltagesExtra.SetSize(4);
    mVoltagesExtra.Zeros();

    mJointsRecalibrationMatrix.SetSize(6,6);
    mJointsRecalibrationMatrix.SetAll(negativeInfinity);
}

void mtsIntuitiveResearchKitSUJQtWidget::Startup(void)
{
    mtsIntuitiveResearchKitArmQtWidget::Startup();
    if (!SetLiftVelocity.IsValid()) {
        QPBLiftDown->hide();
        QPBLiftUp->hide();
    }
}

void mtsIntuitiveResearchKitSUJQtWidget::setupUiDerived(void)
{
    QGridLayout * sujLayout = new QGridLayout;
    MainLayout->addLayout(sujLayout);

    QLabel * labelJoints = new QLabel("Joints");
    sujLayout->addWidget(labelJoints, 0, 0);
    QVJointWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVJointWidget->SetPrecision(5);
    sujLayout->addWidget(QVJointWidget, 0, 1, 1, -1);

    QPushButton * clutchButton = new QPushButton("Clutch");
    sujLayout->addWidget(clutchButton, 1, 0);
    QPBLiftDown = new QPushButton("Lift down");
    sujLayout->addWidget(QPBLiftDown, 1, 1);
    QPBLiftUp = new QPushButton("Lift up");
    sujLayout->addWidget(QPBLiftUp, 1, 2);
    QPBShowMore = new QPushButton("Show more");
    QPBShowMore->setCheckable(true);
    sujLayout->addWidget(QPBShowMore, 1, 3);

    // show more
    QWMore = new QWidget();
    sujLayout->addWidget(QWMore, 2, 0, 1, -1);
    QGridLayout * moreLayout = new QGridLayout();
    QWMore->setLayout(moreLayout);

    // brake current
    QLabel * labelBrakeCurrent = new QLabel("Brake (mA)");
    moreLayout->addWidget(labelBrakeCurrent, 0, 0);
    QVBrakeCurrentWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVBrakeCurrentWidget->SetPrecision(5);
    moreLayout->addWidget(QVBrakeCurrentWidget, 0, 1, 1, 1);

    // extra voltages
    QLabel * labelExtraVoltages = new QLabel("Extra Voltages");
    moreLayout->addWidget(labelExtraVoltages, 1, 0);
    QVExtraVoltageWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVExtraVoltageWidget->SetPrecision(5);
    moreLayout->addWidget(QVExtraVoltageWidget, 1, 1, 1, 5);

    // calibration stuff
    QLabel * labelRecalibrationInputStart = new QLabel("Joint Start");
    moreLayout->addWidget(labelRecalibrationInputStart, 2, 0);
    QVPotentiometerRecalibrationStartWidget
        = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::TEXT_WIDGET);
    moreLayout->addWidget(QVPotentiometerRecalibrationStartWidget, 2, 1, 1, -1);
    QLabel * labelRecalibrationInputFinish = new QLabel("Joint Finish");
    moreLayout->addWidget(labelRecalibrationInputFinish, 3, 0);
    QVPotentiometerRecalibrationFinishWidget
        = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::TEXT_WIDGET);
    moreLayout->addWidget(QVPotentiometerRecalibrationFinishWidget, 3, 1, 1, -1);
    QPushButton * ManualRecalibrationButton = new QPushButton("Manual Recalibration");
    moreLayout->addWidget(ManualRecalibrationButton, 4, 2, 1, 1);
    
    QWMore->hide();

    connect(clutchButton, SIGNAL(pressed()),
            this, SLOT(SlotClutchPressed()));
    connect(clutchButton, SIGNAL(released()),
            this, SLOT(SlotClutchReleased()));

    connect(QPBLiftDown, SIGNAL(pressed()),
            this, SLOT(SlotVelocityDownPressed()));
    connect(QPBLiftDown, SIGNAL(released()),
            this, SLOT(SlotVelocityReleased()));
    connect(QPBLiftUp, SIGNAL(pressed()),
            this, SLOT(SlotVelocityUpPressed()));
    connect(QPBLiftUp, SIGNAL(released()),
            this, SLOT(SlotVelocityReleased()));

    connect(QPBShowMore, SIGNAL(clicked()),
            this, SLOT(SlotShowMore()));

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
    // first axis is a translation, convert to mm
    position.Element(0) *= 1000.0;
    // all others are angles, convert to degrees
    for (size_t index = 1; index < position.size(); ++index) {
        position.Element(index) *= (180.0 / cmnPI);
    }
    QVJointWidget->SetValue(position);

    // display more if needed
    if (mShowMore) {
        // brake voltage
        GetBrakeCurrent(BrakeCurrent);
        QVBrakeCurrentWidget->SetValue(vctDoubleVec(1, BrakeCurrent * 1000.0));
        // extra voltages
        GetExtraVoltages(mVoltagesExtra);
        QVExtraVoltageWidget->SetValue(mVoltagesExtra);
        // calibration data
        QVPotentiometerRecalibrationStartWidget->SetValue(JointPositionStart);
        QVPotentiometerRecalibrationFinishWidget->SetValue(JointPositionFinish);
        GetPrimaryVoltages(mVoltages[0]);
        GetSecondaryVoltages(mVoltages[1]);
    }
}

void mtsIntuitiveResearchKitSUJQtWidget::SlotShowMore(void)
{
    mShowMore = QPBShowMore->isChecked();
    if (mShowMore) {
        QWMore->show();
    } else {
        QWMore->hide();
    }
}

void mtsIntuitiveResearchKitSUJQtWidget::SlotClutchPressed(void)
{
    Clutch(true);
}

void mtsIntuitiveResearchKitSUJQtWidget::SlotClutchReleased(void)
{
    Clutch(false);
}

void mtsIntuitiveResearchKitSUJQtWidget::SlotVelocityDownPressed(void)
{
    SetLiftVelocity(-1.0);
}

void mtsIntuitiveResearchKitSUJQtWidget::SlotVelocityUpPressed(void)
{
    SetLiftVelocity(1.0);
}

void mtsIntuitiveResearchKitSUJQtWidget::SlotVelocityReleased(void)
{
    SetLiftVelocity(0.0);
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
