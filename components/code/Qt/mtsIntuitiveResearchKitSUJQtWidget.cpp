/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Youri Tan
  Created on: 2013-08-24

  (C) Copyright 2013-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// Qt include
#include <QMessageBox>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QPushButton>

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
    InterfaceRequired->AddFunction("GetBrakeCurrent", GetBrakeCurrent, MTS_OPTIONAL); // optional for Si
    InterfaceRequired->AddFunction("Clutch", Clutch);
    InterfaceRequired->AddFunction("SetLiftVelocity", SetLiftVelocity, MTS_OPTIONAL); // only used for Classic PSM3 
    InterfaceRequired->AddFunction("GetVoltagesPrimary", GetPrimaryVoltages);
    InterfaceRequired->AddFunction("GetVoltagesSecondary", GetSecondaryVoltages);
    InterfaceRequired->AddFunction("GetVoltagesExtra", GetExtraVoltages, MTS_OPTIONAL); // optional for Si
    InterfaceRequired->AddFunction("SetRecalibrationMatrix", SetRecalibratioMatrix);

    const double negativeInfinity = cmnTypeTraits<double>::MinusInfinity();

    JointPositionStart.SetSize(6);
    JointPositionStart.SetAll(negativeInfinity);
    JointPositionFinish.SetSize(6);
    JointPositionFinish.SetAll(negativeInfinity);

    mVoltagesExtra.SetSize(4);
    mVoltagesExtra.Zeros();

    mJointsRecalibrationMatrix.SetSize(6, 6);
    mJointsRecalibrationMatrix.SetAll(negativeInfinity);
}

void mtsIntuitiveResearchKitSUJQtWidget::Startup(void)
{
    mtsIntuitiveResearchKitArmQtWidget::Startup();
    if (!SetLiftVelocity.IsValid()) {
        QPBLiftDown->hide();
        QPBLiftUp->hide();
    }
    if (!GetBrakeCurrent.IsValid()) {
        QVBrakeCurrentWidget->hide();
    }
    if (!GetExtraVoltages.IsValid()) {
        QVExtraVoltagesWidget->hide();
    }
}

void mtsIntuitiveResearchKitSUJQtWidget::setupUiDerived(void)
{
    QGridLayout * sujLayout = new QGridLayout;
    MainLayout->addLayout(sujLayout);

    QPushButton * clutchButton = new QPushButton("Clutch");
    sujLayout->addWidget(clutchButton, 0, 0);
    QPBLiftDown = new QPushButton("Lift down");
    sujLayout->addWidget(QPBLiftDown, 0, 1);
    QPBLiftUp = new QPushButton("Lift up");
    sujLayout->addWidget(QPBLiftUp, 0, 2);
    QPBShowMore = new QPushButton("Show more");
    QPBShowMore->setCheckable(true);
    sujLayout->addWidget(QPBShowMore, 0, 3);

    // show more
    QWMore = new QWidget();
    sujLayout->addWidget(QWMore, 1, 0, 1, -1);
    QGridLayout * moreLayout = new QGridLayout();
    QWMore->setLayout(moreLayout);
    int row = 0;

    // brake current
    QLabel * labelBrakeCurrent = new QLabel("Brake (mA)");
    moreLayout->addWidget(labelBrakeCurrent, row, 0);
    QVBrakeCurrentWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVBrakeCurrentWidget->SetPrecision(5);
    moreLayout->addWidget(QVBrakeCurrentWidget, row, 1, 1, 1);
    row++;

    // voltages
    QLabel * labelPrimaryVoltages = new QLabel("Primary Voltages");
    moreLayout->addWidget(labelPrimaryVoltages, row, 0);
    QVPrimaryVoltagesWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVPrimaryVoltagesWidget->SetPrecision(5);
    moreLayout->addWidget(QVPrimaryVoltagesWidget, row, 1, 1, 5);
    row++;

    QLabel * labelSecondaryVoltages = new QLabel("Secondary Voltages");
    moreLayout->addWidget(labelSecondaryVoltages, row, 0);
    QVSecondaryVoltagesWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVSecondaryVoltagesWidget->SetPrecision(5);
    moreLayout->addWidget(QVSecondaryVoltagesWidget, row, 1, 1, 5);
    row++;

    QLabel * labelExtraVoltages = new QLabel("Extra Voltages");
    moreLayout->addWidget(labelExtraVoltages, row, 0);
    QVExtraVoltagesWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVExtraVoltagesWidget->SetPrecision(5);
    moreLayout->addWidget(QVExtraVoltagesWidget, row, 1, 1, 5);
    row++;

    // calibration stuff
    QLabel * labelRecalibrationInputStart = new QLabel("Joint Start");
    labelRecalibrationInputStart->setToolTip("Measured height or angle in mm or degrees");
    moreLayout->addWidget(labelRecalibrationInputStart, row, 0);
    QVPotentiometerRecalibrationStartWidget
        = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::TEXT_WIDGET);
    QVPotentiometerRecalibrationStartWidget->setToolTip("Measured height or angle in mm or degrees");
    moreLayout->addWidget(QVPotentiometerRecalibrationStartWidget, row, 1, 1, -1);
    row++;

    QLabel * labelRecalibrationInputFinish = new QLabel("Joint Finish");
    labelRecalibrationInputFinish->setToolTip("Measured height or angle in mm or degrees");
    moreLayout->addWidget(labelRecalibrationInputFinish, row, 0);
    QVPotentiometerRecalibrationFinishWidget
        = new vctQtWidgetDynamicVectorDoubleWrite(vctQtWidgetDynamicVectorDoubleWrite::TEXT_WIDGET);
    QVPotentiometerRecalibrationFinishWidget->setToolTip("Measured height or angle in mm or degrees");
    moreLayout->addWidget(QVPotentiometerRecalibrationFinishWidget, row, 1, 1, -1);
    QPushButton * ManualRecalibrationButton = new QPushButton("Manual Recalibration");
    row++;

    moreLayout->addWidget(ManualRecalibrationButton, row, 2, 1, 1);

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
    // display more if needed
    if (mShowMore) {
        // brake voltage
        if (GetBrakeCurrent.IsValid()) {
            GetBrakeCurrent(BrakeCurrent);
            QVBrakeCurrentWidget->SetValue(vctDoubleVec(1, BrakeCurrent * 1000.0));
        }
        // extra voltages
        if (GetExtraVoltages.IsValid()) {
            GetExtraVoltages(mVoltagesExtra);
            QVExtraVoltagesWidget->SetValue(mVoltagesExtra);
        }
        // potentiometers
        GetPrimaryVoltages(mVoltages[0]);
        QVPrimaryVoltagesWidget->SetValue(mVoltages[0]);
        GetSecondaryVoltages(mVoltages[1]);
        QVSecondaryVoltagesWidget->SetValue(mVoltages[1]);
        // calibration data
        QVPotentiometerRecalibrationStartWidget->SetValue(JointPositionStart);
        QVPotentiometerRecalibrationFinishWidget->SetValue(JointPositionFinish);
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
