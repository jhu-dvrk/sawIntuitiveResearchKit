/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-08-24

  (C) Copyright 2013-2019 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <QPushButton>
#include <QCheckBox>
#include <QLineEdit>
#include <QMessageBox>
#include <QScrollBar>
#include <QCloseEvent>
#include <QCoreApplication>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmPositionJointGet.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmQtWidget.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitArmQtWidget, mtsComponent, std::string);

mtsIntuitiveResearchKitArmQtWidget::mtsIntuitiveResearchKitArmQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds * 1000),
    DirectControl(false),
    LogEnabled(false)
{
    QMMessage = new mtsMessageQtWidget();

    // Setup CISST Interface
    InterfaceRequired = AddInterfaceRequired("Manipulator");
    if (InterfaceRequired) {
        InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitArmQtWidget::DesiredStateEventHandler,
                                                this, "DesiredState");
        InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitArmQtWidget::CurrentStateEventHandler,
                                                this, "CurrentState");
        InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitArmQtWidget::OperatingStateEventHandler,
                                                this, "OperatingState");
        QMMessage->SetInterfaceRequired(InterfaceRequired);
        InterfaceRequired->AddFunction("GetStateJoint", Arm.GetStateJoint);
        InterfaceRequired->AddFunction("GetPositionCartesian", Arm.GetPositionCartesian);
        InterfaceRequired->AddFunction("GetWrenchBody", Arm.GetWrenchBody, MTS_OPTIONAL);
        InterfaceRequired->AddFunction("SetDesiredState", Arm.SetDesiredState);
        InterfaceRequired->AddFunction("GetPeriodStatistics", Arm.GetPeriodStatistics);
    }
}

void mtsIntuitiveResearchKitArmQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsIntuitiveResearchKitArmQtWidget::Startup(void)
{
    setupUi();
    startTimer(TimerPeriodInMilliseconds); // ms
    if (!LogEnabled) {
        QMMessage->hide();
    }
    if (!parent()) {
        show();
    }
}

void mtsIntuitiveResearchKitArmQtWidget::Cleanup(void)
{
    this->hide();
}

void mtsIntuitiveResearchKitArmQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsIntuitiveResearchKitArmQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsIntuitiveResearchKitArmQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    mtsExecutionResult executionResult;

    executionResult = Arm.GetStateJoint(StateJoint);
    if (executionResult) {
        QSJWidget->SetValue(StateJoint);
    }

    executionResult = Arm.GetPositionCartesian(Position);
    if (executionResult) {
        QCPGWidget->SetValue(Position);
    }

    executionResult = Arm.GetWrenchBody(Wrench);
    if (executionResult) {
        if (Wrench.Valid()) {
            QFTWidget->SetValue(Wrench.F(), Wrench.T(), Wrench.Timestamp());
        }
    }

    Arm.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);

    // for derived classes
    this->timerEventDerived();
}

void mtsIntuitiveResearchKitArmQtWidget::DesiredStateEventHandler(const std::string & state)
{
    emit SignalDesiredState(QString(state.c_str()));
}

void mtsIntuitiveResearchKitArmQtWidget::CurrentStateEventHandler(const std::string & state)
{
    emit SignalCurrentState(QString(state.c_str()));
}

void mtsIntuitiveResearchKitArmQtWidget::OperatingStateEventHandler(const prmOperatingState & state)
{
    emit SignalOperatingState(state);
}

void mtsIntuitiveResearchKitArmQtWidget::SlotLogEnabled(void)
{
    LogEnabled = QPBLog->isChecked();
    if (LogEnabled) {
        QMMessage->show();
    } else {
        QMMessage->hide();
    }
}

void mtsIntuitiveResearchKitArmQtWidget::SlotEnableDirectControl(bool toggle)
{
    DirectControl = toggle;
    QPBHome->setEnabled(toggle);
}

void mtsIntuitiveResearchKitArmQtWidget::SlotHome(void)
{
    Arm.SetDesiredState(std::string("READY"));
}

void mtsIntuitiveResearchKitArmQtWidget::SlotDesiredStateEventHandler(QString state)
{
    QLEDesiredState->setText(state);
}

void mtsIntuitiveResearchKitArmQtWidget::SlotCurrentStateEventHandler(QString state)
{
    QLECurrentState->setText(state);
}

void mtsIntuitiveResearchKitArmQtWidget::SlotOperatingStateEventHandler(const prmOperatingState & state)
{
    QPOState->SetValue(state);
}

void mtsIntuitiveResearchKitArmQtWidget::setupUi(void)
{
    MainLayout = new QVBoxLayout;
    MainLayout->setContentsMargins(2, 2, 2, 2);

    QGridLayout * topLayout = new QGridLayout;
    topLayout->setContentsMargins(2, 2, 2, 2);
    topLayout->setColumnStretch(1, 1);

    MainLayout->addLayout(topLayout);

    // timing
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    topLayout->addWidget(QMIntervalStatistics, 0, 0);

    // joint state
    QSJWidget = new prmStateJointQtWidget();
    QSJWidget->setupUi();
    QSJWidget->SetPrismaticRevoluteFactors(1.0 / cmn_mm, cmn180_PI);
    topLayout->addWidget(QSJWidget, 0, 1);

    // 3D position
    QCPGWidget = new prmPositionCartesianGetQtWidget();
    QCPGWidget->SetPrismaticRevoluteFactors(1.0 / cmn_mm, cmn180_PI);
    topLayout->addWidget(QCPGWidget, 1, 0);

    // wrench, make large
    QFTWidget = new vctForceTorqueQtWidget();
    topLayout->addWidget(QFTWidget, 1, 1);

    // state
    QHBoxLayout * stateLayout = new QHBoxLayout;
    stateLayout->setContentsMargins(2, 2, 2, 2);
    MainLayout->addLayout(stateLayout);

    // messages on/off
    QPBLog = new QPushButton("Messages");
    QPBLog->setCheckable(true);
    stateLayout->addWidget(QPBLog);
    QCBEnableDirectControl = new QCheckBox("Direct control");
    stateLayout->addWidget(QCBEnableDirectControl);
    QPBHome = new QPushButton("Home");
    stateLayout->addWidget(QPBHome);

    QLabel * label = new QLabel("Desired");
    stateLayout->addWidget(label);
    QLEDesiredState = new QLineEdit("");
    QLEDesiredState->setReadOnly(true);
    stateLayout->addWidget(QLEDesiredState);

    label = new QLabel("Current");
    stateLayout->addWidget(label);
    QLECurrentState = new QLineEdit("");
    QLECurrentState->setReadOnly(true);
    stateLayout->addWidget(QLECurrentState);

    // operating state
    QPOState = new prmOperatingStateQtWidget();
    stateLayout->addWidget(QPOState);

    // for derived classes
    this->setupUiDerived();

    // messages
    QMMessage->setupUi();
    MainLayout->addWidget(QMMessage);

    setLayout(MainLayout);
    setWindowTitle("Manipulator");
    resize(sizeHint());

    // setup Qt Connection
    connect(this, SIGNAL(SignalDesiredState(QString)),
            this, SLOT(SlotDesiredStateEventHandler(QString)));
    connect(this, SIGNAL(SignalCurrentState(QString)),
            this, SLOT(SlotCurrentStateEventHandler(QString)));
    connect(this, SIGNAL(SignalOperatingState(prmOperatingState)),
            this, SLOT(SlotOperatingStateEventHandler(prmOperatingState)));
    connect(QPBLog, SIGNAL(clicked()),
            this, SLOT(SlotLogEnabled()));
    connect(QCBEnableDirectControl, SIGNAL(toggled(bool)),
            this, SLOT(SlotEnableDirectControl(bool)));
    connect(QPBHome, SIGNAL(clicked()),
            this, SLOT(SlotHome()));

    // set initial values
    QCBEnableDirectControl->setChecked(DirectControl);
    SlotEnableDirectControl(DirectControl);
}
