/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

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
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QTextEdit>
#include <QLineEdit>
#include <QMessageBox>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QTime>
#include <QScrollBar>
#include <QPushButton>
#include <QHBoxLayout>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>

#include <sawIntuitiveResearchKit/mtsTeleOperationECMQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperationECMQtWidget, mtsComponent, std::string);

mtsTeleOperationECMQtWidget::mtsTeleOperationECMQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds * 1000), // Qt timers are in milliseconds
    LogEnabled(false)
{
    QMMessage = new mtsMessageQtWidget();

    // Setup cisst interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("TeleOperation");
    if (interfaceRequired) {
        QMMessage->SetInterfaceRequired(interfaceRequired);
        interfaceRequired->AddFunction("SetScale", TeleOperation.SetScale);
        interfaceRequired->AddFunction("GetPositionCartesianMTML",
                                       TeleOperation.GetPositionCartesianMTML);
        interfaceRequired->AddFunction("GetPositionCartesianMTMR",
                                       TeleOperation.GetPositionCartesianMTMR);
        interfaceRequired->AddFunction("GetPositionCartesianECM",
                                       TeleOperation.GetPositionCartesianECM);
        interfaceRequired->AddFunction("GetPeriodStatistics", TeleOperation.GetPeriodStatistics);
        // events
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECMQtWidget::DesiredStateEventHandler,
                                                this, "DesiredState");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECMQtWidget::CurrentStateEventHandler,
                                                this, "CurrentState");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECMQtWidget::FollowingEventHandler,
                                                this, "Following");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECMQtWidget::ScaleEventHandler,
                                                this, "Scale");
    }
}

void mtsTeleOperationECMQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsTeleOperationECMQtWidget::Startup(void)
{
    setupUi();
    startTimer(TimerPeriodInMilliseconds);
    if (!LogEnabled) {
        QMMessage->hide();
    }
    if (!parent()) {
        show();
    }
}

void mtsTeleOperationECMQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsTeleOperationECMQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsTeleOperationECMQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsTeleOperationECMQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    // retrieve transformations
    TeleOperation.GetPositionCartesianMTML(PositionMTML);
    QCPGMTMLWidget->SetValue(PositionMTML);

    TeleOperation.GetPositionCartesianMTMR(PositionMTMR);
    QCPGMTMRWidget->SetValue(PositionMTMR);

    TeleOperation.GetPositionCartesianECM(PositionECM);
    QCPGECMWidget->SetValue(PositionECM);

    TeleOperation.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}

void mtsTeleOperationECMQtWidget::SlotSetScale(double scale)
{
    TeleOperation.SetScale(scale);
}

void mtsTeleOperationECMQtWidget::SlotDesiredStateEventHandler(QString state)
{
    QLEDesiredState->setText(state);
}

void mtsTeleOperationECMQtWidget::SlotCurrentStateEventHandler(QString state)
{
    QLECurrentState->setText(state);
}

void mtsTeleOperationECMQtWidget::SlotFollowingEventHandler(bool following)
{
    if (following) {
        QLEFollowing->setText("FOLLOWING");
    } else {
        QLEFollowing->setText("INDEPENDANT");
    }
}

void mtsTeleOperationECMQtWidget::SlotScaleEventHandler(double scale)
{
    QSBScale->setValue(scale);
}

void mtsTeleOperationECMQtWidget::setupUi(void)
{
    // main layout
    QVBoxLayout * mainLayout = new QVBoxLayout();
    setLayout(mainLayout);

    // instructions
    QLabel * instructionsLabel = new QLabel("Operator must be present to operate (sometime using COAG pedal).");
    instructionsLabel->setWordWrap(true);
    mainLayout->addWidget(instructionsLabel);

    // 3D frames
    QGridLayout * frameLayout = new QGridLayout;
    mainLayout->addLayout(frameLayout);
    int column = 0;
    QLabel * masterLabel = new QLabel("<b>MTML</b>");
    masterLabel->setAlignment(Qt::AlignCenter);
    frameLayout->addWidget(masterLabel, 0, column);
    QCPGMTMLWidget = new prmPositionCartesianGetQtWidget();
    frameLayout->addWidget(QCPGMTMLWidget, 1, column);
    column++;
    masterLabel = new QLabel("<b>MTMR</b>");
    masterLabel->setAlignment(Qt::AlignCenter);
    frameLayout->addWidget(masterLabel, 0, column);
    QCPGMTMRWidget = new prmPositionCartesianGetQtWidget();
    frameLayout->addWidget(QCPGMTMRWidget, 1, column);
    column++;
    QLabel * ecmLabel = new QLabel("<b>ECM</b>");
    ecmLabel->setAlignment(Qt::AlignCenter);
    frameLayout->addWidget(ecmLabel, 0, column);
    QCPGECMWidget = new prmPositionCartesianGetQtWidget();
    frameLayout->addWidget(QCPGECMWidget, 1, column);

    // scale/lock/unlock/messages
    QHBoxLayout * buttonsLayout = new QHBoxLayout;
    mainLayout->addLayout(buttonsLayout);

    // scale
    QSBScale = new QDoubleSpinBox();
    QSBScale->setRange(0.1, 1.0);
    QSBScale->setSingleStep(0.1);
    QSBScale->setPrefix("scale ");
    QSBScale->setValue(0.2);
    buttonsLayout->addWidget(QSBScale);

    // messages on/off
    QPBLog = new QPushButton("Messages");
    QPBLog->setCheckable(true);
    buttonsLayout->addWidget(QPBLog);

    // state and timing
    QHBoxLayout * stateAndTimingLayout = new QHBoxLayout();
    mainLayout->addLayout(stateAndTimingLayout);

    // state info
    QVBoxLayout * stateLayout = new QVBoxLayout();
    stateAndTimingLayout->addLayout(stateLayout);

    QHBoxLayout * stateDesiredLayout = new QHBoxLayout;
    stateLayout->addLayout(stateDesiredLayout);
    QLabel * label = new QLabel("Desired state");
    stateDesiredLayout->addWidget(label);
    QLEDesiredState = new QLineEdit("");
    QLEDesiredState->setReadOnly(true);
    stateDesiredLayout->addWidget(QLEDesiredState);
    stateDesiredLayout->addStretch();

    QHBoxLayout * stateCurrentLayout = new QHBoxLayout;
    stateLayout->addLayout(stateCurrentLayout);
    label = new QLabel("Current state");
    stateCurrentLayout->addWidget(label);
    QLECurrentState = new QLineEdit("");
    QLECurrentState->setReadOnly(true);
    stateCurrentLayout->addWidget(QLECurrentState);
    stateCurrentLayout->addStretch();

    QHBoxLayout * followingLayout = new QHBoxLayout;
    stateLayout->addLayout(followingLayout);
    label = new QLabel("Teleoperation \"mode\"");
    followingLayout->addWidget(label);
    QLEFollowing = new QLineEdit("");
    QLEFollowing->setReadOnly(true);
    followingLayout->addWidget(QLEFollowing);
    followingLayout->addStretch();

    // Timing
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    stateAndTimingLayout->addWidget(QMIntervalStatistics);

    // messages
    QMMessage->setupUi();
    mainLayout->addWidget(QMMessage);

    setWindowTitle("TeleOperation Controller");
    resize(sizeHint());

    // setup Qt Connection
    connect(this, SIGNAL(SignalDesiredState(QString)),
            this, SLOT(SlotDesiredStateEventHandler(QString)));
    connect(this, SIGNAL(SignalCurrentState(QString)),
            this, SLOT(SlotCurrentStateEventHandler(QString)));
    connect(this, SIGNAL(SignalFollowing(bool)),
            this, SLOT(SlotFollowingEventHandler(bool)));

    connect(QSBScale, SIGNAL(valueChanged(double)),
            this, SLOT(SlotSetScale(double)));
    connect(this, SIGNAL(SignalScale(double)),
            this, SLOT(SlotScaleEventHandler(double)));

    // messages
    connect(QPBLog, SIGNAL(clicked()),
            this, SLOT(SlotLogEnabled()));
}

void mtsTeleOperationECMQtWidget::DesiredStateEventHandler(const std::string & state)
{
    emit SignalDesiredState(QString(state.c_str()));
}

void mtsTeleOperationECMQtWidget::CurrentStateEventHandler(const std::string & state)
{
    emit SignalCurrentState(QString(state.c_str()));
}

void mtsTeleOperationECMQtWidget::FollowingEventHandler(const bool & following)
{
    emit SignalFollowing(following);
}

void mtsTeleOperationECMQtWidget::ScaleEventHandler(const double & scale)
{
    emit SignalScale(scale);
}

void mtsTeleOperationECMQtWidget::SlotLogEnabled(void)
{
    LogEnabled = QPBLog->isChecked();
    if (LogEnabled) {
        QMMessage->show();
    } else {
        QMMessage->hide();
    }
}
