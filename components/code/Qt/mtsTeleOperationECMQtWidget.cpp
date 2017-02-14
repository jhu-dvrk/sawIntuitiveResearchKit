/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2017 Johns Hopkins University (JHU), All Rights Reserved.

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
        interfaceRequired->AddFunction("GetRegistrationRotation",
                                       TeleOperation.GetRegistrationRotation);
        interfaceRequired->AddFunction("GetPeriodStatistics", TeleOperation.GetPeriodStatistics);
        // events
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECMQtWidget::DesiredStateEventHandler,
                                                this, "DesiredState");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECMQtWidget::CurrentStateEventHandler,
                                                this, "CurrentState");
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
    TeleOperation.GetPositionCartesianMTMR(PositionMTMR);
    TeleOperation.GetPositionCartesianECM(PositionECM);
    TeleOperation.GetRegistrationRotation(RegistrationRotation);

    // apply registration orientation
    vctFrm3 registeredECM;
    RegistrationRotation.ApplyInverseTo(PositionECM.Position().Rotation(), registeredECM.Rotation());
    RegistrationRotation.ApplyInverseTo(PositionECM.Position().Translation(), registeredECM.Translation());

    // update display
    QFRPositionMTMLWidget->SetValue(PositionMTML.Position());
    QFRPositionMTMRWidget->SetValue(PositionMTMR.Position());
    QFRPositionECMWidget->SetValue(registeredECM);

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

void mtsTeleOperationECMQtWidget::SlotScaleEventHandler(double scale)
{
    QSBScale->setValue(scale);
}

void mtsTeleOperationECMQtWidget::setupUi(void)
{
    // 3D frames
    QGridLayout * frameLayout = new QGridLayout;
    QLabel * masterLabel = new QLabel("<b>MTML</b>");
    masterLabel->setAlignment(Qt::AlignCenter);
    frameLayout->addWidget(masterLabel, 0, 0);
    QFRPositionMTMLWidget = new vctQtWidgetFrameDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);
    frameLayout->addWidget(QFRPositionMTMLWidget, 1, 0);
    masterLabel = new QLabel("<b>MTMR</b>");
    masterLabel->setAlignment(Qt::AlignCenter);
    frameLayout->addWidget(masterLabel, 0, 1);
    QFRPositionMTMRWidget = new vctQtWidgetFrameDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);
    frameLayout->addWidget(QFRPositionMTMRWidget, 1, 1);
    QLabel * slaveLabel = new QLabel("<b>ECM</b>");
    slaveLabel->setAlignment(Qt::AlignCenter);
    frameLayout->addWidget(slaveLabel, 2, 0, 1, 2);
    QFRPositionECMWidget = new vctQtWidgetFrameDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);
    frameLayout->addWidget(QFRPositionECMWidget, 3, 0, 1, 2);

    // right side
    QVBoxLayout * controlLayout = new QVBoxLayout;

    QLabel * instructionsLabel = new QLabel("Operator must be present to operate (sometime using COAG pedal).");
    instructionsLabel->setWordWrap(true);
    controlLayout->addWidget(instructionsLabel);

    // scale/lock/unlock/messages
    QHBoxLayout * buttonsLayout = new QHBoxLayout;
    controlLayout->addLayout(buttonsLayout);

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

    // state info
    QHBoxLayout * stateLayout = new QHBoxLayout;
    controlLayout->addLayout(stateLayout);

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

    // Timing
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    controlLayout->addWidget(QMIntervalStatistics);

    // messages
    QMMessage->setupUi();
    controlLayout->addWidget(QMMessage);

    // add stretch
    controlLayout->addStretch();

    QWidget * leftWidget = new QWidget();
    leftWidget->setLayout(frameLayout);
    addWidget(leftWidget);

    QWidget * rightWidget = new QWidget();
    rightWidget->setLayout(controlLayout);
    addWidget(rightWidget);

    setWindowTitle("TeleOperation Controller");
    resize(sizeHint());

    // setup Qt Connection
    connect(this, SIGNAL(SignalDesiredState(QString)),
            this, SLOT(SlotDesiredStateEventHandler(QString)));
    connect(this, SIGNAL(SignalCurrentState(QString)),
            this, SLOT(SlotCurrentStateEventHandler(QString)));

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
