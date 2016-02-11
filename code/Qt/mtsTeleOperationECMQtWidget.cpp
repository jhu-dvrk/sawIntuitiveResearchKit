/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

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
#include <cisstParameterTypes/prmPositionJointSet.h>

#include <sawIntuitiveResearchKit/mtsTeleOperationECMQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperationECMQtWidget, mtsComponent, std::string);

mtsTeleOperationECMQtWidget::mtsTeleOperationECMQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds * 1000) // Qt timers are in milliseconds
{
    // Setup cisst interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("TeleOperation");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("Enable", TeleOperation.Enable);
        interfaceRequired->AddFunction("SetScale", TeleOperation.SetScale);
        interfaceRequired->AddFunction("GetPositionCartesianMasterLeft",
                                       TeleOperation.GetPositionCartesianMasterLeft);
        interfaceRequired->AddFunction("GetPositionCartesianMasterRight",
                                       TeleOperation.GetPositionCartesianMasterRight);
        interfaceRequired->AddFunction("GetPositionCartesianSlave",
                                       TeleOperation.GetPositionCartesianSlave);
        interfaceRequired->AddFunction("GetRegistrationRotation",
                                       TeleOperation.GetRegistrationRotation);
        interfaceRequired->AddFunction("GetPeriodStatistics", TeleOperation.GetPeriodStatistics);
        // events
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECMQtWidget::EnableEventHandler,
                                                this, "Enabled");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECMQtWidget::ScaleEventHandler,
                                                this, "Scale");
        // messages
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECMQtWidget::ErrorEventHandler,
                                                this, "Error");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECMQtWidget::WarningEventHandler,
                                                this, "Warning");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECMQtWidget::StatusEventHandler,
                                                this, "Status");
    }
    setupUi();
    startTimer(TimerPeriodInMilliseconds);
}

void mtsTeleOperationECMQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsTeleOperationECMQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
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

    mtsExecutionResult executionResult;
#if 0
    executionResult = TeleOperation.GetPositionCartesianMasterLeft(PositionMaster);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "TeleOperation.GetPositionCartesianMasterLeft failed, \""
                                << executionResult << "\"" << std::endl;
    }
    executionResult = TeleOperation.GetPositionCartesianSlave(PositionSlave);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "TeleOperation.GetPositionCartesianSlave failed, \""
                                << executionResult << "\"" << std::endl;
    }
    executionResult = TeleOperation.GetRegistrationRotation(RegistrationRotation);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "TeleOperation.GetRegistrationRotation failed, \""
                                << executionResult << "\"" << std::endl;
    }
    // apply registration orientation
    vctFrm3 registeredSlave;
    RegistrationRotation.ApplyInverseTo(PositionSlave.Position().Rotation(), registeredSlave.Rotation());
    RegistrationRotation.ApplyInverseTo(PositionSlave.Position().Translation(), registeredSlave.Translation());

    QFRPositionMasterWidget->SetValue(PositionMaster.Position());
    QFRPositionSlaveWidget->SetValue(registeredSlave);
#endif
    TeleOperation.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}


void mtsTeleOperationECMQtWidget::SlotEnable(bool state)
{
    TeleOperation.Enable(mtsBool(state));
}

void mtsTeleOperationECMQtWidget::SlotSetScale(double scale)
{
    TeleOperation.SetScale(scale);
}

void mtsTeleOperationECMQtWidget::SlotEnableEventHandler(bool state)
{
    QCBEnable->setChecked(state);
}

void mtsTeleOperationECMQtWidget::SlotScaleEventHandler(double scale)
{
    QSBScale->setValue(scale);
}

void mtsTeleOperationECMQtWidget::setupUi(void)
{
    QFont font;
    font.setBold(true);
    font.setPointSize(12);

    QGridLayout * cmdTitleLayout = new QGridLayout;
    QSpacerItem * cmdTitleLeftSpacer = new QSpacerItem(341, 20, QSizePolicy::Expanding);
    QSpacerItem * cmdTitleRightSpacer = new QSpacerItem(341, 20, QSizePolicy::Expanding);
    cmdTitleLayout->addItem(cmdTitleLeftSpacer, 0, 0);
    cmdTitleLayout->addItem(cmdTitleRightSpacer, 0, 2);

    QFrame * cmdTitleLeftLine = new QFrame;
    cmdTitleLeftLine->setFrameShape(QFrame::HLine);
    cmdTitleLeftLine->setFrameShadow(QFrame::Sunken);
    QFrame * cmdTitleRightLine = new QFrame;
    cmdTitleRightLine->setFrameShape(QFrame::HLine);
    cmdTitleRightLine->setFrameShadow(QFrame::Sunken);
    QLabel * cmdTitleLabel = new QLabel("TeleOperation Controller");
    cmdTitleLabel->setFont(font);
    cmdTitleLabel->setAlignment(Qt::AlignCenter);

    cmdTitleLayout->addWidget(cmdTitleLeftLine, 1, 0);
    cmdTitleLayout->addWidget(cmdTitleLabel, 1, 1);
    cmdTitleLayout->addWidget(cmdTitleRightLine, 1, 2);

    QGridLayout * frameLayout = new QGridLayout;
    QLabel * masterLabel = new QLabel("<b>Master Left</b>");
    masterLabel->setAlignment(Qt::AlignCenter);
    frameLayout->addWidget(masterLabel, 0, 0);
    QFRPositionMasterLeftWidget = new vctQtWidgetFrameDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);
    frameLayout->addWidget(QFRPositionMasterLeftWidget, 1, 0);
    masterLabel = new QLabel("<b>Master Right</b>");
    masterLabel->setAlignment(Qt::AlignCenter);
    frameLayout->addWidget(masterLabel, 0, 1);
    QFRPositionMasterRightWidget = new vctQtWidgetFrameDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);
    frameLayout->addWidget(QFRPositionMasterRightWidget, 1, 1);
    QLabel * slaveLabel = new QLabel("<b>Slave</b>");
    slaveLabel->setAlignment(Qt::AlignCenter);
    frameLayout->addWidget(slaveLabel, 2, 0);
    QFRPositionSlaveWidget = new vctQtWidgetFrameDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);
    frameLayout->addWidget(QFRPositionSlaveWidget, 3, 0);


    QVBoxLayout * controlLayout = new QVBoxLayout;

    QLabel * instructionsLabel = new QLabel("To start tele-operation you must first insert the tool past the cannula tip (push tool clutch button and manually insert tool).\nYou must keep your right foot on the COAG/MONO pedal to operate.\nYou can use the clutch pedal to re-position your masters.");
    controlLayout->addWidget(instructionsLabel);

    QHBoxLayout * buttonsLayout = new QHBoxLayout;
    controlLayout->addLayout(buttonsLayout);

    // enable/disable teleoperation
    QCBEnable = new QCheckBox("Enable");
    buttonsLayout->addWidget(QCBEnable);

    // scale
    QSBScale = new QDoubleSpinBox();
    QSBScale->setRange(0.1, 1.0);
    QSBScale->setSingleStep(0.1);
    QSBScale->setPrefix("scale ");
    QSBScale->setValue(0.2);
    controlLayout->addWidget(QSBScale);

    // Timing
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    controlLayout->addWidget(QMIntervalStatistics);

    // messages
    QTEMessages = new QTextEdit();
    QTEMessages->setReadOnly(true);
    QTEMessages->ensureCursorVisible();
    controlLayout->addWidget(QTEMessages);

    QHBoxLayout * mainLayout = new QHBoxLayout;
    mainLayout->addLayout(frameLayout);
    mainLayout->addLayout(controlLayout);

    setLayout(mainLayout);

    setWindowTitle("TeleOperation Controller");
    resize(sizeHint());

    // setup Qt Connection
    connect(QCBEnable, SIGNAL(clicked(bool)),
            this, SLOT(SlotEnable(bool)));
    connect(this, SIGNAL(SignalEnable(bool)),
            this, SLOT(SlotEnableEventHandler(bool)));

    connect(QSBScale, SIGNAL(valueChanged(double)),
            this, SLOT(SlotSetScale(double)));
    connect(this, SIGNAL(SignalScale(double)),
            this, SLOT(SlotScaleEventHandler(double)));

    // messages
    connect(this, SIGNAL(SignalAppendMessage(QString)),
            QTEMessages, SLOT(append(QString)));
    connect(this, SIGNAL(SignalSetColor(QColor)),
            QTEMessages, SLOT(setTextColor(QColor)));
    connect(QTEMessages, SIGNAL(textChanged()),
            this, SLOT(SlotTextChanged()));
}

void mtsTeleOperationECMQtWidget::EnableEventHandler(const bool & enable)
{
    emit SignalEnable(enable);
}

void mtsTeleOperationECMQtWidget::ScaleEventHandler(const double & scale)
{
    emit SignalScale(scale);
}

void mtsTeleOperationECMQtWidget::SlotTextChanged(void)
{
    QTEMessages->verticalScrollBar()->setSliderPosition(QTEMessages->verticalScrollBar()->maximum());
}

void mtsTeleOperationECMQtWidget::ErrorEventHandler(const std::string & message)
{
    emit SignalSetColor(QColor("red"));
    emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Error: ") + QString(message.c_str()));
}

void mtsTeleOperationECMQtWidget::WarningEventHandler(const std::string & message)
{
    emit SignalSetColor(QColor("darkRed"));
    emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Warning: ") + QString(message.c_str()));
}

void mtsTeleOperationECMQtWidget::StatusEventHandler(const std::string & message)
{
    emit SignalSetColor(QColor("black"));
    emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Status: ") + QString(message.c_str()));
}
