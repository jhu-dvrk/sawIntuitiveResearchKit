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

#include <sawIntuitiveResearchKit/mtsTeleOperationPSMQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperationPSMQtWidget, mtsComponent, std::string);

mtsTeleOperationPSMQtWidget::mtsTeleOperationPSMQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds * 1000) // Qt timers are in milliseconds
{
    // Setup cisst interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("TeleOperation");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("Enable", TeleOperation.Enable);
        interfaceRequired->AddFunction("SetScale", TeleOperation.SetScale);
        interfaceRequired->AddFunction("LockRotation", TeleOperation.LockRotation);
        interfaceRequired->AddFunction("LockTranslation", TeleOperation.LockTranslation);
        interfaceRequired->AddFunction("GetPositionCartesianMaster", TeleOperation.GetPositionCartesianMaster);
        interfaceRequired->AddFunction("GetPositionCartesianSlave", TeleOperation.GetPositionCartesianSlave);
        interfaceRequired->AddFunction("GetRegistrationRotation", TeleOperation.GetRegistrationRotation);
        interfaceRequired->AddFunction("GetPeriodStatistics", TeleOperation.GetPeriodStatistics);
        // events
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::EnableEventHandler,
                                                this, "Enabled");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::ScaleEventHandler,
                                                this, "Scale");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::RotationLockedEventHandler,
                                                this, "RotationLocked");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::TranslationLockedEventHandler,
                                                this, "TranslationLocked");
        // messages
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::ErrorEventHandler,
                                                this, "Error");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::WarningEventHandler,
                                                this, "Warning");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::StatusEventHandler,
                                                this, "Status");
    }
    setupUi();
    startTimer(TimerPeriodInMilliseconds);
}

void mtsTeleOperationPSMQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsTeleOperationPSMQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
    if (!parent()) {
        show();
    }
}

void mtsTeleOperationPSMQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsTeleOperationPSMQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsTeleOperationPSMQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsTeleOperationPSMQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    mtsExecutionResult executionResult;
    executionResult = TeleOperation.GetPositionCartesianMaster(PositionMaster);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "TeleOperation.GetPositionCartesianMaster failed, \""
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

    TeleOperation.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}


void mtsTeleOperationPSMQtWidget::SlotEnable(bool state)
{
    TeleOperation.Enable(mtsBool(state));
}

void mtsTeleOperationPSMQtWidget::SlotSetScale(double scale)
{
    TeleOperation.SetScale(scale);
}

void mtsTeleOperationPSMQtWidget::SlotLockRotation(bool lock)
{
    TeleOperation.LockRotation(lock);
}

void mtsTeleOperationPSMQtWidget::SlotLockTranslation(bool lock)
{
    TeleOperation.LockTranslation(lock);
}


void mtsTeleOperationPSMQtWidget::SlotEnableEventHandler(bool state)
{
    QCBEnable->setChecked(state);
}

void mtsTeleOperationPSMQtWidget::SlotScaleEventHandler(double scale)
{
    QSBScale->setValue(scale);
}

void mtsTeleOperationPSMQtWidget::SlotRotationLockedEventHandler(bool lock)
{
    QCBLockRotation->setChecked(lock);
}

void mtsTeleOperationPSMQtWidget::SlotTranslationLockedEventHandler(bool lock)
{
    QCBLockTranslation->setChecked(lock);
}

void mtsTeleOperationPSMQtWidget::setupUi(void)
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
    QLabel * masterLabel = new QLabel("<b>Master</b>");
    masterLabel->setAlignment(Qt::AlignCenter);
    frameLayout->addWidget(masterLabel, 0, 0);
    QFRPositionMasterWidget = new vctQtWidgetFrameDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);
    frameLayout->addWidget(QFRPositionMasterWidget, 1, 0);
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

    QCBLockRotation = new QCheckBox("Lock Rotation");
    buttonsLayout->addWidget(QCBLockRotation);

    QCBLockTranslation = new QCheckBox("Lock Translation");
    buttonsLayout->addWidget(QCBLockTranslation);

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

    connect(QCBLockRotation, SIGNAL(clicked(bool)),
            this, SLOT(SlotLockRotation(bool)));
    connect(this, SIGNAL(SignalRotationLocked(bool)),
            this, SLOT(SlotRotationLockedEventHandler(bool)));

    connect(QCBLockTranslation, SIGNAL(clicked(bool)),
            this, SLOT(SlotLockTranslation(bool)));
    connect(this, SIGNAL(SignalTranslationLocked(bool)),
            this, SLOT(SlotTranslationLockedEventHandler(bool)));

    // messages
    connect(this, SIGNAL(SignalAppendMessage(QString)),
            QTEMessages, SLOT(append(QString)));
    connect(this, SIGNAL(SignalSetColor(QColor)),
            QTEMessages, SLOT(setTextColor(QColor)));
    connect(QTEMessages, SIGNAL(textChanged()),
            this, SLOT(SlotTextChanged()));
}

void mtsTeleOperationPSMQtWidget::EnableEventHandler(const bool & enable)
{
    emit SignalEnable(enable);
}

void mtsTeleOperationPSMQtWidget::ScaleEventHandler(const double & scale)
{
    emit SignalScale(scale);
}

void mtsTeleOperationPSMQtWidget::RotationLockedEventHandler(const bool & lock)
{
    emit SignalRotationLocked(lock);
}

void mtsTeleOperationPSMQtWidget::TranslationLockedEventHandler(const bool & lock)
{
    emit SignalTranslationLocked(lock);
}

void mtsTeleOperationPSMQtWidget::SlotTextChanged(void)
{
    QTEMessages->verticalScrollBar()->setSliderPosition(QTEMessages->verticalScrollBar()->maximum());
}

void mtsTeleOperationPSMQtWidget::ErrorEventHandler(const std::string & message)
{
    emit SignalSetColor(QColor("red"));
    emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Error: ") + QString(message.c_str()));
}

void mtsTeleOperationPSMQtWidget::WarningEventHandler(const std::string & message)
{
    emit SignalSetColor(QColor("darkRed"));
    emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Warning: ") + QString(message.c_str()));
}

void mtsTeleOperationPSMQtWidget::StatusEventHandler(const std::string & message)
{
    emit SignalSetColor(QColor("black"));
    emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Status: ") + QString(message.c_str()));
}
