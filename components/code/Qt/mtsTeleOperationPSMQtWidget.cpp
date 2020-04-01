/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2020 Johns Hopkins University (JHU), All Rights Reserved.

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

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationPSMQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperationPSMQtWidget, mtsComponent, std::string);

mtsTeleOperationPSMQtWidget::mtsTeleOperationPSMQtWidget(const std::string & componentName, double periodInSeconds):
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
        interfaceRequired->AddFunction("LockRotation", TeleOperation.LockRotation);
        interfaceRequired->AddFunction("LockTranslation", TeleOperation.LockTranslation);
        interfaceRequired->AddFunction("SetAlignMTM", TeleOperation.SetAlignMTM);
        interfaceRequired->AddFunction("GetPositionCartesianMTM", TeleOperation.GetPositionCartesianMTM);
        interfaceRequired->AddFunction("GetPositionCartesianPSM", TeleOperation.GetPositionCartesianPSM);
        interfaceRequired->AddFunction("GetAlignMTM", TeleOperation.GetAlignMTM);
        interfaceRequired->AddFunction("GetAlignOffset", TeleOperation.GetAlignOffset);
        interfaceRequired->AddFunction("GetRegistrationRotation", TeleOperation.GetRegistrationRotation);
        interfaceRequired->AddFunction("period_statistics", TeleOperation.period_statistics);
        // events
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::DesiredStateEventHandler,
                                                this, "DesiredState");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::CurrentStateEventHandler,
                                                this, "CurrentState");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::FollowingEventHandler,
                                                this, "Following");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::ScaleEventHandler,
                                                this, "Scale");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::RotationLockedEventHandler,
                                                this, "RotationLocked");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::TranslationLockedEventHandler,
                                                this, "TranslationLocked");
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationPSMQtWidget::AlignMTMEventHandler,
                                                this, "AlignMTM");
    }
}

void mtsTeleOperationPSMQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsTeleOperationPSMQtWidget::Startup(void)
{
    setupUi();
    startTimer(TimerPeriodInMilliseconds);
    if (!LogEnabled) {
        QMMessage->hide();
    }
    if (!parent()) {
        show();
    }

    bool align;
    if (TeleOperation.GetAlignMTM(align)) {
        emit SignalAlignMTM(align);
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

    // retrieve transformations
    TeleOperation.GetPositionCartesianMTM(PositionMTM);
    QCPGMTMWidget->SetValue(PositionMTM);

    // for PSM, check if the registration rotation is needed
    TeleOperation.GetPositionCartesianPSM(PositionPSM);
    TeleOperation.GetRegistrationRotation(RegistrationRotation);
    if (RegistrationRotation.Equal(vctMatRot3::Identity())) {
        QCPGPSMWidget->SetValue(PositionPSM);
    } else {
        prmPositionCartesianGet registeredPSM;
        registeredPSM.Valid() = PositionPSM.Valid();
        registeredPSM.Timestamp() = PositionPSM.Timestamp();
        registeredPSM.MovingFrame() = PositionPSM.MovingFrame();
        registeredPSM.ReferenceFrame() = "rot * " + PositionPSM.ReferenceFrame();
        RegistrationRotation.ApplyInverseTo(PositionPSM.Position().Rotation(),
                                            registeredPSM.Position().Rotation());
        RegistrationRotation.ApplyInverseTo(PositionPSM.Position().Translation(),
                                            registeredPSM.Position().Translation());
        QCPGPSMWidget->SetValue(registeredPSM);
    }

    // alignment offset
    TeleOperation.GetAlignOffset(AlignOffset);
    QVRAlignOffset->SetValue(AlignOffset);

    TeleOperation.period_statistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
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

void mtsTeleOperationPSMQtWidget::SlotSetAlignMTM(bool align)
{
    TeleOperation.SetAlignMTM(align);
}

void mtsTeleOperationPSMQtWidget::SlotDesiredStateEventHandler(QString state)
{
    QLEDesiredState->setText(state);
}

void mtsTeleOperationPSMQtWidget::SlotCurrentStateEventHandler(QString state)
{
    QLECurrentState->setText(state);
}

void mtsTeleOperationPSMQtWidget::SlotFollowingEventHandler(bool following)
{
    if (following) {
        QLEFollowing->setText("FOLLOWING");
    } else {
        QLEFollowing->setText("INDEPENDANT");
    }
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

void mtsTeleOperationPSMQtWidget::SlotAlignMTMEventHandler(bool align)
{
    QCBAlignMTM->setChecked(align);
}

void mtsTeleOperationPSMQtWidget::setupUi(void)
{
    // main layout
    QVBoxLayout * mainLayout = new QVBoxLayout();
    setLayout(mainLayout);

    // instructions
    QLabel * instructionsLabel = new QLabel("To start tele-operation you must first insert the tool past the cannula tip (push tool clutch button and manually insert tool).\nOperator must be present to operate (sometime using COAG pedal).\nYou can use the clutch pedal to re-position your MTMs.");
    instructionsLabel->setWordWrap(true);
    mainLayout->addWidget(instructionsLabel);

    // 3D frames
    QGridLayout * frameLayout = new QGridLayout;
    mainLayout->addLayout(frameLayout);
    int column = 0;
    QLabel * mtmLabel = new QLabel("<b>MTM</b>");
    mtmLabel->setAlignment(Qt::AlignCenter);
    frameLayout->addWidget(mtmLabel, 0, column);
    QCPGMTMWidget = new prmPositionCartesianGetQtWidget();
    frameLayout->addWidget(QCPGMTMWidget, 1, column);
    column++;
    QLabel * psmLabel = new QLabel("<b>PSM</b>");
    psmLabel->setAlignment(Qt::AlignCenter);
    frameLayout->addWidget(psmLabel, 0, column);
    QCPGPSMWidget = new prmPositionCartesianGetQtWidget();
    frameLayout->addWidget(QCPGPSMWidget, 1, column);
    column++;
    QLabel * alignLabel = new QLabel("<b>Alignment</b>");
    alignLabel->setAlignment(Qt::AlignCenter);
    frameLayout->addWidget(alignLabel, 0, column);
    QVRAlignOffset = new vctQtWidgetRotationDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);
    frameLayout->addWidget(QVRAlignOffset, 1, column);

    // scale/lock/unlock/messages
    QHBoxLayout * buttonsLayout = new QHBoxLayout;
    mainLayout->addLayout(buttonsLayout);

    // scale
    QSBScale = new QDoubleSpinBox();
    QSBScale->setRange(0.1, 1.0);
    QSBScale->setSingleStep(0.1);
    QSBScale->setPrefix("scale ");
    QSBScale->setValue(mtsIntuitiveResearchKit::TeleOperationPSM::Scale);
    buttonsLayout->addWidget(QSBScale);

    // enable/disable rotation/translation
    QCBLockRotation = new QCheckBox("Lock Rotation");
    buttonsLayout->addWidget(QCBLockRotation);

    QCBLockTranslation = new QCheckBox("Lock Translation");
    buttonsLayout->addWidget(QCBLockTranslation);

    // align MTM
    QCBAlignMTM = new QCheckBox("Align MTM");
    QCBAlignMTM->setChecked(true);
    buttonsLayout->addWidget(QCBAlignMTM);

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

    connect(QCBLockRotation, SIGNAL(clicked(bool)),
            this, SLOT(SlotLockRotation(bool)));
    connect(this, SIGNAL(SignalRotationLocked(bool)),
            this, SLOT(SlotRotationLockedEventHandler(bool)));

    connect(QCBLockTranslation, SIGNAL(clicked(bool)),
            this, SLOT(SlotLockTranslation(bool)));
    connect(this, SIGNAL(SignalTranslationLocked(bool)),
            this, SLOT(SlotTranslationLockedEventHandler(bool)));

    connect(QCBAlignMTM, SIGNAL(clicked(bool)),
            this, SLOT(SlotSetAlignMTM(bool)));
    connect(this, SIGNAL(SignalAlignMTM(bool)),
            this, SLOT(SlotAlignMTMEventHandler(bool)));

    // messages
    connect(QPBLog, SIGNAL(clicked()),
            this, SLOT(SlotLogEnabled()));
}

void mtsTeleOperationPSMQtWidget::DesiredStateEventHandler(const std::string & state)
{
    emit SignalDesiredState(QString(state.c_str()));
}

void mtsTeleOperationPSMQtWidget::CurrentStateEventHandler(const std::string & state)
{
    emit SignalCurrentState(QString(state.c_str()));
}

void mtsTeleOperationPSMQtWidget::FollowingEventHandler(const bool & following)
{
    emit SignalFollowing(following);
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

void mtsTeleOperationPSMQtWidget::AlignMTMEventHandler(const bool & align)
{
    emit SignalAlignMTM(align);
}

void mtsTeleOperationPSMQtWidget::SlotLogEnabled(void)
{
    LogEnabled = QPBLog->isChecked();
    if (LogEnabled) {
        QMMessage->show();
    } else {
        QMMessage->hide();
    }
}
