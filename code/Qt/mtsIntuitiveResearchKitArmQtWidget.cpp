/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsTeleOperationQtWidget.cpp 4423 2013-08-21 16:43:57Z adeguet1 $

  Author(s):  Anton Deguet
  Created on: 2013-08-24

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

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

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmQtWidget.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitArmQtWidget, mtsComponent, std::string);

mtsIntuitiveResearchKitArmQtWidget::mtsIntuitiveResearchKitArmQtWidget(const std::string & componentName)
    :mtsComponent(componentName)
{
    // Setup CISST Interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Manipulator");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetPositionCartesian", Arm.GetPositionCartesian);
        interfaceRequired->AddFunction("GetPeriodStatistics", Arm.GetPeriodStatistics);
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitArmQtWidget::ErrorMessageEventHandler,
                                                this, "RobotErrorMsg");
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitArmQtWidget::StatusMessageEventHandler,
                                                this, "RobotStatusMsg");
    }
    setupUi();
    startTimer(50); // ms
}

void mtsIntuitiveResearchKitArmQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsIntuitiveResearchKitArmQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsIntuitiveResearchKitManipulatorQtWidget::Startup" << std::endl;
    if (!parent()) {
        show();
    }
}

void mtsIntuitiveResearchKitArmQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsIntuitiveResearchKitManipulatorQtWidget::Cleanup" << std::endl;
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
    mtsExecutionResult executionResult;
    executionResult = Arm.GetPositionCartesian(Position);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "Manipulator.GetPositionCartesian failed, \""
                                << executionResult << "\"" << std::endl;
    }
    QFRPositionWidget->SetValue(Position.Position());

    Arm.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}

void mtsIntuitiveResearchKitArmQtWidget::SlotTextChanged(void)
{
    QTEMessages->verticalScrollBar()->setSliderPosition(QTEMessages->verticalScrollBar()->maximum());
}

void mtsIntuitiveResearchKitArmQtWidget::setupUi(void)
{
    QVBoxLayout * mainLayout = new QVBoxLayout;

    QHBoxLayout * topLayout = new QHBoxLayout;
    mainLayout->addLayout(topLayout);

    // 3D position
    QFRPositionWidget = new vctQtWidgetFrameDoubleRead(vctQtWidgetRotationDoubleRead::OPENGL_WIDGET);
    topLayout->addWidget(QFRPositionWidget, 0, 0);

    // Timing
    QVBoxLayout * timingLayout = new QVBoxLayout();
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    timingLayout->addWidget(QMIntervalStatistics);
    timingLayout->addStretch();
    topLayout->addLayout(timingLayout);

    // Messages
    QTEMessages = new QTextEdit();
    QTEMessages->setReadOnly(true);
    QTEMessages->ensureCursorVisible();
    mainLayout->addWidget(QTEMessages);

    setLayout(mainLayout);
    setWindowTitle("Manipulator");
    resize(sizeHint());

    connect(this, SIGNAL(SignalAppendMessage(QString)),
            QTEMessages, SLOT(append(QString)));
    connect(this, SIGNAL(SignalSetColor(QColor)),
            QTEMessages, SLOT(setTextColor(QColor)));
    connect(QTEMessages, SIGNAL(textChanged()),
            this, SLOT(SlotTextChanged()));
}

void mtsIntuitiveResearchKitArmQtWidget::ErrorMessageEventHandler(const std::string & message)
{
    emit SignalSetColor(QColor("red"));
    emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Error: ") + QString(message.c_str()));
}

void mtsIntuitiveResearchKitArmQtWidget::StatusMessageEventHandler(const std::string & message)
{
    emit SignalSetColor(QColor("black"));
    emit SignalAppendMessage(QTime::currentTime().toString("hh:mm:ss") + QString(" Status: ") + QString(message.c_str()));
}
