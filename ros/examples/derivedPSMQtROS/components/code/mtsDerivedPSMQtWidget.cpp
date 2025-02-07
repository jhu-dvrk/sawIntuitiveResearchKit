/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2021-11-29

  (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <QPushButton>
#include <QHBoxLayout>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <mtsDerivedPSMQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsDerivedPSMQtWidget, mtsComponent, mtsComponentConstructorArg);

mtsDerivedPSMQtWidget::mtsDerivedPSMQtWidget(const std::string & componentName):
    mtsComponent(componentName)
{
    Init();
}

mtsDerivedPSMQtWidget::mtsDerivedPSMQtWidget(const mtsComponentConstructorArg & arg):
    mtsComponent(arg.Name)
{
    Init();
}

void mtsDerivedPSMQtWidget::Init(void)
{
    // Qt timers are in milliseconds, 50 ms
    TimerPeriodInMilliseconds = 0.05;

    // setup cisst interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("NewInterface");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("set_gain", set_gain);
        interfaceRequired->AddFunction("activate", activate);
        // events
        interfaceRequired->AddEventHandlerWrite(&mtsDerivedPSMQtWidget::GainEventHandler,
                                                this, "gain");
        interfaceRequired->AddEventHandlerWrite(&mtsDerivedPSMQtWidget::ActivatedEventHandler,
                                                this, "activated");
    }
}

void mtsDerivedPSMQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsDerivedPSMQtWidget::Startup(void)
{
    setupUi();
    startTimer(TimerPeriodInMilliseconds);
    if (!parent()) {
        show();
    }
}

void mtsDerivedPSMQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsDerivedPSMQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    // this is a placeholder, the current example is event based, nothing to do here
}

void mtsDerivedPSMQtWidget::SlotSetGain(double _gain)
{
    set_gain(_gain);
}

void mtsDerivedPSMQtWidget::SlotActivate(bool _activate)
{
    activate(_activate);
}

void mtsDerivedPSMQtWidget::SlotGainEventHandler(double _gain)
{
    QSBGain->setValue(_gain);
}

void mtsDerivedPSMQtWidget::SlotActivatedEventHandler(bool _activated)
{
    QCBActivate->setChecked(_activated);
}

void mtsDerivedPSMQtWidget::setupUi(void)
{
    // main layout
    QVBoxLayout * mainLayout = new QVBoxLayout();
    setLayout(mainLayout);

    // gain
    QSBGain = new QDoubleSpinBox();
    QSBGain->setRange(0.0, 1.0);
    QSBGain->setSingleStep(0.05);
    QSBGain->setPrefix("gain ");
    QSBGain->setValue(0.5);
    mainLayout->addWidget(QSBGain);

    // enable/disable rotation/translation
    QCBActivate = new QCheckBox("Activate");
    mainLayout->addWidget(QCBActivate);

    setWindowTitle("New Controller");
    resize(sizeHint());

    // setup Qt Connection
    connect(QSBGain, SIGNAL(valueChanged(double)),
            this, SLOT(SlotSetGain(double)));
    connect(this, SIGNAL(SignalGain(double)),
            this, SLOT(SlotGainEventHandler(double)));

    connect(QCBActivate, SIGNAL(clicked(bool)),
            this, SLOT(SlotActivate(bool)));
    connect(this, SIGNAL(SignalActivated(bool)),
            this, SLOT(SlotActivatedEventHandler(bool)));
}

void mtsDerivedPSMQtWidget::GainEventHandler(const double & gain)
{
    emit SignalGain(gain);
}

void mtsDerivedPSMQtWidget::ActivatedEventHandler(const bool & _activated)
{
    emit SignalActivated(_activated);
}
