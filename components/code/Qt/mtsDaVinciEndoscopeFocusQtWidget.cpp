/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2020-02-10

  (C) Copyright 2020 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <sawIntuitiveResearchKit/mtsDaVinciEndoscopeFocusQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsDaVinciEndoscopeFocusQtWidget, mtsComponent, std::string);

mtsDaVinciEndoscopeFocusQtWidget::mtsDaVinciEndoscopeFocusQtWidget(const std::string & componentName):
    mtsComponent(componentName)
{
    // Setup cisst interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Endoscope");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("Lock", Endoscope.Lock);
        interfaceRequired->AddFunction("FocusIn", Endoscope.FocusIn);
        interfaceRequired->AddFunction("FocusOut", Endoscope.FocusOut);
        // events
        interfaceRequired->AddEventHandlerWrite(&mtsDaVinciEndoscopeFocusQtWidget::LockedEventHandler,
                                                this, "Locked");
        interfaceRequired->AddEventHandlerWrite(&mtsDaVinciEndoscopeFocusQtWidget::FocusingInEventHandler,
                                                this, "FocusingIn");
        interfaceRequired->AddEventHandlerWrite(&mtsDaVinciEndoscopeFocusQtWidget::FocusingOutEventHandler,
                                                this, "FocusingOut");
    }
}

void mtsDaVinciEndoscopeFocusQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsDaVinciEndoscopeFocusQtWidget::Startup(void)
{
    setupUi();
    if (!parent()) {
        show();
    }
}

void mtsDaVinciEndoscopeFocusQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsDaVinciEndoscopeFocusQtWidget::SlotLock(void)
{
    const bool isLocked = QPBLock->isChecked();
    Endoscope.Lock(isLocked);
    UpdateLock(isLocked);
}

void mtsDaVinciEndoscopeFocusQtWidget::UpdateLock(const bool & locked)
{
    if (locked) {
        QPBLock->setText("Unlock");
    } else {
        QPBLock->setText("Lock");
    }
    QPBFocusIn->setEnabled(!locked);
    QPBFocusOut->setEnabled(!locked);
}

void mtsDaVinciEndoscopeFocusQtWidget::SlotFocusIn(bool focus)
{
    Endoscope.FocusIn(focus);
}

void mtsDaVinciEndoscopeFocusQtWidget::SlotFocusOut(bool focus)
{
    Endoscope.FocusOut(focus);
}

void mtsDaVinciEndoscopeFocusQtWidget::SlotLockedEventHandler(bool locked)
{
    QPBLock->setChecked(locked);
    UpdateLock(locked);
}

void mtsDaVinciEndoscopeFocusQtWidget::SlotFocusingInEventHandler(bool focusing)
{
    if (focusing) {
        QLFocusingIn->setText("+");
    } else {
        QLFocusingIn->setText("o");
    }
}

void mtsDaVinciEndoscopeFocusQtWidget::SlotFocusingOutEventHandler(bool focusing)
{
    if (focusing) {
        QLFocusingOut->setText("-");
    } else {
        QLFocusingOut->setText("o");
    }
}

void mtsDaVinciEndoscopeFocusQtWidget::setupUi(void)
{
    // main layout
    QHBoxLayout * mainLayout = new QHBoxLayout();
    setLayout(mainLayout);

    // from left to right, focus in/+, lock, focus out/-
    QPBFocusIn = new QPushButton("Focus In");
    mainLayout->addWidget(QPBFocusIn);

    QLFocusingIn = new QLabel("o");
    QLFocusingIn->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(QLFocusingIn);

    QPBLock = new QPushButton("Lock");
    QPBLock->setCheckable(true);
    mainLayout->addWidget(QPBLock);

    QLFocusingOut = new QLabel("o");
    QLFocusingOut->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(QLFocusingOut);

    QPBFocusOut = new QPushButton("Focus Out");
    mainLayout->addWidget(QPBFocusOut);

    setWindowTitle("da Vinci Endoscope Focus");
    resize(sizeHint());

    // setup Qt Connection
    connect(this, SIGNAL(SignalLocked(bool)),
            this, SLOT(SlotLockedEventHandler(bool)));
    connect(this, SIGNAL(SignalFocusingIn(bool)),
            this, SLOT(SlotFocusingInEventHandler(bool)));
    connect(this, SIGNAL(SignalFocusingOut(bool)),
            this, SLOT(SlotFocusingOutEventHandler(bool)));

    connect(QPBLock, SIGNAL(clicked()),
            this, SLOT(SlotLock()));
    connect(QPBFocusIn, &QPushButton::pressed,
            [ = ] { emit SlotFocusIn(true); });
    connect(QPBFocusIn, &QPushButton::released,
            [ = ] { emit SlotFocusIn(false); });
    connect(QPBFocusOut, &QPushButton::pressed,
            [ = ] { emit SlotFocusOut(true); });
    connect(QPBFocusOut, &QPushButton::released,
            [ = ] { emit SlotFocusOut(false); });
}

void mtsDaVinciEndoscopeFocusQtWidget::LockedEventHandler(const bool & locked)
{
    emit SignalLocked(locked);
}

void mtsDaVinciEndoscopeFocusQtWidget::FocusingInEventHandler(const bool & focusing)
{
    emit SignalFocusingIn(focusing);
}

void mtsDaVinciEndoscopeFocusQtWidget::FocusingOutEventHandler(const bool & focusing)
{
    emit SignalFocusingOut(focusing);
}
