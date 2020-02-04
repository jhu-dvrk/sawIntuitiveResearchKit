/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2019-10-10

  (C) Copyright 2019-2020 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <QComboBox>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitECMQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitEndoscopeTypes.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitECMQtWidget, mtsComponent, std::string);

mtsIntuitiveResearchKitECMQtWidget::mtsIntuitiveResearchKitECMQtWidget(const std::string & componentName, double periodInSeconds):
    mtsIntuitiveResearchKitArmQtWidget(componentName, periodInSeconds)
{
    CMN_ASSERT(InterfaceRequired);
    InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitECMQtWidget::EndoscopeTypeEventHandler,
                                            this, "EndoscopeType");
    InterfaceRequired->AddFunction("SetEndoscopeType", SetEndoscopeType);
}

void mtsIntuitiveResearchKitECMQtWidget::setupUiDerived(void)
{
    QHBoxLayout * endoscopeLayout = new QHBoxLayout;
    MainLayout->addLayout(endoscopeLayout);

    // status
    QLabel * label = new QLabel("Endoscope type");
    endoscopeLayout->addWidget(label);
    QLEEndoscopeType = new QLineEdit("");
    QLEEndoscopeType->setReadOnly(true);
    endoscopeLayout->addWidget(QLEEndoscopeType);

    // set endoscope type
    label = new QLabel("Set endoscope type");
    endoscopeLayout->addWidget(label);
    QCBEndoscopeOptions = new QComboBox();
    auto iter = mtsIntuitiveResearchKitEndoscopeTypes::TypeVectorString().begin();
    auto end = mtsIntuitiveResearchKitEndoscopeTypes::TypeVectorString().end();
    for (; iter != end; ++iter) {
        if (*iter != "ERROR") {
            QCBEndoscopeOptions->addItem((*iter).c_str());
        }
    }
    endoscopeLayout->addWidget(QCBEndoscopeOptions);
    endoscopeLayout->addStretch();

    // setup Qt Connection
    connect(this, SIGNAL(SignalEndoscopeType(QString)),
            this, SLOT(SlotEndoscopeTypeEventHandler(QString)));
    connect(QCBEndoscopeOptions, SIGNAL(activated(QString)),
            this, SLOT(SlotEndoscopeTypeSelected(QString)));
}

void mtsIntuitiveResearchKitECMQtWidget::SlotEndoscopeTypeEventHandler(QString endoscopeType)
{
    QLEEndoscopeType->setText(endoscopeType);
}

void mtsIntuitiveResearchKitECMQtWidget::SlotEndoscopeTypeSelected(QString endoscopeType)
{
    std::string message = "Please confirm that the endoscope inserted matches: " + endoscopeType.toStdString();
    int answer = QMessageBox::warning(this, "mtsIntuitiveResearchKitECMQtWidget",
                                      message.c_str(),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        SetEndoscopeType(endoscopeType.toStdString());
    }
}

void mtsIntuitiveResearchKitECMQtWidget::EndoscopeTypeEventHandler(const std::string & endoscopeType)
{
    emit SignalEndoscopeType(QString(endoscopeType.c_str()));
}
