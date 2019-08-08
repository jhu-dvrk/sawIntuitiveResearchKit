/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2019-08-07

  (C) Copyright 2019 Johns Hopkins University (JHU), All Rights Reserved.

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

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSMQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitToolTypes.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitPSMQtWidget, mtsComponent, std::string);

mtsIntuitiveResearchKitPSMQtWidget::mtsIntuitiveResearchKitPSMQtWidget(const std::string & componentName, double periodInSeconds):
    mtsIntuitiveResearchKitArmQtWidget(componentName, periodInSeconds)
{
    CMN_ASSERT(InterfaceRequired);
    InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSMQtWidget::ToolTypeEventHandler,
                                            this, "ToolType");
    InterfaceRequired->AddEventHandlerVoid(&mtsIntuitiveResearchKitPSMQtWidget::ToolTypeRequestEventHandler,
                                           this, "ToolTypeRequest");
    InterfaceRequired->AddFunction("SetToolType", SetToolType);
}

void mtsIntuitiveResearchKitPSMQtWidget::setupUiDerived(void)
{
    QHBoxLayout * toolLayout = new QHBoxLayout;
    MainLayout->addLayout(toolLayout);

    // status
    QLabel * label = new QLabel("Tool type");
    toolLayout->addWidget(label);
    QLEToolType = new QLineEdit("");
    QLEToolType->setReadOnly(true);
    toolLayout->addWidget(QLEToolType);

    // set tool type
    label = new QLabel("Set tool type");
    toolLayout->addWidget(label);
    QCBToolOptions = new QComboBox();
    auto iter = mtsIntuitiveResearchKitToolTypes::TypeVectorString().begin();
    auto end = mtsIntuitiveResearchKitToolTypes::TypeVectorString().end();
    for (; iter != end; ++iter) {
        QCBToolOptions->addItem((*iter).c_str());
    }
    QCBToolOptions->setEnabled(false);
    toolLayout->addWidget(QCBToolOptions);
    

    toolLayout->addStretch();

    // setup Qt Connection
    connect(this, SIGNAL(SignalToolType(QString)),
            this, SLOT(SlotToolTypeEventHandler(QString)));
    connect(this, SIGNAL(SignalToolTypeRequest(void)),
            this, SLOT(SlotToolTypeRequestEventHandler(void)));
    connect(QCBToolOptions, SIGNAL(activated(QString)),
            this, SLOT(SlotToolTypeSelected(QString)));
        
}

void mtsIntuitiveResearchKitPSMQtWidget::SlotToolTypeEventHandler(QString toolType)
{
    QPalette palette;
    palette.setColor(QPalette::Base, Qt::white);
    QLEToolType->setPalette(palette);
    QLEToolType->setText(toolType);
}

void mtsIntuitiveResearchKitPSMQtWidget::SlotToolTypeRequestEventHandler(void)
{
    QPalette palette;
    palette.setColor(QPalette::Base, QColor(255, 100, 100));
    QLEToolType->setPalette(palette);
    QLEToolType->setText("Please select a tool type");
    QCBToolOptions->setEnabled(true);
}

void mtsIntuitiveResearchKitPSMQtWidget::SlotToolTypeSelected(QString toolType)
{
    std::string message = "Please confirm that the tool inserted matches: " + toolType.toStdString();
    int answer = QMessageBox::warning(this, "mtsIntuitiveResearchKitPSMQtWidget",
                                      message.c_str(),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        SetToolType(toolType.toStdString());
    }
}

void mtsIntuitiveResearchKitPSMQtWidget::ToolTypeEventHandler(const std::string & toolType)
{
    emit SignalToolType(QString(toolType.c_str()));
}

void mtsIntuitiveResearchKitPSMQtWidget::ToolTypeRequestEventHandler(void)
{
    emit SignalToolTypeRequest();
}
