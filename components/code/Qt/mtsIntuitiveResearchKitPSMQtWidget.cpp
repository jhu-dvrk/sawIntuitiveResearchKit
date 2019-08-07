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
#include <QCloseEvent>
#include <QCoreApplication>
#include <QPushButton>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmPositionJointGet.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSMQtWidget.h>


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitPSMQtWidget, mtsComponent, std::string);

mtsIntuitiveResearchKitPSMQtWidget::mtsIntuitiveResearchKitPSMQtWidget(const std::string & componentName, double periodInSeconds):
    mtsIntuitiveResearchKitArmQtWidget(componentName, periodInSeconds)
{
    CMN_ASSERT(InterfaceRequired);
    InterfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitPSMQtWidget::ToolTypeEventHandler,
                                            this, "ToolType");
}

void mtsIntuitiveResearchKitPSMQtWidget::setupUiDerived(void)
{
    QHBoxLayout * toolLayout = new QHBoxLayout;
    MainLayout->addLayout(toolLayout);
    QLabel * label = new QLabel("Tool type");
    toolLayout->addWidget(label);
    QLEToolType = new QLineEdit("");
    QLEToolType->setReadOnly(true);
    toolLayout->addWidget(QLEToolType);
    toolLayout->addStretch();
    
    // setup Qt Connection
    connect(this, SIGNAL(SignalToolType(QString)),
            this, SLOT(SlotToolTypeEventHandler(QString)));
}

void mtsIntuitiveResearchKitPSMQtWidget::SlotToolTypeEventHandler(QString toolType)
{
    QLEToolType->setText(toolType);
}

void mtsIntuitiveResearchKitPSMQtWidget::ToolTypeEventHandler(const std::string & toolType)
{
    emit SignalToolType(QString(toolType.c_str()));
}
