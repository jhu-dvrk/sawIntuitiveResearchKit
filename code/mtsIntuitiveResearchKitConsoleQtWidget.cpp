/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id: mtsIntuitiveResearchKitConsoleQtWidget.cpp 4138 2013-04-27 21:48:29Z adeguet1 $

  Author(s):  Anton Deguet
  Created on: 2013-05-17

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQtWidget.h>

CMN_IMPLEMENT_SERVICES(mtsIntuitiveResearchKitConsoleQtWidget);

mtsIntuitiveResearchKitConsoleQtWidget::mtsIntuitiveResearchKitConsoleQtWidget(const std::string & componentName):
    mtsComponent(componentName)
{
    // Setup CISST Interface
#if 0
    mtsInterfaceRequired * requiredInterface = AddInterfaceRequired("TeleOperation");
    if (requiredInterface) {
        requiredInterface->AddFunction("Enable", TeleOperation.Enable);
    }
#endif

    mtsInterfaceRequired * interfaceRequirePSM = AddInterfaceRequired("PSM");
    if (interfaceRequirePSM) {
        interfaceRequirePSM->AddFunction("SetRobotControlState", PSM.SetRobotControlState);
        interfaceRequirePSM->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::StateMsgEventHandlerMaster,
                                                  this, "RobotStatusMsg");
        interfaceRequirePSM->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::ErrorMsgEventHandlerMaster,
                                                  this, "RobotErrorMsg");
    }

    mtsInterfaceRequired * interfaceRequiredMTM = AddInterfaceRequired("MTM");
    if (interfaceRequiredMTM) {
        interfaceRequiredMTM->AddFunction("SetRobotControlState", MTM.SetRobotControlState);
        interfaceRequiredMTM->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::StateMsgEventHandlerSlave,
                                                   this, "RobotStatusMsg");
        interfaceRequiredMTM->AddEventHandlerWrite(&mtsIntuitiveResearchKitConsoleQtWidget::ErrorMsgEventHandlerSlave,
                                                   this, "RobotErrorMsg");
    }

    setupUi();
}

void mtsIntuitiveResearchKitConsoleQtWidget::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsIntuitiveResearchKitConsoleQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsIntuitiveResearchKitConsoleQtWidget::Startup" << std::endl;
    if (!parent()) {
        show();
    }
}

void mtsIntuitiveResearchKitConsoleQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsIntuitiveResearchKitConsoleQtWidget::Cleanup" << std::endl;
}

void mtsIntuitiveResearchKitConsoleQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsIntuitiveResearchKitConsoleQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsIntuitiveResearchKitConsoleQtWidget::SlotSetStateButton(QAbstractButton * radioButton)
{
    std::cout << "---- Radio Button " << radioButton->text().toStdString() << std::endl;
    std::string state = radioButton->text().toStdString();
    PSM.SetRobotControlState(mtsStdString(state));
    MTM.SetRobotControlState(mtsStdString(state));
}

void mtsIntuitiveResearchKitConsoleQtWidget::setupUi(void)
{
    QGridLayout * frameLayout = new QGridLayout;

    QLabelCurrentStateMaster = new QLabel("undefined");
    frameLayout->addWidget(QLabelCurrentStateMaster, 0, 0);

    QLabelCurrentStateSlave = new QLabel("undefined");
    frameLayout->addWidget(QLabelCurrentStateSlave, 1, 0);

    // MTM
    QGroupBox * groupBox = new QGroupBox("Desired state");
    QRadioButton * homeState = new QRadioButton("Home");
    QRadioButton * teleOpMode = new QRadioButton("Teleop");
    homeState->setChecked(true);
    QButtonGroup * group = new QButtonGroup;
	group->addButton(homeState);
	group->addButton(teleOpMode);
	group->setExclusive(true);
    QVBoxLayout * vbox = new QVBoxLayout;
    vbox->addWidget(homeState);
    vbox->addWidget(teleOpMode);
    vbox->addStretch(1);
    groupBox->setLayout(vbox);
    frameLayout->addWidget(groupBox, 2, 0);

    QVBoxLayout * mainLayout = new QVBoxLayout;
    mainLayout->addLayout(frameLayout);
    setLayout(mainLayout);

    setWindowTitle("Intuitive Research Kit");
    resize(sizeHint());

    connect(group, SIGNAL(buttonClicked(QAbstractButton*)),
            this, SLOT(SlotSetStateButton(QAbstractButton*)));
}

void mtsIntuitiveResearchKitConsoleQtWidget::StateMsgEventHandlerMaster(const std::string & newState)
{
    QLabelCurrentStateMaster->setText(newState.c_str());
}

void mtsIntuitiveResearchKitConsoleQtWidget::ErrorMsgEventHandlerMaster(const std::string & newMsg)
{
    // ZC: temp for testing
    std::cerr << newMsg << std::endl;
}

void mtsIntuitiveResearchKitConsoleQtWidget::StateMsgEventHandlerSlave(const std::string & newState)
{
    QLabelCurrentStateSlave->setText(newState.c_str());
}

void mtsIntuitiveResearchKitConsoleQtWidget::ErrorMsgEventHandlerSlave(const std::string & newMsg)
{
    // ZC: temp for testing
    std::cerr << newMsg << std::endl;
}
