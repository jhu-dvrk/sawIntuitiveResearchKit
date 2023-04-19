/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-07-13

  (C) Copyright 2015-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQt.h>

// cisst/saw
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsMessageQtWidget.h>
#include <cisstMultiTask/mtsIntervalStatisticsQtWidget.h>

#include <sawRobotIO1394/mtsRobotIO1394QtWidgetFactory.h>
#include <sawControllers/mtsPIDQtWidget.h>

#include <sawIntuitiveResearchKit/mtsDaVinciEndoscopeFocus.h>
#include <sawIntuitiveResearchKit/mtsDaVinciEndoscopeFocusQtWidget.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitECMQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitMTMQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSMQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitSUJQtWidget.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationPSMQtWidget.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationECMQtWidget.h>
#include <sawIntuitiveResearchKit/mtsSocketBaseQtWidget.h>

#include <QTabWidget>

CMN_IMPLEMENT_SERVICES(mtsIntuitiveResearchKitConsoleQt);

mtsIntuitiveResearchKitConsoleQt::mtsIntuitiveResearchKitConsoleQt(void)
{
}

void mtsIntuitiveResearchKitConsoleQt::Configure(mtsIntuitiveResearchKitConsole * console)
{
    mtsComponentManager * componentManager = mtsComponentManager::GetInstance();

    mtsIntuitiveResearchKitConsoleQtWidget * consoleGUI = new mtsIntuitiveResearchKitConsoleQtWidget("consoleGUI");
    componentManager->AddComponent(consoleGUI);
    // connect consoleGUI to console
    Connections.Add("consoleGUI", "Main", "console", "Main");
    if (console->GetInterfaceRequired("Clutch")) {
        Connections.Add("consoleGUI", "Clutch", "console", "Clutch");
    }
    if (console->GetInterfaceRequired("OperatorPresent")) {
        Connections.Add("consoleGUI", "OperatorPresent", "console", "OperatorPresent");
    }
    if (console->GetInterfaceRequired("Camera")) {
        Connections.Add("consoleGUI", "Camera", "console", "Camera");
    }

    TabWidget = consoleGUI->GetTabWidget();

    // IOs
    if (console->mHasIO) {
        // connect ioGUIMaster to io
        mtsRobotIO1394QtWidgetFactory * robotWidgetFactory = new mtsRobotIO1394QtWidgetFactory("robotWidgetFactory");
        componentManager->AddComponent(robotWidgetFactory);
        // this connect needs to happen now so the factory can figure out the io interfaces
        componentManager->Connect("robotWidgetFactory", "RobotConfiguration", "io", "Configuration");
        robotWidgetFactory->Configure();

        // add all IO GUI to tab
        QTabWidget * ioTabWidget;
        if (robotWidgetFactory->Widgets().size() > 1) {
            ioTabWidget = new QTabWidget();
            TabWidget->addTab(ioTabWidget, "IOs");
        } else {
            ioTabWidget = TabWidget; // use current tab widget
        }
        mtsRobotIO1394QtWidgetFactory::WidgetListType::const_iterator iterator;
        for (iterator = robotWidgetFactory->Widgets().begin();
             iterator != robotWidgetFactory->Widgets().end();
             ++iterator) {
            ioTabWidget->addTab(*iterator, (*iterator)->GetName().c_str());
        }
        if (robotWidgetFactory->ButtonsWidget()) {
            ioTabWidget->addTab(robotWidgetFactory->ButtonsWidget(), "Buttons");
        }
    }

    // Arm and PID widgets
    QTabWidget * pidTabWidget;
    QTabWidget * armTabWidget;
    if (console->mArms.size() > 1) {
        pidTabWidget = new QTabWidget();
        pidTabWidget->setObjectName(QString("PIDs"));
        TabWidget->addTab(pidTabWidget, "PIDs");
        armTabWidget = new QTabWidget();
        armTabWidget->setObjectName(QString("Arms"));
        TabWidget->addTab(armTabWidget, "Arms");
    } else {
        pidTabWidget = TabWidget; // use current tab widget
        armTabWidget = TabWidget; // use current tab widget
    }

    const mtsIntuitiveResearchKitConsole::ArmList::iterator armsEnd = console->mArms.end();
    mtsIntuitiveResearchKitConsole::ArmList::iterator armIter;
    for (armIter = console->mArms.begin(); armIter != armsEnd; ++armIter) {
        mtsIntuitiveResearchKitArmQtWidget * armGUI;
        mtsIntuitiveResearchKitSUJQtWidget * sujGUI;
        mtsPIDQtWidget * pidGUI;
        mtsSocketBaseQtWidget * socketGUI;

        const std::string name = armIter->first;

        switch (armIter->second->m_type) {
        case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM_DERIVED:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_DERIVED:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM_DERIVED:
            // PID widget
            size_t numberOfJoints;
            if (armIter->second->native_or_derived_psm()) {
                numberOfJoints = 7;
            } else if (armIter->second->native_or_derived_mtm()) {
                numberOfJoints = 7;
            } else if (armIter->second->native_or_derived_ecm()) {
                numberOfJoints = 4;
            } else {
                numberOfJoints = 0; // can't happen but prevents compiler warning
            }
            pidGUI = new mtsPIDQtWidget(name + "-PID-GUI", numberOfJoints);
            pidGUI->Configure();
            componentManager->AddComponent(pidGUI);
            Connections.Add(pidGUI->GetName(), "Controller",
                            armIter->second->PIDComponentName(), "Controller");
            pidTabWidget->addTab(pidGUI, (name + " PID").c_str());

            // Arm widget
            if (armIter->second->native_or_derived_psm()) {
                armGUI = new mtsIntuitiveResearchKitPSMQtWidget(name + "-GUI");
            } else if (armIter->second->native_or_derived_mtm()) {
                armGUI = new mtsIntuitiveResearchKitMTMQtWidget(name + "-GUI");
            } else if (armIter->second->native_or_derived_ecm()) {
                armGUI = new mtsIntuitiveResearchKitECMQtWidget(name + "-GUI");
            } else {
                armGUI = new mtsIntuitiveResearchKitArmQtWidget(name + "-GUI");
            }
            armGUI->Configure();
            componentManager->AddComponent(armGUI);
            Connections.Add(armGUI->GetName(), "Manipulator",
                            armIter->second->ComponentName(),
                            armIter->second->InterfaceName());
            armGUI->setObjectName(name.c_str());
            armTabWidget->addTab(armGUI, name.c_str());

            // PSM server
            if (armIter->second->m_socket_server) {
                socketGUI = new mtsSocketBaseQtWidget(name + "-Server-GUI");
                socketGUI->Configure();
                componentManager->AddComponent(socketGUI);
                Connections.Add(socketGUI->GetName(), "SocketBase",
                                armIter->second->m_socket_component_name, "System");
                armTabWidget->addTab(socketGUI, (name + "-Server").c_str());
                break;
            }

            break;

        case mtsIntuitiveResearchKitConsole::Arm::ARM_SUJ_Classic:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_SUJ_Si:

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("PSM1-SUJ");
            componentManager->AddComponent(sujGUI);
            Connections.Add(sujGUI->GetName(), "Manipulator", "SUJ", "PSM1");
            armTabWidget->addTab(sujGUI, "PSM1 SUJ");

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("ECM-SUJ");
            componentManager->AddComponent(sujGUI);
            Connections.Add(sujGUI->GetName(), "Manipulator", "SUJ", "ECM");
            armTabWidget->addTab(sujGUI, "ECM SUJ");

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("PSM2-SUJ");
            componentManager->AddComponent(sujGUI);
            Connections.Add(sujGUI->GetName(), "Manipulator", "SUJ", "PSM2");
            armTabWidget->addTab(sujGUI, "PSM2 SUJ");

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("PSM3-SUJ");
            componentManager->AddComponent(sujGUI);
            Connections.Add(sujGUI->GetName(), "Manipulator", "SUJ", "PSM3");
            armTabWidget->addTab(sujGUI, "PSM3 SUJ");

            break;

        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_SOCKET:

            socketGUI = new mtsSocketBaseQtWidget(name + "-GUI");
            socketGUI->setObjectName(name.c_str());
            socketGUI->Configure();
            componentManager->AddComponent(socketGUI);
            Connections.Add(socketGUI->GetName(), "SocketBase",
                            armIter->second->m_name, "System");
            armTabWidget->addTab(socketGUI, name.c_str());
            break;

        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM_GENERIC:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM_GENERIC:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_GENERIC:
            {
                // Arm widget
                QWidget * genericComponentGUI = new QWidget();
                QHBoxLayout * genericComponentLayout = new QHBoxLayout();
                genericComponentGUI->setLayout(genericComponentLayout);
                mtsMessageQtWidgetComponent * messageGUI
                    = new mtsMessageQtWidgetComponent(name + "-Message-GUI");
                componentManager->AddComponent(messageGUI);
                genericComponentLayout->addWidget(messageGUI);
                Connections.Add(messageGUI->GetName(), "Component",
                                armIter->second->ComponentName(),
                                armIter->second->InterfaceName());
                mtsIntervalStatisticsQtWidgetComponent * timingGUI
                    = new mtsIntervalStatisticsQtWidgetComponent(name + "-Timing-GUI");
                componentManager->AddComponent(timingGUI);
                genericComponentLayout->addWidget(timingGUI);
                Connections.Add(timingGUI->GetName(), "Component",
                                armIter->second->ComponentName(),
                                armIter->second->InterfaceName());
                armTabWidget->addTab(genericComponentGUI, name.c_str());
            }
            break;

        default:
            CMN_LOG_CLASS_INIT_ERROR << "mtsIntuitiveResearchKitConsoleQt: arm "
                                     << name
                                     << ": unable to create appropriate Qt widgets for arm of this type"
                                     << std::endl;
        }
    }

    // add teleop PSM widgets
    bool hasTeleOp = false;

    QTabWidget * teleopTabWidget;
    if (console->mTeleopsPSM.size() > 1) {
        teleopTabWidget = new QTabWidget();
        teleopTabWidget->setObjectName(QString("Teleops"));
        TabWidget->addTab(teleopTabWidget, "Teleops");
    } else {
        teleopTabWidget = TabWidget; // use current tab widget
    }

    const mtsIntuitiveResearchKitConsole::TeleopPSMList::iterator teleopsEnd = console->mTeleopsPSM.end();
    mtsIntuitiveResearchKitConsole::TeleopPSMList::iterator teleopIter;
    for (teleopIter = console->mTeleopsPSM.begin(); teleopIter != teleopsEnd; ++teleopIter) {
        hasTeleOp = true;
        const std::string name = teleopIter->first;
        mtsTeleOperationPSMQtWidget * teleopGUI = new mtsTeleOperationPSMQtWidget(name + "-GUI");
        teleopGUI->setObjectName(name.c_str());
        teleopGUI->Configure();
        componentManager->AddComponent(teleopGUI);
        Connections.Add(teleopGUI->GetName(), "TeleOperation", name, "Setting");
        teleopTabWidget->addTab(teleopGUI, name.c_str());
    }

    // add teleop ECM widget
    if (console->mTeleopECM) {
        hasTeleOp = true;
        const std::string name = console->mTeleopECM->Name();
        mtsTeleOperationECMQtWidget * teleopGUI = new mtsTeleOperationECMQtWidget(name + "-GUI");
        teleopGUI->Configure();
        componentManager->AddComponent(teleopGUI);
        Connections.Add(teleopGUI->GetName(), "TeleOperation", name, "Setting");
        TabWidget->addTab(teleopGUI, name.c_str());
    }

    // add endoscope focus widget
    if (console->mDaVinciEndoscopeFocus) {
        const std::string name = console->mDaVinciEndoscopeFocus->GetName();
        mtsDaVinciEndoscopeFocusQtWidget * endoscopeGUI = new mtsDaVinciEndoscopeFocusQtWidget(name + "-GUI");
        endoscopeGUI->Configure();
        componentManager->AddComponent(endoscopeGUI);
        Connections.Add(endoscopeGUI->GetName(), "Endoscope", name, "Control");
        TabWidget->addTab(endoscopeGUI, "Focus");
    }

    consoleGUI->HasTeleOp(hasTeleOp);

    // show all widgets
    TabWidget->show();
}

void mtsIntuitiveResearchKitConsoleQt::addTab(QWidget * widget, const std::string & name)
{
    TabWidget->addTab(widget, name.c_str());
}
