/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-07-13

  (C) Copyright 2015-2017 Johns Hopkins University (JHU), All Rights Reserved.

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

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmQtWidget.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationPSMQtWidget.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationECMQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitSUJQtWidget.h>
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
    Connections.push_back(new ConnectionType("consoleGUI", "Main", "console", "Main"));
    if (console->GetInterfaceRequired("Clutch")) {
        Connections.push_back(new ConnectionType("consoleGUI", "Clutch", "console", "Clutch"));
    }
    if (console->GetInterfaceRequired("OperatorPresent")) {
        Connections.push_back(new ConnectionType("consoleGUI", "OperatorPresent", "console", "OperatorPresent"));
    }
    if (console->GetInterfaceRequired("Camera")) {
        Connections.push_back(new ConnectionType("consoleGUI", "Camera", "console", "Camera"));
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
        ioTabWidget->addTab(robotWidgetFactory->ButtonsWidget(), "Buttons");
    }

    // Arm and PID widgets
    QTabWidget * pidTabWidget;
    QTabWidget * armTabWidget;
    if (console->mArms.size() > 1) {
        pidTabWidget = new QTabWidget();
        TabWidget->addTab(pidTabWidget, "PIDs");
        armTabWidget = new QTabWidget();
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

        switch(armIter->second->mType)
        {
        case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM_DERIVED:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_DERIVED:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM_DERIVED:
            // PID widget
            unsigned int numberOfJoints;
            if ((armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_PSM) ||
                (armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_DERIVED)) {
                numberOfJoints = 7;
            } else if ((armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_MTM) ||
                       (armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_MTM_DERIVED)) {
                numberOfJoints = 7;
            } else if ((armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_ECM) ||
                       (armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_ECM_DERIVED)) {
                numberOfJoints = 4;
            } else {
                numberOfJoints = 0; // can't happen but prevents compiler warning
            }
            pidGUI = new mtsPIDQtWidget(name + "-PID-GUI", numberOfJoints);
            pidGUI->Configure();
            componentManager->AddComponent(pidGUI);
            Connections.push_back(new ConnectionType(pidGUI->GetName(), "Controller", armIter->second->PIDComponentName(), "Controller"));
            pidTabWidget->addTab(pidGUI, (name + " PID").c_str());

            // Arm widget
            armGUI = new mtsIntuitiveResearchKitArmQtWidget(name + "-GUI");
            armGUI->Configure();
            componentManager->AddComponent(armGUI);
            Connections.push_back(new ConnectionType(armGUI->GetName(), "Manipulator",
                                                     armIter->second->ComponentName(),
                                                     armIter->second->InterfaceName()));
            armTabWidget->addTab(armGUI, name.c_str());

            // PSM server
            if (armIter->second->mSocketServer) {
                socketGUI = new mtsSocketBaseQtWidget(name + "-Server-GUI");
                socketGUI->Configure();
                componentManager->AddComponent(socketGUI);
                Connections.push_back(new ConnectionType(socketGUI->GetName(), "SocketBase", armIter->second->mSocketComponentName, "System"));
                armTabWidget->addTab(socketGUI, (name + "-Server").c_str());
                break;
            }

            break;

        case mtsIntuitiveResearchKitConsole::Arm::ARM_SUJ:

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("PSM1-SUJ");
            componentManager->AddComponent(sujGUI);
            Connections.push_back(new ConnectionType(sujGUI->GetName(), "Manipulator", "SUJ", "PSM1"));
            armTabWidget->addTab(sujGUI, "PSM1 SUJ");

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("ECM-SUJ");
            componentManager->AddComponent(sujGUI);
            Connections.push_back(new ConnectionType(sujGUI->GetName(), "Manipulator", "SUJ", "ECM"));
            armTabWidget->addTab(sujGUI, "ECM SUJ");

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("PSM2-SUJ");
            componentManager->AddComponent(sujGUI);
            Connections.push_back(new ConnectionType(sujGUI->GetName(), "Manipulator", "SUJ", "PSM2"));
            armTabWidget->addTab(sujGUI, "PSM2 SUJ");

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("PSM3-SUJ");
            componentManager->AddComponent(sujGUI);
            Connections.push_back(new ConnectionType(sujGUI->GetName(), "Manipulator", "SUJ", "PSM3"));
            armTabWidget->addTab(sujGUI, "PSM3 SUJ");

            break;

        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_SOCKET:

            socketGUI = new mtsSocketBaseQtWidget(name + "-GUI");
            socketGUI->Configure();
            componentManager->AddComponent(socketGUI);
            Connections.push_back(new ConnectionType(socketGUI->GetName(), "SocketBase",
                                                     armIter->second->mName, "System"));
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
                Connections.push_back(new ConnectionType(messageGUI->GetName(), "Component",
                                                         armIter->second->ComponentName(),
                                                         armIter->second->InterfaceName()));
                mtsIntervalStatisticsQtWidgetComponent * timingGUI
                    = new mtsIntervalStatisticsQtWidgetComponent(name + "-Timing-GUI");
                componentManager->AddComponent(timingGUI);
                genericComponentLayout->addWidget(timingGUI);
                Connections.push_back(new ConnectionType(timingGUI->GetName(), "Component",
                                                         armIter->second->ComponentName(),
                                                         armIter->second->InterfaceName()));
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
    const mtsIntuitiveResearchKitConsole::TeleopPSMList::iterator teleopsEnd = console->mTeleopsPSM.end();
    mtsIntuitiveResearchKitConsole::TeleopPSMList::iterator teleopIter;
    for (teleopIter = console->mTeleopsPSM.begin(); teleopIter != teleopsEnd; ++teleopIter) {
        hasTeleOp = true;
        const std::string name = teleopIter->first;
        mtsTeleOperationPSMQtWidget * teleopGUI = new mtsTeleOperationPSMQtWidget(name + "-GUI");
        teleopGUI->Configure();
        componentManager->AddComponent(teleopGUI);
        Connections.push_back(new ConnectionType(teleopGUI->GetName(), "TeleOperation", name, "Setting"));
        TabWidget->addTab(teleopGUI, name.c_str());
    }

    // add teleop ECM widget
    if (console->mTeleopECM) {
        hasTeleOp = true;
        const std::string name = console->mTeleopECM->Name();
        mtsTeleOperationECMQtWidget * teleopGUI = new mtsTeleOperationECMQtWidget(name + "-GUI");
        teleopGUI->Configure();
        componentManager->AddComponent(teleopGUI);
        Connections.push_back(new ConnectionType(teleopGUI->GetName(), "TeleOperation", name, "Setting"));
        TabWidget->addTab(teleopGUI, name.c_str());
    }

    consoleGUI->HasTeleOp(hasTeleOp);

    // show all widgets
    TabWidget->show();
}

void mtsIntuitiveResearchKitConsoleQt::Connect(void)
{
    mtsComponentManager * componentManager = mtsComponentManager::GetInstance();

    const ConnectionsType::const_iterator end = Connections.end();
    ConnectionsType::const_iterator connectIter;
    for (connectIter = Connections.begin();
         connectIter != end;
         ++connectIter) {
        ConnectionType * connection = *connectIter;
        componentManager->Connect(connection->ClientComponentName,
                                  connection->ClientInterfaceName,
                                  connection->ServerComponentName,
                                  connection->ServerInterfaceName);
    }
}

void mtsIntuitiveResearchKitConsoleQt::addTab(QWidget * widget, const std::string & name)
{
    TabWidget->addTab(widget, name.c_str());
}
