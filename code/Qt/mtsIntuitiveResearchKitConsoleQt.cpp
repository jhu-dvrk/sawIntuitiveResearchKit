/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2015-07-13

  (C) Copyright 2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQt.h>

// cisst/saw
#include <cisstMultiTask/mtsManagerLocal.h>
#include <sawRobotIO1394/mtsRobotIO1394QtWidgetFactory.h>
#include <sawControllers/mtsPIDQtWidget.h>
#include <sawControllers/mtsTeleOperationQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitSUJQtWidget.h>
#include <QTabWidget>

CMN_IMPLEMENT_SERVICES(mtsIntuitiveResearchKitConsoleQt);

mtsIntuitiveResearchKitConsoleQt::mtsIntuitiveResearchKitConsoleQt(void)
{
}

void mtsIntuitiveResearchKitConsoleQt::Configure(mtsIntuitiveResearchKitConsole * console)
{
    mtsComponentManager * componentManager = mtsComponentManager::GetInstance();

    // organize all widgets in a tab widget
    TabWidget = new QTabWidget;

    mtsIntuitiveResearchKitConsoleQtWidget * consoleGUI = new mtsIntuitiveResearchKitConsoleQtWidget("consoleGUI");
    componentManager->AddComponent(consoleGUI);
    // connect consoleGUI to console
    Connections.push_back(new ConnectionType("console", "Main", "consoleGUI", "Main"));
    TabWidget->addTab(consoleGUI, "Main");

    // IOs
    if (console->mHasIO) {
        // connect ioGUIMaster to io
        mtsRobotIO1394QtWidgetFactory * robotWidgetFactory = new mtsRobotIO1394QtWidgetFactory("robotWidgetFactory");
        componentManager->AddComponent(robotWidgetFactory);
        // this connect needs to happen now so the factory can figure out the io interfaces
        componentManager->Connect("robotWidgetFactory", "RobotConfiguration", "io", "Configuration");
        robotWidgetFactory->Configure();

        // add all IO GUI to tab
        mtsRobotIO1394QtWidgetFactory::WidgetListType::const_iterator iterator;
        for (iterator = robotWidgetFactory->Widgets().begin();
             iterator != robotWidgetFactory->Widgets().end();
             ++iterator) {
            TabWidget->addTab(*iterator, (*iterator)->GetName().c_str());
        }
        TabWidget->addTab(robotWidgetFactory->ButtonsWidget(), "Buttons");
    }

    // Arm and PID widgets
    const mtsIntuitiveResearchKitConsole::ArmList::iterator armsEnd = console->mArms.end();
    mtsIntuitiveResearchKitConsole::ArmList::iterator armIter;
    for (armIter = console->mArms.begin(); armIter != armsEnd; ++armIter) {
        mtsIntuitiveResearchKitArmQtWidget * armGUI;
        mtsIntuitiveResearchKitSUJQtWidget * sujGUI;
        mtsPIDQtWidget * pidGUI;

        const std::string name = armIter->first;

        switch(armIter->second->mType)
        {
        case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM_DERIVED:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_MTM_KIN_SIMULATED:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM:        
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_DERIVED:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_KIN_SIMULATED:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM_DERIVED:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM_KIN_SIMULATED:
            // PID widget
            unsigned int numberOfJoints;
            if ((armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_PSM) ||
                (armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_DERIVED) ||
                (armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_PSM_KIN_SIMULATED)) {
                numberOfJoints = 7;
            } else if ((armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_MTM) ||
                       (armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_MTM_DERIVED) || 
                       (armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_MTM_KIN_SIMULATED)) {
                numberOfJoints = 8;
            } else if ((armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_ECM) ||
                       (armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_ECM_DERIVED) ||
                       (armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_ECM_KIN_SIMULATED)) {
                numberOfJoints = 4;
            } else {
                numberOfJoints = 0; // can't happen but prevents compiler warning
            }
            pidGUI = new mtsPIDQtWidget(name + "-PID-GUI", numberOfJoints);
            pidGUI->Configure();
            componentManager->AddComponent(pidGUI);
            Connections.push_back(new ConnectionType(pidGUI->GetName(), "Controller", armIter->second->PIDComponentName(), "Controller"));
            TabWidget->addTab(pidGUI, (name + " PID").c_str());

            // Arm widget
            armGUI = new mtsIntuitiveResearchKitArmQtWidget(name + "-GUI");
            armGUI->Configure();
            componentManager->AddComponent(armGUI);
            Connections.push_back(new ConnectionType(armGUI->GetName(), "Manipulator", armIter->second->mName, "Robot"));
            TabWidget->addTab(armGUI, name.c_str());
            break;

        case mtsIntuitiveResearchKitConsole::Arm::ARM_SUJ:

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("PSM1-SUJ");
            componentManager->AddComponent(sujGUI);
            Connections.push_back(new ConnectionType(sujGUI->GetName(), "Manipulator", "SUJ", "PSM1"));
            TabWidget->addTab(sujGUI, "PSM1 SUJ");

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("ECM-SUJ");
            componentManager->AddComponent(sujGUI);
            Connections.push_back(new ConnectionType(sujGUI->GetName(), "Manipulator", "SUJ", "ECM"));
            TabWidget->addTab(sujGUI, "ECM SUJ");

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("PSM2-SUJ");
            componentManager->AddComponent(sujGUI);
            Connections.push_back(new ConnectionType(sujGUI->GetName(), "Manipulator", "SUJ", "PSM2"));
            TabWidget->addTab(sujGUI, "PSM2 SUJ");

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("PSM3-SUJ");
            componentManager->AddComponent(sujGUI);
            Connections.push_back(new ConnectionType(sujGUI->GetName(), "Manipulator", "SUJ", "PSM3"));
            TabWidget->addTab(sujGUI, "PSM3 SUJ");

            break;

        default:
            CMN_LOG_CLASS_INIT_ERROR << "mtsIntuitiveResearchKitConsoleQt: arm "
                                     << name
                                     << ": unable to create appropriate Qt widgets for arm of this type"
                                     << std::endl;
        }
    }

    // add teleop widgets
    const mtsIntuitiveResearchKitConsole::TeleopList::iterator teleopsEnd = console->mTeleops.end();
    mtsIntuitiveResearchKitConsole::TeleopList::iterator teleopIter;
    for (teleopIter = console->mTeleops.begin(); teleopIter != teleopsEnd; ++teleopIter) {
        const std::string name = teleopIter->first;
        mtsTeleOperationQtWidget * teleopGUI = new mtsTeleOperationQtWidget(name + "-GUI");
        teleopGUI->Configure();
        componentManager->AddComponent(teleopGUI);
        Connections.push_back(new ConnectionType(teleopGUI->GetName(), "TeleOperation", name, "Setting"));
        TabWidget->addTab(teleopGUI, name.c_str());
    }

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
