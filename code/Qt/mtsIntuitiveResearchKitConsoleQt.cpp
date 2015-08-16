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

mtsIntuitiveResearchKitConsoleQt::mtsIntuitiveResearchKitConsoleQt(mtsIntuitiveResearchKitConsole * console)
{
    mtsComponentManager * componentManager = mtsComponentManager::GetInstance();

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;

    mtsIntuitiveResearchKitConsoleQtWidget * consoleGUI = new mtsIntuitiveResearchKitConsoleQtWidget("consoleGUI");
    componentManager->AddComponent(consoleGUI);
    // connect consoleGUI to console
    componentManager->Connect("console", "Main", "consoleGUI", "Main");
    tabWidget->addTab(consoleGUI, "Main");

    // connect ioGUIMaster to io
    mtsRobotIO1394QtWidgetFactory * robotWidgetFactory = new mtsRobotIO1394QtWidgetFactory("robotWidgetFactory");
    componentManager->AddComponent(robotWidgetFactory);
    componentManager->Connect("robotWidgetFactory", "RobotConfiguration", "io", "Configuration");
    robotWidgetFactory->Configure();

    // add all IO GUI to tab
    mtsRobotIO1394QtWidgetFactory::WidgetListType::const_iterator iterator;
    for (iterator = robotWidgetFactory->Widgets().begin();
         iterator != robotWidgetFactory->Widgets().end();
         ++iterator) {
        tabWidget->addTab(*iterator, (*iterator)->GetName().c_str());
    }
    tabWidget->addTab(robotWidgetFactory->ButtonsWidget(), "Buttons");

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
        case mtsIntuitiveResearchKitConsole::Arm::ARM_ECM:
        case mtsIntuitiveResearchKitConsole::Arm::ARM_PSM:
            // PID widget
            unsigned int numberOfJoints;
            if (armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_PSM) {
                numberOfJoints = 7;
            } else if (armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_MTM) {
                numberOfJoints = 8;
            } else if (armIter->second->mType == mtsIntuitiveResearchKitConsole::Arm::ARM_ECM) {
                numberOfJoints = 4;
            }

            pidGUI = new mtsPIDQtWidget(name + "-PID-GUI", numberOfJoints);
            pidGUI->Configure();
            componentManager->AddComponent(pidGUI);
            componentManager->Connect(pidGUI->GetName(), "Controller", armIter->second->PIDComponentName(), "Controller");
            tabWidget->addTab(pidGUI, (name + " PID").c_str());

            // Arm widget
            armGUI = new mtsIntuitiveResearchKitArmQtWidget(name + "-GUI");
            armGUI->Configure();
            componentManager->AddComponent(armGUI);
            componentManager->Connect(armGUI->GetName(), "Manipulator", armIter->second->mName, "Robot");
            tabWidget->addTab(armGUI, name.c_str());
            break;

        case mtsIntuitiveResearchKitConsole::Arm::ARM_SUJ:

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("PSM1-SUJ");
            componentManager->AddComponent(sujGUI);
            componentManager->Connect(sujGUI->GetName(), "Manipulator", "SUJ", "PSM1");
            tabWidget->addTab(sujGUI, "PSM1 SUJ");

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("ECM-SUJ");
            componentManager->AddComponent(sujGUI);
            componentManager->Connect(sujGUI->GetName(), "Manipulator", "SUJ", "ECM");
            tabWidget->addTab(sujGUI, "ECM SUJ");

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("PSM2-SUJ");
            componentManager->AddComponent(sujGUI);
            componentManager->Connect(sujGUI->GetName(), "Manipulator", "SUJ", "PSM2");
            tabWidget->addTab(sujGUI, "PSM2 SUJ");

            sujGUI = new mtsIntuitiveResearchKitSUJQtWidget("PSM3-SUJ");
            componentManager->AddComponent(sujGUI);
            componentManager->Connect(sujGUI->GetName(), "Manipulator", "SUJ", "PSM3");
            tabWidget->addTab(sujGUI, "PSM3 SUJ");

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
        componentManager->Connect(teleopGUI->GetName(), "TeleOperation", name, "Setting");
        tabWidget->addTab(teleopGUI, name.c_str());
    }

    // show all widgets
    tabWidget->show();
}
