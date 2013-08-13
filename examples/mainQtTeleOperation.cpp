/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*
  $Id$

  Author(s):  Zihan Chen
  Created on: 2013-02-07

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

// system
#include <iostream>
#include <map>

// cisst/saw
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsQtApplication.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobotIO1394QtWidgetFactory.h>
#include <sawControllers/mtsPIDQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQtWidget.h>
#include <sawControllers/mtsTeleOperation.h>
#include <sawControllers/mtsTeleOperationQtWidget.h>
#include <sawTextToSpeech/mtsTextToSpeech.h>

#include <QTabWidget>

int main(int argc, char ** argv)
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // parse options
    cmnCommandLineOptions options;
    int firewirePort = 0;
    std::string gcmip = "-1";
    typedef std::map<std::string, std::string> ConfigFilesType;
    ConfigFilesType configFiles;
    std::string masterName, slaveName;

    options.AddOptionOneValue("i", "io-master",
                              "configuration file for master robot IO (see sawRobotIO1394)",
                              cmnCommandLineOptions::REQUIRED, &configFiles["io-master"]);
    options.AddOptionOneValue("p", "pid-master",
                              "configuration file for master PID controller (see sawControllers, mtsPID)",
                              cmnCommandLineOptions::REQUIRED, &configFiles["pid-master"]);
    options.AddOptionOneValue("k", "kinematic-master",
                              "configuration file for master kinematic (see cisstRobot, robManipulator)",
                              cmnCommandLineOptions::REQUIRED, &configFiles["kinematic-master"]);
    options.AddOptionOneValue("I", "io-slave",
                              "configuration file for slave robot IO (see sawRobotIO1394)",
                              cmnCommandLineOptions::REQUIRED, &configFiles["io-slave"]);
    options.AddOptionOneValue("P", "pid-slave",
                              "configuration file for slave PID controller (see sawControllers, mtsPID)",
                              cmnCommandLineOptions::REQUIRED, &configFiles["pid-slave"]);
    options.AddOptionOneValue("K", "kinematic-slave",
                              "configuration file for slave kinematic (see cisstRobot, robManipulator)",
                              cmnCommandLineOptions::REQUIRED, &configFiles["kinematic-slave"]);
    options.AddOptionOneValue("n", "name-master",
                              "MTML or MTMR",
                              cmnCommandLineOptions::REQUIRED, &masterName);
    options.AddOptionOneValue("N", "name-slave",
                              "PSM1 or PSM2",
                              cmnCommandLineOptions::REQUIRED, &slaveName);
    options.AddOptionOneValue("f", "firewire",
                              "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL, &firewirePort);
    options.AddOptionOneValue("g", "gcmip",
                              "global component manager IP address",
                              cmnCommandLineOptions::OPTIONAL, &gcmip);

    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }

    for (ConfigFilesType::const_iterator iter = configFiles.begin();
         iter != configFiles.end();
         ++iter) {
        if (!cmnPath::Exists(iter->second)) {
            std::cerr << "File not found for " << iter->first
                      << ": " << iter->second << std::endl;
            return -1;
        } else {
            std::cout << "Configuration file for " << iter->first
                      << ": " << iter->second << std::endl;
        }
    }
    std::cout << "FirewirePort: " << firewirePort << std::endl;

    std::string processname = "dvTeleop";
    mtsManagerLocal * componentManager = 0;
    if (gcmip != "-1") {
        try {
            componentManager = mtsManagerLocal::GetInstance(gcmip, processname);
        } catch(...) {
            std::cerr << "Failed to get GCM instance." << std::endl;
            return -1;
        }
    } else {
        componentManager = mtsManagerLocal::GetInstance();
    }

    // create a Qt application and tab to hold all widgets
    mtsQtApplication *qtAppTask = new mtsQtApplication("QtApplication", argc, argv);
    qtAppTask->Configure();
    componentManager->AddComponent(qtAppTask);


    // console
    mtsIntuitiveResearchKitConsole * console = new mtsIntuitiveResearchKitConsole("console");
    componentManager->AddComponent(console);
    mtsIntuitiveResearchKitConsoleQtWidget * consoleGUI = new mtsIntuitiveResearchKitConsoleQtWidget("consoleGUI");
    componentManager->AddComponent(consoleGUI);
    // connect teleGUI to tele
    componentManager->Connect("console", "Main", "consoleGUI", "Main");

    // IO
    mtsRobotIO1394 * io = new mtsRobotIO1394("io", 1.0 * cmn_ms, firewirePort);
    io->Configure(configFiles["io-master"]);
    io->Configure(configFiles["io-slave"]);
    componentManager->AddComponent(io);

    mtsIntuitiveResearchKitConsole::Arm * mtm
            = new mtsIntuitiveResearchKitConsole::Arm(masterName, io->GetName());
    mtm->ConfigurePID(configFiles["pid-master"]);
    mtm->ConfigureArm(mtsIntuitiveResearchKitConsole::Arm::ARM_MTM,
                               configFiles["kinematic-master"], 3.0 * cmn_ms);
    console->AddArm(mtm);

    mtsIntuitiveResearchKitConsole::Arm * psm
            = new mtsIntuitiveResearchKitConsole::Arm(slaveName, io->GetName());
    psm->ConfigurePID(configFiles["pid-slave"]);
    psm->ConfigureArm(mtsIntuitiveResearchKitConsole::Arm::ARM_PSM,
                               configFiles["kinematic-slave"], 3.0 * cmn_ms);
    console->AddArm(psm);

    // connect ioGUIMaster to io
    mtsRobotIO1394QtWidgetFactory * robotWidgetFactory = new mtsRobotIO1394QtWidgetFactory("robotWidgetFactory");
    componentManager->AddComponent(robotWidgetFactory);
    componentManager->Connect("robotWidgetFactory", "RobotConfiguration", "io", "Configuration");
    robotWidgetFactory->Configure();

    // PID Master GUI
    mtsPIDQtWidget * pidMasterGUI = new mtsPIDQtWidget("PID Master", 8);
    pidMasterGUI->Configure();
    componentManager->AddComponent(pidMasterGUI);
    componentManager->Connect(pidMasterGUI->GetName(), "Controller", mtm->PIDComponentName(), "Controller");

    // PID Slave GUI
    mtsPIDQtWidget * pidSlaveGUI = new mtsPIDQtWidget("PID Slave", 7);
    pidSlaveGUI->Configure();
    componentManager->AddComponent(pidSlaveGUI);
    componentManager->Connect(pidSlaveGUI->GetName(), "Controller", psm->PIDComponentName(), "Controller");

    // Teleoperation
    mtsTeleOperationQtWidget * teleGUI = new mtsTeleOperationQtWidget("teleGUI");
    teleGUI->Configure();
    componentManager->AddComponent(teleGUI);
    mtsTeleOperation * tele = new mtsTeleOperation("tele", 5.0 * cmn_ms);
    componentManager->AddComponent(tele);
    // connect teleGUI to tele
    componentManager->Connect("teleGUI", "TeleOperation", "tele", "Setting");

    // TextToSpeech
    mtsTextToSpeech* textToSpeech = new mtsTextToSpeech;
    textToSpeech->AddInterfaceRequiredForEventString("ErrorMsg", "RobotErrorMsg");
    componentManager->AddComponent(textToSpeech);
    componentManager->Connect(textToSpeech->GetName(), "ErrorMsg", psm->Name(), "Robot");

    // connect teleop to Master + Slave + Clutch
    componentManager->Connect("tele", "Master",
                              mtm->Name(), "Robot");
    componentManager->Connect("tele", "Slave",
                              psm->Name(), "Robot");
    componentManager->Connect("tele", "CLUTCH", "io", "CLUTCH");
    componentManager->Connect("tele", "COAG", "io", "COAG");

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;
    tabWidget->addTab(consoleGUI, "Main");
    tabWidget->addTab(teleGUI, "Tele-op");
    tabWidget->addTab(pidMasterGUI, "PID Master");
    tabWidget->addTab(pidSlaveGUI, "PID Slave");
    mtsRobotIO1394QtWidgetFactory::WidgetListType::const_iterator iterator;
    for (iterator = robotWidgetFactory->Widgets().begin();
         iterator != robotWidgetFactory->Widgets().end();
         ++iterator) {
        tabWidget->addTab(*iterator, (*iterator)->GetName().c_str());
    }
    tabWidget->show();

    //-------------- create the components ------------------
    io->CreateAndWait(2.0 * cmn_s); // this will also create the pids as they are in same thread
    io->StartAndWait(2.0 * cmn_s);
    componentManager->GetComponent(mtm->PIDComponentName())->StartAndWait(2.0 * cmn_s);
    componentManager->GetComponent(psm->PIDComponentName())->StartAndWait(2.0 * cmn_s);

    // start all other components
    componentManager->CreateAllAndWait(2.0 * cmn_s);
    componentManager->StartAllAndWait(2.0 * cmn_s);

    // QtApplication will run in main thread and return control
    // when exited.

    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    // delete dvgc robot
    delete tele;
    delete pidMasterGUI;
    delete pidSlaveGUI;
    delete io;
    delete robotWidgetFactory;

    // stop all logs
    cmnLogger::Kill();

    return 0;
}
