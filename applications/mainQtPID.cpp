/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-07

  (C) Copyright 2013-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system
#include <iostream>
// cisst/saw
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobotIO1394QtWidgetFactory.h>
#include <sawControllers/mtsPID.h>
#include <sawControllers/mtsPIDQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>

#include <QApplication>
#include <QMainWindow>

int main(int argc, char ** argv)
{
    // period used for IO and PID
    const double periodIOPID = mtsIntuitiveResearchKit::IOPeriod;

    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClass("mtsPID", CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsIntuitiveResearchKit", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // parse options
    cmnCommandLineOptions options;
    std::string portName = mtsRobotIO1394::DefaultPort();
    std::string ioConfigFile, pidConfigFile;
    std::string robotName;

    options.AddOptionOneValue("i", "io",
                              "configuration file for robot IO (see sawRobotIO1394)",
                              cmnCommandLineOptions::REQUIRED_OPTION, &ioConfigFile);
    options.AddOptionOneValue("p", "pid",
                              "configuration file for PID controller (see sawControllers, mtsPID)",
                              cmnCommandLineOptions::REQUIRED_OPTION, &pidConfigFile);
    options.AddOptionOneValue("P", "port",
                              "controller port (X, fw:X, udp, udp:XX.XX.XX.XX)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &portName);
    options.AddOptionOneValue("n", "robot-name",
                              "robot name (i.e. PSM1, PSM2, MTML or MTMR) as defined in the sawRobotIO1394 file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &robotName);

    if (!options.Parse(argc, argv, std::cerr)) {
        return -1;
    }

    if (!cmnPath::Exists(ioConfigFile)) {
        std::cerr << "File not found: " << ioConfigFile << std::endl;
        return -1;
    }
    if (!cmnPath::Exists(pidConfigFile)) {
        std::cerr << "File not found: " << pidConfigFile << std::endl;
        return -1;
    }

    unsigned int numberOfJoints;
    if ((robotName == "PSM1") || (robotName == "PSM2") || (robotName == "PSM3")) {
        numberOfJoints = 7;
    } else if ((robotName == "MTML") || (robotName == "MTMR")) {
        numberOfJoints = 7;
    } else if (robotName == "ECM") {
        numberOfJoints = 4;
    } else if (robotName == "Focus") {
        numberOfJoints = 1;
    } else {
        std::cerr << "Unknown robot name: " << robotName << ", should be one one PSM1, PSM2, PSM3, MTML, MTMR, ECM or Focus" << std::endl;
        return -1;
    }

    std::cout << "Configuration file for IO: " << ioConfigFile << std::endl
              << "Configuration file for PID: " << pidConfigFile << std::endl
              << "Robot name: " << robotName << std::endl
              << "Number of joints: " << numberOfJoints << std::endl
              << "FirewirePort: " << portName << std::endl;

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // create a Qt user interface
    QApplication application(argc, argv);
    application.setWindowIcon(QIcon(":/dVRK.png"));

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;

    // IO
    mtsRobotIO1394 * io = new mtsRobotIO1394("io", periodIOPID, portName);
    io->Configure(ioConfigFile);
    componentManager->AddComponent(io);

    mtsRobotIO1394QtWidgetFactory * robotWidgetFactory = new mtsRobotIO1394QtWidgetFactory("robotWidgetFactory");
    componentManager->AddComponent(robotWidgetFactory);
    componentManager->Connect("robotWidgetFactory", "RobotConfiguration", "io", "Configuration");
    robotWidgetFactory->Configure();

    mtsRobotIO1394QtWidgetFactory::WidgetListType::const_iterator iterator;
    for (iterator = robotWidgetFactory->Widgets().begin();
         iterator != robotWidgetFactory->Widgets().end();
         ++iterator) {
        tabWidget->addTab(*iterator, (*iterator)->GetName().c_str());
    }
    if (robotWidgetFactory->ButtonsWidget()) {
        tabWidget->addTab(robotWidgetFactory->ButtonsWidget(), "Buttons");
    }


    // Qt PID Controller GUI
    mtsPIDQtWidget * pidGUI = new mtsPIDQtWidget("pidGUI", numberOfJoints);
    pidGUI->Configure();
    componentManager->AddComponent(pidGUI);
    tabWidget->addTab(pidGUI, "PID");

    mtsPID * pid = new mtsPID("pid", periodIOPID);
    pid->Configure(pidConfigFile);
    componentManager->AddComponent(pid);
    // connect pidGUI to pid
    componentManager->Connect("pidGUI", "Controller", "pid", "Controller");

    // tie pid execution to io
    componentManager->Connect("pid", "ExecIn", "io", "ExecOut");
    // connect pid to io
    componentManager->Connect("pid", "RobotJointTorqueInterface", "io", robotName);  // see const std::string defined before main()

    //-------------- create the components ------------------
    componentManager->CreateAll();
    componentManager->WaitForStateAll(mtsComponentState::READY, 2.0 * cmn_s);

    // start the periodic Run
    componentManager->StartAll();

    // run Qt user interface
    tabWidget->show();
    application.exec();

    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    // delete dvgc robot
    delete pid;
    delete pidGUI;
    delete io;
    delete robotWidgetFactory;

    // stop all logs
    cmnLogger::Kill();

    return 0;
}
