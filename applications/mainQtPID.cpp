/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-07

  (C) Copyright 2013-2017 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstMultiTask/mtsQtApplication.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobotIO1394QtWidgetFactory.h>
#include <sawControllers/mtsPID.h>
#include <sawControllers/mtsPIDQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>

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
    int firewirePort = 0;
    std::string ioConfigFile, pidConfigFile;
    std::string armName;

    options.AddOptionOneValue("i", "io",
                              "configuration file for robot IO (see sawRobotIO1394)",
                              cmnCommandLineOptions::REQUIRED_OPTION, &ioConfigFile);
    options.AddOptionOneValue("p", "pid",
                              "configuration file for PID controller (see sawControllers, mtsPID)",
                              cmnCommandLineOptions::REQUIRED_OPTION, &pidConfigFile);
    options.AddOptionOneValue("f", "firewire",
                              "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &firewirePort);
    options.AddOptionOneValue("n", "arm-name",
                              "arm name (i.e. PSM1, PSM2, MTML or MTMR) as defined in the sawRobotIO1394 file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &armName);

    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
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
    if ((armName == "PSM1") || (armName == "PSM2") || (armName == "PSM3")) {
        numberOfJoints = 7;
    } else if ((armName == "MTML") || (armName == "MTMR")) {
        numberOfJoints = 8;
    } else if (armName == "ECM") {
        numberOfJoints = 4;
    } else {
        std::cerr << "Unknown arm name: " << armName << ", should be one one PSM1, PSM2, PSM3, MTML, MTMR or ECM" << std::endl;
        return -1;
    }

    std::cout << "Configuration file for IO: " << ioConfigFile << std::endl
              << "Configuration file for PID: " << pidConfigFile << std::endl
              << "Arm name: " << armName << std::endl
              << "Number of joints: " << numberOfJoints << std::endl
              << "FirewirePort: " << firewirePort << std::endl;

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // create a Qt application
    mtsQtApplication *qtAppTask = new mtsQtApplication("QtApplication", argc, argv);
    qtAppTask->Configure();
    componentManager->AddComponent(qtAppTask);

    // IO
    mtsRobotIO1394 * io = new mtsRobotIO1394("io", periodIOPID, firewirePort);
    io->Configure(ioConfigFile);
    componentManager->AddComponent(io);

    mtsRobotIO1394QtWidgetFactory * robotWidgetFactory = new mtsRobotIO1394QtWidgetFactory("robotWidgetFactory");
    componentManager->AddComponent(robotWidgetFactory);
    componentManager->Connect("robotWidgetFactory", "RobotConfiguration", "io", "Configuration");
    robotWidgetFactory->Configure();

    // Qt PID Controller GUI
    mtsPIDQtWidget * pidGUI = new mtsPIDQtWidget("pidGUI", numberOfJoints);
    pidGUI->Configure();
    componentManager->AddComponent(pidGUI);
    mtsPID * pid = new mtsPID("pid", periodIOPID);
    pid->Configure(pidConfigFile);
    componentManager->AddComponent(pid);
    // connect pidGUI to pid
    componentManager->Connect("pidGUI", "Controller", "pid", "Controller");

    // tie pid execution to io
    componentManager->Connect("pid", "ExecIn", "io", "ExecOut");
    // connect pid to io
    componentManager->Connect("pid", "RobotJointTorqueInterface", "io", armName);  // see const std::string defined before main()

    //-------------- create the components ------------------
    componentManager->CreateAll();
    componentManager->WaitForStateAll(mtsComponentState::READY, 2.0 * cmn_s);

    // start the periodic Run
    componentManager->StartAll();

    // QtApplication will run in main thread and return control
    // when exited.

    componentManager->KillAll();
    componentManager->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);
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
