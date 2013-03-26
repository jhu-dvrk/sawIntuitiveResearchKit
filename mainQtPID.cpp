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
// cisst/saw
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobotIO1394QtWidget.h>
#include <sawControllers/mtsPID.h>
#include <sawControllers/mtsPIDQtWidget.h>

int main(int argc, char ** argv)
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // create a Qt application
    QApplication application(argc, argv);

    // parse options
    cmnCommandLineOptions options;
    int firewirePort;
    std::string ioConfigFile, pidConfigFile;
    options.AddOptionOneValue("i", "io",
                              "configuration file for robot IO (see sawRobotIO1394)",
                              cmnCommandLineOptions::REQUIRED, &ioConfigFile);
    options.AddOptionOneValue("p", "pid",
                              "configuration file for PID controller (see sawControllers, mtsPID)",
                              cmnCommandLineOptions::REQUIRED, &pidConfigFile);
    options.AddOptionOneValue("f", "firewire",
                              "firefire port number(s)",
                              cmnCommandLineOptions::REQUIRED, &firewirePort);
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
    std::cout << "Configuration file for IO: " << ioConfigFile << std::endl
              << "Configuration file for PID: " << pidConfigFile << std::endl
              << "FirewirePort: " << firewirePort << std::endl;

    mtsManagerLocal * manager = mtsManagerLocal::GetInstance();

    // IO
    mtsRobotIO1394QtWidget * ioGUI = new mtsRobotIO1394QtWidget("ioGUI");
    ioGUI->Configure();
    manager->AddComponent(ioGUI);
    mtsRobotIO1394 * io = new mtsRobotIO1394("io", 1 * cmn_ms, firewirePort, ioGUI->GetOutputStream());
    io->Configure(ioConfigFile);
    manager->AddComponent(io);
    // connect ioGUI to io
    manager->Connect("ioGUI", "Robot", "io", "MTML");
    manager->Connect("ioGUI", "RobotActuators", "io", "MTMLActuators");

    // Qt PID Controller GUI
    mtsPIDQtWidget * pidGUI = new mtsPIDQtWidget("pidGUI");
    pidGUI->Configure();
    manager->AddComponent(pidGUI);
    mtsPID * pid = new mtsPID("pid", 1 * cmn_ms);
    pid->Configure(pidConfigFile);
    manager->AddComponent(pid);
    // connect pidGUI to pid
    manager->Connect("pidGUI", "Controller", "pid", "Controller");

    // tie pid execution to io
    manager->Connect("pid", "ExecIn", "io", "ExecOut");
    // connect pid to io
    manager->Connect("pid", "RobotJointTorqueInterface", "io", "MTML");

    //-------------- create the components ------------------
    manager->CreateAll();
    manager->WaitForStateAll(mtsComponentState::READY, 2.0 * cmn_s);

    // start the periodic Run
    manager->StartAll();

    // create a main window to hold QWidget
    ioGUI->show();
    pidGUI->show();

    // run Qt app
    application.exec();

    // cleanup
    manager->KillAll();
    manager->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);
    manager->Cleanup();

    // delete dvgc robot
    delete pid;
    delete pidGUI;
    delete io;
    delete ioGUI;

    // stop all logs
    cmnLogger::Kill();

    return 0;
}
