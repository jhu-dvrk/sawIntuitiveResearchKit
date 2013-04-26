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
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobotIO1394QtManager.h>
#include <sawControllers/mtsPID.h>
#include <sawControllers/mtsPIDQtWidget.h>
#include <sawControllers/mtsTeleOperation.h>
#include <sawControllers/mtsTeleOperationQtWidget.h>

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
    std::string gcmip = "-1";
    typedef std::map<std::string, std::string> ConfigFilesType;
    ConfigFilesType configFiles;

    std::string ioConfigFileMaster, pidConfigFileMaster, kinConfigFileMaster;
    std::string ioConfigFileSlave, pidConfigFileSlave, kinConfigFileSlave;
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

    options.AddOptionOneValue("f", "firewire",
                              "firewire port number(s)",
                              cmnCommandLineOptions::REQUIRED, &firewirePort);

    options.AddOptionOneValue("g", "gcmip",
                              "global component manager IP addrress",
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
    mtsManagerLocal *manager = NULL;
    if(gcmip != "-1"){
        try{
            manager = mtsManagerLocal::GetInstance( gcmip, processname );
        }catch(...){
            std::cerr << "Failed to get GCM instance." << std::endl;
            return -1;
        }
    }else{
        manager = mtsManagerLocal::GetInstance();
    }




    // IO
    mtsRobotIO1394 * io = new mtsRobotIO1394("io", 1.0 * cmn_ms, firewirePort);
    io->Configure(configFiles["io-master"]);
    io->Configure(configFiles["io-slave"]);
    manager->AddComponent(io);
    // connect ioGUIMaster to io
    mtsRobotIO1394QtManager * qtManager = new mtsRobotIO1394QtManager("qtManager");
    manager->AddComponent(qtManager);
    manager->Connect("qtManager","Configuration_Qt","io","Configuration");
    qtManager->BuildWidgets();

    // PID
    mtsPIDQtWidget * pidGUI = new mtsPIDQtWidget("pidGUI", 8);
    pidGUI->Configure();
    manager->AddComponent(pidGUI);
    mtsPID * pid = new mtsPID("pid", 1 * cmn_ms);
    pid->Configure(configFiles["pid-master"]);
    manager->AddComponent(pid);
    // connect pidGUI to pid
    manager->Connect("pidGUI", "Controller", "pid", "Controller");

    // Teleoperation
    mtsTeleOperationQtWidget * teleGUI = new mtsTeleOperationQtWidget("teleGUI");
    teleGUI->Configure();
    manager->AddComponent(teleGUI);
    mtsTeleOperation * tele = new mtsTeleOperation("tele", 1.0 * cmn_ms);
    tele->ConfigureMaster(configFiles["kinematic-master"]);
    manager->AddComponent(tele);
    // connect teleGUI to tele
    manager->Connect("teleGUI", "TeleOperation", "tele", "Setting");

    // connect interfaces
    manager->Connect("pid", "RobotJointTorqueInterface", "io", "MTML");
    manager->Connect("tele", "Master", "io", "MTML");

    // execute in following order usign a single thread
    manager->Connect("pid", "ExecIn", "io", "ExecOut");
    manager->Connect("tele", "ExecIn", "pid", "ExecOut");

    //-------------- create the components ------------------
    manager->CreateAll();
    manager->WaitForStateAll(mtsComponentState::READY, 2.0 * cmn_s);

    // start the periodic Run
    manager->StartAll();

    // create a main window to hold QWidget
    pidGUI->show();
    teleGUI->show();

    // run Qt app
    application.exec();

    manager->KillAll();
    manager->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);
    manager->Cleanup();

    // delete dvgc robot
    delete pid;
    delete pidGUI;
    delete io;
    delete qtManager;

    // stop all logs
    cmnLogger::Kill();

    return 0;
}
