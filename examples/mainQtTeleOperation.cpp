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
#include <sawRobotIO1394/mtsRobotIO1394QtManager.h>
#include <sawControllers/mtsPID.h>
#include <sawControllers/mtsPIDQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitMTM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSM.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQtWidget.h>
#include <sawControllers/mtsTeleOperation.h>
#include <sawControllers/mtsTeleOperationQtWidget.h>
#include <sawTextToSpeech/mtsTextToSpeech.h>

int main(int argc, char ** argv)
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
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
    mtsManagerLocal * manager = 0;
    if (gcmip != "-1") {
        try {
            manager = mtsManagerLocal::GetInstance(gcmip, processname);
        } catch(...) {
            std::cerr << "Failed to get GCM instance." << std::endl;
            return -1;
        }
    } else {
        manager = mtsManagerLocal::GetInstance();
    }


    // create a Qt application
    mtsQtApplication *qtAppTask = new mtsQtApplication("QtApplication", argc, argv);
    qtAppTask->Configure();
    manager->AddComponent(qtAppTask);

    // IO
    mtsRobotIO1394 * io = new mtsRobotIO1394("io", 1.0 * cmn_ms, firewirePort);
    io->Configure(configFiles["io-master"]);
    io->Configure(configFiles["io-slave"]);
    manager->AddComponent(io);
    // connect ioGUIMaster to io
    mtsRobotIO1394QtManager * qtManager = new mtsRobotIO1394QtManager("qtManager");
    manager->AddComponent(qtManager);
    manager->Connect("qtManager","Configuration_Qt","io","Configuration");
    qtManager->Configure();

    // PID Master
    mtsPIDQtWidget * pidMasterGUI = new mtsPIDQtWidget("PID Master", 8);
    pidMasterGUI->Configure();
    manager->AddComponent(pidMasterGUI);
    mtsPID * pidMaster = new mtsPID("pid-master", 1.0 * cmn_ms);
    pidMaster->Configure(configFiles["pid-master"]);
    manager->AddComponent(pidMaster);
    // connect pidGUI to pid
    manager->Connect(pidMasterGUI->GetName(), "Controller", pidMaster->GetName(), "Controller");

    // PID Slave
    mtsPIDQtWidget * pidSlaveGUI = new mtsPIDQtWidget("PID Slave", 7);
    pidSlaveGUI->Configure();
    manager->AddComponent(pidSlaveGUI);
    mtsPID * pidSlave = new mtsPID("pid-slave", 1.0 * cmn_ms);
    pidSlave->Configure(configFiles["pid-slave"]);
    manager->AddComponent(pidSlave);
    // connect pidGUI to pid
    manager->Connect(pidSlaveGUI->GetName(), "Controller", pidSlave->GetName(), "Controller");

    // MTM
    mtsIntuitiveResearchKitMTM * master = new mtsIntuitiveResearchKitMTM(masterName, 5.0 * cmn_ms);
    master->Configure(configFiles["kinematic-master"]);
    manager->AddComponent(master);
    manager->Connect(master->GetName(), "RobotIO", "io", masterName);

    // PSM
    mtsIntuitiveResearchKitPSM * slave = new mtsIntuitiveResearchKitPSM(slaveName, 5.0 * cmn_ms);
    slave->Configure(configFiles["kinematic-slave"]);
    manager->AddComponent(slave);
    manager->Connect(slave->GetName(), "RobotIO", "io", slaveName);

    // Teleoperation
    mtsTeleOperationQtWidget * teleGUI = new mtsTeleOperationQtWidget("teleGUI");
    teleGUI->Configure();
    manager->AddComponent(teleGUI);
    mtsTeleOperation * tele = new mtsTeleOperation("tele", 5.0 * cmn_ms);
    manager->AddComponent(tele);
    // connect teleGUI to tele
    manager->Connect("teleGUI", "TeleOperation", "tele", "Setting");

    // TextToSpeech
    // ZC: make this optional based on CMake option (TODO)
    mtsTextToSpeech* textToSpeech = new mtsTextToSpeech;
    textToSpeech->AddInterfaceRequiredForEventString("ErrorMsg", "RobotErrorMsg");
    manager->AddComponent(textToSpeech);
    manager->Connect(textToSpeech->GetName(), "ErrorMsg", slave->GetName(), "Robot");


    // connect interfaces
    manager->Connect(pidMaster->GetName(), "RobotJointTorqueInterface", "io", masterName);
    manager->Connect(pidSlave->GetName(), "RobotJointTorqueInterface", "io", slaveName);

    // Qt console
    mtsIntuitiveResearchKitConsoleQtWidget * console = new mtsIntuitiveResearchKitConsoleQtWidget("console");
    manager->AddComponent(console);

    manager->Connect(master->GetName(), "PID", pidMaster->GetName(), "Controller");
    manager->Connect(slave->GetName(), "PID", pidSlave->GetName(), "Controller");
    manager->Connect(slave->GetName(), "Adapter", "io", slaveName + "-Adapter");
    manager->Connect(slave->GetName(), "Tool", "io", slaveName + "-Tool");
    manager->Connect(slave->GetName(), "ManipClutch", "io", slaveName + "-ManipClutch");
    manager->Connect(slave->GetName(), "SUJClutch", "io", slaveName + "-SUJClutch");


    // connect console to Master & Slave
    manager->Connect("console", "MTM", master->GetName(), "Robot");
    manager->Connect("console", "PSM", slave->GetName(), "Robot");

    // connect teleop to Master + Slave + Clutch
    manager->Connect("tele", "Master", master->GetName(), "Robot");
    manager->Connect("tele", "Slave", slave->GetName(), "Robot");
    manager->Connect("tele", "Clutch", "io", "CLUTCH");

    // execute in following order using a single thread
    manager->Connect(pidMaster->GetName(), "ExecIn", "io", "ExecOut");
    manager->Connect(pidSlave->GetName(), "ExecIn", "io", "ExecOut");

    // execute in following order using a single thread
    manager->Connect(slave->GetName(), "ExecIn", master->GetName(), "ExecOut");
    manager->Connect(tele->GetName(), "ExecIn", slave->GetName(), "ExecOut");

    //-------------- create the components ------------------
    io->CreateAndWait(2.0 * cmn_s); // this will also create the pids as they are in same thread
    io->StartAndWait(2.0 * cmn_s);
    pidMaster->StartAndWait(2.0 * cmn_s);
    pidSlave->StartAndWait(2.0 * cmn_s);

    // start all other components
    manager->CreateAllAndWait(2.0 * cmn_s);
    manager->StartAllAndWait(2.0 * cmn_s);

    // QtApplication will run in main thread and return control
    // when exited.

    manager->KillAllAndWait(2.0 * cmn_s);
    manager->Cleanup();

    // delete dvgc robot
    delete tele;
    delete master;
    delete slave;
    delete pidMaster;
    delete pidSlave;
    delete pidMasterGUI;
    delete pidSlaveGUI;
    delete io;
    delete qtManager;

    // stop all logs
    cmnLogger::Kill();

    return 0;
}
