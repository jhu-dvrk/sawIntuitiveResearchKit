/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-07

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitUDPStreamer.h>
#include <sawControllers/mtsTeleOperation.h>
#include <sawControllers/mtsTeleOperationQtWidget.h>

#include <cisstMultiTask/mtsCollectorFactory.h>
#include <cisstMultiTask/mtsCollectorQtFactory.h>
#include <cisstMultiTask/mtsCollectorQtWidget.h>

#include <QTabWidget>

#include <json/json.h>

void fileExists(const std::string & description, const std::string & filename)
{
    if (!cmnPath::Exists(filename)) {
        std::cerr << "File not found for " << description
                  << ": " << filename << std::endl;
        exit(-1);
    } else {
        std::cout << "File found for " << description
                  << ": " << filename << std::endl;
    }
}


int main(int argc, char ** argv)
{
    // configuration
    const double periodIO = 0.5 * cmn_ms;
    const double periodKinematics = 2.0 * cmn_ms;
    const double periodTeleop = 2.0 * cmn_ms;
    const double periodUDP = 20.0 * cmn_ms;
 
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsIntuitiveResearchKit", CMN_LOG_ALLOW_VERBOSE);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // parse options
    cmnCommandLineOptions options;
    int firewirePort = 0;
    std::string gcmip = "-1";
    std::string jsonMainConfigFile;
    std::string jsonCollectionConfigFile;
    typedef std::map<std::string, std::string> ConfigFilesType;
    ConfigFilesType configFiles;
    std::string masterName, slaveName;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &jsonMainConfigFile);

    options.AddOptionOneValue("f", "firewire",
                              "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &firewirePort);
    options.AddOptionOneValue("g", "gcmip",
                              "global component manager IP address",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &gcmip);

    options.AddOptionOneValue("c", "collection-config",
                              "json configuration file for data collection",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &jsonCollectionConfigFile);

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    // make sure the json config file exists and can be parsed
    fileExists("JSON configuration", jsonMainConfigFile);
    std::ifstream jsonStream;
    jsonStream.open(jsonMainConfigFile.c_str());

    Json::Value jsonConfig;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        std::cerr << "Error: failed to parse configuration\n"
                  << jsonReader.getFormattedErrorMessages();
        return -1;
    }

    // start initializing the cisstMultiTask manager
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
    mtsQtApplication * qtAppTask = new mtsQtApplication("QtApplication", argc, argv);
    qtAppTask->Configure();
    componentManager->AddComponent(qtAppTask);

    // console
    mtsIntuitiveResearchKitConsole * console = new mtsIntuitiveResearchKitConsole("console");
    componentManager->AddComponent(console);
    mtsIntuitiveResearchKitConsoleQtWidget * consoleGUI = new mtsIntuitiveResearchKitConsoleQtWidget("consoleGUI");
    componentManager->AddComponent(consoleGUI);
    // connect consoleGUI to console
    componentManager->Connect("console", "Main", "consoleGUI", "Main");

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;
    tabWidget->addTab(consoleGUI, "Main");

    // IO is shared accross components
    mtsRobotIO1394 * io = new mtsRobotIO1394("io", periodIO, firewirePort);

    // find name of button event used to detect if operator is present
    std::string operatorPresentComponent = jsonConfig["operator-present"]["component"].asString();
    std::string operatorPresentInterface = jsonConfig["operator-present"]["interface"].asString();
    //set defaults
    if (operatorPresentComponent == "") {
        operatorPresentComponent = "io";
    }
    if (operatorPresentInterface == "") {
        operatorPresentInterface = "COAG";
    }
    std::cout << "Using \"" << operatorPresentComponent << "::" << operatorPresentInterface
              << "\" to detect if operator is present" << std::endl;

    // setup io defined in the json configuration file
    const Json::Value pairs = jsonConfig["pairs"];
    for (unsigned int index = 0; index < pairs.size(); ++index) {
        // master
        Json::Value jsonMaster = pairs[index]["master"];
        std::string armName = jsonMaster["name"].asString();
        std::string ioFile = jsonMaster["io"].asString();
        fileExists(armName + " IO", ioFile);
        io->Configure(ioFile);
        // slave
        Json::Value jsonSlave = pairs[index]["slave"];
        armName = jsonSlave["name"].asString();
        ioFile = jsonSlave["io"].asString();
        fileExists(armName + " IO", ioFile);
        io->Configure(ioFile);
    }

    componentManager->AddComponent(io);

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

    // setup arms defined in the json configuration file
    for (unsigned int index = 0; index < pairs.size(); ++index) {
        Json::Value jsonMaster = pairs[index]["master"];
        std::string masterName =  jsonMaster["name"].asString();
        std::string masterPIDFile = jsonMaster["pid"].asString();
        std::string masterKinematicFile = jsonMaster["kinematic"].asString();
        std::string masterUDPIP = jsonMaster["UDP-IP"].asString();
        short masterUDPPort = jsonMaster["UDP-port"].asInt();

        fileExists(masterName + " PID", masterPIDFile);
        fileExists(masterName + " kinematic", masterKinematicFile);
        mtsIntuitiveResearchKitConsole::Arm * mtm
                = new mtsIntuitiveResearchKitConsole::Arm(masterName, io->GetName());
        mtm->ConfigurePID(masterPIDFile);
        mtm->ConfigureArm(mtsIntuitiveResearchKitConsole::Arm::ARM_MTM,
                          masterKinematicFile, periodKinematics);
        console->AddArm(mtm);

        if (masterUDPIP != "" || masterUDPPort != 0) {
            if (masterUDPIP != "" && masterUDPPort != 0) {
                std::cout << "Adding UDPStream component for master " << masterName << " to " << masterUDPIP << ":" << masterUDPPort << std::endl;
                mtsIntuitiveResearchKitUDPStreamer * streamer =
                    new mtsIntuitiveResearchKitUDPStreamer(masterName + "UDP", periodUDP, masterUDPIP, masterUDPPort);
                componentManager->AddComponent(streamer);
                // connect to mtm interface to get cartesian position
                componentManager->Connect(streamer->GetName(), "Robot", mtm->Name(), "Robot");
                // connect to io to get clutch events
                componentManager->Connect(streamer->GetName(), "Clutch", "io", "CLUTCH");
                // connect to io to get coag events
                componentManager->Connect(streamer->GetName(), "Coag", "io", "COAG");
            } else {
                std::cerr << "Error for master arm " << masterName << ", you can provided UDP-IP w/o UDP-port" << std::endl;
                exit(-1);
            }
        }

        Json::Value jsonSlave = pairs[index]["slave"];
        std::string slaveName =  jsonSlave["name"].asString();
        std::string slavePIDFile = jsonSlave["pid"].asString();
        std::string slaveKinematicFile = jsonSlave["kinematic"].asString();

        fileExists(slaveName + " PID", slavePIDFile);
        fileExists(slaveName + " kinematic", slaveKinematicFile);
        mtsIntuitiveResearchKitConsole::Arm * psm
                = new mtsIntuitiveResearchKitConsole::Arm(slaveName, io->GetName());
        psm->ConfigurePID(slavePIDFile);
        psm->ConfigureArm(mtsIntuitiveResearchKitConsole::Arm::ARM_PSM,
                          slaveKinematicFile, periodKinematics);
        console->AddArm(psm);

        // PID Master GUI
        std::string masterPIDName = masterName + " PID";
        mtsPIDQtWidget * pidMasterGUI = new mtsPIDQtWidget(masterPIDName, 8);
        pidMasterGUI->Configure();
        componentManager->AddComponent(pidMasterGUI);
        componentManager->Connect(pidMasterGUI->GetName(), "Controller", mtm->PIDComponentName(), "Controller");
        tabWidget->addTab(pidMasterGUI, masterPIDName.c_str());

        // PID Slave GUI
        std::string slavePIDName = slaveName + " PID";
        mtsPIDQtWidget * pidSlaveGUI = new mtsPIDQtWidget(slavePIDName, 7);
        pidSlaveGUI->Configure();
        componentManager->AddComponent(pidSlaveGUI);
        componentManager->Connect(pidSlaveGUI->GetName(), "Controller", psm->PIDComponentName(), "Controller");
        tabWidget->addTab(pidSlaveGUI, slavePIDName.c_str());

        // Master GUI
        mtsIntuitiveResearchKitArmQtWidget * masterGUI = new mtsIntuitiveResearchKitArmQtWidget(mtm->Name() + "GUI");
        masterGUI->Configure();
        componentManager->AddComponent(masterGUI);
        tabWidget->addTab(masterGUI, mtm->Name().c_str());
        // connect masterGUI to master
        componentManager->Connect(masterGUI->GetName(), "Manipulator", mtm->Name(), "Robot");

        // Slave GUI
        mtsIntuitiveResearchKitArmQtWidget * slaveGUI = new mtsIntuitiveResearchKitArmQtWidget(psm->Name() + "GUI");
        slaveGUI->Configure();
        componentManager->AddComponent(slaveGUI);
        tabWidget->addTab(slaveGUI, psm->Name().c_str());
        // connect slaveGUI to slave
        componentManager->Connect(slaveGUI->GetName(), "Manipulator", psm->Name(), "Robot");

        // Teleoperation
        std::string teleName = masterName + "-" + slaveName;
        mtsTeleOperationQtWidget * teleGUI = new mtsTeleOperationQtWidget(teleName + "GUI");
        teleGUI->Configure();
        componentManager->AddComponent(teleGUI);
        tabWidget->addTab(teleGUI, teleName.c_str());
        mtsTeleOperation * tele = new mtsTeleOperation(teleName, periodTeleop);
        // Default orientation between master and slave
        vctMatRot3 master2slave;
        master2slave.From(vctAxAnRot3(vct3(0.0, 0.0, 1.0), 180.0 * cmnPI_180));
        tele->SetRegistrationRotation(master2slave);
        componentManager->AddComponent(tele);
        // connect teleGUI to tele
        componentManager->Connect(teleGUI->GetName(), "TeleOperation", tele->GetName(), "Setting");

        componentManager->Connect(tele->GetName(), "Master", mtm->Name(), "Robot");
        componentManager->Connect(tele->GetName(), "Slave", psm->Name(), "Robot");
        componentManager->Connect(tele->GetName(), "Clutch", "io", "CLUTCH");
        componentManager->Connect(tele->GetName(), "OperatorPresent", operatorPresentComponent, operatorPresentInterface);
    }

    // configure data collection if needed
    if (options.IsSet("collection-config")) {
        // make sure the json config file exists
        fileExists("JSON data collection configuration", jsonCollectionConfigFile);

        mtsCollectorFactory * collectorFactory = new mtsCollectorFactory("collectors");
        collectorFactory->Configure(jsonCollectionConfigFile);
        componentManager->AddComponent(collectorFactory);
        collectorFactory->Connect();

        mtsCollectorQtWidget * collectorQtWidget = new mtsCollectorQtWidget();
        tabWidget->addTab(collectorQtWidget, "Collection");

        mtsCollectorQtFactory * collectorQtFactory = new mtsCollectorQtFactory("collectorsQt");
        collectorQtFactory->SetFactory("collectors");
        componentManager->AddComponent(collectorQtFactory);
        collectorQtFactory->Connect();
        collectorQtFactory->ConnectToWidget(collectorQtWidget);
    }

    // show all widgets
    tabWidget->show();

    //-------------- create the components ------------------
    io->CreateAndWait(2.0 * cmn_s); // this will also create the pids as they are in same thread
    io->StartAndWait(2.0 * cmn_s);

    // start all other components
    componentManager->CreateAllAndWait(2.0 * cmn_s);
    componentManager->StartAllAndWait(2.0 * cmn_s);

    // QtApplication will run in main thread and return control
    // when exited.

    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    // delete dvgc robot
    delete io;
    delete robotWidgetFactory;

    // stop all logs
    cmnLogger::Kill();

    return 0;
}
