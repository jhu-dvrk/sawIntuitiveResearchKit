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
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQt.h>

// #include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitUDPStreamer.h>
// #include <sawControllers/mtsTeleOperation.h>

//#include <cisstMultiTask/mtsCollectorFactory.h>
//#include <cisstMultiTask/mtsCollectorQtFactory.h>
//#include <cisstMultiTask/mtsCollectorQtWidget.h>

#include <QApplication>

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
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsIntuitiveResearchKit", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // parse options
    cmnCommandLineOptions options;
    std::string gcmip = "-1";
    std::string jsonMainConfigFile;
    std::string jsonCollectionConfigFile;
    typedef std::map<std::string, std::string> ConfigFilesType;
    ConfigFilesType configFiles;
    std::string masterName, slaveName;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &jsonMainConfigFile);

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


    // console
    mtsIntuitiveResearchKitConsole * console = new mtsIntuitiveResearchKitConsole("console");
    console->Configure(jsonMainConfigFile);
    componentManager->AddComponent(console);
    console->Connect();

    // add all Qt widgets
    QApplication application(argc, argv);

    mtsIntuitiveResearchKitConsoleQt * consoleQt = new mtsIntuitiveResearchKitConsoleQt(console);

#if 0
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

    // connect console to IO
    componentManager->Connect(console->GetName(), "Clutch",
                              io->GetName(), "CLUTCH");
    componentManager->Connect(console->GetName(), "Camera",
                              io->GetName(), "CAMERA");
    componentManager->Connect(console->GetName(), "OperatorPresent",
                              operatorPresentComponent, operatorPresentInterface);
#endif

#if 0
    // setup arms defined in the json configuration file
    for (unsigned int index = 0; index < pairs.size(); ++index) {
        std::string masterUDPIP = jsonMaster["UDP-IP"].asString();
        short masterUDPPort = jsonMaster["UDP-port"].asInt();

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
        componentManager->Connect(tele->GetName(), "Clutch", console->GetName(), "Clutch");
        componentManager->Connect(tele->GetName(), "OperatorPresent", console->GetName(), "OperatorPresent");
        console->AddTeleOperation(tele->GetName());
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
#endif

    //-------------- create the components ------------------
    componentManager->CreateAllAndWait(2.0 * cmn_s);
    componentManager->StartAllAndWait(2.0 * cmn_s);

    application.exec();

    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    // stop all logs
    cmnLogger::Kill();

    delete consoleQt;

    return 0;
}
