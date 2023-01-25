/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-07

  (C) Copyright 2013-2023 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnQt.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsCollectorFactory.h>
#include <cisstMultiTask/mtsCollectorQtFactory.h>
#include <cisstMultiTask/mtsCollectorQtWidget.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQt.h>

#include <QApplication>
#include <QIcon>
#include <QLocale>

#include <clocale>

void fileExists(const std::string & description, const std::string & filename)
{
    if (!cmnPath::Exists(filename)) {
        std::cerr << "File not found: " << description
                  << "; " << filename << std::endl;
        exit(-1);
    } else {
        std::cout << "File found: " << description
                  << "; " << filename << std::endl;
    }
}


int main(int argc, char ** argv)
{
    // replace the C++ global locale by C locale
    std::setlocale(LC_ALL, "C");

    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsIntuitiveResearchKit", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);
    // add log file with date so logs don't get overwritten
    std::string currentDateTime;
    osaGetDateTimeString(currentDateTime);
    std::ofstream logFileStream(std::string("cisstLog-" + currentDateTime + ".txt").c_str());
    cmnLogger::AddChannel(logFileStream);
    cmnLogger::HaltDefaultLog(); // stop log to default cisstLog.txt

    // parse options
    cmnCommandLineOptions options;
    std::string jsonMainConfigFile;
    std::list<std::string> jsonIOConfigFiles;
    std::string jsonCollectionConfigFile;
    std::list<std::string> managerConfig;
    std::string qtStyle;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &jsonMainConfigFile);

    options.AddOptionMultipleValues("i", "ros-io-config",
                                    "json config file to configure ROS bridges to collect low level data (IO)",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &jsonIOConfigFiles);

    options.AddOptionNoValue("t", "text-only",
                             "text only interface, do not create Qt widgets");

    options.AddOptionOneValue("c", "collection-config",
                              "json configuration file for data collection using cisstMultiTask state table collector",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &jsonCollectionConfigFile);

    options.AddOptionNoValue("C", "calibration-mode",
                             "run in calibration mode, doesn't use potentiometers to monitor encoder values and always force re-homing.  This mode should only be used when calibrating your potentiometers.");

    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON files to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);

    options.AddOptionOneValue("S", "qt-style",
                              "Qt style, use this option with a random name to see available styles",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &qtStyle);

    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments;

    const bool hasQt = !options.IsSet("text-only");

    // start creating components
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // console
    mtsIntuitiveResearchKitConsole * console = new mtsIntuitiveResearchKitConsole("console");
    console->set_calibration_mode(options.IsSet("calibration-mode"));
    fileExists("console JSON configuration file", jsonMainConfigFile);
    console->Configure(jsonMainConfigFile);
    componentManager->AddComponent(console);
    console->Connect();

    QApplication * application;
    mtsIntuitiveResearchKitConsoleQt * consoleQt = 0;
    // add all Qt widgets if needed
    if (hasQt) {
        QLocale::setDefault(QLocale::English);
        application = new QApplication(argc, argv);
        application->setWindowIcon(QIcon(":/dVRK.png"));
        cmnQt::QApplicationExitsOnCtrlC();
        if (options.IsSet("qt-style")) {
            std::string errorMessage = cmnQt::SetStyle(qtStyle);
            if (errorMessage != "") {
                std::cerr << errorMessage << std::endl;
                return -1;
            }
        }
        if (options.IsSet("dark-mode")) {
            cmnQt::SetDarkMode();
        }
        consoleQt = new mtsIntuitiveResearchKitConsoleQt();
        consoleQt->Configure(console);
        consoleQt->Connect();
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
        consoleQt->addTab(collectorQtWidget, "Collection");

        mtsCollectorQtFactory * collectorQtFactory = new mtsCollectorQtFactory("collectorsQt");
        collectorQtFactory->SetFactory("collectors");
        componentManager->AddComponent(collectorQtFactory);
        collectorQtFactory->Connect();
        collectorQtFactory->ConnectToWidget(collectorQtWidget);
    }

    // custom user component
    if (!componentManager->ConfigureJSON(managerConfig)) {
        CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager, check cisstLog for error messages" << std::endl;
        return -1;
    }

    //-------------- create the components ------------------
    componentManager->CreateAllAndWait(2.0 * cmn_s);
    componentManager->StartAllAndWait(2.0 * cmn_s);

    if (hasQt) {
        application->exec();
    } else {
        do {
            std::cout << "Press 'q' to quit" << std::endl;
        } while (cmnGetChar() != 'q');
    }

    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    // stop all logs
    cmnLogger::Kill();
    cmnLogger::RemoveChannel(logFileStream);

    delete console;
    if (hasQt) {
        delete consoleQt;
    }

    return 0;
}
