/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-07

  (C) Copyright 2013-2025 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsCollectorFactory.h>
#include <cisstMultiTask/mtsCollectorQtFactory.h>
#include <cisstMultiTask/mtsCollectorQtWidget.h>

#include <sawIntuitiveResearchKit/system.h>
#include <sawIntuitiveResearchKit/system_Qt.h>

#include <QApplication>
#include <QIcon>
#include <QLocale>

#include <clocale>

void file_exists(const std::string & description, std::string & filename,
                 dvrk::system * system)
{
    if (!cmnPath::Exists(filename)) {
        const std::string fileInPath = system->find_file(filename);
        if (fileInPath == "") {
            std::cerr << "File not found: " << description
                      << ": " << filename << std::endl;
            exit(-1);
        } else {
            filename = fileInPath;
        }
    }
    std::cout << "Using file: " << description
              << ": " << filename << std::endl;
}

int main(int argc, char ** argv)
{
    // replace the C++ global locale by C locale
    std::setlocale(LC_ALL, "C");

    // log configuration
    mtsIntuitiveResearchKit::Logger logger; 

    // parse options
    cmnCommandLineOptions options;
    std::string jsonMainConfigFile;
    std::string jsonCollectionConfigFile;
    std::list<std::string> managerConfig;
    std::string qtStyle;
    std::string dotFile;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &jsonMainConfigFile);

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

    options.AddOptionOneValue("d", "dot-file",
                              "graphviz compatible dot file",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &dotFile);

    // check that all required options have been provided
    if (!options.Parse(argc, argv, std::cerr)) {
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments;

    const bool hasQt = !options.IsSet("text-only");

    // start creating components
    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // system
    auto * system = new dvrk::system("dVRK_system");
    system->set_calibration_mode(options.IsSet("calibration-mode"));
    file_exists("dVRK system JSON configuration file", jsonMainConfigFile, system);
    system->Configure(jsonMainConfigFile);
    componentManager->AddComponent(system);
    system->Connect();

    QApplication * application;
    dvrk::system_Qt * system_Qt = nullptr;
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
        system_Qt = new dvrk::system_Qt();
        system_Qt->configure(system);
        system_Qt->connect();
    }

    // configure data collection if needed
    if (options.IsSet("collection-config")) {
        // make sure the json config file exists
        file_exists("JSON data collection configuration", jsonCollectionConfigFile, system);

        auto * collectorFactory = new mtsCollectorFactory("collectors");
        collectorFactory->Configure(jsonCollectionConfigFile);
        componentManager->AddComponent(collectorFactory);
        collectorFactory->Connect();

        if (hasQt) {
            auto * collectorQtWidget = new mtsCollectorQtWidget();
            system_Qt->add_tab(collectorQtWidget, "Collection");

            auto * collectorQtFactory = new mtsCollectorQtFactory("collectorsQt");
            collectorQtFactory->SetFactory("collectors");
            componentManager->AddComponent(collectorQtFactory);
            collectorQtFactory->Connect();
            collectorQtFactory->ConnectToWidget(collectorQtWidget);
        }
    }

    // custom user component
    if (!componentManager->ConfigureJSON(managerConfig)) {
        CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager, check cisstLog for error messages" << std::endl;
        return -1;
    }

    //-------------- create the components ------------------
    componentManager->CreateAllAndWait(2.0 * cmn_s);
    componentManager->StartAllAndWait(2.0 * cmn_s);

    if (!dotFile.empty()) {
        std::ofstream dotStream(dotFile, std::ofstream::out);
        componentManager->ToStreamDot(dotStream);
        dotStream.close();
    }
    
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
    logger.Stop();

    delete system;
    if (hasQt) {
        delete system_Qt;
    }

    return 0;
}
