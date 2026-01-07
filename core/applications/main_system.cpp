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
#include <cisstMultiTask/mtsCollectorFactoryQtWidget.h>

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
    std::string _IRE_shell = "";
    std::list<std::string> managerConfig;
    std::string qtStyle;
    std::string dotFile;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &jsonMainConfigFile);

    options.AddOptionOneValue("c", "collection-config",
                              "json configuration file for data collection using cisstMultiTask state table collector",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &jsonCollectionConfigFile);

    options.AddOptionNoValue("C", "calibration-mode",
                             "run in calibration mode, doesn't use potentiometers to monitor encoder values and always force re-homing.  This mode should only be used when calibrating your potentiometers.");

    options.AddOptionOneValue("e", "embedded-python",
                              "start an embedded Python shell to access all dVRK software components",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &_IRE_shell);

    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON files to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);

    options.AddOptionOneValue("S", "qt-style",
                              "Qt style, use this option with a random name to see available styles",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &qtStyle);

    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");

    options.AddOptionNoValue("t", "text-only",
                             "text only interface, do not create Qt widgets");

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
    mtsManagerLocal * component_manager = mtsManagerLocal::GetInstance();

    // system
    auto * system = new dvrk::system("system");
    system->set_calibration_mode(options.IsSet("calibration-mode"));
    system->set_embedded_python(_IRE_shell);
    file_exists("dVRK system JSON configuration file", jsonMainConfigFile, system);
    system->Configure(jsonMainConfigFile);
    component_manager->AddComponent(system);
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

        auto * collectors = new mtsCollectorFactory("collectors");
        collectors->Configure(jsonCollectionConfigFile);
        component_manager->AddComponent(collectors);
        collectors->Connect();

        if (hasQt) {
            auto * collectors_Qt = new mtsCollectorFactoryQtWidget("collectors_Qt");
            system_Qt->add_tab(collectors_Qt, "Collection");
            component_manager->AddComponent(collectors_Qt);
            component_manager->Connect(collectors_Qt->GetName(), "Collector",
                                       collectors->GetName(), "Control");
        }
    }

    // custom user component
    if (!component_manager->ConfigureJSON(managerConfig)) {
        CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager, check cisstLog for error messages" << std::endl;
        return -1;
    }

    //-------------- create the components ------------------
    component_manager->CreateAllAndWait(2.0 * cmn_s);
    component_manager->StartAllAndWait(2.0 * cmn_s);

    if (!dotFile.empty()) {
        std::ofstream dotStream(dotFile, std::ofstream::out);
        component_manager->ToStreamDot(dotStream);
        dotStream.close();
    }
    
    if (hasQt) {
        application->exec();
    } else {
        do {
            std::cout << "Press 'q' to quit" << std::endl;
        } while (cmnGetChar() != 'q');
    }

    component_manager->KillAllAndWait(2.0 * cmn_s);
    component_manager->Cleanup();

    // stop all logs
    logger.Stop();

    delete system;
    if (hasQt) {
        delete system_Qt;
    }

    return 0;
}
