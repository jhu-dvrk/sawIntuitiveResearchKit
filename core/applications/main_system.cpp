/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-07

  (C) Copyright 2013-2026 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnQt.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsCommandLineOptionsQt.h>

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
    mtsCommandLineOptionsQt options;
    std::string jsonMainConfigFile;
    std::string _IRE_shell = "";

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &jsonMainConfigFile);

    options.AddOptionNoValue("C", "calibration-mode",
                             "run in calibration mode, doesn't use potentiometers to monitor encoder values and always force re-homing.  This mode should only be used when calibrating your potentiometers.");

    options.AddOptionOneValue("e", "embedded-python",
                              "start an embedded Python shell to access all dVRK software components",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &_IRE_shell);

    // check that all required options have been provided
    if (!options.Parse(argc, argv, std::cerr)) {
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments;

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

    QLocale::setDefault(QLocale::English);
    QApplication * application = new QApplication(argc, argv);
    application->setWindowIcon(QIcon(":/dVRK.png"));
    dvrk::system_Qt * system_Qt = new dvrk::system_Qt();
    system_Qt->configure(system);
    system_Qt->connect();
 

    // collector factory and component viewer are handled by options.Apply()
    options.Apply();

    //-------------- create the components ------------------
    component_manager->CreateAllAndWait(2.0 * cmn_s);
    component_manager->StartAllAndWait(2.0 * cmn_s);

    application->exec();

    component_manager->KillAllAndWait(2.0 * cmn_s);
    component_manager->Cleanup();

    // stop all logs
    logger.Stop();

    delete system;
    delete system_Qt;

    return 0;
}
