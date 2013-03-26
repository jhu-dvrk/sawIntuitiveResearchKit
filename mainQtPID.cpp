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
    int port;
    std::string configFile;
    options.AddOptionOneValue("c", "config",
                              "configuration file, can be an absolute path or relative to CISST_ROOT share",
                              cmnCommandLineOptions::REQUIRED, &configFile);
    options.AddOptionOneValue("p", "port",
                              "firefire port number(s)",
                              cmnCommandLineOptions::REQUIRED, &port);
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }

    std::string fullFileName;
    if (cmnPath::Exists(configFile)) {
        fullFileName = configFile;
    } else {
        cmnPath path;
        path.AddRelativeToCisstShare("sawRobotIO1394");
        fullFileName = path.Find(configFile);
        if (fullFileName == "") {
            return 0;
        }
    }
    std::cout << "Configuration file: " << fullFileName << std::endl
              << "Port: " << port << std::endl;

    mtsManagerLocal *LCM = mtsManagerLocal::GetInstance();

    // Qt display task
    mtsRobotIO1394QtWidget *disp = new mtsRobotIO1394QtWidget("disp");
    disp->Configure();
    LCM->AddComponent(disp);

    // Robot
    mtsRobotIO1394 *robot = new mtsRobotIO1394("robot", 1 * cmn_ms, port, disp->GetOutputStream());
    robot->Configure(fullFileName);
    LCM->AddComponent(robot);

    // connect disp to Robot & Controller interface
    LCM->Connect("disp", "Robot", "robot", "MTML");
    LCM->Connect("disp", "RobotActuators", "robot", "MTMLActuators");

    //-------------- create the components ------------------
    LCM->CreateAll();
    LCM->WaitForStateAll(mtsComponentState::READY, 2.0 * cmn_s);

    // start the periodic Run
    LCM->StartAll();

    // create a main window to hold QWidget
    disp->show();

    // run Qt app
    application.exec();

    // cleanup
    LCM->KillAll();
    LCM->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);
    LCM->Cleanup();

    // delete dvgc robot
    delete disp;
    delete robot;

    // stop all logs
    cmnLogger::Kill();

    return 0;
}
