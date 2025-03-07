/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2025-02-04

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstCommon/cmnQt.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobotIO1394QtWidgetFactory.h>
#include <sawControllers/mtsPID.h>
#include <sawControllers/mtsPIDQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>

#include <saw_robot_io_1394_ros/mts_ros_crtk_robot_io_bridge.h>
#include <saw_controllers_ros/mts_ros_crtk_controllers_pid_bridge.h>

#include <QApplication>
#include <QMainWindow>

int main(int argc, char ** argv)
{
    // period used for IO and PID
    const double periodIOPID = mtsIntuitiveResearchKit::IOPeriod;

    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClass("mtsPID", CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsIntuitiveResearchKit", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // create ROS node handle
    cisst_ral::ral ral(argc, argv, "dvrk");
    auto rosNode = ral.node();

    // parse options
    cmnCommandLineOptions options;
    std::string portName = mtsRobotIO1394::DefaultPort();
    std::string ioConfigFile, pidConfigFile;
    std::string robotName;
    double publishPeriod = 10.0 * cmn_ms;

    options.AddOptionOneValue("i", "io",
                              "configuration file for robot IO (see sawRobotIO1394)",
                              cmnCommandLineOptions::REQUIRED_OPTION, &ioConfigFile);
    options.AddOptionOneValue("g", "pid",
                              "configuration file for PID controller (see sawControllers, mtsPID)",
                              cmnCommandLineOptions::REQUIRED_OPTION, &pidConfigFile);
    options.AddOptionOneValue("P", "port",
                              "controller port (X, fw:X, udp, udp:XX.XX.XX.XX)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &portName);
    options.AddOptionOneValue("n", "robot-name",
                              "robot name (i.e. PSM1, PSM2, MTML or MTMR) as defined in the sawRobotIO1394 file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &robotName);
    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all arms/teleop components and publish (default 0.01, 10 ms, 100Hz).  There is no point to have a period higher than the arm component's period",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &publishPeriod);

    // check that all required options have been provided
    if (!options.Parse(ral.stripped_arguments(), std::cerr)) {
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments;

    if (!cmnPath::Exists(ioConfigFile)) {
        std::cerr << "File not found: " << ioConfigFile << std::endl;
        return -1;
    }
    if (!cmnPath::Exists(pidConfigFile)) {
        std::cerr << "File not found: " << pidConfigFile << std::endl;
        return -1;
    }

    unsigned int numberOfJoints;
    if ((robotName == "PSM1") || (robotName == "PSM2") || (robotName == "PSM3")) {
        numberOfJoints = 7;
    } else if ((robotName == "MTML") || (robotName == "MTMR")) {
        numberOfJoints = 7;
    } else if (robotName == "ECM") {
        numberOfJoints = 4;
    } else if (robotName == "Focus") {
        numberOfJoints = 1;
    } else {
        std::cerr << "Unknown robot name: " << robotName << ", should be one one PSM1, PSM2, PSM3, MTML, MTMR, ECM or Focus" << std::endl;
        return -1;
    }

    std::cout << "Configuration file for IO: " << ioConfigFile << std::endl
              << "Configuration file for PID: " << pidConfigFile << std::endl
              << "Robot name: " << robotName << std::endl
              << "Number of joints: " << numberOfJoints << std::endl
              << "FirewirePort: " << portName << std::endl;

    mtsManagerLocal * componentManager = mtsManagerLocal::GetInstance();

    // create a Qt user interface
    QApplication application(argc, argv);
    application.setWindowIcon(QIcon(":/dVRK.png"));
    cmnQt::QApplicationExitsOnCtrlC();

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;

    // IO
    mtsRobotIO1394 * io = new mtsRobotIO1394("io", periodIOPID, portName);
    io->Configure(ioConfigFile);
    componentManager->AddComponent(io);

    mtsRobotIO1394QtWidgetFactory * robotWidgetFactory = new mtsRobotIO1394QtWidgetFactory("robotWidgetFactory");
    componentManager->AddComponent(robotWidgetFactory);
    componentManager->Connect("robotWidgetFactory", "RobotConfiguration", "io", "Configuration");
    robotWidgetFactory->Configure();

    mtsRobotIO1394QtWidgetFactory::WidgetListType::const_iterator iterator;
    for (iterator = robotWidgetFactory->Widgets().begin();
         iterator != robotWidgetFactory->Widgets().end();
         ++iterator) {
        tabWidget->addTab(*iterator, (*iterator)->GetName().c_str());
    }
    if (robotWidgetFactory->ButtonsWidget()) {
        tabWidget->addTab(robotWidgetFactory->ButtonsWidget(), "Buttons");
    }


    // Qt PID Controller GUI
    mtsPIDQtWidget * pidGUI = new mtsPIDQtWidget("pidGUI", numberOfJoints);
    pidGUI->Configure();
    componentManager->AddComponent(pidGUI);
    tabWidget->addTab(pidGUI, "PID");

    mtsPID * pid = new mtsPID("pid", periodIOPID);
    pid->Configure(pidConfigFile);
    componentManager->AddComponent(pid);
    // connect pidGUI to pid
    componentManager->Connect("pidGUI", "Controller", "pid", "Controller");

    // tie pid execution to io
    componentManager->Connect("pid", "ExecIn", "io", "ExecOut");
    // connect pid to io
    componentManager->Connect("pid", "RobotJointTorqueInterface", "io", robotName);

    // io bridge uses a factory
    mts_ros_crtk_robot_io_bridge * io_bridge = new mts_ros_crtk_robot_io_bridge("io-bridge", rosNode, "io/",
                                                                                publishPeriod, /* tf */ 0.0,
                                                                                /* rw */ true,
                                                                                /* spin */ true);
    componentManager->AddComponent(io_bridge);
    componentManager->Connect("io-bridge", "RobotConfiguration",
                              "io", "Configuration");
    io_bridge->Configure();

    // controller pid bridge is just derived from crtk bridge
    mts_ros_crtk_controllers_pid_bridge * pid_bridge = new mts_ros_crtk_controllers_pid_bridge("pid-bridge", rosNode,
                                                                                               5.0 * cmn_ms,
                                                                                               /* spin */ false);
    pid_bridge->bridge_interface_provided("pid", "Controller", "pid/" + robotName,
                                          publishPeriod, /* tf */ 0.0,
                                          /* rw */ true);
    componentManager->AddComponent(pid_bridge);
    pid_bridge->Connect();

    //-------------- create the components ------------------
    componentManager->CreateAll();
    componentManager->WaitForStateAll(mtsComponentState::READY, 2.0 * cmn_s);

    // start the periodic Run
    componentManager->StartAll();

    // run Qt user interface
    tabWidget->show();
    application.exec();

    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    // stop all logs
    cmnLogger::Kill();

    // stop ROS node
    cisst_ral::shutdown();

    return 0;
}
