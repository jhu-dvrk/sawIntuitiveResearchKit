/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*

  Author(s):  Anton Deguet
  Created on: 2013-12-20

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

// system
#include <iostream>
#include <limits>

// cisst/saw
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnKbHit.h>
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <sawRobotIO1394/osaConfiguration1394.h>
#include <sawRobotIO1394/osaXML1394.h>
#include <sawRobotIO1394/osaPort1394.h>
#include <sawRobotIO1394/osaRobot1394.h>

using namespace sawRobotIO1394;

int main(int argc, char * argv[])
{
    cmnCommandLineOptions options;
    int portNumber = 0;
    std::string configFile;
    options.AddOptionOneValue("c", "config",
                              "configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &configFile);
    options.AddOptionOneValue("p", "port",
                              "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &portNumber);
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }

    if (!cmnPath::Exists(configFile)) {
        std::cerr << "Can't find file \"" << configFile << "\"." << std::endl;
        return -1;
    }
    std::cout << "Configuration file: " << configFile << std::endl
              << "Port: " << portNumber << std::endl;

    std::cout << "Make sure:" << std::endl
              << " - your computer is connected to the firewire controller." << std::endl
              << " - the MTM arm corresponding to the configuration file \"" << configFile << "\" is connected to the controller." << std::endl
              << " - the E-Stop is opened, this program doesn't require powered actuators." << std::endl
              << " - you have no other device connected to the firewire chain." << std::endl
              << " - you have no other program trying to communicate with the controller." << std::endl
              << std::endl
              << "Press any key to start." << std::endl;
    cmnGetChar();

    std::cout << "Loading config file ..." << std::endl;
    osaPort1394Configuration config;
    osaXML1394ConfigurePort(configFile, config);

    std::cout << "Creating robot ..." << std::endl;
    if (config.Robots.size() == 0) {
        std::cerr << "Error: the config file doesn't define a robot." << std::endl;
        return -1;
    }
    if (config.Robots.size() != 1) {
        std::cerr << "Error: the config file defines more than one robot." << std::endl;
        return -1;
    }
    osaRobot1394 * robot = new osaRobot1394(config.Robots[0]);
    size_t numberOfActuators = robot->NumberOfActuators();

    std::cout << "Creating port ..." << std::endl;
    osaPort1394 * port = new osaPort1394(portNumber);
    port->AddRobot(robot);

    std::cout << std::endl
              << "Press any key to start collecting data." << std::endl;
    cmnGetChar();
    std::cout << "Fully open and close the gripper up to the second spring on the MTM multiple times." << std::endl
              << "NOTE: It is very important to not close the gripper all the way, stop when you feel some resitance from the second spring." << std::endl
              << "+ indicates a new maximum, - indicates a new minimum." << std::endl
              << "Press any key to stop collecting data." << std::endl;

    double minRad = std::numeric_limits<double>::max();
    double maxRad = std::numeric_limits<double>::min();
    size_t counter = 0;
    while (1) {
        if (cmnKbHit()) {
            break;
        }
        port->Read();
        counter++;
        // read values in radians to test existing coeffcients
        double value = robot->PotPosition().at(numberOfActuators - 1);
        if (value > maxRad) {
            maxRad = value;
            std::cout << "+" << std::flush;
        } else if (value < minRad) {
            minRad = value;
            std::cout << "-" << std::flush;
        }
        osaSleep(1.0 * cmn_ms);
    }
    cmnGetChar(); // to read the pressed key

    std::cout << std::endl
              << "Status: found range [" << minRad * 180.0 / cmnPI << ", " << maxRad * 180.0 / cmnPI
              << "] degrees using " << counter << " samples." << std::endl
              << std::endl
              << "Do you want to update the config file with these values? [Y/y]" << std::endl;

    // save if needed
    char key = cmnGetChar();
    if ((key == 'y') || (key == 'Y')) {
        cmnXMLPath xmlConfig;
        xmlConfig.SetInputSource(configFile);
        // query previous current offset and scales
        double previousOffset;
        double previousScale;
        const char * context = "Config";
        xmlConfig.GetXMLValue(context, "Robot[1]/Actuator[8]/AnalogIn/VoltsToPosSI/@Offset", previousOffset);
        xmlConfig.GetXMLValue(context, "Robot[1]/Actuator[8]/AnalogIn/VoltsToPosSI/@Scale", previousScale);

        // compute new offsets assuming a range [0, user-max]
        double userMaxDeg;
        std::cout << "Enter the new desired max for the gripper, 60 (degrees) is recommended to match the maximum tool opening." << std::endl;
        std::cin >> userMaxDeg;
        cmnGetChar(); // to get the CR
        double userMaxRad = userMaxDeg * cmnPI / 180.0;
        // find corresponding pot values using previous scale and offset - carefull, offsets are in degrees in the file
        double minVolt = (minRad - previousOffset * cmnPI / 180.0) / previousScale;
        double maxVolt = (maxRad - previousOffset * cmnPI / 180.0) / previousScale;
        // compute new scale and offset to match pot -> [0, user-max]
        double newScale = (userMaxRad - 0.0) / (maxVolt - minVolt);
        double newOffset = - newScale * minVolt * 180.0 / cmnPI; // also convert back to degrees

        // ask one last confirmation from user
        std::cout << "Status: offset and scale in XML configuration file: " << previousOffset << " " << previousScale << std::endl
                  << "Status: new offset and scale:                       " << newOffset << " " << newScale << std::endl
                  << std::endl
                  << "Do you want to save these values? [S/s]" << std::endl;
        key = cmnGetChar();
        if ((key == 's') || (key == 'S')) {
            const char * context = "Config";
            xmlConfig.SetXMLValue(context, "Robot[1]/Actuator[8]/AnalogIn/VoltsToPosSI/@Offset", newOffset);
            xmlConfig.SetXMLValue(context, "Robot[1]/Actuator[8]/AnalogIn/VoltsToPosSI/@Scale", newScale);
            std::string newConfigFile = configFile + "-new";
            xmlConfig.SaveAs(newConfigFile);
            std::cout << "Status: new config file is \"" << newConfigFile << "\"" << std::endl;
        } else {
            std::cout << "Status: user didn't want to save new offsets." << std::endl;
        }
    } else {
        std::cout << "Status: no data saved in config file." << std::endl;
    }

    delete port;
    return 0;
}
