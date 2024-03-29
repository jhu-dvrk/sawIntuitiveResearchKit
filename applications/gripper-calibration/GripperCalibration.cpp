/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-12-20

  (C) Copyright 2013-2023 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstCommon/cmnXMLPath.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobot1394.h>

using namespace sawRobotIO1394;

int main(int argc, char * argv[])
{
    cmnCommandLineOptions options;
    std::string portName = mtsRobotIO1394::DefaultPort();
    std::string configFile;
    options.AddOptionOneValue("c", "config",
                              "MTM gripper sawRobotIO1394 XML configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &configFile);
    options.AddOptionOneValue("p", "port",
                              "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &portName);

    if (!options.Parse(argc, argv, std::cerr)) {
        return -1;
    }

    if (!cmnPath::Exists(configFile)) {
        std::cerr << "Can't find file \"" << configFile << "\"." << std::endl;
        return -1;
    }
    std::cout << "Configuration file: " << configFile << std::endl
              << "Port: " << portName << std::endl;

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
    mtsRobotIO1394 * port = new mtsRobotIO1394("io", 1.0 * cmn_ms, portName);
    port->Configure(configFile);

    std::cout << "Creating robot ..." << std::endl;
    size_t numberOfRobots;
    port->GetNumberOfRobots(numberOfRobots);
    if (numberOfRobots == 0) {
        std::cerr << "Error: the config file doesn't define a robot." << std::endl;
        return -1;
    }
    if (numberOfRobots != 1) {
        std::cerr << "Error: the config file defines more than one robot." << std::endl;
        return -1;
    }
    mtsRobot1394 * robot = port->Robot(0);
    size_t numberOfActuators = robot->NumberOfActuators();
    if (numberOfActuators != 1) {
        std::cerr << "Error: the config file defines a robot with more than 1 actuator, make sure you use the \"gripper\" configuration file!" << std::endl;
        return -1;
    }

    // make sure we have at least one set of pots values
    try {
        port->Read();
    } catch (const std::runtime_error & e) {
        std::cerr << "Caught exception: " << e.what() << std::endl;
    }

    std::cout << std::endl
              << "Press any key to start collecting data." << std::endl;
    cmnGetChar();
    std::cout << "Fully open and close the gripper up to the second spring on the MTM multiple times." << std::endl
              << "NOTE: It is very important to not close the gripper all the way; stop when you feel some resistance from the second spring." << std::endl
              << "Keep closing and opening until the counter and range stop increasing." << std::endl
              << "Press any key to stop collecting data." << std::endl << std::endl;

    double minRad = std::numeric_limits<double>::max();
    double maxRad = std::numeric_limits<double>::min();
    size_t counter = 0;
    while (1) {
        if (cmnKbHit()) {
            std::cout << std::endl;
            break;
        }
        port->Read();
        // read values in radians to test existing coeffcients
        double value = robot->PotPosition().at(0);
        bool newValue = false;
        if (value > maxRad) {
            counter++;
            maxRad = value;
            newValue = true;
        } else if (value < minRad) {
            minRad = value;
            counter++;
            newValue = true;
        }
        if (newValue) {
            std::cout << '\r' << " Counter: "
                      << std::setfill(' ') << std::setw(5) << counter
                      << ", range: [ "
                      <<  std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(8)
                      << minRad * cmn180_PI << " - "
                      <<  std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(8)
                      << maxRad * cmn180_PI << " ]" << std::flush;
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
        xmlConfig.GetXMLValue(context, "Robot[1]/Actuator[1]/AnalogIn/VoltsToPosSI/@Offset", previousOffset);
        xmlConfig.GetXMLValue(context, "Robot[1]/Actuator[1]/AnalogIn/VoltsToPosSI/@Scale", previousScale);

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
            xmlConfig.SetXMLValue(context, "Robot[1]/Actuator[1]/AnalogIn/VoltsToPosSI/@Offset", newOffset);
            xmlConfig.SetXMLValue(context, "Robot[1]/Actuator[1]/AnalogIn/VoltsToPosSI/@Scale", newScale);
            // rename old file and save in place
            std::string currentDateTime;
            osaGetDateTimeString(currentDateTime);
            std::string newName = configFile + "-backup-" + currentDateTime;
            cmnPath::RenameFile(configFile, newName);
            std::cout << "Existing IO config file has been renamed " << newName << std::endl;
            xmlConfig.SaveAs(configFile);
            std::cout << "Results saved in IO config file " << configFile << std::endl;
        } else {
            std::cout << "Status: user didn't want to save new offsets." << std::endl;
        }
    } else {
        std::cout << "Status: no data saved in config file." << std::endl;
    }

    delete port;
    return 0;
}
