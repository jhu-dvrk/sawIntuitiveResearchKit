/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-12-20

  (C) Copyright 2013-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system
#include <iostream>
#include <limits>
#include <json/json.h>

// cisst/saw
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnKbHit.h>
#include <cisstCommon/cmnGetChar.h>
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
                              "MTM gripper sawRobotIO1394 JSON configuration file",
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
    port->set_calibration_mode(true);
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

    std::cout << "Fully open and close the gripper up to the second spring on the MTM multiple times." << std::endl
              << "NOTE: It is very important to not close the gripper all the way; stop when you feel some resistance from the second spring." << std::endl
              << "Keep closing and opening until the counter and range stop increasing." << std::endl
              << "Press any key to stop collecting data." << std::endl << std::endl;

    double min_v = std::numeric_limits<double>::max();
    double max_v = std::numeric_limits<double>::min();
    double min_p = std::numeric_limits<double>::max();
    double max_p = std::numeric_limits<double>::min();
    size_t counter = 0;
    while (1) {
        if (cmnKbHit()) {
            std::cout << std::endl;
            break;
        }
        port->Read();
        // read values in volts in case scale and offsets are set to 0
        double v = robot->PotentiometerVoltage().at(0);
        bool newValue = false;
        if (v > max_v) {
            max_v = v;
            newValue = true;
        }
        if (v < min_v) {
            min_v = v;
            newValue = true;
        }
        // read values in radians to test existing coefficients
        double p = robot->PotentiometerPosition().at(0);
        if (p > max_p) {
            max_p = p;
        }
        if (p < min_p) {
            min_p = p;
        }
        if (newValue) {
            counter++;
            std::cout << '\r' << " Counter: "
                      << std::setfill(' ') << std::setw(5) << counter
                      << "    Range voltage: [ "
                      <<  std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(8)
                      << min_v << " ; "
                      <<  std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(8)
                      << max_v << " ]    Range angle (deg): [ "
                      <<  std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(8)
                      << min_p * cmn180_PI << " ; "
                      <<  std::fixed << std::setprecision(3) << std::setfill(' ') << std::setw(8)
                      << max_p * cmn180_PI << " ]" << std::flush;
        }
        osaSleep(1.0 * cmn_ms);
    }
    cmnGetChar(); // to read the pressed key
    std::cout << std::endl;

    // load json file
    std::ifstream jsonStream;
    Json::Value jsonConfig;
    Json::Reader jsonReader;

    jsonStream.open(configFile.c_str());
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        std::cerr << "Failed to parse configuration file \""
                  << configFile << "\"\n"
                  << jsonReader.getFormattedErrorMessages();
        exit(EXIT_FAILURE);
    }
    jsonStream.close();

    // query previous current offset and scales
    Json::Value & conversionJson = jsonConfig["robots"][0]["actuators"][0]["potentiometer"]["voltage_to_position"];
    double previousOffset = conversionJson["offset"].asDouble();
    double previousScale = conversionJson["scale"].asDouble();

    // compute new offsets assuming a range [0, user-max]
    double userMaxRad = 60.0 * cmnPI / 180.0;
    // find corresponding pot values using voltage range
    double newScale = userMaxRad / (min_v - max_v);
    double newOffset = - newScale * max_v;

    // ask one last confirmation from user
    std::cout << "Status: offset and scale in JSON configuration file: " << previousOffset << " " << previousScale << std::endl
              << "Status: new offset and scale:                       " << newOffset << " " << newScale << std::endl
              << std::endl
              << "Do you want to save these values? [S/s]" << std::endl;
    char key = cmnGetChar();
    if ((key == 's') || (key == 'S')) {

        // rename old file and save in place
        const double abs_time = osaGetTime();
        std::string date, time;
        osaGetDateString(date, abs_time, '-');
        osaGetTimeString(time, abs_time, ':');
        const std::string newName = configFile + "-backup-" + date + "_" + time;
        cmnPath::RenameFile(configFile, newName);
        std::cout << "Existing IO config file has been renamed " << newName << std::endl;

        // save new data
        Json::StreamWriterBuilder builder;
        builder["indentation"] = "    ";
        std::ofstream outputFile(configFile);
        std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
        jsonConfig["robots"][0]["actuators"][0]["potentiometer"]["voltage_to_position"]["offset"] = newOffset;
        jsonConfig["robots"][0]["actuators"][0]["potentiometer"]["voltage_to_position"]["scale"] = newScale;
        writer->write(jsonConfig, &outputFile);
        outputFile.close();
        std::cout << "Results saved in IO config file " << configFile << std::endl;
    } else {
        std::cout << "Status: user didn't want to save new offsets." << std::endl;
    }

    delete port;
    return 0;
}
