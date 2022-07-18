/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2022-06-20

  (C) Copyright 2022 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobot1394.h>
#include <sawRobotIO1394/mtsDigitalInput1394.h>

using namespace sawRobotIO1394;

int main(int argc, char * argv[])
{
    cmnCommandLineOptions options;
    std::string portName = mtsRobotIO1394::DefaultPort();
    std::string configFile;
    options.AddOptionOneValue("c", "config",
                              "arm sawRobotIO1394 XML configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &configFile);
    options.AddOptionOneValue("p", "port",
                              "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &portName);
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

    // setup XML parsing
    cmnXMLPath xmlConfig;
    xmlConfig.SetInputSource(configFile);
    const char * context = "Config";
    std::string armName;
    xmlConfig.GetXMLValue(context, "Robot[1]/@Name", armName);
    int nbActuators;
    xmlConfig.GetXMLValue(context, "Robot[1]/@NumOfActuator", nbActuators);
    std::string serialNumber;
    xmlConfig.GetXMLValue(context, "Robot[1]/@SN", serialNumber);

    std::cout << "Configuration file: " << configFile
              << " for arm " << armName << " [" << serialNumber << "] ("
              << nbActuators << ")" << std::endl
              << "Port: " << portName << std::endl;

    std::cout << "Make sure:" << std::endl
              << " - your computer is connected to the firewire controller." << std::endl
              << " - the arm corresponding to the configuration file \"" << configFile << "\" is connected to the controller." << std::endl
              << " - the E-Stop is closed, this program requires power." << std::endl
              << " - if you're using a PSM, make sure there is no instrument nor sterile adapter." << std::endl
              << " - you have no other program trying to communicate with the controller." << std::endl
              << std::endl
              << "Press any key to start." << std::endl;
    cmnGetChar();

    std::cout << "Loading config file ..." << std::endl;
    mtsRobotIO1394 * port = new mtsRobotIO1394("io", 1.0 * cmn_ms, portName);
    port->Configure(configFile);

    std::cout << "Creating robot ..." << std::endl;
    int numberOfRobots;
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

    // locate the manip clutch so we can release the brakes
    int numberOfDigitalInput;
    port->GetNumberOfDigitalInputs(numberOfDigitalInput);
    mtsDigitalInput1394 * clutchInput = nullptr;
    const std::string clutchName = armName + "-ManipClutch";
    for (size_t index = 0;
         index < numberOfDigitalInput;
         ++index) {
        if (port->DigitalInput(index)->Name() == clutchName) {
            clutchInput = port->DigitalInput(index);
            break;
        }
    }
    if (!clutchInput) {
        std::cerr << "Error: failed to find digital input \"" << clutchName
                  << "\", make sure you're using a PSM or ECM-S configuration file!" << std::endl;
        return -1;
    }

    // make sure we have at least one set of pots values
    try {
        port->Read();
    } catch (const std::runtime_error & e) {
        std::cerr << "Caught exception: " << e.what() << std::endl;
    }

    std::cout << std::endl
              << "Press any key to power brakes and start collecting data." << std::endl;
    cmnGetChar();

    // turn off pots used to check encoders
    robot->UsePotsForSafetyCheck(false);

    // create memory to store all data
    size_t nbAxis = nbActuators;
    const size_t potRange = 4096;
    const double missing = std::numeric_limits<double>::max();

    vctDoubleMat potToEncoder;
    vctDoubleVec directionEncoder;
    vctDoubleVec minEncoder, maxEncoder;
    vctDynamicVector<size_t> dataCounter;

    potToEncoder.SetSize(nbAxis, potRange);
    potToEncoder.SetAll(missing);

    directionEncoder.SetSize(nbAxis, 1.0);
    // PSM specific
    directionEncoder.at(1) = -1.0;
    directionEncoder.at(3) = -1.0;
    directionEncoder.at(4) = -1.0;
    directionEncoder.at(5) = -1.0;

    minEncoder.SetSize(nbAxis);
    minEncoder.SetAll(std::numeric_limits<double>::max());
    maxEncoder.SetSize(nbAxis);
    maxEncoder.SetAll(std::numeric_limits<double>::min());
    dataCounter.SetSize(nbAxis);
    dataCounter.SetAll(0);

    // enable power and brakes
    vctDoubleVec brakeCurrent(3, 0.0);
    robot->SetBrakeCurrent(brakeCurrent);
    vctDoubleVec actuatorVoltageRatio(nbAxis, 0.0);
    robot->SetActuatorVoltageRatio(actuatorVoltageRatio);
    port->Write();
    robot->WriteSafetyRelay(true);
    robot->WritePowerEnable(true);
    robot->SetBrakeAmpEnable(true);
    robot->SetActuatorAmpEnable(true);
    port->Write();

    // wait a bit to make sure current stabilizes, 100 * 10 ms = 1 second
    for (size_t i = 0; i < 100; ++i) {
        osaSleep(10.0 * cmn_ms);
        port->Read();
        port->Write();
    }

    // check that power is on
    if (!robot->PowerStatus()) {
        std::cerr << "Error: unable to power on controllers, make sure E-Stop is ok." << std::endl;
        return -1;
    }
    if (!robot->BrakeAmpStatus().All()) {
        std::cerr << "Error: failed to turn on brake amplifiers:" << std::endl
                  << " - status:  " << robot->BrakeAmpStatus() << std::endl
                  << " - desired: " << robot->BrakeAmpEnable() << std::endl;
        return -1;
    }

    std::cout << "Fully open and close the gripper up to the second spring on the MTM multiple times." << std::endl
              << "NOTE: It is very important to not close the gripper all the way; stop when you feel some resistance from the second spring." << std::endl
              << "Keep closing and opening until the counter and range stop increasing." << std::endl
              << "Press any key to stop collecting data." << std::endl << std::endl;

    bool brakesReleased = false;

    while (1) {
        if (cmnKbHit()) {
            std::cout << std::endl;
            break;
        }
        port->Read();
        robot->PollState();
        robot->ConvertState();
        robot->CheckState();

        // brakes
        if (brakesReleased != clutchInput->Value()) {
            brakesReleased = clutchInput->Value();
            if (brakesReleased) {
                robot->BrakeRelease();
            } else {
                robot->BrakeEngage();
            }
        }
        port->Write();

        // read values
        vctIntVec potBits = robot->PotBits();
        vctDoubleVec actuatorPos = robot->ActuatorJointState().Position();
        for (size_t axis = 0;
             axis < nbAxis;
             ++axis) {
            if (potToEncoder.at(axis, potBits.at(axis)) == missing) {
                const double encoder = directionEncoder.at(axis) * actuatorPos.at(axis);
                potToEncoder.at(axis, potBits[axis]) = encoder;
                if (encoder < minEncoder.at(axis)) {
                    minEncoder.at(axis) = encoder;
                }
                if (encoder > maxEncoder.at(axis)) {
                    maxEncoder.at(axis) = encoder;
                }
                dataCounter.at(axis)++;
            }
        }
        std::cout << "\rPer axis counters: " << dataCounter << std::flush;
    }

    // finding the deadzone(s)
    for (size_t axis = 0;
         axis < nbAxis;
         ++axis) {
        // start from first element
        std::list<std::pair<size_t, size_t> > encoderAreas;
        bool previousHasEncoder = (potToEncoder.at(axis, 0) != missing);
        bool encoderStarted = previousHasEncoder;
        size_t encoderStart;
        if (encoderStarted) {
            encoderStart = 0;
        }
        // build list of consecutive areas of encoder values
        for (size_t index = 1;
             index < potRange;
             ++index) {
            bool hasEncoder = (potToEncoder.at(axis, index) != missing);
            // transitions
            if (hasEncoder != previousHasEncoder) {
                if (encoderStarted) {
                    // check, this should be a encoder end
                    assert(!hasEncoder);
                    encoderStarted = false;
                    encoderAreas.push_back({encoderStart, index - 1});
                    std::cerr << "Axis " << axis << ", found encoder values from " << encoderStart << " to " << index << std::endl;
                } else {
                    // check, this should be a start
                    assert(hasEncoder);
                    encoderStarted = true;
                    encoderStart = index;
                }
            }
            previousHasEncoder = hasEncoder;
        }
        // cleanup list by merging two consecutive elements if the
        // difference between end and start is small
        if (encoderAreas.size() > 1) {
            auto previous = encoderAreas.begin();
            auto iter = previous++;
            for (; iter != encoderAreas.end(); ++iter) {
                // remove previous and update first of current from first of previous
                if ((iter->first - previous->second) < 20) {
                    iter->first = previous->first;
                    encoderAreas.erase(previous);
                }
                previous = iter;
            }
        }
        // display final results
        for (auto iter : encoderAreas) {
            std::cerr << "Axis " << axis << ", assuming encoder areas from " << iter.first << " to " << iter.second << std::endl;
        }
    }

#if 0
    // this is for a PSM
    vctDoubleVec minEncoderExpected, maxEncoderExpected;
    minEncoderExpected.SetSize(7);
    maxEncoderExpected.SetSize(7);
    minEncoderExpected.at(0) = -170.0 * cmnPI_180;
    maxEncoderExpected.at(0) =  170.0 * cmnPI_180;
    minEncoderExpected.at(1) =  -60.0 * cmnPI_180;
    maxEncoderExpected.at(1) =   85.0 * cmnPI_180;
    // translation
    minEncoderExpected.at(2) =   0.0 * cmn_mm;
    maxEncoderExpected.at(2) = 290.0 * cmn_mm;
    // last 4 elements
    minEncoderExpected.Ref(4, 3) = -172.0 * cmnPI_180;
    maxEncoderExpected.Ref(4, 3) =  172.0 * cmnPI_180;

    vctDoubleVec offsetEncoder(nbAxis, 0.0);

    for (size_t axis = 0;
         axis < nbAxis;
         ++axis) {
        double toh = 1000.0; // human readable
        if (axis != 2) {
            toh = cmn180_PI;
        }
        std::cout << "Axis " << axis << std::endl
                  << " - pot min: " << potMin.at(axis) << std::endl
                  << " - angle min : " << potToEncoder.at(axis, potMin.at(axis)) * toh << std::endl
                  << " - range: [" << minEncoder.at(axis) * toh
                  << ", " << maxEncoder.at(axis) * toh << "]" << std::endl;
        const double encoderRange = maxEncoder.at(axis) - minEncoder.at(axis);
        const double encoderRangeExpected =  maxEncoderExpected.at(axis) - minEncoderExpected.at(axis);
        if ((encoderRange < 0.95 * encoderRangeExpected)
            || (encoderRange > 1.05 * encoderRangeExpected)) {
            std::cerr << "Axis " << axis
                      << ", range for encoder values is not within 5% of what was expected.  Found "
                      << encoderRange * toh << " but we were expecting "
                      << encoderRangeExpected * toh << std::endl;
            return -1;
        }
        // compute offset using mid point
        offsetEncoder.at(axis) = encoderRange / 2.0 - encoderRangeExpected / 2.0;
        std::cout << " - encoder offset: " << offsetEncoder.at(axis) * toh << std::endl;
        // add offset to recorded values
        potToEncoder.Row(axis).Add(offsetEncoder.at(axis));
        // try to find the closest to zero encoder value within pot range
        double homePot = potMin.at(axis);
        double homeEncoder = std::abs(potToEncoder.at(axis, homePot));
        for (size_t index = potMin.at(axis) + 1;
             index <= potMax.at(axis);
             ++index) {
            const double enc = std::abs(potToEncoder.at(axis, homePot));
            if (enc < homeEncoder) {
                homeEncoder = enc;
                homePot = index;
            }
        }
        std::cout << " - home pot index: " << homePot << std::endl
                  << " - home encoder value: " << homeEncoder * toh << std::endl;
    }
#endif
    
    cmnGetChar(); // to read the pressed key
#if 0

    std::cout << std::endl
              << "Status: found range [" << minPot << ", " << maxPot
              << "]" << std::endl
              << std::endl
              << "Do you want to update the config file with these values? [Y/y]" << std::endl;


    // save if needed
    char key = cmnGetChar();
    if ((key == 'y') || (key == 'Y')) {
        // query previous current offset and scales
        double previousOffset;
        double previousScale;
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
            std::string newConfigFile = configFile + "-new";
            xmlConfig.SaveAs(newConfigFile);
            std::cout << "Status: new config file is \"" << newConfigFile << "\"" << std::endl
                      << "You can copy the new file over the old one using:\n  cp -i "
                      << newConfigFile << " " << configFile << std::endl;
        } else {
            std::cout << "Status: user didn't want to save new offsets." << std::endl;
        }
    } else {
        std::cout << "Status: no data saved in config file." << std::endl;
    }
#endif

    // power off
    robot->SetBrakeAmpEnable(false);
    port->Write();
    robot->WritePowerEnable(false);
    robot->WriteSafetyRelay(false);

    delete port;
    return 0;
}
