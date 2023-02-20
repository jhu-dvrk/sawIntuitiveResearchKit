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
    std::string savedData;
    options.AddOptionOneValue("c", "config",
                              "arm sawRobotIO1394 XML configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &configFile);
    options.AddOptionOneValue("p", "port",
                              "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &portName);
    options.AddOptionOneValue("d", "debug-data",
                              "run using data previously saved (for debug only)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &savedData);

    std::string errorMessage;
    if (!options.Parse(argc, argv, std::cerr)) {
        return -1;
    }
    std::string parsedArguments;
    options.PrintParsedArguments(parsedArguments);
    std::cout << "Arguments:" << std::endl << parsedArguments << std::endl;

    const bool collectData = savedData.empty();

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
              << nbActuators << " actuators)" << std::endl
              << "Port: " << portName << std::endl;

    std::cout << "Make sure:" << std::endl
              << " - your computer is connected to the firewire controller." << std::endl
              << " - the arm corresponding to the configuration file \"" << configFile << "\" is connected to the controller." << std::endl
              << " - the E-Stop is closed, this program requires power." << std::endl
              << " - if you're using a PSM, make sure there is no instrument nor sterile adapter." << std::endl
              << " - you have no other program trying to communicate with the controller." << std::endl
              << std::endl;

    if (collectData) {
        std::cout << std::endl << "Press any key to start." << std::endl;
        cmnGetChar();
    }

    // create memory to store all data
    size_t nbAxis = nbActuators;
    const size_t potRange = 4096;

    vctDoubleMat potToEncoder;
    vctDoubleVec directionEncoder;
    vctDoubleVec minEncoder, maxEncoder;
    vctDynamicVector<size_t> minPotIndex, maxPotIndex, zeroPotIndex;
    vctDynamicVector<size_t> dataCounter;

    potToEncoder.SetSize(nbAxis, potRange);
    potToEncoder.SetAll(mtsRobot1394::GetMissingPotValue());

    // human readable, works for PSM and ECM
    std::vector<double> toh = {cmn180_PI, cmn180_PI, 1000.0, cmn180_PI, cmn180_PI, cmn180_PI, cmn180_PI};

    directionEncoder.SetSize(nbAxis, 1.0);
    if (nbActuators == 7) {
        // PSM specific
        directionEncoder.at(1) = -1.0;
        directionEncoder.at(3) = -1.0;
        directionEncoder.at(4) = -1.0;
        directionEncoder.at(5) = -1.0;
    } else if (nbActuators == 4) {

    }

    minEncoder.SetSize(nbAxis);
    minEncoder.SetAll(std::numeric_limits<double>::max());
    maxEncoder.SetSize(nbAxis);
    maxEncoder.SetAll(std::numeric_limits<double>::min());
    minPotIndex.SetSize(nbAxis);
    maxPotIndex.SetSize(nbAxis);
    zeroPotIndex.SetSize(nbAxis);
    dataCounter.SetSize(nbAxis);
    dataCounter.SetAll(0);

    // JSON writer used to save data/calibration
    Json::StreamWriterBuilder builder;
    builder["commentStyle"] = "None";
    builder["indentation"] = "  ";
    builder["precision"] = 9;
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());

    if (collectData) {
        std::cout << "Loading config file ..." << std::endl;
        mtsRobotIO1394 * port = new mtsRobotIO1394("io", 1.0 * cmn_ms, portName);
        // to support cases when user doesn't have an existing lookup table
        port->SetCalibrationMode(true);
        std::cout << "Configuring ..." << std::endl
                  << "If this application just quit, check the error messages in cisstLog.txt" << std::endl;
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

        // locate the manip clutch so we can release the brakes
        size_t numberOfDigitalInput;
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
            std::cout << "Caught exception: " << e.what() << std::endl;
        }

        std::cout << std::endl
                  << "Press any key to power brakes and start collecting data." << std::endl;
        cmnGetChar();

        // set watchdog to a reasonable default
        robot->SetWatchdogPeriod(30.0 * cmn_ms);

        // make sure we reset the encoders to 0
        robot->SetEncoderPosition(vctDoubleVec(nbAxis, 0.0));

        // turn off pots used to check encoders
        robot->UsePotsForSafetyCheck(false);

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

        std::cout << std::endl
                  << "Move every joint one by one from physical limit to pysical limit." << std::endl
                  << "Press the arm clutch to release the brakes if needed (first 3 joints)." << std::endl
                  << "The counters below will increase as you cover new positions." << std::endl
                  << "The counters should reach values in the 3000s." << std::endl
                  << "Press any key to stop collecting data." << std::endl << std::endl;

        bool brakesReleased = false;

        // data collection
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
                if (mtsRobot1394::IsMissingPotValue(potToEncoder.at(axis, potBits.at(axis)))) {
                    potToEncoder.at(axis, potBits[axis]) = directionEncoder.at(axis) * actuatorPos.at(axis);
                    dataCounter.at(axis)++;
                }
            }
            std::cout << "\rPer axis counters: " << dataCounter << std::flush;
        }

        // power off
        robot->SetBrakeAmpEnable(false);
        port->Write();
        robot->WritePowerEnable(false);
        robot->WriteSafetyRelay(false);
        delete port;

        // record the raw data (for debugging purposes)
        std::ofstream rawFile;
        std::string rawFileName = "sawRobotIO1394-" + armName + "-" + serialNumber + "-PotentiometerLookupTable-Raw.json";
        rawFile.open(rawFileName);
        Json::Value jsonValue;
        cmnDataJSON<vctDoubleMat>::SerializeText(potToEncoder, jsonValue);
        writer->write(jsonValue, &rawFile);
        rawFile.close();
        std::cout << std::endl << "Raw data saved to " << rawFileName << std::endl
                  << "You can plot the data in python using:" << std::endl
                  << "import matplotlib.pyplot as plt" << std::endl
                  << "import json, sys" << std::endl
                  << "data = json.load(open('" << rawFileName << "'))" << std::endl
                  << "data = [[(a if a < 31.0 else 0) for a in row] for row in data]" << std::endl
                  << "plt.plot(data[0]) # or whatever axis index you need" << std::endl
                  << "plt.show()" << std::endl << std::endl;
    }
    // using recorded data
    else {
        // make sure file exists
        if (!cmnPath::Exists(savedData)) {
            std::cerr << "Can't find file \"" << savedData << "\"." << std::endl;
            return -1;
        }
        // load content
        std::ifstream jsonStream;
        Json::Value jsonValue;
        Json::Reader jsonReader;
        jsonStream.open(savedData.c_str());
        if (!jsonReader.parse(jsonStream, jsonValue)) {
            std::cerr << "JSON parsing failed for " << savedData << ", "
                      << jsonReader.getFormattedErrorMessages();
            return false;
        }
        cmnDataJSON<vctDoubleMat>::DeSerializeText(potToEncoder, jsonValue);
    }

    // finding the deadzone(s)
    for (size_t axis = 0;
         axis < nbAxis;
         ++axis) {
        // start from first element
        std::list<std::pair<size_t, size_t> > encoderAreas;
        bool previousHasEncoder = !mtsRobot1394::IsMissingPotValue(potToEncoder.at(axis, 0));
        bool encoderStarted = previousHasEncoder;
        size_t encoderStart;
        if (encoderStarted) {
            encoderStart = 0;
        }
        // build list of consecutive areas of encoder values
        std::cout << "Axis[" << axis << "]: found encoder values for ranges";
        for (size_t index = 1;
             index < potRange;
             ++index) {
            const bool hasEncoder = !mtsRobot1394::IsMissingPotValue(potToEncoder.at(axis, index));
            // transitions
            if ((hasEncoder != previousHasEncoder)
                || (index == (potRange - 1))) {
                if (encoderStarted) {
                    encoderStarted = false;
                    encoderAreas.push_back({encoderStart, index - 1});
                    std::cout << " [" << encoderStart << ", " << index - 1 << "]";
                } else {
                    encoderStarted = true;
                    encoderStart = index;
                }
            }
            // update min/max
            if (hasEncoder) {
                const double enc = potToEncoder.at(axis, index);
                if (enc < minEncoder.at(axis)) {
                    minEncoder.at(axis) = enc;
                    minPotIndex.at(axis) = index;
                }
                if (enc > maxEncoder.at(axis)) {
                    maxEncoder.at(axis) = enc;
                    maxPotIndex.at(axis) = index;
                }
            }
            previousHasEncoder = hasEncoder;
        }
        std::cout << std::endl;

        // cleanup list by merging two consecutive elements if the
        // difference between end and start is small
        if (encoderAreas.size() > 1) {
            auto previous = encoderAreas.begin();
            auto iter = previous++;
            for (; iter != encoderAreas.end(); ++iter) {
                // remove previous and update first of current from first of previous
                if ((iter->first - previous->second) < 20) {
                    // simple interpolation to fill in the missing values
                    const double start = potToEncoder.at(axis, previous->second);
                    const double end = potToEncoder.at(axis, iter->first);
                    const size_t nbMissing = iter->first - previous->second - 1;
                    std::cout << "Axis[" << axis << "]: found " << nbMissing << " missing consecutive pot value(s)" << std::endl;
                    const double delta = (end - start) / nbMissing;
                    size_t counter = 1;
                    for (size_t index = previous->second + 1;
                         index < iter->first;
                         ++index, ++counter) {
                        if (!mtsRobot1394::IsMissingPotValue(potToEncoder.at(axis, index))) {
                            std::cerr << "!!!! this shouldn't be happening" << std::endl;
                        }
                        potToEncoder.at(axis, index) = start + counter * delta;
                    }
                    // concatenate lists of indices
                    iter->first = previous->first;
                    encoderAreas.erase(previous);
                }
                previous = iter;
            }
        }

        // display final results
        for (auto iter : encoderAreas) {
            std::cout << "Axis[" << axis << "]: using encoder for range [" << iter.first
                      << " (" << potToEncoder.at(axis, iter.first) * toh.at(axis)
                      << "), " << iter.second << " ("
                      << potToEncoder.at(axis, iter.second) * toh.at(axis)<< ")]" << std::endl;
        }
    }

    vctDoubleVec minEncoderExpected, maxEncoderExpected;
    if (nbActuators == 7) {
        // this is for a PSM.  We don't have the exact numbers but we have 3
        // calibration files.  Ranges differ between arms but the midpoint
        // is always the same.  Values are from jhu-dVRK-Si/cal-files 292409 586288 334809
        minEncoderExpected.SetSize(7);
        maxEncoderExpected.SetSize(7);
        minEncoderExpected.at(0) = (-2.9646 + -2.9668 + -2.9675) / 3.0;
        maxEncoderExpected.at(0) = ( 2.9646 +  2.9668 +  2.9675) / 3.0;
        minEncoderExpected.at(1) = (-1.2656 + -1.2661 + -1.2667) / 3.0;
        maxEncoderExpected.at(1) = ( 1.3010 +  1.3015 +  1.3021) / 3.0;
        // translation
        minEncoderExpected.at(2) = (-0.00099057 + -0.00099124 + -0.0009907) / 3.0;
        maxEncoderExpected.at(2) = ( 0.29035    +  0.29055    +  0.29039) / 3.0;
        // last 4 elements
        minEncoderExpected.at(3) = (-3.0346 + -3.0283 + -3.03) / 3.0;
        maxEncoderExpected.at(3) = ( 3.0346 +  3.0283 +  3.03) / 3.0;
        minEncoderExpected.at(4) = (-3.0358 + -3.0271 + -3.0271) / 3.0;
        maxEncoderExpected.at(4) = ( 3.0358 +  3.0271 +  3.0271) / 3.0;
        minEncoderExpected.at(5) = (-3.0350 + -3.0303 + -3.0294) / 3.0;
        maxEncoderExpected.at(5) = ( 3.0350 +  3.0303 +  3.0294) / 3.0;
        minEncoderExpected.at(6) = (-3.0378 + -3.03   + -3.0268) / 3.0;
        maxEncoderExpected.at(6) = ( 3.0378 +  3.03   +  3.0268) / 3.0;
    } else if (nbActuators == 4) {
        // this is for an ECM, using 267579 438610
        minEncoderExpected.SetSize(4);
        maxEncoderExpected.SetSize(4);
        minEncoderExpected.at(0) = (-2.9657 + -2.968) / 2.0;
        maxEncoderExpected.at(0) = ( 2.9657 +  2.968) / 2.0;
        minEncoderExpected.at(1) = (-1.1253 + -1.1061) / 2.0;
        maxEncoderExpected.at(1) = ( 1.0612 +  1.0431) / 2.0;
        // translation
        minEncoderExpected.at(2) = ( 0.001  +  0.001) / 2.0;
        maxEncoderExpected.at(2) = ( 0.2595 +  0.25894) / 2.0;
        // last 4 elements
        minEncoderExpected.at(3) = (-1.5434 + -1.5414) / 2.0;
        maxEncoderExpected.at(3) = ( 1.5434 +  1.5414) / 2.0;
    }

    vctDoubleVec offsetEncoder(nbAxis, 0.0);

    for (size_t axis = 0;
         axis < nbAxis;
         ++axis) {
        std::cout << "Axis[" << axis
                  << "]: range [" << minEncoder.at(axis) * toh.at(axis)
                  << ", " << maxEncoder.at(axis) * toh.at(axis) << "] = "
                  << (maxEncoder.at(axis) - minEncoder.at(axis)) * toh.at(axis) << std::endl;
        const double encoderRange = maxEncoder.at(axis) - minEncoder.at(axis);
        const double encoderRangeExpected =  maxEncoderExpected.at(axis) - minEncoderExpected.at(axis);
        if ((encoderRange < 0.95 * encoderRangeExpected)
            || (encoderRange > 1.05 * encoderRangeExpected)) {
            std::cerr << "Axis[" << axis
                      << "]: range for encoder values is not within 5% of what was expected.  Found "
                      << encoderRange * toh.at(axis) << " but we were expecting "
                      << encoderRangeExpected * toh.at(axis) << std::endl;
            return -1;
        }
        // compute offset using mid point
        offsetEncoder.at(axis) = (minEncoderExpected.at(axis) + maxEncoderExpected.at(axis)) / 2.0
            - (minEncoder.at(axis) + maxEncoder.at(axis)) / 2.0;
        std::cout << " - encoder offset: " << offsetEncoder.at(axis) * toh.at(axis) << std::endl;
        // add offset to recorded values and try to find the closest
        // to zero encoder value within pot range
        double zeroEncoder = std::numeric_limits<double>::max();
        for (size_t index = 0;
             index < potRange;
             ++index) {
            const bool hasEncoder = !mtsRobot1394::IsMissingPotValue(potToEncoder.at(axis, index));
            if (hasEncoder) {
                // add offset
                potToEncoder.at(axis, index) += offsetEncoder.at(axis);
                potToEncoder.at(axis, index) *= directionEncoder.at(axis);
                // try to find lowest encoder value
                const double enc = std::abs(potToEncoder.at(axis, index));
                if (enc < zeroEncoder) {
                    zeroEncoder = enc;
                    zeroPotIndex.at(axis) = index;
                }
            }
        }
        std::cout << " - zero pot index: " << zeroPotIndex.at(axis) << std::endl
                  << " - zero encoder value: " << zeroEncoder * toh.at(axis) << std::endl;

        // padding
        bool previousValueIsMissing = mtsRobot1394::IsMissingPotValue(potToEncoder(axis, 0));
        const size_t paddingWidth = 30;
        for (size_t index = 1;
             index < potRange;
             ++index) {
            // two cases, from missing to set
            bool currentValueIsMissing = mtsRobot1394::IsMissingPotValue(potToEncoder(axis, index));
            if (previousValueIsMissing && !currentValueIsMissing) {
                const size_t startPadding = std::max(static_cast<size_t>(0), index - paddingWidth);
                for (size_t indexPadding = startPadding;
                     indexPadding < index;
                     ++indexPadding) {
                    potToEncoder.at(axis, indexPadding) = potToEncoder.at(axis, index);
                }
            } else if (!previousValueIsMissing && currentValueIsMissing) {
                const size_t endPadding = std::min(potRange, index + paddingWidth);
                for (size_t indexPadding = index;
                     indexPadding < endPadding;
                     ++indexPadding) {
                    potToEncoder.at(axis, indexPadding) = potToEncoder.at(axis, index - 1);
                }
                // then skip forward
                index = endPadding;
            }
            previousValueIsMissing = currentValueIsMissing;
        }
    }

    // record the pot to encoder file
    std::ofstream potToEncoderFile;
    std::string potToEncoderFileName = "sawRobotIO1394-" + armName + "-" + serialNumber + "-PotentiometerLookupTable.json";
    std::string potToEncoderFileNameNew = potToEncoderFileName + "-new";
    potToEncoderFile.open(potToEncoderFileNameNew);
    Json::Value jsonValue;
    cmnDataJSON<vctDoubleMat>::SerializeText(potToEncoder, jsonValue);
    writer->write(jsonValue, &potToEncoderFile);
    potToEncoderFile.close();

    std::cout << std::endl
              << "Results saved in " << potToEncoderFileNameNew << std::endl
              << "You can move the new file over the existing one using:" << std::endl
              << "mv -i " << potToEncoderFileNameNew << " " << potToEncoderFileName << std::endl;
    return 0;
}
