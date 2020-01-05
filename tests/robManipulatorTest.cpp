/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2019-11-11

  (C) Copyright 2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "robManipulatorTest.h"

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitConfig.h>


class ManipulatorTestDataECM: public ManipulatorTestData {
public:
    ManipulatorTestDataECM(void)
    {
        Name = "ECM";
        NumberOfLinks = 4;
        Manipulator = new robManipulatorECM;
    };

    void CheckIKResults(void) {
        vctDoubleVec jointErrors(NumberOfLinks), jointErrorsAbsolute(NumberOfLinks);
        jointErrors.DifferenceOf(SolutionJoints, ActualJoints);
        jointErrorsAbsolute.AbsOf(jointErrors);

        // compare joint values
        CPPUNIT_ASSERT_MESSAGE("Joint 0 solution is incorrect",
                               (jointErrorsAbsolute[0] < 0.0000001 * cmn180_PI)); // asinl is not that precise

        CPPUNIT_ASSERT_MESSAGE("Joint 1 solution is incorrect",
                               (jointErrorsAbsolute[1] < 0.0000001 * cmn180_PI)); // asinl

        CPPUNIT_ASSERT_MESSAGE("Joint 2 solution is incorrect",
                               (jointErrorsAbsolute[2] < 0.000000001 * cmn_mm)); // just a sqrt, higher precision expected

        CPPUNIT_ASSERT_MESSAGE("Joint 3 solution is incorrect",
                               (jointErrorsAbsolute[3] < 0.0000001 * cmn180_PI)); // acosl

        // translation
        vct3 positionTranslationError = ActualPose.Translation() - SolutionPose.Translation();
        CPPUNIT_ASSERT_MESSAGE("Cartesian translation error is too high",
                               positionTranslationError.Norm() < 0.005 * cmn_mm);

        // rotation
        vctMatRot3 positionRotationError;
        ActualPose.Rotation().ApplyInverseTo(SolutionPose.Rotation(), positionRotationError);
        CPPUNIT_ASSERT_MESSAGE("Cartesian rotation error is too high",
                               vctAxAnRot3(positionRotationError).Angle() < 0.0000001 * cmn180_PI); // acosl
    }
};


class ManipulatorTestDataMTM: public ManipulatorTestData {
public:
    ManipulatorTestDataMTM(void)
    {
        Name = "MTM";
        NumberOfLinks = 7;
        Manipulator = new robManipulatorMTM;
    };

    void CheckIKResults(void) {
        vctDoubleVec jointErrors(NumberOfLinks), jointErrorsAbsolute(NumberOfLinks);
        jointErrors.DifferenceOf(SolutionJoints, ActualJoints);
        jointErrorsAbsolute.AbsOf(jointErrors);

        // compare joint values
        CPPUNIT_ASSERT_MESSAGE("Joint 0 solution is incorrect",
                               (jointErrorsAbsolute[0] < 0.0000001 * cmn180_PI)); // asinl is not that precise

        CPPUNIT_ASSERT_MESSAGE("Joint 1 solution is incorrect",
                               (jointErrorsAbsolute[1] < 0.0000001 * cmn180_PI)); // asinl

        CPPUNIT_ASSERT_MESSAGE("Joint 2 solution is incorrect",
                               (jointErrorsAbsolute[2] < 0.0000001 * cmn180_PI)); // asinl

        // translation
        vct3 positionTranslationError = ActualPose.Translation() - SolutionPose.Translation();
        CPPUNIT_ASSERT_MESSAGE("Cartesian translation error is too high",
                               positionTranslationError.Norm() < 0.005 * cmn_mm);

        // rotation
#if 0
        vctMatRot3 positionRotationError;
        ActualPose.Rotation().ApplyInverseTo(SolutionPose.Rotation(), positionRotationError);
        CPPUNIT_ASSERT_MESSAGE("Cartesian rotation error is too high",
                               vctAxAnRot3(positionRotationError).Angle() < 0.0000001 * cmn180_PI); // acosl
#endif
    }
};


void robManipulatorTest::SetupTestData(ManipulatorTestData & data,
                                       const std::string & filename)
{
    // find the file
    cmnPath path;
    path.Add(std::string(sawIntuitiveResearchKit_SOURCE_DIR) + "/../share/kinematic", cmnPath::TAIL);
    const std::string configFile = path.Find(filename);
    CPPUNIT_ASSERT_MESSAGE("Can't find full path for " + filename,
                           configFile != std::string(""));

    // json parsing
    std::ifstream jsonStream;
    Json::Value jsonConfig;
    Json::Reader jsonReader;
    jsonStream.open(configFile.c_str());
    bool fileParsed = jsonReader.parse(jsonStream, jsonConfig);
    CPPUNIT_ASSERT_MESSAGE("Failed to parse JSON file " + configFile + ": " + jsonReader.getFormattedErrorMessages(),
                           fileParsed);

    // look for DH in file
    const Json::Value jsonDH = jsonConfig["DH"];
    CPPUNIT_ASSERT_MESSAGE("Can't find \"DH\" in " + configFile,
                           !jsonDH.isNull());

    // try to load the DH parameters
    CPPUNIT_ASSERT_MESSAGE("Failed while loading from JSON \"DH\" value in " + configFile,
                           data.Manipulator->LoadRobot(jsonDH) == robManipulator::ESUCCESS);

    // verify number of links in robManipulator
    CPPUNIT_ASSERT_EQUAL_MESSAGE("Expected number of links for " + filename,
                                 data.NumberOfLinks, data.Manipulator->links.size());

    // allocate test data
    data.LowerLimits.SetSize(data.NumberOfLinks);
    data.UpperLimits.SetSize(data.NumberOfLinks);
    data.Manipulator->GetJointLimits(data.LowerLimits,
                                     data.UpperLimits);
    data.Increments.SetSize(data.NumberOfLinks);
    data.ActualJoints.SetSize(data.NumberOfLinks);
    data.PreviousActualJoints.SetSize(data.NumberOfLinks);
    data.SolutionJoints.SetSize(data.NumberOfLinks);
}

void robManipulatorTest::ComputeAndTestIK(ManipulatorTestData & data)
{
    // compute FK
    data.ActualPose = data.Manipulator->ForwardKinematics(data.ActualJoints);
    // compute IK
    data.SolutionJoints.Assign(data.ActualJoints);
    robManipulator::Errno result = data.Manipulator->InverseKinematics(data.SolutionJoints,
                                                                       data.ActualPose);
    // make sure IK didn't complain
    CPPUNIT_ASSERT_EQUAL_MESSAGE("robManipulator::InverseKinematics result for " + data.Name,
                                 robManipulator::ESUCCESS,
                                 result);

    // compute solution pose
    data.SolutionPose = data.Manipulator->ForwardKinematics(data.SolutionJoints);

    // arm specific testing
    data.CheckIKResults();
}

void robManipulatorTest::TestSampleJointSpace(ManipulatorTestData & data)
{
    vctDoubleVec directions(data.NumberOfLinks, 1.0);

    data.ActualJoints.Assign(data.LowerLimits);
    data.PreviousActualJoints.Assign(data.LowerLimits);
    ComputeAndTestIK(data);

    bool allReached = false;
    while (!allReached) {
        bool nextDimensionIncrement = true;
        for (size_t index = 0; index < data.NumberOfLinks; ++index) {
            if (nextDimensionIncrement) {
                double future = data.ActualJoints[index] + directions[index] * data.Increments[index];
                if (future > data.UpperLimits[index]) {
                    directions[index] = -1.0;
                    if (index == (data.NumberOfLinks - 1)) {
                        allReached = true;
                    }
                } else if (future < data.LowerLimits[index]) {
                    directions[index] = 1.0;
                } else {
                    data.PreviousActualJoints.Assign(data.ActualJoints);
                    data.ActualJoints[index] = future;
                    nextDimensionIncrement = false;
                    ComputeAndTestIK(data);
                }
            }
        }
    }
}

void robManipulatorTest::TestECMIKSampleJointSpace(void)
{
    // load manipulator
    ManipulatorTestDataECM data;
    SetupTestData(data, "ecm.json");

    data.Increments.SetAll(3.0 * cmnPI_180); // use 3 degrees sampling
    data.Increments.at(2) = 2.0 * cmn_cm; // except for the translation stage

    TestSampleJointSpace(data);
}


void robManipulatorTest::TestMTMIKSampleJointSpace(void)
{
    // load manipulator
    ManipulatorTestDataMTM data;
    SetupTestData(data, "mtmr.json");

    data.Increments.SetAll(15.0 * cmnPI_180); // use 15 degrees sampling
    data.LowerLimits.at(6) = -cmnPI;
    data.UpperLimits.at(6) =  cmnPI;

    TestSampleJointSpace(data);
}
