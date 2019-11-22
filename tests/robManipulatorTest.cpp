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

void robManipulatorTest::LoadKinematics(robManipulator & manipulator,
                                        const std::string & filename,
                                        const size_t expectedNumberOfLinks)
{
    // find the file
    cmnPath path;
    path.Add(std::string(sawIntuitiveResearchKit_SOURCE_DIR) + "/../share/kinematic", cmnPath::TAIL);
    const std::string ecmConfigFile = path.Find(filename);
    CPPUNIT_ASSERT_MESSAGE("Can't find full path for " + filename,
                           ecmConfigFile != std::string(""));

    // json parsing
    std::ifstream jsonStream;
    Json::Value jsonConfig;
    Json::Reader jsonReader;
    jsonStream.open(ecmConfigFile.c_str());
    bool fileParsed = jsonReader.parse(jsonStream, jsonConfig);
    CPPUNIT_ASSERT_MESSAGE("Failed to parse JSON file " + ecmConfigFile + ": " + jsonReader.getFormattedErrorMessages(),
                           fileParsed);

    // look for DH in file
    const Json::Value jsonDH = jsonConfig["DH"];
    CPPUNIT_ASSERT_MESSAGE("Can't find \"DH\" in " + ecmConfigFile,
                           !jsonDH.isNull());

    // try to load the DH parameters
    CPPUNIT_ASSERT_MESSAGE("Failed while loading from JSON \"DH\" value in " + ecmConfigFile,
                           manipulator.LoadRobot(jsonDH) == robManipulator::ESUCCESS);

    // verify number of links in robManipulator
    CPPUNIT_ASSERT_EQUAL_MESSAGE("Expected number of links for ECM",
                                 expectedNumberOfLinks, manipulator.links.size());
}

void robManipulatorTest::SampleJointSpace(const robManipulator & manipulator,
                                          const vctDoubleVec & lowerLimits,
                                          const vctDoubleVec & upperLimits,
                                          const vctDoubleVec & increments,
                                          std::list<vctDoubleVec> & joints,
                                          std::list<vctFrm4x4> & positions)
{
    const size_t numberOfLinks = manipulator.links.size();
    CPPUNIT_ASSERT_EQUAL_MESSAGE("Size of lower limits vector",
                                 numberOfLinks,
                                 lowerLimits.size());
    CPPUNIT_ASSERT_EQUAL_MESSAGE("Size of upper limits vector",
                                 numberOfLinks,
                                 upperLimits.size());
    CPPUNIT_ASSERT_EQUAL_MESSAGE("Size of increments vector",
                                 numberOfLinks,
                                 increments.size());

    vctDoubleVec directions(numberOfLinks, 1.0);
    vctDoubleVec currentJoints(lowerLimits);

    joints.push_back(currentJoints);
    positions.push_back(manipulator.ForwardKinematics(currentJoints));

    bool allReached = false;
    while (!allReached) {
        bool nextDimensionIncrement = true;
        for (size_t index = 0; index < numberOfLinks; ++index) {
            if (nextDimensionIncrement) {
                double future = currentJoints[index] + directions[index] * increments[index];
                if (future > upperLimits[index]) {
                    directions[index] = -1.0;
                    if (index == (numberOfLinks - 1)) {
                        allReached = true;
                    }
                } else if (future < lowerLimits[index]) {
                    directions[index] = 1.0;
                } else {
                    currentJoints[index] = future;
                    nextDimensionIncrement = false;
                }
            }
        }
        joints.push_back(currentJoints);
        positions.push_back(manipulator.ForwardKinematics(currentJoints));
    }
}

void robManipulatorTest::TestECMIKSampleJointSpace(void)
{
    // load manipulator
    const size_t numberOfLinks = 4;
    robManipulatorECM manipulator;
    robManipulatorTest::LoadKinematics(manipulator, "ecm.json", numberOfLinks);

    std::list<vctDoubleVec> joints;
    std::list<vctFrm4x4> positions;

    // generate trajectory
    vctDoubleVec lowerLimits(numberOfLinks), upperLimits(numberOfLinks);
    manipulator.GetJointLimits(lowerLimits,
                               upperLimits);
    vctDoubleVec increments(numberOfLinks, 3.0 * cmnPI_180); // use 3 degrees sampling
    increments.at(2) = 2.0 * cmn_cm; // except for the translation stage
    robManipulatorTest::SampleJointSpace(manipulator,
                                         lowerLimits,
                                         upperLimits,
                                         increments,
                                         joints,
                                         positions);

    CPPUNIT_ASSERT_EQUAL_MESSAGE("SampleJointSpace should generate same number of joints and positions",
                                 joints.size(), positions.size());

    // initialize the solution with first position
    vctDoubleVec solution = joints.front();
    for (size_t positionIndex = 0;
         positionIndex < joints.size();
         ++positionIndex) {
        // pop joint and position
        vctDoubleVec joint = joints.front();
        joints.pop_front();
        vctFrm4x4 position = positions.front();
        positions.pop_front();
        // solve IK
        robManipulator::Errno result;
        result = manipulator.InverseKinematics(solution, position);
        // make sure IK didn't complain
        CPPUNIT_ASSERT_EQUAL_MESSAGE("robManipulatorECM::InverseKinematics result",
                                     robManipulator::ESUCCESS,
                                     result);

        vctDoubleVec jointError(numberOfLinks), jointErrorAbsolute(numberOfLinks);
        jointError.DifferenceOf(solution, joint);
        jointErrorAbsolute.AbsOf(jointError);

        // compare joint values
        CPPUNIT_ASSERT_MESSAGE("Joint 0 solution is incorrect",
                               (jointErrorAbsolute[0] < 0.0000001 * cmn180_PI)); // asinl is not that precise

        CPPUNIT_ASSERT_MESSAGE("Joint 1 solution is incorrect",
                               (jointErrorAbsolute[1] < 0.0000001 * cmn180_PI)); // asinl

        CPPUNIT_ASSERT_MESSAGE("Joint 2 solution is incorrect",
                               (jointErrorAbsolute[2] < 0.000000001 * cmn_mm)); // just a sqrt, higher precision expected

        CPPUNIT_ASSERT_MESSAGE("Joint 3 solution is incorrect",
                               (jointErrorAbsolute[3] < 0.0000001 * cmn180_PI)); // acosl

        // compare cartesian positions
        vctFrm4x4 solutionPosition = manipulator.ForwardKinematics(solution);

        // translation
        vct3 positionTranslationError = position.Translation() - solutionPosition.Translation();
        CPPUNIT_ASSERT_MESSAGE("Cartesian translation error is too high",
                               positionTranslationError.Norm() < 0.005 * cmn_mm);

        // rotation
        vctMatRot3 positionRotationError;
        position.Rotation().ApplyInverseTo(solutionPosition.Rotation(), positionRotationError);
        CPPUNIT_ASSERT_MESSAGE("Cartesian rotation error is too high",
                               vctAxAnRot3(positionRotationError).Angle() < 0.0000001 * cmn180_PI); // acosl

        // set solution from current joint for next loop, it's close
        // to the next joint so it somewhat simulates a continuous
        // trajectory for the IK
        solution = joint;
    }
}


void robManipulatorTest::TestMTMIKSampleJointSpace(void)
{
    // load manipulator
    const size_t numberOfLinks = 7;
    robManipulatorMTM manipulator;
    robManipulatorTest::LoadKinematics(manipulator, "mtmr.json", numberOfLinks);

    std::list<vctDoubleVec> joints;
    std::list<vctFrm4x4> positions;

    // generate trajectory
    vctDoubleVec lowerLimits(numberOfLinks), upperLimits(numberOfLinks);
    manipulator.GetJointLimits(lowerLimits,
                               upperLimits);
    vctDoubleVec increments(numberOfLinks, 15.0 * cmnPI_180); // use 15 degrees sampling
    // reduce space on roll
    lowerLimits[6] = -cmnPI;
    upperLimits[6] =  cmnPI;

    robManipulatorTest::SampleJointSpace(manipulator,
                                         lowerLimits,
                                         upperLimits,
                                         increments,
                                         joints,
                                         positions);

    CPPUNIT_ASSERT_EQUAL_MESSAGE("SampleJointSpace should generate same number of joints and positions",
                                 joints.size(), positions.size());

    // initialize the solution with first position
    vctDoubleVec solution = joints.front();
    for (size_t positionIndex = 0;
         positionIndex < joints.size();
         ++positionIndex) {
        // pop joint and position
        vctDoubleVec joint = joints.front();
        joints.pop_front();
        vctFrm4x4 position = positions.front();
        positions.pop_front();
        // solve IK
        robManipulator::Errno result;
        result = manipulator.InverseKinematics(solution, position);
        // make sure IK didn't complain
        CPPUNIT_ASSERT_EQUAL_MESSAGE("robManipulatorMTM::InverseKinematics result",
                                     robManipulator::ESUCCESS,
                                     result);

        vctDoubleVec jointError(numberOfLinks), jointErrorAbsolute(numberOfLinks);
        jointError.DifferenceOf(solution, joint);
        jointErrorAbsolute.AbsOf(jointError);

        // compare joint values
        CPPUNIT_ASSERT_MESSAGE("Joint 0 solution is incorrect",
                               (jointErrorAbsolute[0] < 0.0000001 * cmn180_PI)); // asinl is not that precise

        CPPUNIT_ASSERT_MESSAGE("Joint 1 solution is incorrect",
                               (jointErrorAbsolute[1] < 0.0000001 * cmn180_PI)); // asinl

        CPPUNIT_ASSERT_MESSAGE("Joint 2 solution is incorrect",
                               (jointErrorAbsolute[2] < 0.0000001 * cmn180_PI)); // asinl

        // compare cartesian positions
        vctFrm4x4 solutionPosition = manipulator.ForwardKinematics(solution);

        // translation
        vct3 positionTranslationError = position.Translation() - solutionPosition.Translation();
        CPPUNIT_ASSERT_MESSAGE("Cartesian translation error is too high",
                               positionTranslationError.Norm() < 0.005 * cmn_mm);

#if 0
        // rotation
        vctMatRot3 positionRotationError;
        position.Rotation().ApplyInverseTo(solutionPosition.Rotation(), positionRotationError);
        CPPUNIT_ASSERT_MESSAGE("Cartesian rotation error is too high",
                               vctAxAnRot3(positionRotationError).Angle() < 0.0000001 * cmn180_PI); // acosl
#endif

        // set solution from current joint for next loop, it's close
        // to the next joint so it somewhat simulates a continuous
        // trajectory for the IK
        solution = joint;
    }
}
