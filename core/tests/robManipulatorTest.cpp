/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2019-11-11

  (C) Copyright 2019-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "robManipulatorTest.h"

#include <cisstCommon/cmnSearchPath.h>
#include <cisstCommon/cmnUnits.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitConfig.h>
#include "sawIntuitiveResearchKit/robManipulatorECM.h"


class ManipulatorTestDataECM: public ManipulatorTestData {
public:
    ManipulatorTestDataECM(void)
    {
        Name = "ECM";
        NumberOfLinks = 4;
    };

    void CheckIKResults(void) {
        Eigen::VectorXd jointErrors = SolutionJoints - ActualJoints;
        Eigen::VectorXd jointErrorsAbsolute = jointErrors.cwiseAbs();

        // compare joint values
        CPPUNIT_ASSERT_MESSAGE("Joint 0 solution is incorrect",
                               (jointErrorsAbsolute[0] < 0.001 * cmnPI_180)); // asinl is not that precise

        CPPUNIT_ASSERT_MESSAGE("Joint 1 solution is incorrect",
                               (jointErrorsAbsolute[1] < 0.001 * cmnPI_180)); // asinl

        CPPUNIT_ASSERT_MESSAGE("Joint 2 solution is incorrect",
                               (jointErrorsAbsolute[2] < 0.000000001 * cmn_mm)); // just a sqrt, higher precision expected

        CPPUNIT_ASSERT_MESSAGE("Joint 3 solution is incorrect",
                               (jointErrorsAbsolute[3] < 0.001 * cmnPI_180)); // acosl

        // translation
        Eigen::Vector3d positionTranslationError = ActualPose.translation() - SolutionPose.translation();
        CPPUNIT_ASSERT_MESSAGE("Cartesian translation error is too high",
                               positionTranslationError.norm() < 0.005 * cmn_mm);

        // rotation
        Eigen::Matrix3d positionRotationError = ActualPose.rotation().inverse() * SolutionPose.rotation();
        CPPUNIT_ASSERT_MESSAGE("Cartesian rotation error is too high",
                               Eigen::AngleAxisd(positionRotationError).angle() < 0.0000001 * cmnPI_180);
    }
};


class ManipulatorTestDataMTM: public ManipulatorTestData {
public:
    ManipulatorTestDataMTM(void)
    {
        Name = "MTM";
        NumberOfLinks = 7;
    };

    void CheckIKResults(void) {
        Eigen::VectorXd jointErrors = SolutionJoints - ActualJoints;
        Eigen::VectorXd jointErrorsAbsolute = jointErrors.cwiseAbs();

        std::stringstream details;
        details << "Actual joints: " << (SolutionJoints * cmn180_PI).transpose() << "\n"
                   "Solution     : " << (ActualJoints * cmn180_PI).transpose()   << "\n"
                   "Error        : " << (jointErrors * cmn180_PI).transpose()    << "\n";
        const std::string detail_str = details.str();

        // compare joint values
        CPPUNIT_ASSERT_MESSAGE("Joint 0 solution is incorrect\n" + detail_str,
                               (jointErrorsAbsolute[0] < 0.001 * cmnPI_180)); // asinl is not that precise

        CPPUNIT_ASSERT_MESSAGE("Joint 1 solution is incorrect\n" + detail_str,
                               (jointErrorsAbsolute[1] < 0.001 * cmnPI_180)); // asinl

        CPPUNIT_ASSERT_MESSAGE("Joint 2 solution is incorrect\n" + detail_str,
                               (jointErrorsAbsolute[2] < 0.001 * cmnPI_180)); // asinl

        // translation
        const Eigen::Vector3d positionTranslationError = ActualPose.translation() - SolutionPose.translation();
        CPPUNIT_ASSERT_MESSAGE("Cartesian translation error is too high",
                               positionTranslationError.norm() < 0.005 * cmn_mm);

        // rotation
        const Eigen::Matrix3d positionRotationError = ActualPose.inverse() * SolutionPose.rotation();
        const double rotation_error = Eigen::AngleAxisd(positionRotationError).angle();
        CPPUNIT_ASSERT_MESSAGE("Cartesian rotation error is too high", rotation_error < 0.0000001 * cmn180_PI);
    }
};


void robManipulatorTest::SetupTestData(ManipulatorTestData & data,
                                       const std::string & filename)
{
    // find the file
    cmnSearchPath path;
    path.Add(std::string(sawIntuitiveResearchKit_SOURCE_CONFIG_DIR) + "/kinematic", cmnSearchPath::Position::TAIL);
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
    CPPUNIT_ASSERT_MESSAGE("Can't find \"DH\" in " + configFile, !jsonDH.isNull());

    const auto kdl_chain = loadKDLChain(jsonDH);
    CPPUNIT_ASSERT_MESSAGE("Failed to load kinematic chain from " + configFile, kdl_chain.has_value());

    const auto joint_config = loadJointConfig(jsonDH);
    CPPUNIT_ASSERT_MESSAGE("Failed to load joint config from " + configFile, joint_config.has_value());
    data.Manipulator = std::make_unique<robManipulatorECM>(joint_config.value(), kdl_chain.value());

    // verify number of links in robManipulator
    CPPUNIT_ASSERT_EQUAL_MESSAGE("Expected number of links for " + filename,
                                 (int)data.NumberOfLinks, data.Manipulator->numJoints());

    // allocate test data
    data.LowerLimits = joint_config->GetPositionMin();
    data.UpperLimits = joint_config->GetPositionMax();
    data.Increments = Eigen::VectorXd::Zero(data.NumberOfLinks);
    data.ActualJoints = Eigen::VectorXd::Zero(data.NumberOfLinks);
    data.PreviousActualJoints = Eigen::VectorXd::Zero(data.NumberOfLinks);
    data.SolutionJoints = Eigen::VectorXd::Zero(data.NumberOfLinks);
}

void robManipulatorTest::ComputeAndTestIK(ManipulatorTestData & data)
{
    // compute FK
    data.ActualPose = data.Manipulator->forwardKinematics(data.ActualJoints);
    // compute IK
    data.SolutionJoints = data.ActualJoints;
    std::string ik_error;
    bool ik_ok = data.Manipulator->inverseKinematics(
        data.ActualPose,
        data.SolutionJoints,
        ik_error
    );
    // make sure IK didn't complain
    CPPUNIT_ASSERT_MESSAGE("robManipulator::InverseKinematics result for "
                                 + data.Name + ": " + ik_error, ik_ok);

    // compute solution pose
    data.SolutionPose = data.Manipulator->forwardKinematics(data.SolutionJoints);

    // arm specific testing
    data.CheckIKResults();
}

void robManipulatorTest::TestSampleJointSpace(ManipulatorTestData & data)
{
    Eigen::VectorXd directions = Eigen::VectorXd::Constant(data.NumberOfLinks, 1.0);

    // initial position
    data.ActualJoints = data.LowerLimits;
    data.PreviousActualJoints = data.LowerLimits;
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
                    // valid position
                    data.PreviousActualJoints = data.ActualJoints;
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

    data.Increments.fill(3.0 * cmnPI_180); // use 3 degrees sampling
    data.Increments(2) = 2.0 * cmn_cm; // except for the translation stage

    TestSampleJointSpace(data);
}


void robManipulatorTest::TestMTMIKSampleJointSpace(void)
{
    // load manipulator
    ManipulatorTestDataMTM data;
    SetupTestData(data, "mtmr.json");

    data.Increments.fill(15.0 * cmnPI_180); // use 15 degrees sampling
    data.LowerLimits(6) = -cmnPI;
    data.UpperLimits(6) =  cmnPI;

    TestSampleJointSpace(data);
}
