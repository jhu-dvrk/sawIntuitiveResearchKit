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

#include <cppunit/TestCase.h>
#include <cppunit/extensions/HelperMacros.h>

#include <cisstVector/vctDynamicVectorTypes.h>
#include <sawIntuitiveResearchKit/robManipulatorECM.h>
#include <sawIntuitiveResearchKit/robManipulatorMTM.h>

class ManipulatorTestData {
public:
    std::string Name;
    size_t NumberOfLinks;
    robManipulator * Manipulator;
    vctDoubleVec LowerLimits;
    vctDoubleVec UpperLimits;
    vctDoubleVec Increments;
    vctDoubleVec ActualJoints, PreviousActualJoints;
    vctFrm4x4 ActualPose;
    vctDoubleVec SolutionJoints;
    vctFrm4x4 SolutionPose;

    virtual void CheckIKResults(void) = 0;
};

class robManipulatorTest : public CppUnit::TestFixture
{
protected:

    CPPUNIT_TEST_SUITE(robManipulatorTest);
    {
        CPPUNIT_TEST(TestECMIKSampleJointSpace);
        CPPUNIT_TEST(TestMTMIKSampleJointSpace);
    }
    CPPUNIT_TEST_SUITE_END();

    // helper method to load kinematics with some basic tests
    void SetupTestData(ManipulatorTestData & data,
                       const std::string & filename);

    void ComputeAndTestIK(ManipulatorTestData & data);

    // create a trajectory of points over the whole joint space and
    // returns joint values as well as forward kinematic
    void TestSampleJointSpace(ManipulatorTestData & data);

public:

    void setUp(void) {
    }

    void tearDown(void) {
    }

    void TestECMIKSampleJointSpace(void);

    void TestMTMIKSampleJointSpace(void);
};

CPPUNIT_TEST_SUITE_REGISTRATION(robManipulatorTest);
