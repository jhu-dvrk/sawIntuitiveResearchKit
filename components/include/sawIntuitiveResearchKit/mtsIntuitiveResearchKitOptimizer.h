/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Preetham Chalasani
  Created on: 2014-08-13

  (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitOptimizer_h
#define _mtsIntuitiveResearchKitOptimizer_h


// system include
#include <iostream>
#include <time.h>

// cisst
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstRobot/robManipulator.h>

// sawConstraintController
#include <sawConstraintController/prmKinematicsState.h>
#include <sawConstraintController/mtsVFController.h>
#include <sawConstraintController/mtsVFDataPlane.h>
#include <sawIntuitiveResearchKit/mtsVFDataFollow.h>
#include <sawIntuitiveResearchKit/mtsVFFollow.h>
#include <sawIntuitiveResearchKit/mtsVFFollowJacobian.h>

class mtsIntuitiveResearchKitOptimizer : public mtsVFController
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION, CMN_LOG_LOD_RUN_ERROR);

public:

    mtsIntuitiveResearchKitOptimizer(void) {};
    mtsIntuitiveResearchKitOptimizer(const size_t numOfJoints);
    inline ~mtsIntuitiveResearchKitOptimizer() {}
    void UpdateJacobian(const robManipulator & manip);
    void UpdateKinematics(vctDoubleVec & qCurr,
                          vctFrm4x4 cartesianCurrent,
                          vctFrm4x4 cartesianDesired);

    void UpdateParams(vctDoubleVec & qCurr,
                      const robManipulator & manip,
                      const double tickTime,
                      vctFrm4x4 cartesianCurrent,
                      vctFrm4x4 cartesianDesired);

    void InitializeFollowVF(const size_t objectiveRows,
                            const std::string & vfName,
                            const std::string & currentKinName,
                            const std::string & desiredKinName = "");

    void InitializePlaneVF(const size_t rows,
                           mtsVFDataPlane & planeData,
                           const std::string & currentKinName,
                           const std::string & desiredKinName);

    bool Solve(vctDoubleVec & dq);

protected:
    // VF variables
    prmKinematicsState CurrentSlaveKinematics;
    prmKinematicsState DesiredSlaveKinematics;
    mtsVFDataFollow FollowData;
    vctDoubleVec ControllerOutput;
    prmJointState CurrentJointState;

    size_t NumOfJoints;

    struct {
        vctDoubleMat BodyJacobian;
        vctDoubleMat Adjoint;
    } Cached;

private:

    //! Adds/Updates a vf data object
    void AddVFFollow(const mtsVFDataBase & vf);

    //! Adds/Updates a vf data object
    void AddVFFollowJacobian(const mtsVFDataBase & vf);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitOptimizer);

#endif // _mtsIntuitiveResearchKitOptimizer_h
