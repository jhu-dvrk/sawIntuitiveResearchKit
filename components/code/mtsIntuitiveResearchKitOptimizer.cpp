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

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitOptimizer.h>

CMN_IMPLEMENT_SERVICES(mtsIntuitiveResearchKitOptimizer);

vctReturnDynamicMatrix<double> ComputeAdjointMatrix(const vctFrame4x4<double> & Rt)
{
    vctDynamicMatrix<double> Ad(6, 6);
    Ad.Zeros(); // fast zero using memset

    // upper left block
    Ad[0][0] = Rt[0][0];     Ad[0][1] = Rt[0][1];     Ad[0][2] = Rt[0][2];
    Ad[1][0] = Rt[1][0];     Ad[1][1] = Rt[1][1];     Ad[1][2] = Rt[1][2];
    Ad[2][0] = Rt[2][0];     Ad[2][1] = Rt[2][1];     Ad[2][2] = Rt[2][2];

    // upper right block
    Ad[0][3] = 0.0;
    Ad[0][4] = 0.0;
    Ad[0][5] = 0.0;

    Ad[1][3] = 0.0;
    Ad[1][4] = 0.0;
    Ad[1][5] = 0.0;

    Ad[2][3] = 0.0;
    Ad[2][4] = 0.0;
    Ad[2][5] = 0.0;

    // lower right block
    Ad[3][3] = Rt[0][0];     Ad[3][4] = Rt[0][1];     Ad[3][5] = Rt[0][2];
    Ad[4][3] = Rt[1][0];     Ad[4][4] = Rt[1][1];     Ad[4][5] = Rt[1][2];
    Ad[5][3] = Rt[2][0];     Ad[5][4] = Rt[2][1];     Ad[5][5] = Rt[2][2];

    return vctReturnDynamicMatrix<double>(Ad);
}

mtsIntuitiveResearchKitOptimizer::mtsIntuitiveResearchKitOptimizer(const size_t numOfJoints) :
    mtsVFController(numOfJoints, mtsVFBase::JPOS)
{
    NumOfJoints = numOfJoints;
    CurrentJointState.JointPosition.SetSize(NumOfJoints);
}

void mtsIntuitiveResearchKitOptimizer::InitializeFollowVF(const size_t rows,
                                                          const std::string & vfName,
                                                          const std::string & currentKinName,
                                                          const std::string & desiredKinName)
{
    // Set the Kinematics
    CurrentSlaveKinematics.Name = currentKinName.data();
    DesiredSlaveKinematics.Name = desiredKinName.data();
    CurrentSlaveKinematics.JointState = &CurrentJointState;

    FollowData.Name = vfName.data();
    FollowData.ObjectiveRows = rows;
    FollowData.KinNames.clear();
    FollowData.KinNames.push_back(currentKinName);
    FollowData.KinNames.push_back(desiredKinName);
    AddVFFollowJacobian(FollowData);

    CurrentSlaveKinematics.Jacobian.SetSize(rows, NumOfJoints, VCT_COL_MAJOR);
}

void mtsIntuitiveResearchKitOptimizer::InitializePlaneVF(size_t rows,
                                                         mtsVFDataPlane & planeData,
                                                         const std::string & currentKinName,
                                                         const std::string & desiredKinName)
{
    planeData.KinNames.push_back(currentKinName);
    planeData.IneqConstraintRows = rows;
    AddVFPlane(planeData);
}

void mtsIntuitiveResearchKitOptimizer::UpdateParams(vctDoubleVec & qCurr,
                                                    const robManipulator & manip,
                                                    double tickTime,
                                                    vctFrm4x4 cartesianCurrent,
                                                    vctFrm4x4 cartesianDesired)
{
    UpdateKinematics(qCurr, cartesianCurrent, cartesianDesired);
    UpdateJacobian(manip);
    UpdateOptimizer(tickTime);
}

void mtsIntuitiveResearchKitOptimizer::UpdateKinematics(vctDoubleVec & qCurr,
                                                        vctFrm4x4 cartesianCurrent,
                                                        vctFrm4x4 cartesianDesired)
{
    CurrentJointState.JointPosition = qCurr;

    // Populating the currest slave kinematics object
    CurrentSlaveKinematics.Frame.FromNormalized(cartesianCurrent);
    SetKinematics(CurrentSlaveKinematics);

    // Populating the desired slave kinematics object
    DesiredSlaveKinematics.Frame.FromNormalized(cartesianDesired);
    SetKinematics(DesiredSlaveKinematics);
}

void mtsIntuitiveResearchKitOptimizer::UpdateJacobian(const robManipulator & manip)
{
    // ask manipulator to compute body jacobian
    manip.JacobianBody(CurrentJointState.JointPosition);

    // to make sure we have the right size
    Cached.BodyJacobian.SetSize(6, 6);
    // get a copy of the body jacobian
    Cached.BodyJacobian.Assign(manip.Jn[0], VCT_COL_MAJOR);

    // Base of the body jacobian is the tip frame and the ref point is the tip
    // Have to convert the base to the world frame

    vctFrame4x4<double> Rt0n = manip.ForwardKinematics(CurrentJointState.JointPosition);

    Cached.Adjoint = ComputeAdjointMatrix(Rt0n);

    CurrentSlaveKinematics.Jacobian = Cached.Adjoint * Cached.BodyJacobian;
}

bool mtsIntuitiveResearchKitOptimizer::Solve(vctDoubleVec & dq)
{
    nmrConstraintOptimizer::STATUS OptimizerStatus;
    OptimizerStatus = mtsVFController::Solve(dq);

    if (OptimizerStatus == nmrConstraintOptimizer::NMR_OK) {
        return true;
    }

    // If the optimizer fails set the value of dq to be 0
    // This will make sure the robot doesn't move
    dq.SetAll(0);
    CMN_LOG_CLASS_RUN_ERROR << "Control Optimizer returned status:  " << GetOptimizer().GetStatusString(OptimizerStatus) << std::endl;
    return false;
}

//! Adds/updates a sensor compliance virtual fixture in the map and increments users of kinematics and sensors
/*! AddVFFollow
@param vf virtual fixture to be added
*/
void mtsIntuitiveResearchKitOptimizer::AddVFFollow(const mtsVFDataBase & vf)
{
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
    if (!SetVFData(vf, typeid(mtsVFFollow)))
    {
        // Adds a new virtual fixture to the active vector
        VFMap.insert(std::pair<std::string,mtsVFFollow *>(vf.Name,new mtsVFFollow(vf.Name,new mtsVFDataBase(vf))));
        // Increment users of each kinematics and sensor object found
        IncrementUsers(vf.KinNames,vf.SensorNames);
    }
}

//! Adds/updates a sensor compliance virtual fixture in the map and increments users of kinematics and sensors
/*! AddVFFollow
@param vf virtual fixture to be added
*/
void mtsIntuitiveResearchKitOptimizer::AddVFFollowJacobian(const mtsVFDataBase & vf)
{
    // If we can find the VF, only change its data. Otherwise, create a new VF object.
    if (!SetVFData(vf, typeid(mtsVFFollowJacobian)))
    {
        // Adds a new virtual fixture to the active vector
        VFMap.insert(std::pair<std::string,mtsVFFollowJacobian *>(vf.Name,new mtsVFFollowJacobian(vf.Name,new mtsVFDataBase(vf))));
        // Increment users of each kinematics and sensor object found
        IncrementUsers(vf.KinNames,vf.SensorNames);
    }
}
