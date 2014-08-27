/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Paul Wilkening
  Created on: 2014

  (C) Copyright 2014 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#include <sawIntuitiveResearchKit/mtsVFFollow.h>

CMN_IMPLEMENT_SERVICES(mtsVFFollow)

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFFollow::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{
    // fill in refs
    // min || I*dq - (q_des - q_curr) ||
    // I is the identity matrix, q_des is the desired joint set, q_curr is the current joint set

    //check desired frame, current frame dependencies
    if(Kinematics.size() < 2)
    {
        CMN_LOG_CLASS_RUN_ERROR << "FillInTableauRefs: Follow VF given improper input" << std::endl;
        cmnThrow("FillInTableauRefs: Follow VF given improper input");
    }

    // pointers to kinematics
    CurrentKinematics = Kinematics.at(0);
    DesiredKinematics = Kinematics.at(1);

    // current kinematics gives us current joint set
    int nJoints = CurrentKinematics->Jacobian.cols();
    CurrentJointSet.SetSize(nJoints);
    CurrentJointSet.Assign(CurrentKinematics->JointState->JointPosition, nJoints);    

    // desired kinematics gives us desired frame
    DesiredFrame.FromNormalized(DesiredKinematics->Frame);

    // use desired frame to solve for desired joint set
    DesiredJointSet.SetSize(nJoints);
    DesiredJointSet.Assign(CurrentJointSet,nJoints);    

    // make sure manipulator object exists
    if(Manipulator)
    {
        // make sure inverse kinematics call has succeeded
        if(Manipulator->InverseKinematics(DesiredJointSet, DesiredFrame) == robManipulator::ESUCCESS)
        {

            // Identity matrix
            ObjectiveMatrixRef.Diagonal().SetAll(1.0);
            // q_des - q_curr
            ObjectiveVectorRef.Assign(DesiredJointSet - CurrentJointSet);

//            ObjectiveVectorRef.Assign(DesiredKinematics->Frame.Translation() - CurrentKinematics->Frame.Translation());

            // make conversion, if necessary
            ConvertRefs(mode,TickTime);
        }

    }
    else
    {
        cmnThrow("FillInTableauRefs: Inverse Kinematics failed");
    }

}
