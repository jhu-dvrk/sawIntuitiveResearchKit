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

#include <sawIntuitiveResearchKit/mtsVFFollowJacobian.h>
#include <stdlib.h>
#include <cisstNumerical/nmrInverse.h>

CMN_IMPLEMENT_SERVICES(mtsVFFollowJacobian)

//! Updates co with virtual fixture data.
/*! FillInTableauRefs
*/
void mtsVFFollowJacobian::FillInTableauRefs(const CONTROLLERMODE mode, const double TickTime)
{
    // fill in refs
    // min || J*dq - [v_T ; v_R] ||
    // v_T = p_des - R_des * R^(-1)_cur * p_cur
    // v_R = sk(A)^(-1) * A - sk(A)^(-1) * R_des * R^(-1)_cur * A
    // J is the da vinci's jacobian, p_des is the desired frame's translation,
    // R_des is the desired frame's rotation, R^(-1)_cur is the inverse of the current frame's rotation,
    // p_cur is the current frame's translation, A is an arbitrary vector, sk(A)^(-1) is the inverse of the skew matrix of A

    //check desired frame, current frame dependencies
    if(Kinematics.size() < 2)
    {
        CMN_LOG_CLASS_RUN_ERROR << "FillInTableauRefs: Follow VF given improper input" << std::endl;
        cmnThrow("FillInTableauRefs: Follow VF given improper input");
    }

    // pointers to kinematics
    CurrentKinematics = Kinematics.at(0);
    DesiredKinematics = Kinematics.at(1);

    // Current Frame
    vctFrm4x4 CurrentFrame;
    CurrentFrame.FromNormalized(CurrentKinematics->Frame);

    // Desired Frame
    vctFrm4x4 DesiredFrame;
    DesiredFrame.FromNormalized(DesiredKinematics->Frame);

    // Incremental Frame
    vctFrm4x4 Pose_dx = CurrentFrame.Inverse() * DesiredFrame;


    vct3 dx_translation, dx_rotation;

    // Translation Part
    dx_translation[0] = Pose_dx.Translation().X();
    dx_translation[1] = Pose_dx.Translation().Y();
    dx_translation[2] = Pose_dx.Translation().Z();
    dx_translation = CurrentFrame.Rotation() * dx_translation;

    // Rotation part
    vctAxAnRot3 dxRot;
    vct3 dxRotVec;
    dxRot.FromNormalized(Pose_dx.Rotation());
    dxRotVec = dxRot.Axis() * dxRot.Angle();
    dx_rotation[0] = dxRotVec[0];
    dx_rotation[1] = dxRotVec[1];
    dx_rotation[2] = dxRotVec[2];
    dx_rotation = CurrentFrame.Rotation() * dx_rotation;

    // Constructing the dx parameter
    vctDoubleVec dx(6);
    std::copy(dx_translation.begin(), dx_translation.end(), dx.begin()  );
    std::copy(dx_rotation.begin()   , dx_rotation.end()   , dx.begin()+3);

    // Delta x
    ObjectiveVectorRef.Assign(dx);

    // Put Jacobian into matrix ref
    ObjectiveMatrixRef.Assign(CurrentKinematics->Jacobian);


    // make conversion, if necessary
    ConvertRefs(mode,TickTime);


}
