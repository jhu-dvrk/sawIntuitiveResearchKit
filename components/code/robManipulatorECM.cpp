/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2019-11-11

  (C) Copyright 2019-2020 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

#include <sawIntuitiveResearchKit/robManipulatorECM.h>

#include <cisstCommon/cmnUnits.h>
#include <math.h>

robManipulatorECM::robManipulatorECM(const std::vector<robKinematics *> linkParms,
                                     const vctFrame4x4<double> &Rtw0)
    : robManipulator(linkParms, Rtw0)
{
}

robManipulatorECM::robManipulatorECM(const std::string &robotfilename,
                                     const vctFrame4x4<double> &Rtw0)
    : robManipulator(robotfilename, Rtw0)
{
}

robManipulatorECM::robManipulatorECM(const vctFrame4x4<double> &Rtw0)
    : robManipulator(Rtw0)
{
}

robManipulator::Errno
robManipulatorECM::InverseKinematics(vctDynamicVector<double> & q,
                                     const vctFrame4x4<double> & Rts,
                                     double CMN_UNUSED(tolerance),
                                     size_t CMN_UNUSED(Niterations),
                                     double CMN_UNUSED(LAMBDA))
{
    if (q.size() != links.size()) {
        std::stringstream ss;
        ss << "robManipulatorECM::InverseKinematics: expected " << links.size()
           << " joints values but received " << q.size();
        mLastError = ss.str();
        CMN_LOG_RUN_ERROR << mLastError << std::endl;
        return robManipulator::EFAILURE;
    }

    if (links.size() == 0) {
        mLastError = "robManipulatorECM::InverseKinematics: the manipulator has no links";
        CMN_LOG_RUN_ERROR << mLastError << std::endl;
        return robManipulator::EFAILURE;
    }

    // take Rtw0 into account
    vctFrm4x4 Rt04t, Rt04; // t for "with tool"
    Rtw0.ApplyInverseTo(Rts, Rt04t);

    // take tool into account -> Rt04 from Rt04t
    if (tools.size() > 1) {
        mLastError = "robManipulatorECM::InverseKinematics: the manipulator has more than one tool attached";
        CMN_LOG_RUN_ERROR << mLastError << std::endl;
        return robManipulator::EFAILURE;
    } else if (tools.size() == 1) {
        CMN_ASSERT(tools[0]);
        Rt04t.ApplyTo(tools[0]->Rtw0.Inverse(), Rt04);
    } else {
        Rt04 = Rt04t;
    }

    // re-align desired frame to 4 axis direction to reduce free space
    vctFrm4x4 Rt04a; // a for "aligned"
    Rt04a.Translation().Assign(Rt04.Translation());

    vctDouble3 shaft = Rt04.Translation();
    const double shaftNorm = shaft.Norm();
    // we should not allow anything in the cannula but at least make
    // sure it's numerically stable using 1mm
    if (shaftNorm < 0.1 * cmn_mm) {
        mLastError = "robManipulatorECM::InverseKinematics: cartesian goal is too close to RCM point";
        CMN_LOG_RUN_ERROR << mLastError << std::endl;
        return robManipulator::EFAILURE;
    }
    // normalize
    shaft.Divide(shaftNorm);
    const vctDouble3 Z = Rt04.Rotation().Column(2).Ref<3>(); // last column of rotation matrix

    if (! Z.AlmostEqual(shaft, 0.0001)) {
        vctMatRot3 reAlign;
        vct3 axis;
        double angle;
        axis.CrossProductOf(Z, shaft);
        axis.NormalizedSelf();
        angle = acos(vctDotProduct(Z, shaft));
        reAlign.From(vctAxAnRot3(axis, angle, VCT_NORMALIZE));
        Rt04a.Rotation().ProductOf(reAlign, Rt04.Rotation());
    } else {
        Rt04a.Rotation().Assign(Rt04.Rotation());
    }

    // now compute close kinematics without tool nor base frame and
    // knowing that solution exists
    const double x = Rt04a.Translation().X();
    const double x2 = x * x;
    const double y = Rt04a.Translation().Y();
    const double y2 = y * y;
    const double z = Rt04a.Translation().Z();
    const double z2 = z * z;

    // if we encounter a joint limit, keep computing a solution but at
    // the end return failure
    bool hasReachedJointLimit = false;

    // hard coded check for dVRK classic, wouldn't work on S/Si system
    if (z > cmnTypeTraits<double>::Tolerance()) {
        mLastError = "robManipulatorECM::InverseKinematics: z is positive, out of reach";
        hasReachedJointLimit = true;
    }

    // first joint controls x position
    if (std::abs(x) < cmnTypeTraits<double>::Tolerance()) {
        q[0] = 0.0;
    } else {
        const double depthX = std::sqrt(x2 + z2);
        if (depthX < cmnTypeTraits<double>::Tolerance()) {
            q[0] = 0.0;
        } else {
            q[0] = asinl(x / depthX);
        }
    }

    // similar computation for second joint
    const double depth = std::sqrt(x2 + y2 + z2);
    if (std::abs(y) < cmnTypeTraits<double>::Tolerance()) {
        q[1] = 0.0;
    } else {
        const double depth = std::sqrt(x2 + y2 + z2);
        if (depth < cmnTypeTraits<double>::Tolerance()) {
            q[1] = 0.0;
        } else {
            q[1] = -asinl(y / depth);
        }
    }

    // make sure we respect joint limits
    if (ClampJointValueAndUpdateError(0, q[0], 1e-5)) {
        hasReachedJointLimit = true;
    }
    if (ClampJointValueAndUpdateError(1, q[1], 1e-5)) {
        hasReachedJointLimit = true;
    }

    // third is translation, i.e. depth
    q[2] = depth - 0.0007; // 0.0007 is from DH link_3.offset - link_4.D

    // check that depth is reachable
    if (q[2] > links[2].GetKinematics()->PositionMax()) {
        mLastError = "robManipulatorECM::InverseKinematics: goal is too far, out of reach for q[2]";
        hasReachedJointLimit = true;
    }

    // for orientation, we assume the goal is reachable
    vctFrm4x4 Rt03w = ForwardKinematics(q, 3);
    vctFrm4x4 Rt03; // same but w/o Rtw0
    Rtw0.ApplyInverseTo(Rt03w, Rt03);

    vctFrm4x4 Rt34; // rotation for last link
    Rt03.ApplyInverseTo(Rt04a, Rt34);

    vctMatRot3 rotationMatrix(Rt34.Rotation());
    vctAxAnRot3 axisAngle(rotationMatrix);
    // it's a rotation along z or -z
    if (axisAngle.Axis()[2] < 0) {
        q[3] = -axisAngle.Angle();
    } else {
        q[3] = axisAngle.Angle();
    }

    if (ClampJointValueAndUpdateError(3, q[3], 1e-5)) {
        hasReachedJointLimit = true;
    }

    if (hasReachedJointLimit) {
        return robManipulator::EFAILURE;
    }

    return robManipulator::ESUCCESS;
}
