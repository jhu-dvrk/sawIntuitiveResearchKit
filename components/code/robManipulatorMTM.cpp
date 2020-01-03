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

#include <sawIntuitiveResearchKit/robManipulatorMTM.h>
#include <math.h>

robManipulatorMTM::robManipulatorMTM(const std::vector<robKinematics *> linkParms,
                                     const vctFrame4x4<double> &Rtw0)
    : robManipulator(linkParms, Rtw0)
{
}

robManipulatorMTM::robManipulatorMTM(const std::string &robotfilename,
                                     const vctFrame4x4<double> &Rtw0)
    : robManipulator(robotfilename, Rtw0)
{
}

robManipulatorMTM::robManipulatorMTM(const vctFrame4x4<double> &Rtw0)
    : robManipulator(Rtw0)
{
}

robManipulator::Errno
robManipulatorMTM::InverseKinematics(vctDynamicVector<double> & q,
                                     const vctFrame4x4<double> & Rts,
                                     double CMN_UNUSED(tolerance),
                                     size_t CMN_UNUSED(Niterations),
                                     double CMN_UNUSED(LAMBDA))
{
    if (q.size() != links.size()) {
        std::stringstream ss;
        ss << "robManipulatorMTM::InverseKinematics: expected " << links.size()
           << " joints values but received " << q.size();
        mLastError = ss.str();
        CMN_LOG_RUN_ERROR << mLastError << std::endl;
        return robManipulator::EFAILURE;
    }

    if (links.size() == 0) {
        mLastError = "robManipulatorMTM::InverseKinematics: the manipulator has no links";
        CMN_LOG_RUN_ERROR << mLastError << std::endl;
        return robManipulator::EFAILURE;
    }

    // if we encounter a joint limit, keep computing a solution but at
    // the end return failure
    bool hasReachedJointLimit = false;

    // take Rtw0 into account
    vctFrm4x4 Rt07;
    Rtw0.ApplyInverseTo(Rts, Rt07);

    q[0] = atan2l(Rt07.Translation().X(),
                  -Rt07.Translation().Y());

    // arm is provided in ISI DH
    const double l1 = 0.2794;
    const double l1_sqr = l1 * l1;

    // create a triangle "above" forarm to find position
    const double forarmBase = 0.3645; // from ISI documentation
    const double forarmHeight = 0.1506; // for ISI documentation
    const double l2_sqr = forarmBase * forarmBase + forarmHeight * forarmHeight;
    const double l2 = sqrt(l2_sqr) ;
    const double angleOffset = asinl(forarmHeight / l2);

    // project in plane formed by links 2 & 3 to find q2 and q3 (joint[1] and joint[2])
    const double x = -Rt07.Translation().Z();
    const double y = sqrt(Rt07.Translation().X() * Rt07.Translation().X()
                          + Rt07.Translation().Y() * Rt07.Translation().Y());

    // 2 dof IK in plane
    const double d_sqr = x * x + y * y;
    const double d = sqrt(d_sqr);
    const double a1 = atan2l(y, x);
    const double a2 = acosl((l1_sqr - l2_sqr + d_sqr) / (2.0 * l1 * d));
    const double q1 = a1 - a2;
    const double q2 = -acosl((l1_sqr + l2_sqr - d_sqr) / (2.0 * l1 * l2));

    q[1] = q1;
    q[2] = q2 - angleOffset + cmnPI_2;

    // check joint limits for first 3 joints
    for (size_t joint = 0; joint < 3; joint++) {
        if (ClampJointValueAndUpdateError(joint, q[joint], 1e-5)) {
            hasReachedJointLimit = true;
        }
    }

    // optimized placement of platform
    // compute projection of roll axis on platform plane
    const vctFrm4x4 Rt03 = this->ForwardKinematics(q, 3);
    vctFrm4x4 Rt37;
    Rt03.ApplyInverseTo(Rt07, Rt37);
    // work in progress......
    //std::cerr << Rt37.Column(2) << std::endl;
    q[3] = q[3];

    // make sure we respect joint limits
    const double q3Max = links[3].GetKinematics()->PositionMax();
    const double q3Min = links[3].GetKinematics()->PositionMin();
    if (q[3] > q3Max) {
        q[3] = q3Max;
    } else if (q[3] < q3Min) {
        q[3] = q3Min;
    }

    // compute orientation of platform
    const vctFrm4x4 Rt04 = this->ForwardKinematics(q, 4);
    vctFrm4x4 Rt47;
    Rt04.ApplyInverseTo(Rt07, Rt47);
    vctEulerZXZRotation3 closed57(Rt47.Rotation());

    // applying DH offsets
    q[4] = closed57.alpha() + cmnPI_2;
    q[5] = -closed57.beta() + cmnPI_2;
    q[6] = closed57.gamma() + cmnPI;

    if (hasReachedJointLimit) {
        return robManipulator::EFAILURE;
    }

    return robManipulator::ESUCCESS;
}
