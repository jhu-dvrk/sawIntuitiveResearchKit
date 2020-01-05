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

template <typename _rotationMatrix>
vct3 SO3toRPY(const _rotationMatrix & R)
{
    vct3 rpy;
    if (fabs(R[2][2]) < 1e-12 && fabs(R[1][2]) < 1e-12) {
        rpy[0] = 0.0;
        rpy[1] = atan2(R[0][2], R[2][2]);
        rpy[2] = atan2(R[1][0], R[1][1]);
    }
    else {
        rpy[0] = atan2(-R[1][2], R[2][2]);
        double sr = sin(rpy[0]);
        double cr = cos(rpy[0]);
        rpy[1] = atan2(R[0][2], cr*R[2][2] - sr*R[1][2]);
        rpy[2] = atan2(-R[0][1], R[0][0] );
    }
    return rpy;
}


robManipulator::Errno
robManipulatorMTM::InverseKinematics(vctDynamicVector<double> & q,
                                     const vctFrame4x4<double> & Rts,
                                     double CMN_UNUSED(tolerance),
                                     size_t CMN_UNUSED(Niterations),
                                     double CMN_UNUSED(LAMBDA))
{
    if (q.size() != links.size()) {
        CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
                          << ": robManipulatorMTM::InverseKinematics: expected " << links.size() << " joints values. "
                          << " Got " << q.size()
                          << std::endl;
        return robManipulator::EFAILURE;
    }

    if (links.size() == 0) {
        CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
                          << ": robManipulatorMTM::InverseKinematics: the manipulator has no links."
                          << std::endl;
        return robManipulator::EFAILURE;
    }

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

    // this needs to be replaced by optimized placement of platform
    q[3] = q[3];

    // compute orientation of platform
    const vctFrm4x4 fwd04 = this->ForwardKinematics(q, 4);
    vctFrm4x4 Rt57;
    fwd04.ApplyInverseTo(Rt07, Rt57);
    vct3 closed57 = SO3toRPY(Rt57.Rotation());

    // applying DH offsets
    q[4] = closed57.Element(1) + cmnPI_2;
    q[5] = -closed57.Element(0) + cmnPI_2;
    q[6] = closed57.Element(2) + cmnPI;

    return robManipulator::ESUCCESS;
}
