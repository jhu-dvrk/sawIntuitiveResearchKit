/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Simon Leonard, Anton Deguet
  Created on: 2017-03-07

  (C) Copyright 2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/robManipulatorPSMSnake.h>
#include <cisstNumerical/nmrLSEISolver.h>

robManipulatorPSMSnake::robManipulatorPSMSnake(const std::vector<robKinematics *> linkParms,
                                               const vctFrame4x4<double> &Rtw0)
    : robManipulator(linkParms, Rtw0)
{
}

robManipulatorPSMSnake::robManipulatorPSMSnake(const std::string &robotfilename,
                                               const vctFrame4x4<double> &Rtw0)
    : robManipulator(robotfilename, Rtw0)
{
}

robManipulatorPSMSnake::robManipulatorPSMSnake(const vctFrame4x4<double> &Rtw0)
    : robManipulator(Rtw0)
{
}

void robManipulatorPSMSnake::Resize(void)
{
    if (m.E.cols() == links.size()) {
        return;
    }

    // Ex = f
    m.E.SetSize(2, links.size(), VCT_COL_MAJOR);
    m.E.SetAll(0.0);
    m.f.SetSize(2, 1, VCT_COL_MAJOR);
    m.f.SetAll(0.0);
    m.E.at(0, 4) = 1.0;     m.E.at(0, 7) = -1.0;
    m.E.at(1, 5) = 1.0;     m.E.at(1, 6) = -1.0;

    // || Ax - B ||
    m.A.SetSize(6, links.size(), VCT_COL_MAJOR);
    m.A.SetAll(0.0);
    m.b.SetSize(6, 1, VCT_COL_MAJOR);
    m.b.SetAll(0.0);
}

vctReturnDynamicVector<double>
robManipulatorPSMSnake::ConstrainedRMRC(const vctDynamicVector<double> & q,
                                        const vctFixedSizeVector<double, 6> & vw)
{
    JacobianSpatial(q, m.A);

    m.b.Column(0) = vw;

    nmrLSEISolver lsei( m.E, m.A, m.G );
    lsei.Solve( m.E, m.f, m.A, m.b, m.G, m.h );

    // solution is first column of LSEI X
    return vctReturnDynamicVector<double>(lsei.GetX().Column(0));
}

robManipulator::Errno
robManipulatorPSMSnake::InverseKinematics(vctDynamicVector<double> & q,
                                          const vctFrame4x4<double> & Rts,
                                          double tolerance, size_t Niterations)
{
    if (q.size() != links.size()) {
        CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
                          << ": robManipulatorPSMSnake::InverseKinematics: expected " << links.size() << " joints values. "
                          << " Got " << q.size()
                          << std::endl;
        return robManipulator::EFAILURE;
    }

    if (links.size() == 0) {
        CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
                          << ": robManipulatorPSMSnake::InverseKinematics: the manipulator has no links."
                          << std::endl;
        return robManipulator::EFAILURE;
    }

    Resize();

    double ndq = 1.0;               // norm of the iteration error
    size_t i = 0;
    // loop until Niter are executed or the error is bellow the tolerance
    for (i = 0; i < Niterations && tolerance < ndq; i++) {

        // Evaluate the forward kinematics
        vctFrame4x4<double,VCT_ROW_MAJOR> Rt = ForwardKinematics( q );

        // compute the translation error
        vctFixedSizeVector<double,3> dt( Rts[0][3] - Rt[0][3],
                                         Rts[1][3] - Rt[1][3],
                                         Rts[2][3] - Rt[2][3] );

        // compute the orientation error
        // This is the orientation error
        vctFixedSizeVector<double,3> dr = 0.5 * ( (Rt.Rotation().Column(0) % Rts.Rotation().Column(0)) +
                                                  (Rt.Rotation().Column(1) % Rts.Rotation().Column(1)) +
                                                  (Rt.Rotation().Column(2) % Rts.Rotation().Column(2)) );

        // combine both errors in one R^6 vector
        vctFixedSizeVector<double,6> e( dt[0], dt[1], dt[2], dr[0], dr[1], dr[2] );

        vctDynamicVector<double> dq = ConstrainedRMRC( q, e );

        // compute the L2 norm of the error
        ndq = dq.Norm();

        // update the solution
        q.Add(dq);
    }

    NormalizeAngles(q);

    if (i == Niterations) {
      return robManipulator::EFAILURE;
    }
    return robManipulator::ESUCCESS;
}
