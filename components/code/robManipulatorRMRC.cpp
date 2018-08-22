/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Simon Leonard, Anton Deguet, Preetham Chalasani
  Created on: 2017-03-07

  (C) Copyright 2017-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/robManipulatorRMRC.h>

robManipulatorRMRC::robManipulatorRMRC(const std::vector<robKinematics *> linkParms,
                                       const vctFrame4x4<double> & Rtw0)
    : robManipulator(linkParms, Rtw0)
{
}

robManipulatorRMRC::robManipulatorRMRC(const std::string & robotfilename,
                                       const vctFrame4x4<double> & Rtw0)
    : robManipulator(robotfilename, Rtw0)
{
}

robManipulatorRMRC::robManipulatorRMRC(const vctFrame4x4<double> & Rtw0)
    : robManipulator(Rtw0)
{
}

void robManipulatorRMRC::Resize(void)
{
    // use A to figure out if the LSEI memory is already allocated
    if (m.A.cols() != links.size()) {
        // Ex = f
        m.E.SetSize(0, 0, VCT_COL_MAJOR);
        m.f.SetSize(0, 0, VCT_COL_MAJOR);

        // || Ax - B ||
        m.A.SetSize(6, links.size(), VCT_COL_MAJOR);
        m.A.SetAll(0.0);
        m.b.SetSize(6, 1, VCT_COL_MAJOR);
        m.b.SetAll(0.0);

        // Gx >= h, the matrix is 2 nb links * nb links to have all
        // the min and max joint limits
        m.G.SetSize(2 * links.size(), links.size(), VCT_COL_MAJOR);
        m.G.SetAll(0.0);
        m.h.SetSize(2 * links.size(), 1, VCT_COL_MAJOR );
        m.h.SetAll(0.0);

        // activated inequality constraints, the "top" diagonal is all
        // +1 for negative limits, i.e. dq will have to be positive to
        // respect constaint, "bottom" diagonal is all -1 for positive
        // limits.
        for (size_t i = 0; i < links.size(); i++) {
            m.G.Element(i, i) = 1.0;
            m.G.Element(links.size() + i, i) = -1.0;
        }
    }
    // allocate should be called every time before calling Solve
    m.lsei.Allocate(m.E, m.A, m.G);
}

vctReturnDynamicVector<double>
robManipulatorRMRC::ConstrainedRMRC(const vctDynamicVector<double> & q,
                                    const vctFixedSizeVector<double, 6> & vw)
{
    // resize and reset, this needs to be called before modifying m
    // members and before calling m.lsei
    Resize();

    JacobianSpatial(q, m.A);
    m.b.Column(0) = vw;

    // h has to be recalculated to represent which new inequality
    // constraints on dq based on q and q_limits
    for (size_t i = 0; i < links.size(); i++) {
        m.h.Element(i, 0) = links[i].GetKinematics()->PositionMin() - q[i];
        m.h.Element(links.size() + i, 0) = q[i] - links[i].GetKinematics()->PositionMax();
    }

    // solve for dq
    m.lsei.Solve(m.E, m.f, m.A, m.b, m.G, m.h);

    return vctReturnDynamicVector<double>(m.lsei.GetX().Column(0));
}

robManipulator::Errno
robManipulatorRMRC::InverseKinematics(vctDynamicVector<double> & q,
                                      const vctFrame4x4<double> & Rts,
                                      double tolerance, size_t Niterations, double)
{
    if (q.size() != links.size()) {
        CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
                          << ": robManipulatorRMRC::InverseKinematics: expected " << links.size() << " joints values. "
                          << " Got " << q.size()
                          << std::endl;
        return robManipulator::EFAILURE;
    }

    if (links.size() == 0) {
        CMN_LOG_RUN_ERROR << CMN_LOG_DETAILS
                          << ": robManipulatorRMRC::InverseKinematics: the manipulator has no links."
                          << std::endl;
        return robManipulator::EFAILURE;
    }

    double ndq = 1.0;               // norm of the iteration error
    size_t i = 0;
    // loop until Niter are executed or the error is bellow the tolerance
    for (i = 0; i < Niterations && tolerance < ndq; i++) {

        // Evaluate the forward kinematics
        vctFrame4x4<double,VCT_ROW_MAJOR> Rt = ForwardKinematics(q);

        // compute the translation error
        vctFixedSizeVector<double,3> dt(Rts[0][3] - Rt[0][3],
                                        Rts[1][3] - Rt[1][3],
                                        Rts[2][3] - Rt[2][3]);

        // compute the orientation error
        vctFixedSizeVector<double,3> dr = 0.5 * ((Rt.Rotation().Column(0) % Rts.Rotation().Column(0)) +
                                                 (Rt.Rotation().Column(1) % Rts.Rotation().Column(1)) +
                                                 (Rt.Rotation().Column(2) % Rts.Rotation().Column(2)));

        // combine both errors in one R^6 vector
        vctFixedSizeVector<double,6> e(dt[0], dt[1], dt[2], dr[0], dr[1], dr[2]);

        // get dq from optimizer with constaints
        vctDynamicVector<double> dq = ConstrainedRMRC(q, e);

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
