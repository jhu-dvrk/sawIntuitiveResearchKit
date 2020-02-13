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

#ifndef _robManipulatorMTM_h
#define _robManipulatorMTM_h

#include <cisstRobot/robManipulator.h>
#include <cisstNumerical/nmrLSEISolver.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class robManipulatorMTM: public robManipulator
{

public:
    robManipulatorMTM(const vctFrame4x4<double>& Rtw0 = vctFrame4x4<double>());

    robManipulatorMTM(const std::string& robotfilename,
                      const vctFrame4x4<double>& Rtw0 = vctFrame4x4<double>());

    robManipulatorMTM(const std::vector<robKinematics *> linkParms,
                      const vctFrame4x4<double>& Rtw0 = vctFrame4x4<double>());

    ~robManipulatorMTM() {}

    robManipulator::Errno
    InverseKinematics(vctDynamicVector<double> & q,
                      const vctFrame4x4<double> & Rts,
                      double tolerance = 1e-12,
                      size_t Niterations = 1000,
                      double LAMBDA = 0.001);

    double FindOptimalPlatformAngle(const vctDynamicVector<double> & q,
                                    const vctFrame4x4<double> & Rt07) const;

    double ComputeGimbalIK(vctDynamicVector<double> & q, const vctFrame4x4<double> & Rt07) const;
};

#endif // _robManipulatorMTM_h
