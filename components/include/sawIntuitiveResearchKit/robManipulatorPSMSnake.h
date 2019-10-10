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

#ifndef _robManipulatorPSMSnake_h
#define _robManipulatorPSMSnake_h

#include <cisstRobot/robManipulator.h>
#include <cisstNumerical/nmrLSEISolver.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class robManipulatorPSMSnake: public robManipulator
{

public:
    robManipulatorPSMSnake(const vctFrame4x4<double>& Rtw0 = vctFrame4x4<double>());

    robManipulatorPSMSnake(const std::string& robotfilename,
                           const vctFrame4x4<double>& Rtw0 = vctFrame4x4<double>());

    robManipulatorPSMSnake(const std::vector<robKinematics *> linkParms,
                           const vctFrame4x4<double>& Rtw0 = vctFrame4x4<double>());

    ~robManipulatorPSMSnake() {}

    vctReturnDynamicVector<double>
    ConstrainedRMRC(const vctDynamicVector<double> & q,
                    const vctFixedSizeVector<double, 6> & vw);

    robManipulator::Errno
    InverseKinematics(vctDynamicVector<double> & q,
                      const vctFrame4x4<double> & Rts,
                      double tolerance = 1e-12,
                      size_t Niterations = 1000,
                      double LAMBDA = 0.001);

private:
    void Resize(void);

    struct {
        // Ex = f
        vctDynamicMatrix<double> E;
        vctDynamicMatrix<double> f;

        // || Ax - B ||
        vctDynamicMatrix<double> A;
        vctDynamicMatrix<double> b;

        // storage for LSEI
        vctDynamicMatrix<double> G;
        vctDynamicMatrix<double> h;

        // solver
        nmrLSEISolver lsei;
    } m;
};

#endif // _robManipulatorPSMSnake_h
