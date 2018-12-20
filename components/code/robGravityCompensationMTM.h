/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):
  Created on: 2018

  (C) Copyright 2018, Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _robGravityCompensationMTM_h
#define _robGravityCompensationMTM_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctFixedSizeMatrix.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <json/json.h>

using vct40 = vctFixedSizeVector<double, 40>;
using vct7x40 = vctFixedSizeMatrix<double, 7, 40>;

// always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>
class CISST_EXPORT robGravityCompensationMTM
{
public:
    struct CreationResult {
      robGravityCompensationMTM * Pointer = nullptr;
      std::string ErrorMessage = "";
    };

    struct Parameters {
        Parameters()
            : Pos(0.0), Neg(0.0), BetaVelAmp(0.0), UpperEffortsLimit(0.0),
              LowerEffortsLimit(0.0) {}
      vct40 Pos;
      vct40 Neg;
      vct7 BetaVelAmp;
      vct7 UpperEffortsLimit;
      vct7 LowerEffortsLimit;
    };

    static CreationResult Create(const Json::Value & jsonConfig);
    robGravityCompensationMTM(const Parameters & parameters);
    void AddGravityCompensationEfforts(const vctVec & q, const vctVec & q_dot,
                                       vctVec & totalEfforts);

private:
    static void AssignRegressor(const vctVec & q, vct7x40 & regressor);
    void LimitEfforts(vct7 & efforts) const;
    vct7 ComputeBetaVel(const vctVec & q_dot) const;
    Parameters mParameters;
    vct7x40 mRegressor;
    const vct7 mOnes;
    vct7 mGravityEfforts;
};

#endif // _robGravityCompensationMTM_h
