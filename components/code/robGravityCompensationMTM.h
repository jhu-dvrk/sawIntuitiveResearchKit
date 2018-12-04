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

class robGravityCompensationMTM;

class CISST_EXPORT robGravityCompensationMTM
{
public:
    struct CreationResult {
      robGravityCompensationMTM *gc = nullptr;
      std::string errorMsg = "";
    };

    struct Params {
      Params()
          : pos(0.0), neg(0.0), betaVelAmp(0.0), upperEffortsLimit(0.0),
            lowerEffortsLimit(0.0) {}
      vct40 pos;
      vct40 neg;
      vct7 betaVelAmp;
      vct7 upperEffortsLimit;
      vct7 lowerEffortsLimit;
    };

    static CreationResult Create(const Json::Value &jsonConfig);
    robGravityCompensationMTM(const Params &params);
    void AddGCeffortsTo(const vctVec &q, const vctVec &q_dot,
                        vctVec &totalEfforts);

private:
    static void AssignRegressor(const vctVec &q, vct7x40 &regressor);
    void LimitEfforts(vct7 &efforts);
    vct7 computeBetaVel(const vctVec &q_dot);
    Params p_;
    vct7x40 regressor_;
    const vct7 ones_;
    vct7 gravityEfforts_;
};

#endif // _robGravityCompensationMTM_h
