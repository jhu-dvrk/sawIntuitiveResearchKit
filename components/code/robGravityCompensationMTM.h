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

#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstVector/vctDynamicVectorTypes.h>
#include <json/json.h>

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

      vctVec Pos;
      vctVec Neg;
      vctVec BetaVelAmp;
      vctVec UpperEffortsLimit;
      vctVec LowerEffortsLimit;
      vctVec UpperPositionsLimit;
      vctVec LowerPositionsLimit;

      size_t JointCount() const {
          return BetaVelAmp.size();
      }
      size_t DynamicParameterCount() const {
          return Pos.size();
      }
    };

    static CreationResult Create(const Json::Value & jsonConfig);
    robGravityCompensationMTM(const Parameters & parameters);
    void AddGravityCompensationEfforts(const vctVec & q, const vctVec & q_dot,
                                       vctVec & totalEfforts);

private:
    static void AssignRegressor(const vctVec & q, vctMat & regressor);
    void LimitEfforts(vctVec & efforts) const;
    void ComputeBetaVel(const vctVec & q_dot);
    Parameters mParameters;
    vctMat mRegressor;
    vctVec mOnes;
    vctVec mGravityEfforts;
    vctVec mTauPos;
    vctVec mTauNeg;
    vctVec mBeta;
    vctVec mOneMinusBeta;
    vctVec mClippedQ;
};

#endif // _robGravityCompensationMTM_h
