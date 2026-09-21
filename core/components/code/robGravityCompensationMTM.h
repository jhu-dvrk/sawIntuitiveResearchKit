/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):
  Created on: 2018

  (C) Copyright 2018-2025, Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _robGravityCompensationMTM_h
#define _robGravityCompensationMTM_h

#include <Eigen/Dense>
#include <json/json.h>

#include <sawIntuitiveResearchKit/robGravityCompensation.h>

// always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>
class CISST_EXPORT robGravityCompensationMTM : public robGravityCompensation
{
public:
    struct CreationResult {
        inline CreationResult(robGravityCompensationMTM * ptr, const std::string & msg):
            Pointer(ptr),
            ErrorMessage(msg) {}

        robGravityCompensationMTM * Pointer = nullptr;
        std::string ErrorMessage = "";
    };

    struct Parameters {
        Eigen::VectorXd Pos;
        Eigen::VectorXd Neg;
        Eigen::VectorXd BetaVelAmp;
        Eigen::VectorXd UpperEffortsLimit;
        Eigen::VectorXd LowerEffortsLimit;
        Eigen::VectorXd DBVel;
        Eigen::VectorXd SatVel;
        Eigen::VectorXd FricCompRatio;

        size_t JointCount(void) const {
            return UpperEffortsLimit.size();
        }
        size_t DynamicParameterCount(void) const {
            return Pos.size();
        }
    };

    static CreationResult Create(const Json::Value & jsonConfig);
    robGravityCompensationMTM(const Parameters & parameters,int version);
    Eigen::VectorXd compute(const prmStateJoint& state, Eigen::Vector3d& CMN_UNUSED(gravity)) override;

private:
    static void AssignRegressor(const Eigen::VectorXd& q, Eigen::MatrixXd& regressor);
    void LimitEfforts(Eigen::VectorXd& efforts) const;
    void ComputeAlphaVel(const Eigen::VectorXd& q_dot);
    void ComputeBetaVel(const Eigen::VectorXd& q_dot);

    const Parameters mParameters;
    Eigen::MatrixXd mRegressor;
    Eigen::VectorXd mOnes;
    Eigen::VectorXd mGravityEfforts;
    Eigen::VectorXd mTauPos;
    Eigen::VectorXd mTauNeg;
    Eigen::VectorXd mBeta;
    Eigen::VectorXd mOneMinusBeta;
    Eigen::VectorXd mAlpha;
    Eigen::VectorXd mOneMinusAlpha;
    const int mVersion = 0;
};

#endif // _robGravityCompensationMTM_h
