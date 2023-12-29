/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s): Vincent Hui
  Created on: 2018

  (C) Copyright 2018-2023, Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "robGravityCompensationMTM.h"
#include <cisstCommon/cmnDataFunctionsJSON.h>
#include <cisstCommon/cmnLogger.h>
#include <cisstCommon/cmnConstants.h>
#include <cmath>
#include <iostream>

robGravityCompensationMTM::robGravityCompensationMTM(const robGravityCompensationMTM::Parameters & parameters, int version)
    : mParameters(parameters)
    , mRegressor(parameters.JointCount(), parameters.DynamicParameterCount(), 0.0)
    , mOnes(parameters.JointCount(), 1.0)
    , mGravityEfforts(parameters.JointCount(), 0.0)
    , mTauPos(parameters.JointCount(), 0.0)
    , mTauNeg(parameters.JointCount(), 0.0)
    , mBeta(parameters.JointCount(), 0.0)
    , mOneMinusBeta(parameters.JointCount(), 0.0)
    , mAlpha(parameters.JointCount(), 0.0)
    , mOneMinusAlpha(parameters.JointCount(), 0.0)
    , mVersion(version)
{}

void robGravityCompensationMTM::AssignRegressor(const vctVec & q, vctMat & regressor)
{
    constexpr double g = 9.81;
    const double q1 = q[0];
    const double q2 = q[1];
    const double q3 = q[2];
    const double q4 = q[3];
    const double q5 = q[4];
    const double q6 = q[5];
    const double sq2 = sin(q2);
    const double cq2 = cos(q2);
    const double sq3 = sin(q3);
    const double cq3 = cos(q3);
    const double sq4 = sin(q4);
    const double cq4 = cos(q4);
    const double sq5 = sin(q5);
    const double cq5 = cos(q5);
    const double sq6 = sin(q6);
    const double cq6 = cos(q6);
    const double sq2pq3 = sin(q2 + q3);

    regressor.Element(0, 10) = 1.0;
    regressor.Element(0, 11) = q1;
    regressor.Element(0, 12) = q1 * q1;
    regressor.Element(0, 13) = q1 * q1 * q1;
    regressor.Element(0, 14) = q1 * q1 * q1 * q1;
    regressor.Element(1, 0) = g * sq2;
    regressor.Element(1, 1) = g * cq2;
    regressor.Element(1, 2) = g * cq2 * cq3 - g * sq2 * sq3;
    regressor.Element(1, 3) = -g * cq2 * sq3 - g * cq3 * sq2;
    regressor.Element(1, 4) =
        g * cq2 * cq3 * cq4 - g * cq4 * sq2 * sq3;
    regressor.Element(1, 5) =
        g * sq2 * sq3 * sq4 - g * cq2 * cq3 * sq4;
    regressor.Element(1, 6) = g * cq4 * sq2 * sq3 * sq5 -
        g * cq3 * cq5 * sq2 -
        g * cq2 * cq3 * cq4 * sq5 -
        g * cq2 * cq5 * sq3;
    regressor.Element(1, 7) = g * cq2 * cq3 * cq4 * cq5 -
        g * cq3 * sq2 * sq5 -
        g * cq2 * sq3 * sq5 -
        g * cq4 * cq5 * sq2 * sq3;
    regressor.Element(1, 8) = g * cq2 * cq3 * sq4 * sq6 +
        g * cq2 * cq6 * sq3 * sq5 +
        g * cq3 * cq6 * sq2 * sq5 -
        g * sq2 * sq3 * sq4 * sq6 +
        g * cq4 * cq5 * cq6 * sq2 * sq3 -
        g * cq2 * cq3 * cq4 * cq5 * cq6;
    regressor.Element(1, 9) = g * cq2 * cq3 * cq6 * sq4 -
        g * cq6 * sq2 * sq3 * sq4 -
        g * cq2 * sq3 * sq5 * sq6 -
        g * cq3 * sq2 * sq5 * sq6 -
        g * cq4 * cq5 * sq2 * sq3 * sq6 +
        g * cq2 * cq3 * cq4 * cq5 * sq6;
    regressor.Element(1, 15) = 1.0;
    regressor.Element(1, 16) = q2;
    regressor.Element(1, 17) = q2 * q2;
    regressor.Element(1, 18) = q2 * q2 * q2;
    regressor.Element(1, 19) = q2 * q2 * q2 * q2;
    regressor.Element(2, 2) = g * cq2 * cq3 - g * sq2 * sq3;
    regressor.Element(2, 3) = -g * cq2 * sq3 - g * cq3 * sq2;
    regressor.Element(2, 4) =
        g * cq2 * cq3 * cq4 - g * cq4 * sq2 * sq3;
    regressor.Element(2, 5) =
        g * sq2 * sq3 * sq4 - g * cq2 * cq3 * sq4;
    regressor.Element(2, 6) = g * cq4 * sq2 * sq3 * sq5 -
        g * cq3 * cq5 * sq2 -
        g * cq2 * cq3 * cq4 * sq5 -
        g * cq2 * cq5 * sq3;
    regressor.Element(2, 7) = g * cq2 * cq3 * cq4 * cq5 -
        g * cq3 * sq2 * sq5 -
        g * cq2 * sq3 * sq5 -
        g * cq4 * cq5 * sq2 * sq3;
    regressor.Element(2, 8) = g * cq2 * cq3 * sq4 * sq6 +
        g * cq2 * cq6 * sq3 * sq5 +
        g * cq3 * cq6 * sq2 * sq5 -
        g * sq2 * sq3 * sq4 * sq6 +
        g * cq4 * cq5 * cq6 * sq2 * sq3 -
        g * cq2 * cq3 * cq4 * cq5 * cq6;
    regressor.Element(2, 9) = g * cq2 * cq3 * cq6 * sq4 -
        g * cq6 * sq2 * sq3 * sq4 -
        g * cq2 * sq3 * sq5 * sq6 -
        g * cq3 * sq2 * sq5 * sq6 -
        g * cq4 * cq5 * sq2 * sq3 * sq6 +
        g * cq2 * cq3 * cq4 * cq5 * sq6;
    regressor.Element(2, 20) = 1.0;
    regressor.Element(2, 21) = q3;
    regressor.Element(2, 22) = q3 * q3;
    regressor.Element(2, 23) = q3 * q3 * q3;
    regressor.Element(2, 24) = q3 * q3 * q3 * q3;
    regressor.Element(3, 4) = -g * sq2pq3 * sq4;
    regressor.Element(3, 5) = -g * sq2pq3 * cq4;
    regressor.Element(3, 6) = g * sq2pq3 * sq4 * sq5;
    regressor.Element(3, 7) = -g * sq2pq3 * cq5 * sq4;
    regressor.Element(3, 8) =
        g * sq2pq3 * (cq4 * sq6 + cq5 * cq6 * sq4);
    regressor.Element(3, 9) =
        g * sq2pq3 * (cq4 * cq6 - cq5 * sq4 * sq6);
    regressor.Element(3, 25) = 1.0;
    regressor.Element(3, 26) = q4;
    regressor.Element(3, 27) = q4 * q4;
    regressor.Element(3, 28) = q4 * q4 * q4;
    regressor.Element(3, 29) = q4 * q4 * q4 * q4;
    regressor.Element(4, 6) =
        -g * (cq2 * cq3 * sq5 - sq2 * sq3 * sq5 +
              cq2 * cq4 * cq5 * sq3 +
              cq3 * cq4 * cq5 * sq2);
    regressor.Element(4, 7) =
        -g * (cq5 * sq2 * sq3 - cq2 * cq3 * cq5 +
              cq2 * cq4 * sq3 * sq5 +
              cq3 * cq4 * sq2 * sq5);
    regressor.Element(4, 8) = g * (cq5 * cq6 * sq2 * sq3 -
                                  cq2 * cq3 * cq5 * cq6 +
                                  cq2 * cq4 * cq6 * sq3 * sq5 +
                                  cq3 * cq4 * cq6 * sq2 * sq5);
    regressor.Element(4, 9) = -g * (cq5 * sq2 * sq3 * sq6 -
                                   cq2 * cq3 * cq5 * sq6 +
                                   cq2 * cq4 * sq3 * sq5 * sq6 +
                                   cq3 * cq4 * sq2 * sq5 * sq6);
    regressor.Element(4, 30) = 1.0;
    regressor.Element(4, 31) = q5;
    regressor.Element(4, 32) = q5 * q5;
    regressor.Element(4, 33) = q5 * q5 * q5;
    regressor.Element(4, 34) = q5 * q5 * q5 * q5;
    regressor.Element(5, 8) = g * (cq2 * cq6 * sq3 * sq4 +
                                  cq3 * cq6 * sq2 * sq4 +
                                  cq2 * cq3 * sq5 * sq6 -
                                  sq2 * sq3 * sq5 * sq6 +
                                  cq2 * cq4 * cq5 * sq3 * sq6 +
                                  cq3 * cq4 * cq5 * sq2 * sq6);
    regressor.Element(5, 9) = g * (cq2 * cq3 * cq6 * sq5 -
                                  cq2 * sq3 * sq4 * sq6 -
                                  cq3 * sq2 * sq4 * sq6 -
                                  cq6 * sq2 * sq3 * sq5 +
                                  cq2 * cq4 * cq5 * cq6 * sq3 +
                                  cq3 * cq4 * cq5 * cq6 * sq2);
    regressor.Element(5, 35) = 1.0;
    regressor.Element(5, 36) = q6;
    regressor.Element(5, 37) = q6 * q6;
    regressor.Element(5, 38) = q6 * q6 * q6;
    regressor.Element(5, 39) = q6 * q6 * q6 * q6;
}

void robGravityCompensationMTM::AddGravityCompensationEfforts(const vctVec & q,
                                                              const vctVec & q_dot,
                                                              vctVec & totalEfforts)
{
    AssignRegressor(q, mRegressor);
    if ( 1 == mVersion ) {
        ComputeBetaVel(q_dot);
        mOnes.SetAll(1.0);
        mOneMinusBeta = mOnes.Subtract(mBeta);
        mTauPos.ProductOf(mRegressor, mParameters.Pos).ElementwiseMultiply(mBeta);
        mTauNeg.ProductOf(mRegressor, mParameters.Neg).ElementwiseMultiply(mOneMinusBeta);
    } else if ( 2 == mVersion ) {
        ComputeAlphaVel(q_dot);
        mOnes.SetAll(1.0);
        mOneMinusAlpha = mOnes.Subtract(mAlpha);
        mTauPos.ProductOf(mRegressor, mParameters.Pos).ElementwiseMultiply(mAlpha);
        mTauNeg.ProductOf(mRegressor, mParameters.Neg).ElementwiseMultiply(mOneMinusAlpha);

    } else {
        mTauPos.SetAll(0.0);
        mTauNeg.SetAll(0.0);
    }

    mGravityEfforts.SetAll(0.0);
    mGravityEfforts.Add(mTauPos);
    mGravityEfforts.Add(mTauNeg);
    LimitEfforts(mGravityEfforts);
    totalEfforts.Add(mGravityEfforts);
}

void robGravityCompensationMTM::LimitEfforts(vctVec & efforts) const
{
    auto end = efforts.end();
    auto effort = efforts.begin();
    auto upper = mParameters.UpperEffortsLimit.begin();
    auto lower = mParameters.LowerEffortsLimit.begin();
    for (;
         effort != end;
         ++effort,
             ++upper,
             ++lower) {
        if (*effort > *upper) {
            *effort = *upper;
        } else if (*effort < *lower) {
            *effort = *lower;
        }
    }
}

void robGravityCompensationMTM::ComputeAlphaVel(const vctVec & q_dot)
{
    auto end = q_dot.end();
    auto qd = q_dot.begin();
    auto bd_vel = mParameters.DBVel.begin();
    auto sat_vel = mParameters.SatVel.begin();
    auto fric_comp_ratio = mParameters.FricCompRatio.begin();
    auto alpha = mAlpha.begin();

    for (;
         qd != end;
         ++qd,
         ++bd_vel,
         ++sat_vel,
         ++fric_comp_ratio,
         ++alpha) {

        if (*qd >= *sat_vel) {
            *alpha = 0.5 + 0.5*(*fric_comp_ratio);
        } else if (*qd <= -(*sat_vel)) {
            *alpha = 0.5 - 0.5*(*fric_comp_ratio);
        } else if ( (*qd <= *bd_vel) && (*qd >= -(*bd_vel)) ) {
            *alpha = 0.5;
        } else if ( (*qd > *bd_vel) && (*qd < *sat_vel) ) {
            *alpha = 0.5*(*fric_comp_ratio)*(*qd - *bd_vel)/(*sat_vel - *bd_vel) + 0.5;
        } else {
            *alpha = -0.5*(*fric_comp_ratio)*((-*qd) - *bd_vel)/(*sat_vel - *bd_vel) + 0.5;
        }
    }
}

void robGravityCompensationMTM::ComputeBetaVel(const vctVec & q_dot)
{
    auto end = q_dot.end();
    auto qd = q_dot.begin();
    auto beta = mBeta.begin();
    auto betaVelAmp = mParameters.BetaVelAmp.begin();
    for (;
         qd != end;
         ++qd,
             ++beta,
             ++betaVelAmp) {
        if (*qd > *betaVelAmp) {
            *beta = 1.0;
        }
        if (*qd < -(*betaVelAmp)) {
            *beta = 0.0;
        } else {
            *beta = 0.5 + sin(*qd * cmnPI / (2.0 * (*betaVelAmp))) / 2.0;
        }
    }
}

robGravityCompensationMTM::CreationResult
robGravityCompensationMTM::Create(const Json::Value & jsonConfig)
{
    // check version
    const int version = (int)std::stod(jsonConfig["version"].asString());

    auto getTuple = [&jsonConfig](const std::string &fieldName) {
        auto jarray = jsonConfig["GC_controller"][fieldName];
        if (jarray.empty()) {
            std::string errorMessage = std::string("the field name \"") + fieldName + std::string("\" cannot be empty.");
            return std::make_tuple(jarray, true, errorMessage);
        }
        return std::make_tuple(jarray, false, std::string(""));
    };

    auto isEmpty = [](const auto &paramTuple) {
        return std::get<1>(paramTuple);
    };

    auto createReturnValue = [](const auto &paramTuple) {
        return robGravityCompensationMTM::CreationResult({nullptr, std::get<2>(paramTuple)});
    };

#define GCMTM_GetParam(fieldname, param)                            \
    {                                                               \
        auto paramTuple = getTuple(fieldname);                      \
        if (isEmpty(paramTuple))                                    \
            return createReturnValue(paramTuple);                   \
        cmnDataJSON<vctVec>::DeSerializeText(param, std::get<0>(paramTuple)); \
    }

    robGravityCompensationMTM::Parameters params;

    if ( 1 == version) {

        GCMTM_GetParam("gc_dynamic_params_pos", params.Pos);
        GCMTM_GetParam("gc_dynamic_params_neg", params.Neg);
        GCMTM_GetParam("beta_vel_amplitude", params.BetaVelAmp);
        GCMTM_GetParam("safe_upper_torque_limit", params.UpperEffortsLimit);
        GCMTM_GetParam("safe_lower_torque_limit", params.LowerEffortsLimit);
        return {new robGravityCompensationMTM(params,version), "version 1 is still supported but you should recalibrate your MTM for version 2"};

    }
    if ( 2 == version) {

        GCMTM_GetParam("gc_dynamic_params_pos", params.Pos);
        GCMTM_GetParam("gc_dynamic_params_neg", params.Neg);
        GCMTM_GetParam("safe_upper_torque_limit", params.UpperEffortsLimit);
        GCMTM_GetParam("safe_lower_torque_limit", params.LowerEffortsLimit);
        GCMTM_GetParam("db_vel_vec", params.DBVel);
        GCMTM_GetParam("sat_vec_vec", params.SatVel);
        GCMTM_GetParam("fric_comp_ratio_vec", params.FricCompRatio);
        return {new robGravityCompensationMTM(params,version), ""};
    }

    return {nullptr, "the version of dynamic parameters is invalid"};
}

#undef GCMTM_GetParam
