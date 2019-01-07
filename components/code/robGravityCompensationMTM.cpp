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

#include "robGravityCompensationMTM.h"
#include <cisstCommon/cmnDataFunctionsJSON.h>
#include <cisstCommon/cmnLogger.h>
#include <cmath>
#include <iostream>

namespace {
    inline void Add(const vct7 & gravityEfforts, vctVec & totalEfforts)
    {
        auto geItr = gravityEfforts.begin();
        for (auto &te : totalEfforts) {
            te += *geItr;
            geItr++;
        }
    }
}

robGravityCompensationMTM::robGravityCompensationMTM(const robGravityCompensationMTM::Parameters & parameters)
    : mParameters(parameters), mRegressor(0.0), mOnes(1.0), mGravityEfforts(0.0) {}

void robGravityCompensationMTM::AssignRegressor(const vctVec & q,
                                                vct7x40 & regressor)
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

    regressor[0][10] = 1.0;
    regressor[0][11] = q1;
    regressor[0][12] = std::pow(q1, 2);
    regressor[0][13] = std::pow(q1, 3);
    regressor[0][14] = std::pow(q1, 4);
    regressor[1][0] = g * sq2;
    regressor[1][1] = g * cq2;
    regressor[1][2] = g * cq2 * cq3 - g * sq2 *sq3;
    regressor[1][3] = -g * cq2 *sq3 - g * cq3 * sq2;
    regressor[1][4] =
        g * cq2 * cq3 * cq4 - g * cq4 * sq2 *sq3;
    regressor[1][5] =
        g * sq2 *sq3 *sq4 - g * cq2 * cq3 *sq4;
    regressor[1][6] = g * cq4 * sq2 *sq3 *sq5 -
        g * cq3 * cq5 * sq2 -
        g * cq2 * cq3 * cq4 *sq5 -
        g * cq2 * cq5 *sq3;
    regressor[1][7] = g * cq2 * cq3 * cq4 * cq5 -
        g * cq3 * sq2 *sq5 -
        g * cq2 *sq3 *sq5 -
        g * cq4 * cq5 * sq2 *sq3;
    regressor[1][8] = g * cq2 * cq3 *sq4 *sq6 +
        g * cq2 * cq6 *sq3 *sq5 +
        g * cq3 * cq6 * sq2 *sq5 -
        g * sq2 *sq3 *sq4 *sq6 +
        g * cq4 * cq5 * cq6 * sq2 *sq3 -
        g * cq2 * cq3 * cq4 * cq5 * cq6;
    regressor[1][9] = g * cq2 * cq3 * cq6 *sq4 -
        g * cq6 * sq2 *sq3 *sq4 -
        g * cq2 *sq3 *sq5 *sq6 -
        g * cq3 * sq2 *sq5 *sq6 -
        g * cq4 * cq5 * sq2 *sq3 *sq6 +
        g * cq2 * cq3 * cq4 * cq5 *sq6;
    regressor[1][15] = 1.0;
    regressor[1][16] = q2;
    regressor[1][17] = std::pow(q2, 2);
    regressor[1][18] = std::pow(q2, 3);
    regressor[1][19] = std::pow(q2, 4);
    regressor[2][2] = g * cq2 * cq3 - g * sq2 *sq3;
    regressor[2][3] = -g * cq2 *sq3 - g * cq3 * sq2;
    regressor[2][4] =
        g * cq2 * cq3 * cq4 - g * cq4 * sq2 *sq3;
    regressor[2][5] =
        g * sq2 *sq3 *sq4 - g * cq2 * cq3 *sq4;
    regressor[2][6] = g * cq4 * sq2 *sq3 *sq5 -
        g * cq3 * cq5 * sq2 -
        g * cq2 * cq3 * cq4 *sq5 -
        g * cq2 * cq5 *sq3;
    regressor[2][7] = g * cq2 * cq3 * cq4 * cq5 -
        g * cq3 * sq2 *sq5 -
        g * cq2 *sq3 *sq5 -
        g * cq4 * cq5 * sq2 *sq3;
    regressor[2][8] = g * cq2 * cq3 *sq4 *sq6 +
        g * cq2 * cq6 *sq3 *sq5 +
        g * cq3 * cq6 * sq2 *sq5 -
        g * sq2 *sq3 *sq4 *sq6 +
        g * cq4 * cq5 * cq6 * sq2 *sq3 -
        g * cq2 * cq3 * cq4 * cq5 * cq6;
    regressor[2][9] = g * cq2 * cq3 * cq6 *sq4 -
        g * cq6 * sq2 *sq3 *sq4 -
        g * cq2 *sq3 *sq5 *sq6 -
        g * cq3 * sq2 *sq5 *sq6 -
        g * cq4 * cq5 * sq2 *sq3 *sq6 +
        g * cq2 * cq3 * cq4 * cq5 *sq6;
    regressor[2][20] = 1.0;
    regressor[2][21] = q3;
    regressor[2][22] = std::pow(q3, 2);
    regressor[2][23] = std::pow(q3, 3);
    regressor[2][24] = std::pow(q3, 4);
    regressor[3][4] = -g * sq2pq3 *sq4;
    regressor[3][5] = -g * sq2pq3 * cq4;
    regressor[3][6] = g * sq2pq3 *sq4 *sq5;
    regressor[3][7] = -g * sq2pq3 * cq5 *sq4;
    regressor[3][8] =
        g * sq2pq3 * (cq4 *sq6 + cq5 * cq6 *sq4);
    regressor[3][9] =
        g * sq2pq3 * (cq4 * cq6 - cq5 *sq4 *sq6);
    regressor[3][25] = 1.0;
    regressor[3][26] = q4;
    regressor[3][27] = std::pow(q4, 2);
    regressor[3][28] = std::pow(q4, 3);
    regressor[3][29] = std::pow(q4, 4);
    regressor[4][6] =
        -g * (cq2 * cq3 *sq5 - sq2 *sq3 *sq5 +
              cq2 * cq4 * cq5 *sq3 +
              cq3 * cq4 * cq5 * sq2);
    regressor[4][7] =
        -g * (cq5 * sq2 *sq3 - cq2 * cq3 * cq5 +
              cq2 * cq4 *sq3 *sq5 +
              cq3 * cq4 * sq2 *sq5);
    regressor[4][8] = g * (cq5 * cq6 * sq2 *sq3 -
                           cq2 * cq3 * cq5 * cq6 +
                           cq2 * cq4 * cq6 *sq3 *sq5 +
                           cq3 * cq4 * cq6 * sq2 *sq5);
    regressor[4][9] = -g * (cq5 * sq2 *sq3 *sq6 -
                            cq2 * cq3 * cq5 *sq6 +
                            cq2 * cq4 *sq3 *sq5 *sq6 +
                            cq3 * cq4 * sq2 *sq5 *sq6);
    regressor[4][30] = 1.0;
    regressor[4][31] = q5;
    regressor[4][32] = std::pow(q5, 2);
    regressor[4][33] = std::pow(q5, 3);
    regressor[4][34] = std::pow(q5, 4);
    regressor[5][8] = g * (cq2 * cq6 *sq3 *sq4 +
                           cq3 * cq6 * sq2 *sq4 +
                           cq2 * cq3 *sq5 *sq6 -
                           sq2 *sq3 *sq5 *sq6 +
                           cq2 * cq4 * cq5 *sq3 *sq6 +
                           cq3 * cq4 * cq5 * sq2 *sq6);
    regressor[5][9] = g * (cq2 * cq3 * cq6 *sq5 -
                           cq2 *sq3 *sq4 *sq6 -
                           cq3 * sq2 *sq4 *sq6 -
                           cq6 * sq2 *sq3 *sq5 +
                           cq2 * cq4 * cq5 * cq6 *sq3 +
                           cq3 * cq4 * cq5 * cq6 * sq2);
    regressor[5][35] = 1.0;
    regressor[5][36] = q6;
    regressor[5][37] = std::pow(q6, 2);
    regressor[5][38] = std::pow(q6, 3);
    regressor[5][39] = std::pow(q6, 4);
}

void robGravityCompensationMTM::AddGravityCompensationEfforts(const vctVec & q,
                                                              const vctVec & q_dot,
                                                              vctVec & totalEfforts)
{
    AssignRegressor(q, mRegressor);
    const vct7 beta = ComputeBetaVel(q_dot);
    vct7 tau_pos = mRegressor * mParameters.Pos;
    vct7 tau_neg = mRegressor * mParameters.Neg;

    mGravityEfforts = tau_pos.ElementwiseMultiply(beta) + tau_neg.ElementwiseMultiply(mOnes - beta);
    LimitEfforts(mGravityEfforts);
    Add(mGravityEfforts, totalEfforts);
}

void robGravityCompensationMTM::LimitEfforts(vct7 & efforts) const
{
    for (size_t i = 0; i < efforts.size(); i++) {
        if (efforts[i] > mParameters.UpperEffortsLimit[i]) {
            efforts[i] = mParameters.UpperEffortsLimit[i];
        } else if (efforts[i] < mParameters.LowerEffortsLimit[i]) {
            efforts[i] = mParameters.LowerEffortsLimit[i];
        }
    }
}

vct7 robGravityCompensationMTM::ComputeBetaVel(const vctVec & q_dot) const
{
    vct7 beta;
    for (size_t i = 0; i < q_dot.size(); i++) {
        if (q_dot[i] > mParameters.BetaVelAmp[i]) {
            beta[i] = 1.0;
        }
        if (q_dot[i] < -mParameters.BetaVelAmp[i]) {
            beta[i] = 0.0;
        } else {
            beta[i] = 0.5 + sin(q_dot[i] * M_PI / (2.0 * mParameters.BetaVelAmp[i])) / 2.0;
        }
    }
    return beta;
}

robGravityCompensationMTM::CreationResult
robGravityCompensationMTM::Create(const Json::Value & jsonConfig)
{
    // check version
    if (jsonConfig["version"].asString() != "1.0") {
        return {nullptr, "the version of dynamic parameters is not 1.0"};
    }

    // load
    robGravityCompensationMTM::Parameters params;
    auto jpos = jsonConfig["GC_controller"]["gc_dynamic_params_pos"];
    auto jneg = jsonConfig["GC_controller"]["gc_dynamic_params_neg"];
    auto jbeta = jsonConfig["GC_controller"]["beta_vel_amplitude"];
    auto jupper = jsonConfig["GC_controller"]["safe_upper_torque_limit"];
    auto jlower = jsonConfig["GC_controller"]["safe_lower_torque_limit"];

    // check sizes
    if (!(params.Pos.size() == jpos.size() &&
          params.Neg.size() == jneg.size() &&
          params.BetaVelAmp.size() == jbeta.size() &&
          params.LowerEffortsLimit.size() == jlower.size() &&
          params.UpperEffortsLimit.size() == jupper.size())) {
        return {nullptr,
                "the arrays size in the JSON object are not matching the "
                "expected size"};        
    }

    cmnDataJSON<vct40>::DeSerializeText(params.Pos, jpos);
    cmnDataJSON<vct40>::DeSerializeText(params.Neg, jneg);
    cmnDataJSON<vct7>::DeSerializeText(params.BetaVelAmp, jbeta);
    cmnDataJSON<vct7>::DeSerializeText(params.UpperEffortsLimit, jupper);
    cmnDataJSON<vct7>::DeSerializeText(params.LowerEffortsLimit, jlower);
    
    return {new robGravityCompensationMTM(params), ""};
}
