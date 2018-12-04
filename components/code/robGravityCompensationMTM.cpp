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
inline void Add(const vct7 &gravityEfforts, vctVec &totalEfforts)
{
    auto geItr = gravityEfforts.begin();
    for (auto &te : totalEfforts) {
        te += *geItr;
        geItr++;
    }
}
}

robGravityCompensationMTM::robGravityCompensationMTM(
    const robGravityCompensationMTM::Params &params)
    : p_(params), regressor_(0.0), ones_(1.0), gravityEfforts_(0.0) {}

void robGravityCompensationMTM::AssignRegressor(const vctVec &q,
                                                vct7x40 &regressor)
{
    constexpr double g = 9.81;
    const double q1 = q[0];
    const double q2 = q[1];
    const double q3 = q[2];
    const double q4 = q[3];
    const double q5 = q[4];
    const double q6 = q[5];

    regressor[0][10] = 1.0;
    regressor[0][11] = q1;
    regressor[0][12] = std::pow(q1, 2);
    regressor[0][13] = std::pow(q1, 3);
    regressor[0][14] = std::pow(q1, 4);
    regressor[1][0] = g * sin(q2);
    regressor[1][1] = g * cos(q2);
    regressor[1][2] = g * cos(q2) * cos(q3) - g * sin(q2) * sin(q3);
    regressor[1][3] = -g * cos(q2) * sin(q3) - g * cos(q3) * sin(q2);
    regressor[1][4] =
        g * cos(q2) * cos(q3) * cos(q4) - g * cos(q4) * sin(q2) * sin(q3);
    regressor[1][5] =
        g * sin(q2) * sin(q3) * sin(q4) - g * cos(q2) * cos(q3) * sin(q4);
    regressor[1][6] = g * cos(q4) * sin(q2) * sin(q3) * sin(q5) -
                      g * cos(q3) * cos(q5) * sin(q2) -
                      g * cos(q2) * cos(q3) * cos(q4) * sin(q5) -
                      g * cos(q2) * cos(q5) * sin(q3);
    regressor[1][7] = g * cos(q2) * cos(q3) * cos(q4) * cos(q5) -
                      g * cos(q3) * sin(q2) * sin(q5) -
                      g * cos(q2) * sin(q3) * sin(q5) -
                      g * cos(q4) * cos(q5) * sin(q2) * sin(q3);
    regressor[1][8] = g * cos(q2) * cos(q3) * sin(q4) * sin(q6) +
                      g * cos(q2) * cos(q6) * sin(q3) * sin(q5) +
                      g * cos(q3) * cos(q6) * sin(q2) * sin(q5) -
                      g * sin(q2) * sin(q3) * sin(q4) * sin(q6) +
                      g * cos(q4) * cos(q5) * cos(q6) * sin(q2) * sin(q3) -
                      g * cos(q2) * cos(q3) * cos(q4) * cos(q5) * cos(q6);
    regressor[1][9] = g * cos(q2) * cos(q3) * cos(q6) * sin(q4) -
                      g * cos(q6) * sin(q2) * sin(q3) * sin(q4) -
                      g * cos(q2) * sin(q3) * sin(q5) * sin(q6) -
                      g * cos(q3) * sin(q2) * sin(q5) * sin(q6) -
                      g * cos(q4) * cos(q5) * sin(q2) * sin(q3) * sin(q6) +
                      g * cos(q2) * cos(q3) * cos(q4) * cos(q5) * sin(q6);
    regressor[1][15] = 1.0;
    regressor[1][16] = q2;
    regressor[1][17] = std::pow(q2, 2);
    regressor[1][18] = std::pow(q2, 3);
    regressor[1][19] = std::pow(q2, 4);
    regressor[2][2] = g * cos(q2) * cos(q3) - g * sin(q2) * sin(q3);
    regressor[2][3] = -g * cos(q2) * sin(q3) - g * cos(q3) * sin(q2);
    regressor[2][4] =
        g * cos(q2) * cos(q3) * cos(q4) - g * cos(q4) * sin(q2) * sin(q3);
    regressor[2][5] =
        g * sin(q2) * sin(q3) * sin(q4) - g * cos(q2) * cos(q3) * sin(q4);
    regressor[2][6] = g * cos(q4) * sin(q2) * sin(q3) * sin(q5) -
                      g * cos(q3) * cos(q5) * sin(q2) -
                      g * cos(q2) * cos(q3) * cos(q4) * sin(q5) -
                      g * cos(q2) * cos(q5) * sin(q3);
    regressor[2][7] = g * cos(q2) * cos(q3) * cos(q4) * cos(q5) -
                      g * cos(q3) * sin(q2) * sin(q5) -
                      g * cos(q2) * sin(q3) * sin(q5) -
                      g * cos(q4) * cos(q5) * sin(q2) * sin(q3);
    regressor[2][8] = g * cos(q2) * cos(q3) * sin(q4) * sin(q6) +
                      g * cos(q2) * cos(q6) * sin(q3) * sin(q5) +
                      g * cos(q3) * cos(q6) * sin(q2) * sin(q5) -
                      g * sin(q2) * sin(q3) * sin(q4) * sin(q6) +
                      g * cos(q4) * cos(q5) * cos(q6) * sin(q2) * sin(q3) -
                      g * cos(q2) * cos(q3) * cos(q4) * cos(q5) * cos(q6);
    regressor[2][9] = g * cos(q2) * cos(q3) * cos(q6) * sin(q4) -
                      g * cos(q6) * sin(q2) * sin(q3) * sin(q4) -
                      g * cos(q2) * sin(q3) * sin(q5) * sin(q6) -
                      g * cos(q3) * sin(q2) * sin(q5) * sin(q6) -
                      g * cos(q4) * cos(q5) * sin(q2) * sin(q3) * sin(q6) +
                      g * cos(q2) * cos(q3) * cos(q4) * cos(q5) * sin(q6);
    regressor[2][20] = 1.0;
    regressor[2][21] = q3;
    regressor[2][22] = std::pow(q3, 2);
    regressor[2][23] = std::pow(q3, 3);
    regressor[2][24] = std::pow(q3, 4);
    regressor[3][4] = -g * sin(q2 + q3) * sin(q4);
    regressor[3][5] = -g * sin(q2 + q3) * cos(q4);
    regressor[3][6] = g * sin(q2 + q3) * sin(q4) * sin(q5);
    regressor[3][7] = -g * sin(q2 + q3) * cos(q5) * sin(q4);
    regressor[3][8] =
        g * sin(q2 + q3) * (cos(q4) * sin(q6) + cos(q5) * cos(q6) * sin(q4));
    regressor[3][9] =
        g * sin(q2 + q3) * (cos(q4) * cos(q6) - cos(q5) * sin(q4) * sin(q6));
    regressor[3][25] = 1.0;
    regressor[3][26] = q4;
    regressor[3][27] = std::pow(q4, 2);
    regressor[3][28] = std::pow(q4, 3);
    regressor[3][29] = std::pow(q4, 4);
    regressor[4][6] =
        -g * (cos(q2) * cos(q3) * sin(q5) - sin(q2) * sin(q3) * sin(q5) +
              cos(q2) * cos(q4) * cos(q5) * sin(q3) +
              cos(q3) * cos(q4) * cos(q5) * sin(q2));
    regressor[4][7] =
        -g * (cos(q5) * sin(q2) * sin(q3) - cos(q2) * cos(q3) * cos(q5) +
              cos(q2) * cos(q4) * sin(q3) * sin(q5) +
              cos(q3) * cos(q4) * sin(q2) * sin(q5));
    regressor[4][8] = g * (cos(q5) * cos(q6) * sin(q2) * sin(q3) -
                           cos(q2) * cos(q3) * cos(q5) * cos(q6) +
                           cos(q2) * cos(q4) * cos(q6) * sin(q3) * sin(q5) +
                           cos(q3) * cos(q4) * cos(q6) * sin(q2) * sin(q5));
    regressor[4][9] = -g * (cos(q5) * sin(q2) * sin(q3) * sin(q6) -
                            cos(q2) * cos(q3) * cos(q5) * sin(q6) +
                            cos(q2) * cos(q4) * sin(q3) * sin(q5) * sin(q6) +
                            cos(q3) * cos(q4) * sin(q2) * sin(q5) * sin(q6));
    regressor[4][30] = 1.0;
    regressor[4][31] = q5;
    regressor[4][32] = std::pow(q5, 2);
    regressor[4][33] = std::pow(q5, 3);
    regressor[4][34] = std::pow(q5, 4);
    regressor[5][8] = g * (cos(q2) * cos(q6) * sin(q3) * sin(q4) +
                           cos(q3) * cos(q6) * sin(q2) * sin(q4) +
                           cos(q2) * cos(q3) * sin(q5) * sin(q6) -
                           sin(q2) * sin(q3) * sin(q5) * sin(q6) +
                           cos(q2) * cos(q4) * cos(q5) * sin(q3) * sin(q6) +
                           cos(q3) * cos(q4) * cos(q5) * sin(q2) * sin(q6));
    regressor[5][9] = g * (cos(q2) * cos(q3) * cos(q6) * sin(q5) -
                           cos(q2) * sin(q3) * sin(q4) * sin(q6) -
                           cos(q3) * sin(q2) * sin(q4) * sin(q6) -
                           cos(q6) * sin(q2) * sin(q3) * sin(q5) +
                           cos(q2) * cos(q4) * cos(q5) * cos(q6) * sin(q3) +
                           cos(q3) * cos(q4) * cos(q5) * cos(q6) * sin(q2));
    regressor[5][35] = 1.0;
    regressor[5][36] = q6;
    regressor[5][37] = std::pow(q6, 2);
    regressor[5][38] = std::pow(q6, 3);
    regressor[5][39] = std::pow(q6, 4);
}

void robGravityCompensationMTM::AddGCeffortsTo(const vctVec &q,
                                               const vctVec &q_dot,
                                               vctVec &totalEfforts)
{
    AssignRegressor(q, regressor_);
    const vct7 beta = computeBetaVel(q_dot);
    vct7 tau_pos = regressor_ * p_.pos;
    vct7 tau_neg = regressor_ * p_.neg;

    gravityEfforts_ = tau_pos.ElementwiseMultiply(beta) + tau_neg.ElementwiseMultiply(ones_ - beta);
    LimitEfforts(gravityEfforts_);
    Add(gravityEfforts_, totalEfforts);
}

void robGravityCompensationMTM::LimitEfforts(vct7 &efforts)
{
    for (size_t i = 0; i < efforts.size(); i++) {
        if (efforts[i] > p_.upperEffortsLimit[i]) {
            efforts[i] = p_.upperEffortsLimit[i];
        } else if (efforts[i] < p_.lowerEffortsLimit[i]) {
            efforts[i] = p_.lowerEffortsLimit[i];
        }
    }
}

vct7 robGravityCompensationMTM::computeBetaVel(const vctVec &q_dot)
{
    vct7 beta;
    for (size_t i = 0; i < q_dot.size(); i++) {
        if (q_dot[i] > p_.betaVelAmp[i]) {
            beta[i] = 1.0;
        }
        if (q_dot[i] < -p_.betaVelAmp[i]) {
            beta[i] = 0;
        } else {
            beta[i] = 0.5 + sin(q_dot[i] * M_PI / (2 * p_.betaVelAmp[i])) / 2;
        }
    }
    return beta;
}

robGravityCompensationMTM::CreationResult
robGravityCompensationMTM::Create(const Json::Value &jsonConfig)
{
    if (jsonConfig["version"].asString() == "1.0") {

        robGravityCompensationMTM::Params params;
        auto jpos = jsonConfig["GC_controller"]["gc_dynamic_params_pos"];
        auto jneg = jsonConfig["GC_controller"]["gc_dynamic_params_neg"];
        auto jbeta = jsonConfig["GC_controller"]["beta_vel_amplitude"];
        auto jupper = jsonConfig["GC_controller"]["safe_upper_torque_limit"];
        auto jlower = jsonConfig["GC_controller"]["safe_lower_torque_limit"];

        if (params.pos.size() == jpos.size() &&
            params.neg.size() == jneg.size() &&
            params.betaVelAmp.size() == jbeta.size() &&
            params.lowerEffortsLimit.size() == jlower.size() &&
            params.upperEffortsLimit.size() == jupper.size()) {

            cmnDataJSON<vct40>::DeSerializeText(params.pos, jpos);
            cmnDataJSON<vct40>::DeSerializeText(params.neg, jneg);
            cmnDataJSON<vct7>::DeSerializeText(params.betaVelAmp, jbeta);
            cmnDataJSON<vct7>::DeSerializeText(params.upperEffortsLimit, jupper);
            cmnDataJSON<vct7>::DeSerializeText(params.lowerEffortsLimit, jlower);

            return {new robGravityCompensationMTM(params), ""};
        }
        return {nullptr,
                "the array size in the JSON object is not matched with the "
                "expected size\n"};
    }
    return {nullptr, "the version of dynamic parameters is not 1.0\n"};
}
