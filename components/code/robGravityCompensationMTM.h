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

#include <cisstVector/vctMatrixRotation3.h>
#include <cisstVector/vctFrame4x4.h>
#if CISST_HAS_JSON
#include <json/json.h>
#endif
#include <cisstRobot/robExport.h>
#include <vector>
#include <math.h>
#include <string>
#include <iostream>
#include <cisstVector/vctFixedSizeMatrix.h>
#include <cisstVector/vctDynamicVector.h>

// number of gravity compensation dynamics parameters
#define PARAM_NUM 70

// number of parameters related to center of mass
#define CM_PARAM_NUM 10

// row size of regressor matrix
#define R_ROWS 7

// column size of regressor matrix
#define R_COLUMNS 40

#define GRAVITY_CONSTANT 9.81

// hard code path (still finding some other solution like JSON)
#define DEFAULT_JSON_PATH  "/home/ben/dvrk_ws/src/cisst-saw/sawIntuitiveResearchKit/share/cuhk-daVinci/lgc_config.json"

// maximum torque output for safety
#define Tau_Max_Amplitude 1

// velocity amplitude for low friction compensation
#define Beta_Vel_Amplitude 2


class CISST_EXPORT robGravityCompensationMTM
{
 public:

    double q1,q2,q3,q4,q5,q6,q7;
    double g;
    vctFixedSizeVector<double, R_ROWS> Torques_Max_Limit;
    vctFixedSizeVector<double, R_ROWS> Beta_Vel_Amplitude_List;


    vctFixedSizeVector<double, R_COLUMNS> MTML_dynamic_parameter_pos;
    vctFixedSizeVector<double, R_COLUMNS> MTML_dynamic_parameter_neg;
    vctFixedSizeVector<double, R_COLUMNS> MTMR_dynamic_parameter_pos;
    vctFixedSizeVector<double, R_COLUMNS> MTMR_dynamic_parameter_neg;
    vctFixedSizeMatrix<double, R_ROWS, R_COLUMNS> regressor;

    robGravityCompensationMTM();
    bool Load_Default_JSON();
    bool Load_Param(const Json::Value &config);

    bool assign_regressor(vctFixedSizeVector<double, R_ROWS> q);
    vctDynamicVector<double> LGC(const vctDynamicVector<double>& q, const vctDynamicVector<double>& q_dot);
    vctDynamicVector<double> Torque_Safe_Limit(vctDynamicVector<double> &tau);
    vctFixedSizeVector<double, R_ROWS> beta_vel(const vctDynamicVector<double>& q_dot);
};

#endif // _robGravityCompensationMTM_h
