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
#include <cisstCommon/cmnLogger.h>

// class contructor
robGravityCompensationMTM::robGravityCompensationMTM()
{
    this->g = GRAVITY_CONSTANT;
    for (size_t i = 0; i < R_ROWS; i++) {
        this->Torques_Max_Limit[i] = Tau_Max_Amplitude;
    }
}

bool robGravityCompensationMTM::Load_Default_JSON(void)
{
    std::ifstream jsonStream;
    jsonStream.open(DEFAULT_JSON_PATH);
    Json::Value config;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, config)) {
        CMN_LOG_INIT_ERROR << "Cannot Find Json file or Empty file" << std::endl;
    }
    return Load_Param(config);
}

bool robGravityCompensationMTM::Load_Param(const Json::Value & config)
{
    std::string param_string[PARAM_NUM] = {"d_param_1",
                                           "d_param_2",
                                           "d_param_3",
                                           "d_param_4",
                                           "d_param_5",
                                           "d_param_6",
                                           "d_param_7",
                                           "d_param_8",
                                           "d_param_9",
                                           "d_param_10",
                                           "d_param_11",
                                           "d_param_12",
                                           "d_param_13",
                                           "d_param_14",
                                           "d_param_15",
                                           "d_param_16",
                                           "d_param_17",
                                           "d_param_18",
                                           "d_param_19",
                                           "d_param_20",
                                           "d_param_21",
                                           "d_param_22",
                                           "d_param_23",
                                           "d_param_24",
                                           "d_param_25",
                                           "d_param_26",
                                           "d_param_27",
                                           "d_param_28",
                                           "d_param_29",
                                           "d_param_30",
                                           "d_param_31",
                                           "d_param_32",
                                           "d_param_33",
                                           "d_param_34",
                                           "d_param_35",
                                           "d_param_36",
                                           "d_param_37",
                                           "d_param_38",
                                           "d_param_39",
                                           "d_param_40",
                                           "d_param_41",
                                           "d_param_42",
                                           "d_param_43",
                                           "d_param_44",
                                           "d_param_45",
                                           "d_param_46",
                                           "d_param_47",
                                           "d_param_48",
                                           "d_param_49",
                                           "d_param_50",
                                           "d_param_51",
                                           "d_param_52",
                                           "d_param_53",
                                           "d_param_54",
                                           "d_param_55",
                                           "d_param_56",
                                           "d_param_57",
                                           "d_param_58",
                                           "d_param_59",
                                           "d_param_60",
                                           "d_param_61",
                                           "d_param_62",
                                           "d_param_63",
                                           "d_param_64",
                                           "d_param_65",
                                           "d_param_66",
                                           "d_param_67",
                                           "d_param_68",
                                           "d_param_69",
                                           "d_param_70"};

    std::string Beta_Vel_String_List[R_ROWS] = {"Beta_Vel_Amplitude_1",
                                                "Beta_Vel_Amplitude_2",
                                                "Beta_Vel_Amplitude_3",
                                                "Beta_Vel_Amplitude_4",
                                                "Beta_Vel_Amplitude_5",
                                                "Beta_Vel_Amplitude_6",
                                                "Beta_Vel_Amplitude_7"};

    if (config.isNull()) {
        CMN_LOG_INIT_ERROR << "JSON file is empty" << std::endl;
        return false;
    }

    const Json::Value Beta_Amplitude_Config = config["Beta_Vel_Amplitude"];
    if (Beta_Amplitude_Config.isNull()) {
        CMN_LOG_INIT_ERROR << " \"Beta_Vel_Amplitude\" in JSON file is empty, please check lgc json files" << std::endl;
        return false;
    }
    for (int i = 0; i < R_ROWS; i++) {
        this->Beta_Vel_Amplitude_List[i] = Beta_Amplitude_Config[Beta_Vel_String_List[i]].asDouble();
        std::cout << Beta_Vel_String_List[i] << ": " << this->Beta_Vel_Amplitude_List[i] << std::endl;
    }

    // Load MTML Dynamic Parameters
    Json::Value dynamic_param;
    const Json::Value dynamic_param_config = config["dynamic_param"];
    dynamic_param = dynamic_param_config["MTML"];
    if (dynamic_param.isNull()) {
        CMN_LOG_INIT_ERROR << "dynamic_param is empty, need to fill in parameters" << std::endl;
        return false;
    }
    else {
        // Loading Model Param
        const unsigned int param_num = dynamic_param.size();
        if (param_num != PARAM_NUM) {
            CMN_LOG_INIT_ERROR << "Please Check, Param length in JSON FILE in Model should be " << PARAM_NUM <<std::endl;
            return false;
        }
        else {
            std::cout << "The Param in JSON FILE are: " << std::endl;
            Json::Value d_param;
            int pos_index = 0;
            int neg_index = 0;
            for (int i = 0; i < PARAM_NUM; i++) {
                d_param = dynamic_param[param_string[i]];
                std::cout << " " << param_string[i] << ": [" << d_param.asDouble() << "]" <<std::endl;
                if (i < CM_PARAM_NUM) {
                    this->MTML_dynamic_parameter_pos[pos_index] = d_param.asDouble();
                    this->MTML_dynamic_parameter_neg[neg_index] = d_param.asDouble();
                    pos_index++;
                    neg_index++;
                }
                else if (i >= CM_PARAM_NUM && i < (PARAM_NUM-CM_PARAM_NUM) / 2 + CM_PARAM_NUM) {
                    this->MTML_dynamic_parameter_pos[pos_index] = d_param.asDouble();
                    pos_index++;
                }
                else {
                    this->MTML_dynamic_parameter_neg[neg_index] = d_param.asDouble();
                    neg_index++;
                }
            }
        }
        std::cout << "load MTML  param_i: [pos_param],[neg_param]" << std::endl;
        for (size_t i = 0; i < MTML_dynamic_parameter_pos.size(); i++) {
            std::cout << "load MTML  param_" << i+1 << ": " << this->MTML_dynamic_parameter_pos[i]
                      << " ,             " << this->MTML_dynamic_parameter_neg[i] << "," <<std::endl;
        }
    }

    // Load MTMR Dynamic Parameters
    dynamic_param = dynamic_param_config["MTMR"];
    if (dynamic_param.isNull()) {
            CMN_LOG_INIT_ERROR << "dynamic_param is empty, need to fill in parameters" << std::endl;
            return false;
    }
    else {
        // Loading Model Param
        const unsigned int param_num = dynamic_param.size();
        if (param_num != PARAM_NUM) {
            CMN_LOG_INIT_ERROR << "Please Check, Param length in JSON FILE in Model should be" << PARAM_NUM << std::endl;
            return false;
        }
        else {
            std::cout << "The Param in JSON FILE are: " << std::endl;
            Json::Value d_param;
            int pos_index = 0;
            int neg_index = 0;
            for (int i=0; i<PARAM_NUM; i++) {
                d_param = dynamic_param[param_string[i]];
                std::cout << " " << param_string[i] << ":   [" << d_param.asDouble() << "]" << std::endl;
                if (i < CM_PARAM_NUM) {
                    this->MTMR_dynamic_parameter_pos[pos_index] = d_param.asDouble();
                    this->MTMR_dynamic_parameter_neg[neg_index] = d_param.asDouble();
                    pos_index++;
                    neg_index++;
                }
                else if (i >= CM_PARAM_NUM && i < (PARAM_NUM - CM_PARAM_NUM) / 2 + CM_PARAM_NUM) {
                    this->MTMR_dynamic_parameter_pos[pos_index] = d_param.asDouble();
                    pos_index++;
                }
                else {
                    this->MTMR_dynamic_parameter_neg[neg_index] = d_param.asDouble();
                    neg_index++;
                }
            }
        }
        std::cout << "load MTMR  param_i: [pos_param],[neg_param]" << std::endl;
        for (size_t i = 0; i < MTMR_dynamic_parameter_pos.size(); i++) {
            std::cout << "load MTMR  param_" << i+1 << ": " << this->MTMR_dynamic_parameter_pos[i]
                      <<" ,             " << this->MTMR_dynamic_parameter_neg[i] << "," << std::endl;
        }
    }
    return true;
}

bool
robGravityCompensationMTM::assign_regressor(vctFixedSizeVector<double, R_ROWS> q)
{
    q1 = q[0];
    q2 = q[1];
    q3 = q[2];
    q4 = q[3];
    q5 = q[4];
    q6 = q[5];
    q7 = q[6];
    // regressor matrix
    double regressor_list[R_ROWS][R_COLUMNS] =  {{         0,         0,                                     0,                                       0,                                                     0,                                                             0,                                                                                                                                      0,                                                                                                                                      0,                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                              0, 1, q1, pow(q1,2), pow(q1,3), pow(q1,4), 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0},
                                                 { g*sin(q2), g*cos(q2), g*cos(q2)*cos(q3) - g*sin(q2)*sin(q3), - g*cos(q2)*sin(q3) - g*cos(q3)*sin(q2), g*cos(q2)*cos(q3)*cos(q4) - g*cos(q4)*sin(q2)*sin(q3),         g*sin(q2)*sin(q3)*sin(q4) - g*cos(q2)*cos(q3)*sin(q4),          g*cos(q4)*sin(q2)*sin(q3)*sin(q5) - g*cos(q3)*cos(q5)*sin(q2) - g*cos(q2)*cos(q3)*cos(q4)*sin(q5) - g*cos(q2)*cos(q5)*sin(q3),          g*cos(q2)*cos(q3)*cos(q4)*cos(q5) - g*cos(q3)*sin(q2)*sin(q5) - g*cos(q2)*sin(q3)*sin(q5) - g*cos(q4)*cos(q5)*sin(q2)*sin(q3),         g*cos(q2)*cos(q3)*sin(q4)*sin(q6) + g*cos(q2)*cos(q6)*sin(q3)*sin(q5) + g*cos(q3)*cos(q6)*sin(q2)*sin(q5) - g*sin(q2)*sin(q3)*sin(q4)*sin(q6) + g*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) - g*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6),          g*cos(q2)*cos(q3)*cos(q6)*sin(q4) - g*cos(q6)*sin(q2)*sin(q3)*sin(q4) - g*cos(q2)*sin(q3)*sin(q5)*sin(q6) - g*cos(q3)*sin(q2)*sin(q5)*sin(q6) - g*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) + g*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6), 0,  0,    0,    0,    0, 1, q2, pow(q2,2), pow(q2,3), pow(q2,4), 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0},
                                                 {         0,         0,                        g*cos(q2 + q3),                         -g*sin(q2 + q3),         (g*(cos(q2 + q3 + q4) + cos(q2 + q3 - q4)))/2, (g*(2*sin(q2)*sin(q3)*sin(q4) - 2*cos(q2)*cos(q3)*sin(q4)))/2, -(g*(2*cos(q2)*cos(q5)*sin(q3) + 2*cos(q3)*cos(q5)*sin(q2) + 2*cos(q2)*cos(q3)*cos(q4)*sin(q5) - 2*cos(q4)*sin(q2)*sin(q3)*sin(q5)))/2, -(g*(2*cos(q2)*sin(q3)*sin(q5) + 2*cos(q3)*sin(q2)*sin(q5) - 2*cos(q2)*cos(q3)*cos(q4)*cos(q5) + 2*cos(q4)*cos(q5)*sin(q2)*sin(q3)))/2, (g*(2*cos(q2)*cos(q3)*sin(q4)*sin(q6) + 2*cos(q2)*cos(q6)*sin(q3)*sin(q5) + 2*cos(q3)*cos(q6)*sin(q2)*sin(q5) - 2*sin(q2)*sin(q3)*sin(q4)*sin(q6) - 2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) + 2*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3)))/2, -(g*(2*cos(q6)*sin(q2)*sin(q3)*sin(q4) - 2*cos(q2)*cos(q3)*cos(q6)*sin(q4) + 2*cos(q2)*sin(q3)*sin(q5)*sin(q6) + 2*cos(q3)*sin(q2)*sin(q5)*sin(q6) - 2*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) + 2*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6)))/2, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 1, q3, pow(q3,2), pow(q3,3), pow(q3,4), 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0},
                                                 {         0,         0,                                     0,                                       0,                               -g*sin(q2 + q3)*sin(q4),                                       -g*sin(q2 + q3)*cos(q4),                                                                                                         g*sin(q2 + q3)*sin(q4)*sin(q5),                                                                                                        -g*sin(q2 + q3)*cos(q5)*sin(q4),                                                                                                                                                                                    g*sin(q2 + q3)*(cos(q4)*sin(q6) + cos(q5)*cos(q6)*sin(q4)),                                                                                                                                                                                     g*sin(q2 + q3)*(cos(q4)*cos(q6) - cos(q5)*sin(q4)*sin(q6)), 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 1, q4, pow(q4,2), pow(q4,3), pow(q4,4), 0,  0,    0,    0,    0, 0,  0,    0,    0,    0},
                                                 {         0,         0,                                     0,                                       0,                                                     0,                                                             0,             -g*(cos(q2)*cos(q3)*sin(q5) - sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*sin(q3) + cos(q3)*cos(q4)*cos(q5)*sin(q2)),             -g*(cos(q5)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*cos(q5) + cos(q2)*cos(q4)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*sin(q2)*sin(q5)),                                                                                     g*(cos(q5)*cos(q6)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*cos(q5)*cos(q6) + cos(q2)*cos(q4)*cos(q6)*sin(q3)*sin(q5) + cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5)),                                                                                     -g*(cos(q5)*sin(q2)*sin(q3)*sin(q6) - cos(q2)*cos(q3)*cos(q5)*sin(q6) + cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q6) + cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6)), 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 1, q5, pow(q5,2), pow(q5,3), pow(q5,4), 0,  0,    0,    0,    0},
                                                 {         0,         0,                                     0,                                       0,                                                     0,                                                             0,                                                                                                                                      0,                                                                                                                                      0,                 g*(cos(q2)*cos(q6)*sin(q3)*sin(q4) + cos(q3)*cos(q6)*sin(q2)*sin(q4) + cos(q2)*cos(q3)*sin(q5)*sin(q6) - sin(q2)*sin(q3)*sin(q5)*sin(q6) + cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)),                  g*(cos(q2)*cos(q3)*cos(q6)*sin(q5) - cos(q2)*sin(q3)*sin(q4)*sin(q6) - cos(q3)*sin(q2)*sin(q4)*sin(q6) - cos(q6)*sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)), 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 1, q6, pow(q6,2), pow(q6,3), pow(q6,4)},
                                                 {         0,         0,                                     0,                                       0,                                                     0,                                                             0,                                                                                                                                      0,                                                                                                                                      0,                                                                                                                                                                                                                                             0,                                                                                                                                                                                                                                              0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0, 0,  0,    0,    0,    0}};

    for (size_t i = 0; i < R_ROWS; i++) {
        for(size_t j = 0; j < R_COLUMNS; j++) {
            this->regressor[i][j] = regressor_list[i][j];
        }
    }

    return true;
}



vctDynamicVector<double>
robGravityCompensationMTM::LGC(const vctDynamicVector<double>& q,
                               const vctDynamicVector<double>& q_dot)
{
    vctFixedSizeMatrix<double, R_ROWS, R_COLUMNS> _Regressor;
    vctFixedSizeVector<double, R_ROWS> _tau_pos, _tau_neg;
    vctFixedSizeVector<double, R_ROWS> beta;
    vctDynamicVector<double> tau(7,0.0);
    if (assign_regressor(q)) {


        beta = beta_vel(q_dot);
        for (size_t i = 0; i < R_ROWS; i++) {
            tau[i] = _tau_pos[i] * beta[i] + _tau_neg[i] * (1 - beta[i]);
        }
        tau = Torque_Safe_Limit(tau);
        return tau;
    }
}

vctDynamicVector<double> robGravityCompensationMTM::Torque_Safe_Limit(vctDynamicVector<double> & tau)
{
    for (size_t i = 0; i < R_ROWS; i++) {
        if (tau[i] > this->Torques_Max_Limit[i]) {
            tau[i] = this->Torques_Max_Limit[i];
        }
        else if (tau[i] < -(this->Torques_Max_Limit[i])) {
            tau[i] = -(this->Torques_Max_Limit[i]);
        }
    }
    return tau;
}

vctFixedSizeVector<double, R_ROWS> robGravityCompensationMTM::beta_vel(const vctDynamicVector<double> & q_dot)
{
    vctFixedSizeVector<double, R_ROWS> beta;
    for (size_t i = 0; i < R_ROWS; i++) {
        if (q_dot[i] > Beta_Vel_Amplitude_List[i]) {
            beta[i] = 1;
        }
        if (q_dot[i] < -Beta_Vel_Amplitude_List[i]) {
            beta[i] = 0;
        }
        else {
            beta[i] = 0.5 + sin(q_dot[i] * M_PI / (2 * this->Beta_Vel_Amplitude_List[i])) / 2;
        }
    }
    return beta;
}

robGravityCompensationMTM::CreationResult robGravityCompensationMTM::create(const Json::Value &jsonConfig)
{
    return {nullptr,"WIP"};
}
