/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Grace Chrysilla
  Created on: 2017-07-18

  (C) Copyright 2013-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/
#include "mtsCompensation.h"
// #include <cmath>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#if CISST_HAS_JSON
#include <json/json.h>
#endif

CMN_IMPLEMENT_SERVICES_DERIVED(compensation, mtsTaskPeriodic);

mstCompensation::mstCompensation(const std::string & componentName, double periodInSeconds){
    mtsTaskPeriodic(componentName, periodInSeconds, false, 500);

    StateTable.AddData(JointState, "JointState");
    // CompensatedStateTable(50, "Compensated");
    // // adding the state table to the component
    // AddStateTable(&CompensatedStateTable);
    // // make sure we control when the table advances
    // CompensatedStateTable.SetAutomaticAdvance(false);
    // add data to the state table
    StateTable.AddData(CorrectedJointState, "CorrectedJointState");

    setupInterfaces();

    // get JointState from PID
    // JointState = GetJointState();
}

void mstCompensation::setupInterfaces(void){

    // add provided interfaces
    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("JointStateCorrected");
    if (!interfaceProvided){
        CMN_LOG_CLASS_INIT_ERROR << "failed to add \"PSM-RO\" to component \""
            << this->GetName() << "\"" << std::endl;
        return;
    }
  
    // get the latest values from both the state tables for comparison
    interfaceProvided->AddCommandReadState(StateTable, CorrectedJointState, "GetJointState");

    // add another provided interfaces
    interfaceProvided = AddInterfaceProvided("JointState");
    if (!interfaceProvided){
        CMN_LOG_CLASS_INIT_ERROR << "failed to add \"PSM-RO\" to component \""
            << this->GetName() << "\"" << std::endl;
        return;
    }
  
    // get the latest values from both the state tables for comparison
    interfaceProvided->AddCommandReadState(StateTable, JointState, "GetJointState");

    // add required interfaces
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("PID");
    if (!interfaceRequired){
        CMN_LOG_CLASS_INIT_ERROR << "failed to add \"PSM-RO\" to component \""
            << this->GetName() << "\"" << std::endl;
        return;
    }

    // add commands to the interface
    interfaceRequired->AddFunction("GetJointState", this->GetJointState);

}

void mstCompensation::Configure(const std::string & filename) {
#if CISST_HAS_JSON
    std::ifstream jsonStream;
    jsonStream.open(filename.c_str());

    Json::Value jsonConfig, jsonValue;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
         CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration" << std::endl
                             << "File: " << filename << std::endl << "Error(s):" << std::endl
                             << jsonReader.getFormattedErrorMessages();
        return;
    }

    // search for array of parameters  
    const Json::Value parameters = jsonConfig["parameters"];
    if (parameters.size() == 0) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: configuration files needs \"parameters\"" << std::endl;
        return;
    }

    for (unsigned int i=0; i<parameters.size(); ++i) {
        Json::Value parameter = parameters[i];
        std::string parameter_name;

        jsonValue = parameter["parameter"];
        if (! jsonValue.empty()) {
            parameter_name = jsonValue.asString();
            
            if (parameter_name == "compliance_first") { // store data for first joint
                complianceA1 = parameter["value-a"];
                complianceB1 = parameter["value-b"];
                complianceC1 = parameter["value-c"];
                complianceD1 = parameter["value-d"];
            } else if (parameter_name == "torque_offset_first") {
                torqueOffsetA1 = parameter["value-a"];
            } else if (parameter_name == "backlash_first"){
                backlash1 = parameter["value-a"];
            } else if (parameter_name == "compliance_second") { // store data for second joint
                complianceA2 = parameter["value-a"];
                complianceB2 = parameter["value-b"];
                complianceC2 = parameter["value-c"];
                complianceD2 = parameter["value-d"];
            }else if (parameter_name == "torque_offset_second") {
                torqueOffsetA2 = parameter["value-a"];
                torqueOffsetB2 = parameter["value-b"];
            } else if (parameter_name == "backlash_second"){
                backlash2 = parameter["value-a"];
            } 
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find \"parameter\" for parameters["
                                 << index << "]" << std::endl;
            return;
        }
    }
#else
    CMN_LOG_CLASS_INIT_ERROR << "Configure: this method requires CISST_HAS_JSON, reconfigure cisst with CISST_HAS_JSON" << std::endl;
#endif
}

void mtsCompensation::setCorrectedPosition() {
    prmStateJoint correctedJointState = JointState;

    // calculate compliance
    double A = (complianceA1 * pow(JointState.Position[2], 3));
    double B = (complianceB1 * pow(JointState.Position[2], 2));
    double C = complianceC1 * JointState.Position[2];
    double D = complianceD1;
    double complianceFirstJoint = A + B + C + D;

    // calculate first joint correction
    correctedJointState[0] = JointState[0] - (backlash1 * (JointState.Effort[0] - torqueOffsetA1) +
        (complianceFirstJoint * (JointState.Effort[0] - torqueOffsetA1)));

    // calculate compliance
    A = (complianceA2 * pow(JointState.Position[2], 3));
    B = (complianceB2 * pow(JointState.Position[2], 2));
    C = complianceC2 * JointState.Position[2];
    D = complianceD2;
    double complianceSecondJoint = A + B + C + D;

    // calculate torque offset
    double torqueOffset2 = (torqueOffsetA2 * JointState.Position[2]) + torqueOffsetB2;

    // calculate second joint correction
    correctedJointState[1] = JointState[1] - (backlash2 * (JointState.Effort[1] - torqueOffset2) +
        (complianceSecondJoint * (JointState.Effort[1] - torqueOffset2)));

    // CompensatedStateTable.Start();
    CorrectedJointState = correctedJointState;
    // CompensatedStateTable.Advance();

    // TODO: copy timestamp of the JointState obj, isValid, Velocity, Effort, Name 
    // for name: for JointStateCompensated, if the name vector size is 0, take the name from JointStateEncoders
    // save temp computation as var
}

prmStateJoint mtsCompensation::getCorrectedJointState() const {
    return CorrectedJointState;
}

void mtsCompensation::Startup(void){}

void mtsCompensation::Run(void){

    ProcessQueuedCommands();
    setCorrectedPosition();
}