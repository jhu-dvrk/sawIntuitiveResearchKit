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
#include <sawIntuitiveResearchKit/mtsPSMCompensation.h>
// #include <cmath>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#if CISST_HAS_JSON
#include <json/json.h>
#endif

CMN_IMPLEMENT_SERVICES_DERIVED(mtsPSMCompensation, mtsTaskPeriodic);

mtsPSMCompensation::mtsPSMCompensation(const std::string & componentName, double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds, false, 500)
{
    StateTable.AddData(mJointStateEncoder, "JointStateEncoder");
    StateTable.AddData(mJointStateCompensated, "CorrectedJointState");

    SetupInterfaces();
}

void mtsPSMCompensation::SetupInterfaces(void){

    // add provided interfaces
    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("JointStateCorrected");
    if (!interfaceProvided){
        CMN_LOG_CLASS_INIT_ERROR << "failed to add \"JointStateCorrected\" to component \""
            << this->GetName() << "\"" << std::endl;
        return;
    }
  
    // get the latest values from both the state tables for comparison
    interfaceProvided->AddCommandReadState(StateTable, mJointStateCompensated, "GetJointState");

    // add another provided interfaces
    interfaceProvided = AddInterfaceProvided("JointStateEncoder");
    if (!interfaceProvided){
        CMN_LOG_CLASS_INIT_ERROR << "failed to add \"JointStateEncoder\" to component \""
            << this->GetName() << "\"" << std::endl;
        return;
    }
  
    // get the latest values from both the state tables for comparison
    interfaceProvided->AddCommandReadState(StateTable, mJointStateEncoder, "GetJointState");

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

void mtsPSMCompensation::Configure(const std::string & filename) {
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
                complianceA1 = parameter["value-a"].asDouble();
                complianceB1 = parameter["value-b"].asDouble();
                complianceC1 = parameter["value-c"].asDouble();
                complianceD1 = parameter["value-d"].asDouble();
            } else if (parameter_name == "torque_offset_first") {
                torqueOffsetA1 = parameter["value-a"].asDouble();
            } else if (parameter_name == "backlash_first"){
                backlash1 = parameter["value-a"].asDouble();
            } else if (parameter_name == "compliance_second") { // store data for second joint
                complianceA2 = parameter["value-a"].asDouble();
                complianceB2 = parameter["value-b"].asDouble();
                complianceC2 = parameter["value-c"].asDouble();
                complianceD2 = parameter["value-d"].asDouble();
            }else if (parameter_name == "torque_offset_second") {
                torqueOffsetA2 = parameter["value-a"].asDouble();
                torqueOffsetB2 = parameter["value-b"].asDouble();
            } else if (parameter_name == "backlash_second"){
                backlash2 = parameter["value-a"].asDouble();
            } 
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: can't find \"parameter\" for parameters["
                                 << i << "]" << std::endl;
            return;
        }
    }
#else
    CMN_LOG_CLASS_INIT_ERROR << "Configure: this method requires CISST_HAS_JSON, reconfigure cisst with CISST_HAS_JSON" << std::endl;
#endif
}

void mtsPSMCompensation::ComputeCompensation() {
    mJointStateCompensated = mJointStateEncoder;

    double polyThird = pow(mJointStateEncoder.Position()[2], 3);
    double polySecond = pow(mJointStateEncoder.Position()[2], 2);
    double polyFirst = mJointStateEncoder.Position()[2];

    // calculate compliance
    double A = complianceA1 * polyThird;
    double B = complianceB1 * polySecond;
    double C = complianceC1 * polyFirst;
    double D = complianceD1;
    double complianceFirstJoint = A + B + C + D;

    // calculate first joint correction
    mJointStateCompensated.Position()[0] = mJointStateEncoder.Position()[0] - (backlash1 * (mJointStateEncoder.Effort()[0] - torqueOffsetA1) +
        (complianceFirstJoint * (mJointStateEncoder.Effort()[0] - torqueOffsetA1)));

    // calculate compliance
    A = complianceA2 * polyThird;
    B = complianceB2 * polySecond;
    C = complianceC2 * polyFirst;
    D = complianceD2;
    double complianceSecondJoint = A + B + C + D;

    // calculate torque offset
    double torqueOffset2 = (torqueOffsetA2 * mJointStateEncoder.Position()[2]) + torqueOffsetB2;

    // calculate second joint correction
    mJointStateCompensated.Position()[1] = mJointStateEncoder.Position()[1] - (backlash2 * (mJointStateEncoder.Effort()[1] - torqueOffset2) +
        (complianceSecondJoint * (mJointStateEncoder.Effort()[1] - torqueOffset2)));

   
    // assign other members of encoder joint state to the compensated joint state
    mJointStateCompensated.Timestamp() = mJointStateEncoder.Timestamp();
    mJointStateCompensated.Velocity().Assign(mJointStateEncoder.Velocity());
    mJointStateCompensated.Effort().Assign(mJointStateEncoder.Effort());
    mJointStateCompensated.Valid() = mJointStateEncoder.Valid();
    if (mJointStateCompensated.Name().size() == 0) {
        mJointStateCompensated.Name() = mJointStateEncoder.Name();
    }
    // save temp computation as var
}

prmStateJoint mtsPSMCompensation::GetCorrectedJointState() const {
    return mJointStateCompensated;
}

void mtsPSMCompensation::Startup(void){}

void mtsPSMCompensation::Run(void){

    ProcessQueuedCommands();
    ComputeCompensation();
}