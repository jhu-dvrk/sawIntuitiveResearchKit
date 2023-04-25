/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2022-07-27

  (C) Copyright 2022-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>
#include <time.h>

// cisst
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitSUJSi.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstRobot/robManipulator.h>

// BLE arduino over gatt
#include <gattlib.h>

static const char ATTRIB_POTS[] = "babb122c-de4c-11ec-9d64-0242ac120101";
static const std::map<std::string, size_t> BASE_POT_INDEX = {{"PSM3", 0}, {"ECM", 1}, {"PSM2", 2}, {"PSM1", 3}};
static const std::map<std::string, size_t> NB_JOINTS = {{"PSM1", 4}, {"PSM2", 4}, {"PSM3", 5}, {"ECM", 4}};

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitSUJSi, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg)

class mtsIntuitiveResearchKitSUJSiArduino
{
public:
    inline mtsIntuitiveResearchKitSUJSiArduino(const std::string & arduinoMAC,
                                               const std::string & name):
        m_MAC(arduinoMAC),
        m_name(name)
    {
        gattlib_string_to_uuid(ATTRIB_POTS, strlen(ATTRIB_POTS) + 1, &m_g_uuid);
    }

    inline bool UpdateRawPots(void)
    {
        if (m_MAC.empty()) {
            return false;
        }
        // try to connect if not connected
        if (!m_connected) {
            m_connection = gattlib_connect(nullptr, m_MAC.c_str(),
                                           GATTLIB_CONNECTION_OPTIONS_LEGACY_DEFAULT);
            if (m_connection != nullptr) {
                m_connected = true;
            } else {
                std::cerr << CMN_LOG_DETAILS << " -- " << m_name << " -- BT connection issue " << std::endl;
                gattlib_disconnect(m_connection);
                m_connected = false;
                m_connection = nullptr;
            }
        }
        // get data if connected
        if (m_connected) {
            char * buffer = nullptr;
            int ret;
            size_t len;
            ret = gattlib_read_char_by_uuid(m_connection, &m_g_uuid, (void **)&buffer, &len);
            if (ret == GATTLIB_SUCCESS) {
                // fill last 4 elements from dESSJ over BLE
                m_json_reader.parse(std::string(buffer, len), m_json_value);
                Json::Value jsonPots = m_json_value["pots"];
                cmnDataJSON<vctDoubleMat>::DeSerializeText(m_raw_pots, jsonPots);
                gattlib_characteristic_free_value(buffer);
                return true;
            }
            gattlib_characteristic_free_value(buffer); // needed?
        }
        return false;
    }

    // arduino/gatt
    std::string m_MAC;
    std::string m_name;
    bool m_connected = false;
    gatt_connection_t * m_connection = nullptr;
    uuid_t m_g_uuid;

    // json parsing
    Json::Reader m_json_reader;
    Json::Value m_json_value;
    vctDoubleMat m_raw_pots;
};

class mtsIntuitiveResearchKitSUJSiArmData: public mtsIntuitiveResearchKitSUJSiArduino
{
public:

    typedef enum {SUJ_UNDEFINED, SUJ_PSM, SUJ_ECM, SUJ_MOTORIZED_PSM} SujType;

    inline mtsIntuitiveResearchKitSUJSiArmData(const std::string & name,
                                               const std::string & arduinoMAC,
                                               const bool isSimulated,
                                               mtsInterfaceProvided * interfaceProvided,
                                               mtsInterfaceRequired * interfaceRequired):
        mtsIntuitiveResearchKitSUJSiArduino(arduinoMAC, name),
        m_name(name),
        m_nb_joints(NB_JOINTS.at(name)),
        m_base_arduino_pot_index(BASE_POT_INDEX.at(name)),
        m_simulated(isSimulated),
        mStateTable(500, name),
        mStateTableConfiguration(100, name + "Configuration")
    {
        // base frame
        mBaseFrameValid = true;

        // recalibration matrix
        mRecalibrationMatrix.SetSize(m_nb_joints, m_nb_joints);
        mRecalibrationMatrix.Zeros();
        mNewJointScales[0].SetSize(m_nb_joints);
        mNewJointScales[0].Zeros();
        mNewJointScales[1].SetSize(m_nb_joints);
        mNewJointScales[1].Zeros();

        mNewJointOffsets[0].SetSize(m_nb_joints);
        mNewJointOffsets[0].Zeros();
        mNewJointOffsets[1].SetSize(m_nb_joints);
        mNewJointOffsets[1].Zeros();

        // state table doesn't always advance, only when pots are stable
        mStateTable.SetAutomaticAdvance(false);
        mStateTableConfiguration.SetAutomaticAdvance(false);

        for (size_t potArray = 0; potArray < 2; ++potArray) {
            mVoltages[potArray].SetSize(m_nb_joints);
            mPositions[potArray].SetSize(m_nb_joints);
        }
        mPositionDifference.SetSize(m_nb_joints);
        mPotsAgree = true;

        m_measured_js.Position().SetSize(m_nb_joints);
        m_measured_js.Name().SetSize(m_nb_joints);
        m_configuration_js.Name().SetSize(m_nb_joints);
        std::stringstream jointName;
        for (size_t index = 0; index < m_nb_joints; ++index) {
            jointName.str("");
            jointName << "SUJ_" << name << "_J" << index;
            m_measured_js.Name().at(index) = jointName.str();
            m_configuration_js.Name().at(index) = jointName.str();
        }

        m_configuration_js.Type().SetSize(m_nb_joints);
        m_configuration_js.Type().SetAll(PRM_JOINT_REVOLUTE);
        m_configuration_js.Type().at(0) = PRM_JOINT_PRISMATIC;
        // joint limits are only used in simulation mode so we can set
        // arbitrarily wide limits
        m_configuration_js.PositionMin().SetSize(m_nb_joints);
        m_configuration_js.PositionMax().SetSize(m_nb_joints);
        m_configuration_js.PositionMin().SetAll(-2.0 * cmnPI);
        m_configuration_js.PositionMax().SetAll( 2.0 * cmnPI);
        m_configuration_js.PositionMin().at(0) = -2.0 * cmn_m;
        m_configuration_js.PositionMax().at(0) =  2.0 * cmn_m;

        mStateTable.AddData(mVoltages[0], "PrimaryVoltage");
        mStateTable.AddData(mVoltages[1], "SecondaryVoltage");
        mStateTable.AddData(m_measured_js, "measured_js");
        mStateTable.AddData(m_configuration_js, "configuration_js");

        m_measured_cp.SetReferenceFrame("Cart");
        m_measured_cp.SetMovingFrame(name + "_base");
        mStateTable.AddData(m_measured_cp, "measured_cp");

        m_local_measured_cp.SetReferenceFrame("Cart");
        m_local_measured_cp.SetMovingFrame(name + "_base");
        mStateTable.AddData(m_local_measured_cp, "local/measured_cp");

        mStateTable.AddData(mBaseFrame, "base_frame");
        mStateTableConfiguration.AddData(m_name, "name");
        mStateTableConfiguration.AddData(mSerialNumber, "serial_number");
        mStateTableConfiguration.AddData(mVoltageToPositionOffsets[0], "PrimaryJointOffset");
        mStateTableConfiguration.AddData(mVoltageToPositionOffsets[1], "SecondaryJointOffset");

        CMN_ASSERT(interfaceProvided);
        mInterfaceProvided = interfaceProvided;
        // read commands
        mInterfaceProvided->AddCommandReadState(mStateTable, m_measured_js, "measured_js");
        mInterfaceProvided->AddCommandReadState(mStateTable, m_configuration_js, "configuration_js");
        // set position is only for simulation, allows both servo and move
        mInterfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitSUJSiArmData::servo_jp,
                                            this, "servo_jp");
        mInterfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitSUJSiArmData::servo_jp,
                                            this, "move_jp");
        mInterfaceProvided->AddCommandReadState(mStateTableConfiguration, mVoltageToPositionOffsets[0],
                                                "GetPrimaryJointOffset");
        mInterfaceProvided->AddCommandReadState(mStateTableConfiguration, mVoltageToPositionOffsets[1],
                                                "GetSecondaryJointOffset");
        mInterfaceProvided->AddCommandReadState(mStateTable, m_measured_cp,
                                                "measured_cp");
        mInterfaceProvided->AddCommandReadState(mStateTable, m_local_measured_cp,
                                                "local/measured_cp");
        mInterfaceProvided->AddCommandReadState(mStateTable, mBaseFrame, "base_frame");
        mInterfaceProvided->AddCommandReadState(mStateTable, mVoltages[0], "GetVoltagesPrimary");
        mInterfaceProvided->AddCommandReadState(mStateTable, mVoltages[1], "GetVoltagesSecondary");
        mInterfaceProvided->AddCommandReadState(mStateTableConfiguration, m_name, "GetName");
        mInterfaceProvided->AddCommandReadState(mStateTableConfiguration, mSerialNumber, "GetSerialNumber");

        // write commands
        mInterfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitSUJSiArmData::ClutchCommand, this,
                                            "Clutch", false);
        mInterfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitSUJSiArmData::CalibratePotentiometers, this,
                                            "SetRecalibrationMatrix", mRecalibrationMatrix);

        // cartesian position events
        // m_base_frame is send everytime the mux has found all joint values
        mInterfaceProvided->AddEventWrite(EventPositionCartesian, "PositionCartesian", prmPositionCartesianGet());
        mInterfaceProvided->AddEventWrite(EventPositionCartesianLocal, "PositionCartesianLocal", prmPositionCartesianGet());

        // Events
        mInterfaceProvided->AddEventWrite(state_events.current_state, "current_state", std::string(""));
        mInterfaceProvided->AddEventWrite(state_events.desired_state, "desired_state", std::string(""));
        mInterfaceProvided->AddEventWrite(state_events.operating_state, "operating_state", prmOperatingState());
        mInterfaceProvided->AddMessageEvents();
        // Stats
        mInterfaceProvided->AddCommandReadState(mStateTable, mStateTable.PeriodStats,
                                                "period_statistics");

        CMN_ASSERT(interfaceRequired);
        mInterfaceRequired = interfaceRequired;
        mInterfaceRequired->AddFunction("set_base_frame", mSetArmBaseFrame);
        mInterfaceRequired->AddFunction("local/measured_cp", mGetArmPositionCartesianLocal);
    }

    inline void ClutchCallback(const prmEventButton & button)
    {
        if (button.Type() == prmEventButton::PRESSED) {
            // clutch is pressed, arm is moving around and we know the pots are slow, we mark position as invalid
            mInterfaceProvided->SendStatus(m_name + ": SUJ clutched");
            m_measured_cp.SetTimestamp(m_measured_js.Timestamp());
            m_measured_cp.SetValid(false);
            EventPositionCartesian(m_measured_cp);
            m_local_measured_cp.SetTimestamp(m_measured_js.Timestamp());
            m_local_measured_cp.SetValid(false);
            EventPositionCartesianLocal(m_local_measured_cp);
        } else {
            m_measured_cp.SetValid(true);
            m_local_measured_cp.SetValid(true);
            mInterfaceProvided->SendStatus(m_name + ": SUJ not clutched");
        }
    }

    inline void servo_jp(const prmPositionJointSet & newPosition) {
        if (!m_simulated) {
            mInterfaceProvided->SendWarning(m_name + ": servo_jp can't be used unless the SUJs are in simulated mode");
            return;
        }
        // save the desired position
        m_measured_js.Position().Assign(newPosition.Goal());
        m_measured_js.Timestamp() = newPosition.Timestamp();
    }

    inline void ClutchCommand(const bool & clutch) {
        prmEventButton button;
        if (clutch) {
            button.SetType(prmEventButton::PRESSED);
        } else {
            button.SetType(prmEventButton::RELEASED);
        }
        ClutchCallback(button);
    }

    inline void CalibratePotentiometers(const vctMat & mat) {
        for (size_t col = 0; col < m_nb_joints; col++) {
            // IF:                                      Pi = Offset + Vi * Scale
            // Given P1 / V1 & P2 / V2, THEN:           Scale = (P1 - P2) / (V1 - V2)

            // Delta_P = P1 - P2
            const double deltaJointPosition = mat.Element(0, col) - mat.Element(3, col);

            // Delta_V = V1 - V2 (primary)
            const double deltaPrimaryVoltage = mat.Element(1, col) - mat.Element(4, col);

            // V1 - V2 (secondary)
            const double deltaSecondaryVoltage = mat.Element(2, col) - mat.Element(5, col);

            // Scale = Delta_P / Delta_V
            mNewJointScales[0][col] = deltaJointPosition / deltaPrimaryVoltage;
            mNewJointScales[1][col] = deltaJointPosition / deltaSecondaryVoltage;

            mNewJointOffsets[0][col] = mat.Element(0, col) - mat.Element(1, col) * mNewJointScales[0][col];
            mNewJointOffsets[1][col] = mat.Element(0, col) - mat.Element(2, col) * mNewJointScales[1][col];
        }


        std::cerr << "SUJ scales and offsets for arm: " << m_name << std::endl
                  << "Please update your suj.json file using these values" << std::endl
                  << "\"primary-offsets\": [ "
                  << mNewJointOffsets[0][0] << ", "
                  << mNewJointOffsets[0][1] << ", "
                  << mNewJointOffsets[0][2] << ", "
                  << mNewJointOffsets[0][3] << "],"  << std::endl
                  << "\"primary-scales\": [ "
                  << mNewJointScales[0][0] << ", "
                  << mNewJointScales[0][1] << ", "
                  << mNewJointScales[0][2] << ", "
                  << mNewJointScales[0][3] << "],"  << std::endl
                  << "\"secondary-offsets\": [ "
                  << mNewJointOffsets[1][0] << ", "
                  << mNewJointOffsets[1][1] << ", "
                  << mNewJointOffsets[1][2] << ", "
                  << mNewJointOffsets[1][3] << "]," << std::endl
                  << "\"secondary-scales\": [ "
                  << mNewJointScales[1][0] << ", "
                  << mNewJointScales[1][1] << ", "
                  << mNewJointScales[1][2] << ", "
                  << mNewJointScales[1][3] << "],"  << std::endl;
    }

    // name of this SUJ arm (ECM, PSM1, ...)
    std::string m_name;
    // serial number
    std::string mSerialNumber;

    // number of joints for this arm
    size_t m_nb_joints;

    // index of prismatic pot on base arduino
    size_t m_base_arduino_pot_index;

    // simulated or not
    bool m_simulated;

    // interfaces
    mtsInterfaceProvided * mInterfaceProvided = nullptr;
    mtsInterfaceRequired * mInterfaceRequired = nullptr;

    // state of this SUJ arm
    mtsStateTable mStateTable; // for positions, fairly slow, i.e 12 * delay for a2d
    mtsStateTable mStateTableConfiguration; // changes only at config and if recalibrate

    // 2 arrays, one for each set of potentiometers
    vctDoubleVec mVoltages[2];
    vctDoubleVec mPositions[2];
    vctDoubleVec mPositionDifference;
    bool mPotsAgree;

    vctDoubleVec mVoltageToPositionScales[2];
    vctDoubleVec mVoltageToPositionOffsets[2];
    prmStateJoint m_measured_js;
    prmConfigurationJoint m_configuration_js;
    // 0 is no, 1 tells we need to send, 2 is for first full mux cycle has started
    unsigned int mNumberOfMuxCyclesBeforeStable;
    vctFrm4x4 mPositionCartesianLocal;
    prmPositionCartesianGet m_measured_cp;
    prmPositionCartesianGet m_local_measured_cp;

    // kinematics
    robManipulator mManipulator;
    vctMat mRecalibrationMatrix;
    vctDoubleVec mNewJointScales[2];
    vctDoubleVec mNewJointOffsets[2];

    // setup transformations from json file
    vctFrame4x4<double> mWorldToSUJ;
    vctFrame4x4<double> mSUJToArmBase;

    // base frame
    mtsFunctionWrite mSetArmBaseFrame;
    vctFrame4x4<double> mBaseFrame;
    bool mBaseFrameValid;
    // for ECM only, get current position
    mtsFunctionRead mGetArmPositionCartesianLocal;

    // functions for events
    mtsFunctionWrite EventPositionCartesian;
    mtsFunctionWrite EventPositionCartesianLocal;

    struct {
        mtsFunctionWrite current_state;
        mtsFunctionWrite desired_state;
        mtsFunctionWrite operating_state;
    } state_events;
};

mtsIntuitiveResearchKitSUJSi::mtsIntuitiveResearchKitSUJSi(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    mArmState(componentName, "DISABLED"),
    mStateTableState(100, "State")
{
    Init();
}

mtsIntuitiveResearchKitSUJSi::mtsIntuitiveResearchKitSUJSi(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    mArmState(arg.Name, "DISABLED"),
    mStateTableState(100, "State")
{
    Init();
}

void mtsIntuitiveResearchKitSUJSi::Init(void)
{
    mSimulatedTimer = 0.0;

    // initialize arm pointers
    Arms.SetAll(nullptr);

    // configure state machine common to all arms (ECM/MTM/PSM)
    // possible states
    mArmState.AddState("ENABLED");

    // possible desired states
    mArmState.AddAllowedDesiredState("DISABLED");
    mArmState.AddAllowedDesiredState("ENABLED");

    // state change, to convert to string events for users (Qt, ROS)
    mArmState.SetStateChangedCallback(&mtsIntuitiveResearchKitSUJSi::StateChanged,
                                      this);

    // run for all states
    mArmState.SetRunCallback(&mtsIntuitiveResearchKitSUJSi::RunAllStates,
                             this);

    // disabled
    mArmState.SetEnterCallback("DISABLED",
                               &mtsIntuitiveResearchKitSUJSi::EnterDisabled,
                               this);

    mArmState.SetTransitionCallback("DISABLED",
                                    &mtsIntuitiveResearchKitSUJSi::TransitionDisabled,
                                    this);

    // enabled
    mArmState.SetEnterCallback("ENABLED",
                               &mtsIntuitiveResearchKitSUJSi::EnterEnabled,
                               this);

    mArmState.SetRunCallback("ENABLED",
                             &mtsIntuitiveResearchKitSUJSi::RunEnabled,
                             this);

    mArmState.SetTransitionCallback("ENABLED",
                                    &mtsIntuitiveResearchKitSUJSi::TransitionEnabled,
                                    this);

    // state table to maintain state :-)
    mStateTableState.AddData(mStateTableStateDesired, "desired_state");
    m_operating_state.SetValid(true);
    mStateTableState.AddData(m_operating_state, "operating_state");
    AddStateTable(&mStateTableState);
    mStateTableState.SetAutomaticAdvance(false);

    // default values
    m_simulated = false;

    mInterface = AddInterfaceProvided("Arm");
    if (mInterface) {
        // Arm State
        mInterface->AddCommandWrite(&mtsIntuitiveResearchKitSUJSi::state_command,
                                    this, "state_command", std::string(""));
        mInterface->AddCommandReadState(mStateTableState,
                                        m_operating_state, "operating_state");
        // Events
        mInterface->AddMessageEvents();
        mInterface->AddEventWrite(state_events.desired_state, "desired_state", std::string(""));
        mInterface->AddEventWrite(state_events.current_state, "current_state", std::string(""));
        mInterface->AddEventWrite(state_events.operating_state, "operating_state", prmOperatingState());
        // Stats
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                        "period_statistics");
    }
}


void mtsIntuitiveResearchKitSUJSi::Configure(const std::string & filename)
{
    std::ifstream jsonStream;
    jsonStream.open(filename.c_str());

    Json::Value jsonConfig;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure "<< this->GetName()
                                 << ": failed to parse configuration file \""
                                 << filename << "\"\n"
                                 << jsonReader.getFormattedErrorMessages();
        exit(EXIT_FAILURE);
    }

    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << this->GetName()
                               << " using file \"" << filename << "\"" << std::endl
                               << "----> content of configuration file: " << std::endl
                               << jsonConfig << std::endl
                               << "<----" << std::endl;

    // base component configuration
    mtsComponent::ConfigureJSON(jsonConfig);

    // get the reference arm if defined.  By default ECM and should be
    // used only and only if user want to use a PSM to hold a camera
    std::string referenceArm_name = "ECM";
    const Json::Value jsonReferenceArm = jsonConfig["reference-arm"];
    if (!jsonReferenceArm.isNull()) {
        referenceArm_name = jsonReferenceArm.asString();
        CMN_LOG_CLASS_INIT_WARNING << "Configure: \"reference-arm\" is user defined.  This should only happen if you are using a PSM to hold a camera.  Most users shouldn't define \"reference-arm\".  If undefined, all arm cartesian positions will be defined with respect to the ECM" << std::endl;
    }

    // base arduino used for SUJ prismatic joints
    if (!jsonConfig["base-arduino-mac"]) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: \"base-arduino-mac\" is missing" << std::endl;
        exit(EXIT_FAILURE);
    }
    const std::string mac = jsonConfig["base-arduino-mac"].asString();
    m_base_arduino = new mtsIntuitiveResearchKitSUJSiArduino(mac, "column");

    // find all arms, there should be 4 of them
    const Json::Value jsonArms = jsonConfig["arms"];
    if (jsonArms.size() != 4) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to find 4 SUJ arms" << std::endl;
        exit(EXIT_FAILURE);
    }

    mtsIntuitiveResearchKitSUJSiArmData * arm;
    for (unsigned int index = 0; index < jsonArms.size(); ++index) {
        // name
        Json::Value jsonArm = jsonArms[index];
        std::string name = jsonArm["name"].asString();
        if (!((name == "ECM") || (name == "PSM1") || (name == "PSM2") || (name == "PSM3"))) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: incorrect arm name for SUJ \""
                                     << name << "\", must be one of \"PSM1\", \"PSM2\", \"PSM3\" or \"ECM\""
                                     << std::endl;
            exit(EXIT_FAILURE);
        }

        if (!jsonArm["arduino-mac"]) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: \"arduino-mac\" is missing for SUJ \""
                                     << name << "\"" << std::endl;
            exit(EXIT_FAILURE);
        }
        const std::string mac = jsonArm["arduino-mac"].asString();

        // add interfaces, one is provided so users can find the SUJ
        // info, the other is required to the SUJ can get position of
        // ECM and change base frame on attached arms
        mtsInterfaceProvided * interfaceProvided = this->AddInterfaceProvided(name);
        mtsInterfaceRequired * interfaceRequired = this->AddInterfaceRequired(name, MTS_OPTIONAL);
        arm = new mtsIntuitiveResearchKitSUJSiArmData(name, mac,
                                                      m_simulated,
                                                      interfaceProvided,
                                                      interfaceRequired);
        Arms[index] = arm;

        // save which arm is the Reference Arm
        if (name == referenceArm_name) {
            BaseFrameArmIndex = index;
        }

        // Arm State so GUI widget for each arm can set/get state
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitSUJSi::state_command,
                                           this, "state_command", std::string(""));

        // create a required interface for each arm to handle clutch button
        if (!m_simulated) {
            std::string itf = "SUJ-Clutch-" + name;
            mtsInterfaceRequired * requiredInterface = this->AddInterfaceRequired(itf, MTS_OPTIONAL);
            if (requiredInterface) {
                requiredInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitSUJSiArmData::ClutchCallback, arm,
                                                        "Button");
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: can't add required interface \"" << itf
                                         << "\" for SUJ \"" << name << "\" clutch button"
                                         << std::endl;
                exit(EXIT_FAILURE);
            }
        } else {
            // look for hard coded position if available - users can always push new joint values using ROS
            Json::Value jsonPosition = jsonArm["simulated-position"];
            if (!jsonPosition.empty()) {
                vctDoubleVec position;
                cmnDataJSON<vctDoubleVec>::DeSerializeText(position, jsonPosition);
                if (position.size() == arm->m_measured_js.Position().size()) {
                    arm->m_measured_js.Position().Assign(position);
                } else {
                    CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to load \"position-simulated\" for \""
                                             << name << "\", expected vector size is "
                                             << arm->m_measured_js.Position().size() << " but vector in configuration file has "
                                             << position.size() << " element(s)"
                                             << std::endl;
                    exit(EXIT_FAILURE);
                }
            }
        }

        // find serial number
        arm->mSerialNumber = jsonArm["serial-number"].asString();

        // read pot settings
        arm->mStateTableConfiguration.Start();
        cmnDataJSON<vctDoubleVec>::DeSerializeText(arm->mVoltageToPositionOffsets[0], jsonArm["primary-offsets"]);
        cmnDataJSON<vctDoubleVec>::DeSerializeText(arm->mVoltageToPositionOffsets[1], jsonArm["secondary-offsets"]);
        cmnDataJSON<vctDoubleVec>::DeSerializeText(arm->mVoltageToPositionScales[0], jsonArm["primary-scales"]);
        cmnDataJSON<vctDoubleVec>::DeSerializeText(arm->mVoltageToPositionScales[1], jsonArm["secondary-scales"]);
        for (auto vec : arm->mVoltageToPositionOffsets) {
            if (vec.size() != NB_JOINTS.at(name)) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: incorrect number of voltage to position offsets for \""
                                         << name << "\", expected " << NB_JOINTS.at(name)
                                         << " but found " << vec.size() << " elements" << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        for (auto vec : arm->mVoltageToPositionScales) {
            if (vec.size() != NB_JOINTS.at(name)) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: incorrect number of voltage to position scales for \""
                                         << name << "\", expected " << NB_JOINTS.at(name)
                                         << " but found " << vec.size() << " elements" << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        arm->mStateTableConfiguration.Advance();

        // DH and transforms should be loaded from the share/kinematic
        // directory bit if DH is defined in this scope, assumes user
        // want to provide DH and base/tip transforms
        Json::Value jsonDH = jsonArm["DH"];
        if (!jsonDH.empty()) {
            // look for DH
            arm->mManipulator.LoadRobot(jsonArm["DH"]);
            // read setup transforms
            vctFrm3 transform;
            cmnDataJSON<vctFrm3>::DeSerializeText(transform, jsonArm["world-origin-to-suj"]);
            arm->mWorldToSUJ.From(transform);
            cmnDataJSON<vctFrm3>::DeSerializeText(transform, jsonArm["suj-tip-to-tool-origin"]);
            arm->mSUJToArmBase.From(transform);
        } else {
            // look for configuration file in share/kinematics

        }
    }
}

void mtsIntuitiveResearchKitSUJSi::UpdateOperatingStateAndBusy(const prmOperatingState::StateType & state,
                                                               const bool isBusy)
{
    mStateTableState.Start();
    m_operating_state.State() = state;
    m_operating_state.IsBusy() = isBusy;
    mStateTableState.Advance();
    // push only operating_state since it's the only one changing
    DispatchOperatingState();
}

void mtsIntuitiveResearchKitSUJSi::StateChanged(void)
{
    const std::string newState = mArmState.CurrentState();
    // update state table
    mStateTableState.Start();
    mStateTableStateCurrent = newState;
    mStateTableState.Advance();
    // event
    DispatchStatus(this->GetName() + ": current state " + newState);
    DispatchState();
}

void mtsIntuitiveResearchKitSUJSi::RunAllStates(void)
{
    // get robot data, i.e. process mux/pots
    GetRobotData();
}

void mtsIntuitiveResearchKitSUJSi::EnterDisabled(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::DISABLED, false);
}

void mtsIntuitiveResearchKitSUJSi::TransitionDisabled(void)
{
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState("ENABLED");
    }
}

void mtsIntuitiveResearchKitSUJSi::EnterEnabled(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, false);

    if (m_simulated) {
        // set all data to be valid
        for (auto arm : Arms) {
            arm->m_measured_js.Valid() = true;
            arm->m_measured_cp.Valid() = true;
            arm->m_local_measured_cp.Valid() = true;
        }
        SetHomed(true);
        return;
    }
}

void mtsIntuitiveResearchKitSUJSi::TransitionEnabled(void)
{
    // move to next stage if desired state is different
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState(mArmState.DesiredState());
    }
}

void mtsIntuitiveResearchKitSUJSi::Startup(void)
{
    SetDesiredState("DISABLED");
}

void mtsIntuitiveResearchKitSUJSi::Run(void)
{
    // collect data from required interfaces
    ProcessQueuedEvents();
    try {
        mArmState.Run();
    } catch (std::exception & e) {
        DispatchError(this->GetName() + ": in state " + mArmState.CurrentState()
                      + ", caught exception \"" + e.what() + "\"");
        SetDesiredState("DISABLED");
    }
    // trigger ExecOut event
    RunEvent();
    ProcessQueuedCommands();

    // update all base frame kinematics
    // first see if there's an BaseFrameArm connected
    prmPositionCartesianGet baseFrameArmPositionParam;
    prmPositionCartesianGet baseFrameArmTipToSUJBase;

    mtsIntuitiveResearchKitSUJSiArmData * arm = Arms[BaseFrameArmIndex];
    if (! (Arms[BaseFrameArmIndex]->mGetArmPositionCartesianLocal(baseFrameArmPositionParam))) {
        // interface not connected, reporting wrt cart
        baseFrameArmTipToSUJBase.SetValid(true);
        baseFrameArmTipToSUJBase.SetReferenceFrame("Cart");
    } else {
        // get position from BaseFrameArm and convert to useful type
        vctFrm3 sujBaseToSUJTip = arm->m_local_measured_cp.Position() * baseFrameArmPositionParam.Position();
        // compute and send new base frame for all SUJs (SUJ will handle BaseFrameArm differently)
        baseFrameArmTipToSUJBase.Position().From(sujBaseToSUJTip.Inverse());
        // it's an inverse, swap moving and reference frames
        baseFrameArmTipToSUJBase.SetReferenceFrame(baseFrameArmPositionParam.MovingFrame());
        baseFrameArmTipToSUJBase.SetMovingFrame(arm->m_local_measured_cp.ReferenceFrame());
        // valid only if both are valid
        baseFrameArmTipToSUJBase.SetValid(arm->m_local_measured_cp.Valid()
                                          && baseFrameArmPositionParam.Valid());
        baseFrameArmTipToSUJBase.SetTimestamp(baseFrameArmPositionParam.Timestamp());
    }

    for (size_t armIndex = 0; armIndex < 4; ++armIndex) {
        arm = Arms[armIndex];
        // update positions with base frame, local positions are only
        // updated from FK when joints are ready
        if (armIndex != BaseFrameArmIndex) {
            arm->mBaseFrame.From(baseFrameArmTipToSUJBase.Position());
            arm->mBaseFrameValid = baseFrameArmTipToSUJBase.Valid();
            arm->m_measured_cp.SetReferenceFrame(baseFrameArmTipToSUJBase.ReferenceFrame());
        }
        vctFrm4x4 armLocal(arm->m_local_measured_cp.Position());
        vctFrm4x4 armBase = arm->mBaseFrame * armLocal;
        // - with base frame
        arm->m_measured_cp.Position().From(armBase);
        arm->m_measured_cp.SetTimestamp(arm->m_measured_js.Timestamp());
        arm->EventPositionCartesian(arm->m_measured_cp);
        // - set base frame for the arm
        prmPositionCartesianSet positionSet;
        positionSet.Goal().Assign(arm->m_measured_cp.Position());
        positionSet.Valid() = arm->m_measured_cp.Valid();
        positionSet.Timestamp() = arm->m_measured_cp.Timestamp();
        positionSet.ReferenceFrame() = arm->m_measured_cp.ReferenceFrame();
        positionSet.MovingFrame() = arm->m_measured_cp.MovingFrame();
        arm->mSetArmBaseFrame(positionSet);
    }
}

void mtsIntuitiveResearchKitSUJSi::Cleanup(void)
{
}

void mtsIntuitiveResearchKitSUJSi::set_simulated(void)
{
    m_simulated = true;
    // set all arms simulated
    for (auto arm : Arms) {
        if (arm != nullptr) {
            arm->m_simulated = true;
        }
    }
    // in simulation mode, we don't need IOs
    RemoveInterfaceRequired("RobotIO");
}

void mtsIntuitiveResearchKitSUJSi::GetRobotData(void)
{
    if (m_simulated) {
        return;
    }

    // update pot values from base arduino
    if (m_base_arduino) {
        m_base_arduino->UpdateRawPots();
    }

    for (auto arm : Arms) {
        if (arm != nullptr) {
            // see if we can get updated pot values
            if (arm->UpdateRawPots()) {
                arm->mStateTable.Start();

                // first joint comes from base arduino
                if (m_base_arduino && m_base_arduino->m_connected) {
                    arm->mVoltages[0].at(0) = m_base_arduino->m_raw_pots.Row(0).at(arm->m_base_arduino_pot_index);
                    arm->mVoltages[1].at(0) = m_base_arduino->m_raw_pots.Row(1).at(arm->m_base_arduino_pot_index);
                }

                // last 3 to 4 joints come from this arm's arduino
                size_t joints_to_copy = arm->m_nb_joints - 1;
                arm->mVoltages[0].Ref(joints_to_copy, 1) = arm->m_raw_pots.Row(0).Ref(joints_to_copy);
                arm->mVoltages[1].Ref(joints_to_copy, 1) = arm->m_raw_pots.Row(1).Ref(joints_to_copy);

                // convert to SI
                arm->mPositions[0].Assign(arm->mVoltageToPositionOffsets[0]);
                arm->mPositions[0].AddElementwiseProductOf(arm->mVoltageToPositionScales[0], arm->mVoltages[0]);
                arm->mPositions[1].Assign(arm->mVoltageToPositionOffsets[1]);
                arm->mPositions[1].AddElementwiseProductOf(arm->mVoltageToPositionScales[1], arm->mVoltages[1]);

                // compare primary and secondary pots when arm is not clutched
                const double angleTolerance = 1.0 * cmnPI_180;
                const double distanceTolerance = 2.0 * cmn_mm;
                arm->mPositionDifference.DifferenceOf(arm->mPositions[0], arm->mPositions[1]);
                if ((arm->mPositionDifference[0] > distanceTolerance) ||
                    (arm->mPositionDifference.Ref(3, 1).MaxAbsElement() > angleTolerance)) {
                    // send messages if this is new
                    if (arm->mPotsAgree) {
                        mInterface->SendWarning(this->GetName() + ": " + arm->m_name + " primary and secondary potentiometers don't seem to agree.");
                        CMN_LOG_CLASS_RUN_WARNING << "GetAndConvertPotentiometerValues, error: " << std::endl
                                                  << " - " << this->GetName() << ": " << arm->m_name << std::endl
                                                  << " - primary:   " << arm->mPositions[0] << std::endl
                                                  << " - secondary: " << arm->mPositions[1] << std::endl;
                        arm->mPotsAgree = false;
                    }
                } else {
                    if (!arm->mPotsAgree) {
                        mInterface->SendStatus(this->GetName() + ": " + arm->m_name + " primary and secondary potentiometers agree.");
                        CMN_LOG_CLASS_RUN_VERBOSE << "GetAndConvertPotentiometerValues recovery" << std::endl
                                                  << " - " << this->GetName() << ": " << arm->m_name << std::endl;
                        arm->mPotsAgree = true;
                    }
                }

                // use average of positions reported by potentiometers
                arm->m_measured_js.Position().SumOf(arm->mPositions[0],
                                                    arm->mPositions[1]);
                arm->m_measured_js.Position().Divide(2.0);
                arm->m_measured_js.SetValid(true);

                // advance this arm state table
                arm->mStateTable.Advance();
            }
        }
    }

#if 0
    arm->m_local_measured_cp.SetValid(true);

    // always update the global position valid flag to take into account base frame valid
    arm->m_measured_cp.SetValid(arm->mBaseFrameValid && arm->m_local_measured_cp.Valid());

    // forward kinematic
    vctFrm4x4 suj = arm->mManipulator.ForwardKinematics(arm->mJointGet, 4);
    // pre and post transformations loaded from JSON file, base frame updated using events
    arm->mPositionCartesianLocal = arm->mWorldToSUJ * suj * arm->mSUJToArmBase;
    // update local only
    arm->m_local_measured_cp.Position().From(arm->mPositionCartesianLocal);
    arm->m_local_measured_cp.SetTimestamp(arm->m_measured_js.Timestamp());
    arm->EventPositionCartesianLocal(arm->m_local_measured_cp);

#endif

}

void mtsIntuitiveResearchKitSUJSi::SetDesiredState(const std::string & state)
{
    // setting desired state triggers a new event so user nows which state is current
    DispatchState();
    // try to find the state in state machine
    if (!mArmState.StateExists(state)) {
        DispatchError(this->GetName() + ": unsupported state " + state);
        return;
    }
    // try to set the desired state
    try {
        mArmState.SetDesiredState(state);
    } catch (...) {
        DispatchError(this->GetName() + ": " + state + " is not an allowed desired state");
        return;
    }
    // update state table
    mStateTableState.Start();
    mStateTableStateDesired = state;
    mStateTableState.Advance();

    DispatchState();
    DispatchStatus(this->GetName() + ": desired state " + state);

    // state transitions with direct transitions
    if (state == "DISABLED") {
        mArmState.SetCurrentState(state);
    }
}

void mtsIntuitiveResearchKitSUJSi::state_command(const std::string & command)
{
    std::string humanReadableMessage;
    prmOperatingState::StateType newOperatingState;
    try {
        if (m_operating_state.ValidCommand(prmOperatingState::CommandTypeFromString(command),
                                           newOperatingState, humanReadableMessage)) {
            if (command == "enable") {
                SetDesiredState("ENABLED");
                return;
            }
            if (command == "disable") {
                SetDesiredState("DISABLED");
                return;
            }
            if (command == "home") {
                SetDesiredState("ENABLED");
                SetHomed(true);
                return;
            }
            if (command == "unhome") {
                SetHomed(false);
                return;
            }
            if (command == "pause") {
                std::cerr << CMN_LOG_DETAILS << " not implemented yet" << std::endl;
                return;
            }
            if (command == "resume") {
                std::cerr << CMN_LOG_DETAILS << " not implemented yet" << std::endl;
                return;
            }
        } else {
            DispatchWarning(this->GetName() + ": " + humanReadableMessage);
        }
    } catch (std::runtime_error & e) {
        DispatchWarning(this->GetName() + ": " + command + " doesn't seem to be a valid state_command (" + e.what() + ")");
    }
}

void mtsIntuitiveResearchKitSUJSi::RunEnabled(void)
{
    if (m_simulated) {
        const double currentTime = this->StateTable.GetTic();
        if (currentTime - mSimulatedTimer > 1.0 * cmn_s) {
            mSimulatedTimer = currentTime;
            for (auto arm : Arms) {
                arm->mStateTable.Start();
                // forward kinematic
                vctFrm4x4 suj = arm->mManipulator.ForwardKinematics(arm->m_measured_js.Position(), arm->m_nb_joints);
                // pre and post transformations loaded from JSON file, base frame updated using events
                vctFrm4x4 armLocal = arm->mWorldToSUJ * suj * arm->mSUJToArmBase;
                // apply base frame
                vctFrm4x4 armBase = arm->mBaseFrame * armLocal;
                // emit events for continuous positions
                // - joint state
                arm->m_measured_js.SetValid(true);
                // - with base
                arm->m_measured_cp.Position().From(armBase);
                arm->m_measured_cp.SetTimestamp(arm->m_measured_js.Timestamp());
                arm->m_measured_cp.SetValid(arm->mBaseFrameValid);
                arm->EventPositionCartesian(arm->m_measured_cp);
                // - local
                arm->m_local_measured_cp.Position().From(armLocal);
                arm->m_local_measured_cp.SetTimestamp(arm->m_measured_js.Timestamp());
                arm->m_local_measured_cp.SetValid(arm->mBaseFrameValid);
                arm->EventPositionCartesianLocal(arm->m_local_measured_cp);
                arm->mStateTable.Advance();
            }
        }
        return;
    }
}

void mtsIntuitiveResearchKitSUJSi::SetHomed(const bool homed)
{
    if (homed != m_operating_state.IsHomed()) {
        m_operating_state.IsHomed() = homed;
        DispatchOperatingState();
    }
}

void mtsIntuitiveResearchKitSUJSi::ErrorEventHandler(const mtsMessage & message)
{
    DispatchError(this->GetName() + ": received [" + message.Message + "]");
    SetDesiredState("DISABLED");
}

void mtsIntuitiveResearchKitSUJSi::DispatchError(const std::string & message)
{
    mInterface->SendError(message);
    for (auto arm : Arms) {
        arm->mInterfaceProvided->SendError(arm->m_name + " " + message);
    }
}

void mtsIntuitiveResearchKitSUJSi::DispatchWarning(const std::string & message)
{
    mInterface->SendWarning(message);
    for (auto arm : Arms) {
        arm->mInterfaceProvided->SendWarning(arm->m_name + " " + message);
    }
}

void mtsIntuitiveResearchKitSUJSi::DispatchStatus(const std::string & message)
{
    mInterface->SendStatus(message);
    for (auto arm : Arms) {
        arm->mInterfaceProvided->SendStatus(arm->m_name + " " + message);
    }
}

void mtsIntuitiveResearchKitSUJSi::DispatchState(void)
{
    state_events.current_state(mArmState.CurrentState());
    for (auto arm : Arms) {
        arm->state_events.current_state(mArmState.CurrentState());
    }
    state_events.desired_state(mArmState.DesiredState());
    for (auto arm : Arms) {
        arm->state_events.desired_state(mArmState.DesiredState());
    }
    DispatchOperatingState();
}

void mtsIntuitiveResearchKitSUJSi::DispatchOperatingState(void)
{
    state_events.operating_state(m_operating_state);
    for (auto arm : Arms) {
        arm->state_events.operating_state(m_operating_state);
    }
}
