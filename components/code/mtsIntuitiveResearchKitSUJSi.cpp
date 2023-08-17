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
#include <cisstMultiTask/mtsManagerLocal.h>
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

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitSUJSi, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

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


    inline bool check_connection(void)
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
        return m_connected;
    }


    inline bool update_raw_pots(void)
    {
        // try to connect if not connected
        if (!check_connection()) {
            return false;
        }

        // get data if connected
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
                                               mtsInterfaceProvided * interfaceProvided,
                                               mtsInterfaceRequired * interfaceRequired):
        mtsIntuitiveResearchKitSUJSiArduino(arduinoMAC, name),
        m_name(name),
        m_nb_joints(NB_JOINTS.at(name)),
        m_base_arduino_pot_index(BASE_POT_INDEX.at(name)),
        m_state_table(500, name),
        m_state_table_configuration(100, name + "Configuration")
    {
        // recalibration matrix
        m_recalibration_matrix.SetSize(m_nb_joints, m_nb_joints);
        m_recalibration_matrix.Zeros();
        m_new_joint_scales[0].SetSize(m_nb_joints);
        m_new_joint_scales[0].Zeros();
        m_new_joint_scales[1].SetSize(m_nb_joints);
        m_new_joint_scales[1].Zeros();

        m_new_joint_offsets[0].SetSize(m_nb_joints);
        m_new_joint_offsets[0].Zeros();
        m_new_joint_offsets[1].SetSize(m_nb_joints);
        m_new_joint_offsets[1].Zeros();

        // state table doesn't always advance, only when pots are stable
        m_state_table.SetAutomaticAdvance(false);
        m_state_table_configuration.SetAutomaticAdvance(false);

        for (size_t potArray = 0; potArray < 2; ++potArray) {
            m_voltages[potArray].SetSize(m_nb_joints);
            m_positions[potArray].SetSize(m_nb_joints);
        }
        m_delta_measured_js.SetSize(m_nb_joints);
        m_pots_agree = true;

        m_measured_js.Position().SetSize(m_nb_joints, 0.0);
        m_measured_js.Name().SetSize(m_nb_joints);
        m_configuration_js.Name().SetSize(m_nb_joints);
        std::stringstream jointName;
        for (size_t index = 0; index < m_nb_joints; ++index) {
            jointName.str("");
            jointName << "SUJ_" << name << "_J" << index;
            m_measured_js.Name().at(index) = jointName.str();
            m_configuration_js.Name().at(index) = jointName.str();
        }
        m_live_measured_js.Position().ForceAssign(m_measured_js.Position());
        m_live_measured_js.Name().ForceAssign(m_measured_js.Name());

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

        m_state_table.AddData(m_voltages[0], "PrimaryVoltage");
        m_state_table.AddData(m_voltages[1], "SecondaryVoltage");
        m_state_table.AddData(m_measured_js, "measured_js");
        m_state_table.AddData(m_live_measured_js, "live_measured_js");
        m_state_table.AddData(m_configuration_js, "configuration_js");

        m_measured_cp.SetReferenceFrame("Cart");
        m_measured_cp.SetMovingFrame(name + "_base");
        m_state_table.AddData(m_measured_cp, "measured_cp");

        m_local_measured_cp.SetReferenceFrame("Cart");
        m_local_measured_cp.SetMovingFrame(name + "_base");
        m_state_table.AddData(m_local_measured_cp, "local/measured_cp");

        m_state_table_configuration.AddData(m_name, "name");
        m_state_table_configuration.AddData(m_serial_number, "serial_number");

        CMN_ASSERT(interfaceProvided);
        m_interface_provided = interfaceProvided;
        // read commands
        m_interface_provided->AddCommandReadState(m_state_table, m_measured_js, "measured_js");
        m_interface_provided->AddCommandReadState(m_state_table, m_live_measured_js, "live/measured_js");
        m_interface_provided->AddCommandReadState(m_state_table, m_configuration_js, "configuration_js");
        m_interface_provided->AddCommandReadState(m_state_table, m_measured_cp,
                                                  "measured_cp");
        m_interface_provided->AddCommandReadState(m_state_table, m_local_measured_cp,
                                                  "local/measured_cp");
        m_interface_provided->AddCommandReadState(m_state_table, m_voltages[0], "GetVoltagesPrimary");
        m_interface_provided->AddCommandReadState(m_state_table, m_voltages[1], "GetVoltagesSecondary");
        m_interface_provided->AddCommandReadState(m_state_table_configuration, m_name, "GetName");
        m_interface_provided->AddCommandReadState(m_state_table_configuration, m_serial_number, "GetSerialNumber");

        // write commands
        m_interface_provided->AddCommandWrite(&mtsIntuitiveResearchKitSUJSiArmData::clutch_command, this,
                                              "Clutch", false);
        m_interface_provided->AddCommandWrite(&mtsIntuitiveResearchKitSUJSiArmData::calibrate_potentiometers, this,
                                              "SetRecalibrationMatrix", m_recalibration_matrix);

        // cartesian position events
        // m_base_frame is send everytime the mux has found all joint values
        m_interface_provided->AddEventWrite(EventPositionCartesian, "PositionCartesian", prmPositionCartesianGet());
        m_interface_provided->AddEventWrite(EventPositionCartesianLocal, "PositionCartesianLocal", prmPositionCartesianGet());

        // Events
        m_interface_provided->AddEventWrite(state_events.current_state, "current_state", std::string(""));
        m_interface_provided->AddEventWrite(state_events.desired_state, "desired_state", std::string(""));
        m_interface_provided->AddEventWrite(state_events.operating_state, "operating_state", prmOperatingState());
        m_interface_provided->AddMessageEvents();
        // Stats
        m_interface_provided->AddCommandReadState(m_state_table, m_state_table.PeriodStats,
                                                  "period_statistics");

        CMN_ASSERT(interfaceRequired);
        m_interface_required = interfaceRequired;
        m_interface_required->AddFunction("set_base_frame", m_arm_set_base_frame);
        m_interface_required->AddFunction("local/measured_cp", m_get_local_measured_cp);
    }


    inline void clutch_callback(const prmEventButton & button)
    {
        if (button.Type() == prmEventButton::PRESSED) {
            // clutch is pressed, arm is moving around and we know the pots are slow, we mark position as invalid
            m_interface_provided->SendStatus(m_name + ": SUJ clutched");
            m_measured_cp.SetTimestamp(m_measured_js.Timestamp());
            m_measured_cp.SetValid(false);
            EventPositionCartesian(m_measured_cp);
            m_local_measured_cp.SetTimestamp(m_measured_js.Timestamp());
            m_local_measured_cp.SetValid(false);
            EventPositionCartesianLocal(m_local_measured_cp);
        } else if (button.Type() == prmEventButton::RELEASED) {
            m_interface_provided->SendStatus(m_name + ": SUJ not clutched");
            m_waiting_for_live = true;
        }
    }


    inline void clutch_command(const bool & clutch)
    {
        prmEventButton button;
        if (clutch) {
            button.SetType(prmEventButton::PRESSED);
        } else {
            button.SetType(prmEventButton::RELEASED);
        }
        clutch_callback(button);
    }


    inline void calibrate_potentiometers(const vctMat & mat)
    {
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
            m_new_joint_scales[0][col] = deltaJointPosition / deltaPrimaryVoltage;
            m_new_joint_scales[1][col] = deltaJointPosition / deltaSecondaryVoltage;

            m_new_joint_offsets[0][col] = mat.Element(0, col) - mat.Element(1, col) * m_new_joint_scales[0][col];
            m_new_joint_offsets[1][col] = mat.Element(0, col) - mat.Element(2, col) * m_new_joint_scales[1][col];
        }

        std::cerr << "SUJ scales and offsets for arm: " << m_name << std::endl
                  << "Please update your suj.json file using these values" << std::endl
                  << "\"primary-offsets\": [ "
                  << m_new_joint_offsets[0][0] << ", "
                  << m_new_joint_offsets[0][1] << ", "
                  << m_new_joint_offsets[0][2] << ", "
                  << m_new_joint_offsets[0][3] << "],"  << std::endl
                  << "\"primary-scales\": [ "
                  << m_new_joint_scales[0][0] << ", "
                  << m_new_joint_scales[0][1] << ", "
                  << m_new_joint_scales[0][2] << ", "
                  << m_new_joint_scales[0][3] << "],"  << std::endl
                  << "\"secondary-offsets\": [ "
                  << m_new_joint_offsets[1][0] << ", "
                  << m_new_joint_offsets[1][1] << ", "
                  << m_new_joint_offsets[1][2] << ", "
                  << m_new_joint_offsets[1][3] << "]," << std::endl
                  << "\"secondary-scales\": [ "
                  << m_new_joint_scales[1][0] << ", "
                  << m_new_joint_scales[1][1] << ", "
                  << m_new_joint_scales[1][2] << ", "
                  << m_new_joint_scales[1][3] << "],"  << std::endl;
    }


    // name of this SUJ arm (ECM, PSM1, ...)
    std::string m_name;
    // serial number
    std::string m_serial_number;

    // number of joints for this arm
    size_t m_nb_joints;

    // index of prismatic pot on base arduino
    size_t m_base_arduino_pot_index;

    // interfaces
    mtsInterfaceProvided * m_interface_provided = nullptr;
    mtsInterfaceRequired * m_interface_required = nullptr;

    // state of this SUJ arm
    mtsStateTable m_state_table; // for positions, fairly slow, i.e 12 * delay for a2d
    mtsStateTable m_state_table_configuration; // changes only at config and if recalibrate

    // 2 arrays, one for each set of potentiometers
    vctDoubleVec m_voltages[2];
    vctDoubleVec m_positions[2];
    vctDoubleVec m_delta_measured_js;
    bool m_pots_agree;
    bool m_waiting_for_live = true;

    vctDoubleVec m_voltage_to_position_scales[2];
    vctDoubleVec m_voltage_to_position_offsets[2];
    prmStateJoint m_measured_js, m_live_measured_js;
    prmConfigurationJoint m_configuration_js;

    // kinematics
    bool m_need_update_forward_kinemactics = false;
    prmPositionCartesianGet m_measured_cp;
    prmPositionCartesianGet m_local_measured_cp;

    robManipulator m_manipulator;
    vctMat m_recalibration_matrix;
    vctDoubleVec m_new_joint_scales[2];
    vctDoubleVec m_new_joint_offsets[2];

    // setup transformations from json file
    vctFrame4x4<double> m_world_to_SUJ;
    vctFrame4x4<double> m_SUJ_to_arm_base;

    // base frame
    mtsFunctionWrite m_arm_set_base_frame;
    // for reference arm only, get current position
    mtsFunctionRead m_get_local_measured_cp;

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
    m_state_machine(componentName, "DISABLED")
{
    init();
}


mtsIntuitiveResearchKitSUJSi::mtsIntuitiveResearchKitSUJSi(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    m_state_machine(arg.Name, "DISABLED")
{
    init();
}


void mtsIntuitiveResearchKitSUJSi::init(void)
{
    // initialize arm pointers
    m_sarms.SetAll(nullptr);

    // configure state machine common to all arms (ECM/MTM/PSM)
    // possible states
    m_state_machine.AddState("ENABLED");

    // possible desired states
    m_state_machine.AddAllowedDesiredState("DISABLED");
    m_state_machine.AddAllowedDesiredState("ENABLED");

    // state change, to convert to string events for users (Qt, ROS)
    m_state_machine.SetStateChangedCallback(&mtsIntuitiveResearchKitSUJSi::state_changed,
                                            this);

    // run for all states
    m_state_machine.SetRunCallback(&mtsIntuitiveResearchKitSUJSi::run_all_states,
                                   this);

    // disabled
    m_state_machine.SetEnterCallback("DISABLED",
                                     &mtsIntuitiveResearchKitSUJSi::enter_DISABLED,
                                     this);

    m_state_machine.SetTransitionCallback("DISABLED",
                                          &mtsIntuitiveResearchKitSUJSi::transition_DISABLED,
                                          this);

    // enabled
    m_state_machine.SetEnterCallback("ENABLED",
                                     &mtsIntuitiveResearchKitSUJSi::enter_ENABLED,
                                     this);

    m_state_machine.SetTransitionCallback("ENABLED",
                                          &mtsIntuitiveResearchKitSUJSi::transition_ENABLED,
                                          this);

    m_operating_state.SetValid(true);
    m_operating_state.SetState(prmOperatingState::DISABLED);
    m_operating_state.SetIsHomed(true);
    StateTable.AddData(m_operating_state, "operating_state");

    m_interface = AddInterfaceProvided("Arm");
    if (m_interface) {
        // Arm State
        m_interface->AddCommandWrite(&mtsIntuitiveResearchKitSUJSi::state_command,
                                     this, "state_command", std::string(""));
        m_interface->AddCommandReadState(StateTable,
                                         m_operating_state, "operating_state");
        // Events
        m_interface->AddMessageEvents();
        m_interface->AddEventWrite(state_events.operating_state, "operating_state", prmOperatingState());
        // Stats
        m_interface->AddCommandReadState(StateTable, StateTable.PeriodStats,
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
    std::string reference_sarm_name = "ECM";
    const Json::Value jsonReferenceArm = jsonConfig["reference-arm"];
    if (!jsonReferenceArm.isNull()) {
        reference_sarm_name = jsonReferenceArm.asString();
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

    mtsIntuitiveResearchKitSUJSiArmData * sarm;
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
        sarm = new mtsIntuitiveResearchKitSUJSiArmData(name, mac,
                                                       interfaceProvided,
                                                       interfaceRequired);
        m_sarms[index] = sarm;

        // save which arm is the Reference Arm
        if (name == reference_sarm_name) {
            m_reference_arm_index = index;
        }

        // Arm State so GUI widget for each arm can set/get state
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitSUJSi::state_command,
                                           this, "state_command", std::string(""));

        // create a required interface for each arm to handle clutch button
        std::string itf = "SUJ-Clutch-" + name;
        mtsInterfaceRequired * requiredInterface = this->AddInterfaceRequired(itf, MTS_OPTIONAL);
        if (requiredInterface) {
            requiredInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitSUJSiArmData::clutch_callback, sarm,
                                                    "Button");
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: can't add required interface \"" << itf
                                     << "\" for SUJ \"" << name << "\" clutch button"
                                     << std::endl;
            exit(EXIT_FAILURE);
        }

        // find serial number
        sarm->m_serial_number = jsonArm["serial-number"].asString();

        // read pot settings
        sarm->m_state_table_configuration.Start();
        cmnDataJSON<vctDoubleVec>::DeSerializeText(sarm->m_voltage_to_position_offsets[0], jsonArm["primary-offsets"]);
        cmnDataJSON<vctDoubleVec>::DeSerializeText(sarm->m_voltage_to_position_offsets[1], jsonArm["secondary-offsets"]);
        cmnDataJSON<vctDoubleVec>::DeSerializeText(sarm->m_voltage_to_position_scales[0], jsonArm["primary-scales"]);
        cmnDataJSON<vctDoubleVec>::DeSerializeText(sarm->m_voltage_to_position_scales[1], jsonArm["secondary-scales"]);
        const size_t nb_joints = NB_JOINTS.at(name);
        for (auto vec : sarm->m_voltage_to_position_offsets) {
            if (vec.size() != nb_joints) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: incorrect number of voltage to position offsets for \""
                                         << name << "\", expected " << nb_joints
                                         << " but found " << vec.size() << " elements" << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        for (auto vec : sarm->m_voltage_to_position_scales) {
            if (vec.size() != nb_joints) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: incorrect number of voltage to position scales for \""
                                         << name << "\", expected " << nb_joints
                                         << " but found " << vec.size() << " elements" << std::endl;
                exit(EXIT_FAILURE);
            }
        }
        sarm->m_state_table_configuration.Advance();

        // DH and transforms should be loaded from the share/kinematic
        // directory bit if DH is defined in this scope, assumes user
        // want to provide DH and base/tip transforms
        Json::Value jsonDH = jsonArm["DH"];
        if (!jsonDH.empty()) {
            // look for DH
            sarm->m_manipulator.LoadRobot(jsonArm["DH"]);
            if (sarm->m_manipulator.links.size() != (nb_joints + 2)) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: incorrect kinematic chain lenght for \""
                                         << name << "\", expected " << nb_joints + 2
                                         << " but found " << sarm->m_manipulator.links.size() << " elements" << std::endl;
                exit(EXIT_FAILURE);

            }
            // read setup transforms
            vctFrm3 transform;
            cmnDataJSON<vctFrm3>::DeSerializeText(transform, jsonArm["world-origin-to-suj"]);
            sarm->m_world_to_SUJ.From(transform);
            cmnDataJSON<vctFrm3>::DeSerializeText(transform, jsonArm["suj-tip-to-tool-origin"]);
            sarm->m_SUJ_to_arm_base.From(transform);
        }
    }
}


void mtsIntuitiveResearchKitSUJSi::update_operating_state_and_busy(const prmOperatingState::StateType & state,
                                                                   const bool isBusy)
{
    m_operating_state.State() = state;
    m_operating_state.IsBusy() = isBusy;
    dispatch_operating_state();
}


void mtsIntuitiveResearchKitSUJSi::state_changed(void)
{
    dispatch_status("current state " + m_state_machine.CurrentState());
    dispatch_operating_state();
}


void mtsIntuitiveResearchKitSUJSi::run_all_states(void)
{
    start_state_tables();
    // get robot data, i.e. process mux/pots
    get_robot_data();
    // update all forward kinematics
    update_forward_kinematics();
    advance_state_tables();
}


void mtsIntuitiveResearchKitSUJSi::enter_DISABLED(void)
{
    update_operating_state_and_busy(prmOperatingState::DISABLED, false);
}


void mtsIntuitiveResearchKitSUJSi::transition_DISABLED(void)
{
    if (m_state_machine.DesiredStateIsNotCurrent()) {
        m_state_machine.SetCurrentState("ENABLED");
    }
}


void mtsIntuitiveResearchKitSUJSi::enter_ENABLED(void)
{
    update_operating_state_and_busy(prmOperatingState::ENABLED, false);
}


void mtsIntuitiveResearchKitSUJSi::transition_ENABLED(void)
{
    // move to next stage if desired state is different
    if (m_state_machine.DesiredStateIsNotCurrent()) {
        m_state_machine.SetCurrentState(m_state_machine.DesiredState());
    }
}


void mtsIntuitiveResearchKitSUJSi::Startup(void)
{
    set_desired_state("DISABLED");
}


void mtsIntuitiveResearchKitSUJSi::Run(void)
{
    // collect data from required interfaces
    ProcessQueuedEvents();
    try {
        m_state_machine.Run();
    } catch (std::exception & e) {
        dispatch_error("in state " + m_state_machine.CurrentState()
                       + ", caught exception \"" + e.what() + "\"");
        set_desired_state("DISABLED");
    }
    // trigger ExecOut event
    RunEvent();
    ProcessQueuedCommands();
}


void mtsIntuitiveResearchKitSUJSi::Cleanup(void)
{
}


void mtsIntuitiveResearchKitSUJSi::set_simulated(void)
{
    dispatch_error("simulated mode not supported on SUJ Si");
}


void mtsIntuitiveResearchKitSUJSi::start_state_tables(void)
{
    for (auto sarm : m_sarms) {
        if (sarm != nullptr) {
            sarm->m_state_table.Start();
        }
    }
}


void mtsIntuitiveResearchKitSUJSi::advance_state_tables(void)
{
    for (auto sarm : m_sarms) {
        if (sarm != nullptr) {
            sarm->m_state_table.Advance();
        }
    }
}


void mtsIntuitiveResearchKitSUJSi::get_robot_data(void)
{
    // update pot values from base arduino
    if (m_base_arduino) {
        m_base_arduino->update_raw_pots();
    }

    for (auto sarm : m_sarms) {
        if (sarm != nullptr) {
            // see if we can get updated pot values
            if (sarm->update_raw_pots()) {

                // first joint comes from base arduino
                if (m_base_arduino && m_base_arduino->m_connected) {
                    sarm->m_voltages[0].at(0) = m_base_arduino->m_raw_pots.Row(0).at(sarm->m_base_arduino_pot_index);
                    sarm->m_voltages[1].at(0) = m_base_arduino->m_raw_pots.Row(1).at(sarm->m_base_arduino_pot_index);
                }

                // last 3 to 4 joints come from this arm's arduino
                size_t joints_to_copy = sarm->m_nb_joints - 1;
                sarm->m_voltages[0].Ref(joints_to_copy, 1) = sarm->m_raw_pots.Row(0).Ref(joints_to_copy);
                sarm->m_voltages[1].Ref(joints_to_copy, 1) = sarm->m_raw_pots.Row(1).Ref(joints_to_copy);

                // convert to SI
                sarm->m_positions[0].Assign(sarm->m_voltage_to_position_offsets[0]);
                sarm->m_positions[0].AddElementwiseProductOf(sarm->m_voltage_to_position_scales[0], sarm->m_voltages[0]);
                sarm->m_positions[1].Assign(sarm->m_voltage_to_position_offsets[1]);
                sarm->m_positions[1].AddElementwiseProductOf(sarm->m_voltage_to_position_scales[1], sarm->m_voltages[1]);

                // compare primary and secondary pots when arm is not clutched
                const double angleTolerance = 2.0 * cmnPI_180;
                const double distanceTolerance = 3.0 * cmn_mm;
                sarm->m_delta_measured_js.DifferenceOf(sarm->m_positions[0], sarm->m_positions[1]);
                if ((sarm->m_delta_measured_js[0] > distanceTolerance) ||
                    (sarm->m_delta_measured_js.Ref(3, 1).MaxAbsElement() > angleTolerance)) {
                    // send messages if this is new
                    if (sarm->m_pots_agree) {
                        m_interface->SendWarning(sarm->m_name + " SUJ: primary and secondary potentiometers don't seem to agree.");
                        CMN_LOG_CLASS_RUN_WARNING << "get_and_convert_potentiometers, error: " << std::endl
                                                  << " - " << this->GetName() << ": " << sarm->m_name << std::endl
                                                  << " - primary:   " << sarm->m_positions[0] << std::endl
                                                  << " - secondary: " << sarm->m_positions[1] << std::endl;
                        sarm->m_pots_agree = false;
                    }
                } else {
                    if (!sarm->m_pots_agree) {
                        m_interface->SendStatus(sarm->m_name + " SUJ: primary and secondary potentiometers agree.");
                        CMN_LOG_CLASS_RUN_VERBOSE << "get_and_convert_potentiometers recovery" << std::endl
                                                  << " - " << this->GetName() << ": " << sarm->m_name << std::endl;
                        sarm->m_pots_agree = true;
                    }
                }

                // use average of positions reported by potentiometers
                sarm->m_live_measured_js.Position().SumOf(sarm->m_positions[0],
                                                          sarm->m_positions[1]);
                sarm->m_live_measured_js.Position().Divide(2.0);
                sarm->m_live_measured_js.SetValid(true);

                if (sarm->m_waiting_for_live) {
                    // copy live jp
                    sarm->m_measured_js.Position().Assign(sarm->m_live_measured_js.Position());
                    sarm->m_measured_js.SetValid(true);
                    sarm->m_measured_js.SetTimestamp(mtsComponentManager::GetInstance()->GetTimeServer().GetRelativeTime());
                    sarm->m_waiting_for_live = false;
                    sarm->m_need_update_forward_kinemactics = true;
                }
            }
        }
    }
}


void mtsIntuitiveResearchKitSUJSi::update_forward_kinematics(void)
{
    // first update all the local_measured_cp if all arms are ready
    for (auto sarm : m_sarms) {
        if (sarm != nullptr) {
            if (sarm->m_measured_js.Valid()) {
                if (sarm->m_need_update_forward_kinemactics) {
                    sarm->m_need_update_forward_kinemactics = false;
                    // forward kinematic
                    vctDoubleVec jp(sarm->m_manipulator.links.size(), 0.0);
                    jp.Ref(sarm->m_measured_js.Position().size()).Assign(sarm->m_measured_js.Position());
                    vctFrm4x4 dh_cp = sarm->m_manipulator.ForwardKinematics(jp);
                    // pre and post transformations loaded from JSON file, base frame updated using events
                    vctFrm4x4 local_cp = sarm->m_world_to_SUJ * dh_cp * sarm->m_SUJ_to_arm_base;
                    // update local only
                    sarm->m_local_measured_cp.Position().From(local_cp);
                    sarm->m_local_measured_cp.SetTimestamp(sarm->m_measured_js.Timestamp());
                    sarm->m_local_measured_cp.SetValid(true);
                    sarm->EventPositionCartesianLocal(sarm->m_local_measured_cp);
                    sarm->m_interface_provided->SendStatus(sarm->m_name + " SUJ: measured_cp updated");
                }
            } else {
                sarm->m_local_measured_cp.SetValid(false);
            }
        }
    }

    // find the reference arm (usually ECM)
    prmPositionCartesianGet reference_arm_local_cp;
    prmPositionCartesianGet reference_arm_to_cart_cp;

    mtsIntuitiveResearchKitSUJSiArmData * reference_sarm = m_sarms[m_reference_arm_index];
    if (! (reference_sarm->m_get_local_measured_cp(reference_arm_local_cp))) {
        // interface not connected, reporting wrt cart
        reference_arm_to_cart_cp.Position().Assign(vctFrm3::Identity());
        reference_arm_to_cart_cp.SetValid(true);
        reference_arm_to_cart_cp.SetReferenceFrame("Cart");
    } else {
        // get position from BaseFrameArm and convert to useful type
        vctFrm3 cart_to_reference_arm_cp = reference_sarm->m_local_measured_cp.Position() * reference_arm_local_cp.Position();
        // compute and send new base frame for all SUJs (SUJ will handle BaseFrameArm differently)
        reference_arm_to_cart_cp.Position().From(cart_to_reference_arm_cp.Inverse());
        // it's an inverse, swap moving and reference frames
        reference_arm_to_cart_cp.SetReferenceFrame(reference_arm_local_cp.MovingFrame());
        reference_arm_to_cart_cp.SetMovingFrame(reference_sarm->m_local_measured_cp.ReferenceFrame());
        // valid only if both are valid
        reference_arm_to_cart_cp.SetValid(reference_sarm->m_local_measured_cp.Valid()
                                          && reference_arm_local_cp.Valid());
        // take most recent timestamp
        reference_arm_to_cart_cp.SetTimestamp(std::max(reference_sarm->m_local_measured_cp.Timestamp(),
                                                       reference_arm_local_cp.Timestamp()));
    }
    // reference sarm measured_cp is always with respect to cart, same as local
    reference_sarm->m_measured_cp.Position().Assign(reference_sarm->m_local_measured_cp.Position());
    reference_sarm->m_measured_cp.SetValid(reference_sarm->m_local_measured_cp.Valid());
    reference_sarm->m_measured_cp.SetTimestamp(reference_sarm->m_local_measured_cp.Timestamp());

    // update other arms
    vctFrm4x4 reference_frame(reference_arm_to_cart_cp.Position());
    vctFrm4x4 local_cp, cp;
    for (size_t arm_index = 0; arm_index < 4; ++arm_index) {
        mtsIntuitiveResearchKitSUJSiArmData * sarm = m_sarms[arm_index];
        // update positions with base frame, local positions are only
        // updated from FK when joints are ready
        if (arm_index != m_reference_arm_index) {
            sarm->m_measured_cp.SetReferenceFrame(reference_arm_to_cart_cp.ReferenceFrame());
            local_cp.From(sarm->m_local_measured_cp.Position());
            cp = reference_frame * local_cp;
            // - with base frame
            sarm->m_measured_cp.Position().From(cp);
            sarm->m_measured_cp.SetValid(reference_sarm->m_local_measured_cp.Valid()
                                         && reference_arm_local_cp.Valid());
            sarm->m_measured_cp.SetTimestamp(std::max(reference_sarm->m_local_measured_cp.Timestamp(),
                                                      sarm->m_local_measured_cp.Timestamp()));
            sarm->EventPositionCartesian(sarm->m_measured_cp);
            // - set base frame for the arm
            prmPositionCartesianSet setpoint_cp;
            setpoint_cp.Goal().Assign(sarm->m_measured_cp.Position());
            setpoint_cp.SetValid(sarm->m_measured_cp.Valid());
            setpoint_cp.SetTimestamp(sarm->m_measured_cp.Timestamp());
            setpoint_cp.SetReferenceFrame(sarm->m_measured_cp.ReferenceFrame());
            setpoint_cp.SetMovingFrame(sarm->m_measured_cp.MovingFrame());
            sarm->m_arm_set_base_frame(setpoint_cp);
        }
    }
}


void mtsIntuitiveResearchKitSUJSi::set_desired_state(const std::string & state)
{
    // setting desired state triggers a new event so user nows which state is current
    dispatch_operating_state();
    // try to find the state in state machine
    if (!m_state_machine.StateExists(state)) {
        dispatch_error("unsupported state " + state);
        return;
    }
    // try to set the desired state
    try {
        m_state_machine.SetDesiredState(state);
    } catch (...) {
        dispatch_error(state + " is not an allowed desired state");
        return;
    }

    dispatch_operating_state();
    dispatch_status("desired state " + state);

    // state transitions with direct transitions
    if (state == "DISABLED") {
        m_state_machine.SetCurrentState(state);
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
                set_desired_state("ENABLED");
                return;
            }
            if (command == "disable") {
                set_desired_state("DISABLED");
                return;
            }
            if (command == "home") {
                set_desired_state("ENABLED");
                set_homed(true);
                return;
            }
            if (command == "unhome") {
                set_homed(false);
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
            dispatch_warning(humanReadableMessage);
        }
    } catch (std::runtime_error & e) {
        dispatch_warning(command + " doesn't seem to be a valid state_command (" + e.what() + ")");
    }
}


void mtsIntuitiveResearchKitSUJSi::set_homed(const bool homed)
{
    if (homed != m_operating_state.IsHomed()) {
        m_operating_state.IsHomed() = homed;
        dispatch_operating_state();
    }
}


void mtsIntuitiveResearchKitSUJSi::dispatch_error(const std::string & message)
{
    m_interface->SendError(this->GetName() + ": " + message);
    for (auto sarm : m_sarms) {
        sarm->m_interface_provided->SendError(sarm->m_name + " SUJ: " + message);
    }
}


void mtsIntuitiveResearchKitSUJSi::dispatch_warning(const std::string & message)
{
    m_interface->SendWarning(this->GetName() + ": " + message);
    for (auto sarm : m_sarms) {
        sarm->m_interface_provided->SendWarning(sarm->m_name + " SUJ: " + message);
    }
}


void mtsIntuitiveResearchKitSUJSi::dispatch_status(const std::string & message)
{
    m_interface->SendStatus(this->GetName() + ": " + message);
    for (auto sarm : m_sarms) {
        sarm->m_interface_provided->SendStatus(sarm->m_name + " SUJ: " + message);
    }
}


void mtsIntuitiveResearchKitSUJSi::dispatch_operating_state(void)
{
    state_events.operating_state(m_operating_state);
    for (auto sarm : m_sarms) {
        sarm->state_events.operating_state(m_operating_state);
        sarm->state_events.current_state(m_state_machine.CurrentState());
        sarm->state_events.desired_state(m_state_machine.DesiredState());
    }
}
