/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Youri Tan
  Created on: 2014-11-07

  (C) Copyright 2014-2024 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitSUJ.h>
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

const size_t MUX_ARRAY_SIZE = 6;
const size_t MUX_MAX_INDEX = 15;

// empirical value, tradeoff between speed and stability of analog
// input
const size_t ANALOG_SAMPLE_NUMBER = 60;

// DO NOT set value below 3, this value might go down when the
// QLA/dSIB are properly grounded.
const size_t NUMBER_OF_MUX_CYCLE_BEFORE_STABLE = 3;


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitSUJ, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg)

class mtsIntuitiveResearchKitSUJArmData
{
public:

    typedef enum {SUJ_UNDEFINED, SUJ_PSM, SUJ_ECM, SUJ_MOTORIZED_PSM} SujType;

    inline mtsIntuitiveResearchKitSUJArmData(const std::string & name,
                                             const SujType type,
                                             const unsigned int plugNumber,
                                             const bool simulated,
                                             mtsInterfaceProvided * interfaceProvided,
                                             mtsInterfaceRequired * interfaceRequired):
        m_name(name),
        m_type(type),
        m_plug_number(plugNumber),
        m_simulated(simulated),
        m_state_table(500, name),
        m_state_table_configuration(100, name + "Configuration"),
        m_state_table_brake_current(100, name + "BrakeCurrent")
    {
        // recalibration matrix
        m_recalibration_matrix.SetSize(6, 6);
        m_recalibration_matrix.Zeros();
        m_new_joint_scales[0].SetSize(6);
        m_new_joint_scales[0].Zeros();
        m_new_joint_scales[1].SetSize(6);
        m_new_joint_scales[1].Zeros();

        m_new_joint_offsets[0].SetSize(6);
        m_new_joint_offsets[0].Zeros();
        m_new_joint_offsets[1].SetSize(6);
        m_new_joint_offsets[1].Zeros();

        // state table doesn't always advance, only when values are changed
        m_state_table_configuration.SetAutomaticAdvance(false);

        for (size_t potArray = 0; potArray < 2; ++potArray) {
            m_voltages[potArray].SetSize(MUX_ARRAY_SIZE);
            m_positions[potArray].SetSize(MUX_ARRAY_SIZE);
            m_voltage_to_position_scales[potArray].SetSize(MUX_ARRAY_SIZE);
            m_voltage_to_position_offsets[potArray].SetSize(MUX_ARRAY_SIZE);
        }
        m_delta_measured_js.SetSize(MUX_ARRAY_SIZE);
        m_voltages_extra.SetSize(MUX_MAX_INDEX - 2 * MUX_ARRAY_SIZE + 1);

        m_measured_js.Position().SetSize(MUX_ARRAY_SIZE);
        m_measured_js.Name().resize(MUX_ARRAY_SIZE);
        m_configuration_js.Name().resize(MUX_ARRAY_SIZE);
        std::stringstream jointName;
        for (size_t index = 0; index < MUX_ARRAY_SIZE; ++index) {
            jointName.str("");
            jointName << "SUJ_" << name << "_J" << index;
            m_measured_js.Name().at(index) = jointName.str();
            m_configuration_js.Name().at(index) = jointName.str();
        }

        m_configuration_js.Type().SetSize(MUX_ARRAY_SIZE);
        m_configuration_js.Type().SetAll(CMN_JOINT_REVOLUTE);
        m_configuration_js.Type().at(0) = CMN_JOINT_PRISMATIC;
        // joint limits are only used in simulation mode to we can set
        // arbitrarily wide limits
        m_configuration_js.PositionMin().SetSize(MUX_ARRAY_SIZE);
        m_configuration_js.PositionMax().SetSize(MUX_ARRAY_SIZE);
        m_configuration_js.PositionMin().SetAll(-2.0 * cmnPI);
        m_configuration_js.PositionMax().SetAll( 2.0 * cmnPI);
        m_configuration_js.PositionMin().at(0) = -2.0 * cmn_m;
        m_configuration_js.PositionMax().at(0) =  2.0 * cmn_m;

        m_state_table.AddData(m_voltages[0], "PrimaryVoltage");
        m_state_table.AddData(m_voltages[1], "SecondaryVoltage");
        m_state_table.AddData(m_voltages_extra, "VoltagesExtra");
        m_state_table.AddData(m_measured_js, "measured_js");
        m_state_table.AddData(m_configuration_js, "configuration_js");

        m_measured_cp.SetReferenceFrame("Cart");
        m_measured_cp.SetMovingFrame(name + "_base");
        m_state_table.AddData(m_measured_cp, "measured_cp");

        m_local_measured_cp.SetReferenceFrame("Cart");
        m_local_measured_cp.SetMovingFrame(name + "_base");
        m_state_table.AddData(m_local_measured_cp, "local/measured_cp");

        m_state_table_configuration.AddData(m_name, "name");
        m_state_table_configuration.AddData(m_serial_number, "serial_number");
        m_state_table_configuration.AddData(m_plug_number, "plug_number");
        m_state_table_brake_current.AddData(m_brake_desired_current, "BrakeCurrent");

        CMN_ASSERT(interfaceProvided);
        m_interface_provided = interfaceProvided;
        // read commands
        m_interface_provided->AddCommandReadState(m_state_table, m_measured_js, "measured_js");
        m_interface_provided->AddCommandReadState(m_state_table, m_configuration_js, "configuration_js");
        // set position is only for simulation, allows both servo and move
        m_interface_provided->AddCommandWrite(&mtsIntuitiveResearchKitSUJArmData::servo_jp,
                                              this, "servo_jp");
        m_interface_provided->AddCommandWrite(&mtsIntuitiveResearchKitSUJArmData::servo_jp,
                                              this, "move_jp");
        m_interface_provided->AddCommandReadState(m_state_table, m_measured_cp,
                                                  "measured_cp");
        m_interface_provided->AddCommandReadState(m_state_table, m_local_measured_cp,
                                                  "local/measured_cp");
        m_interface_provided->AddCommandReadState(m_state_table, m_voltages[0], "GetVoltagesPrimary");
        m_interface_provided->AddCommandReadState(m_state_table, m_voltages[1], "GetVoltagesSecondary");
        m_interface_provided->AddCommandReadState(m_state_table, m_voltages_extra, "GetVoltagesExtra");
        m_interface_provided->AddCommandReadState(m_state_table_configuration, m_name, "GetName");
        m_interface_provided->AddCommandReadState(m_state_table_configuration, m_serial_number, "GetSerialNumber");
        m_interface_provided->AddCommandReadState(m_state_table_configuration, m_plug_number, "GetPlugNumber");
        m_interface_provided->AddCommandReadState(m_state_table_brake_current, m_brake_desired_current, "GetBrakeCurrent");

        // write commands
        m_interface_provided->AddCommandWrite(&mtsIntuitiveResearchKitSUJArmData::clutch_command, this,
                                              "clutch", false);
        m_interface_provided->AddCommandWrite(&mtsIntuitiveResearchKitSUJArmData::calibrate_potentiometers, this,
                                              "SetRecalibrationMatrix", m_recalibration_matrix);

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


    inline void clutch_callback(const prmEventButton & button) {
        if (button.Type() == prmEventButton::PRESSED) {
            m_clutched += 1;
            if (m_clutched == 1) {
                // clutch is pressed, arm is moving around and we know the pots are slow, we mark position as invalid
                m_interface_provided->SendStatus(m_name + " SUJ: clutched");
                m_measured_js.SetValid(false);
                m_measured_cp.SetTimestamp(m_measured_js.Timestamp());
                m_measured_cp.SetValid(false);
                m_local_measured_cp.SetTimestamp(m_measured_js.Timestamp());
                m_local_measured_cp.SetValid(false);
            }
        } else {
            // first event to release (physical button or GUI) forces release
            m_clutched = 0;
            m_waiting_for_live = true;
            m_interface_provided->SendStatus(m_name + " SUJ: not clutched");
        }
    }


    inline void servo_jp(const prmPositionJointSet & newPosition)
    {
        if (!m_simulated) {
            m_interface_provided->SendWarning(m_name + " SUJ: servo_jp can't be used unless the SUJs are in simulated mode");
            return;
        }
        // save the desired position
        m_measured_js.Position().Assign(newPosition.Goal());
        m_measured_js.SetValid(true);
        m_measured_js.SetTimestamp(newPosition.Timestamp());
        m_need_update_forward_kinemactics = true;
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
        for (size_t col = 0; col < 6; col++) {
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
                  << "\"primary_offsets\": [ "
                  << m_new_joint_offsets[0][0] << ", "
                  << m_new_joint_offsets[0][1] << ", "
                  << m_new_joint_offsets[0][2] << ", "
                  << m_new_joint_offsets[0][3] << ", "
                  << m_new_joint_offsets[0][4] << ", "
                  << m_new_joint_offsets[0][5] << "],"  << std::endl
                  << "\"primary_scales\": [ "
                  << m_new_joint_scales[0][0] << ", "
                  << m_new_joint_scales[0][1] << ", "
                  << m_new_joint_scales[0][2] << ", "
                  << m_new_joint_scales[0][3] << ", "
                  << m_new_joint_scales[0][4] << ", "
                  << m_new_joint_scales[0][5] << "],"  << std::endl
                  << "\"secondary_offsets\": [ "
                  << m_new_joint_offsets[1][0] << ", "
                  << m_new_joint_offsets[1][1] << ", "
                  << m_new_joint_offsets[1][2] << ", "
                  << m_new_joint_offsets[1][3] << ", "
                  << m_new_joint_offsets[1][4] << ", "
                  << m_new_joint_offsets[1][5] << "]," << std::endl
                  << "\"secondary_scales\": [ "
                  << m_new_joint_scales[1][0] << ", "
                  << m_new_joint_scales[1][1] << ", "
                  << m_new_joint_scales[1][2] << ", "
                  << m_new_joint_scales[1][3] << ", "
                  << m_new_joint_scales[1][4] << ", "
                  << m_new_joint_scales[1][5] << "],"  << std::endl;
    }


    // name of this SUJ arm (ECM, PSM1, ...)
    std::string m_name;
    // suj type
    SujType m_type;
    // serial number
    std::string m_serial_number;
    // plug on back of controller, 1 to 4
    unsigned int m_plug_number;

    // simulated or not
    bool m_simulated;

    // interfaces
    mtsInterfaceProvided * m_interface_provided = nullptr;
    mtsInterfaceRequired * m_interface_required = nullptr;

    // state of this SUJ arm
    mtsStateTable m_state_table; // for positions, fairly slow, i.e 12 * delay for a2d
    mtsStateTable m_state_table_configuration; // changes only at config and if recalibrate
    mtsStateTable m_state_table_brake_current; // changes when requested current changes

    // 2 arrays, one for each set of potentiometers
    vctDoubleVec m_voltages[2];
    vctDoubleVec m_positions[2];
    vctDoubleVec m_delta_measured_js;
    bool m_pots_agree = false;
    bool m_waiting_for_live = true;

    vctDoubleVec m_voltage_to_position_scales[2];
    vctDoubleVec m_voltage_to_position_offsets[2];
    prmStateJoint m_measured_js;
    prmConfigurationJoint m_configuration_js;
    // 0 is no, 1 tells we need to send, 2 is for first full mux cycle has started
    unsigned int m_number_of_mux_cycles = 0;

    // kinematics
    bool m_need_update_forward_kinemactics = false;
    prmPositionCartesianGet m_measured_cp;
    prmPositionCartesianGet m_local_measured_cp;

    // exta analog feedback
    // plugs 1-3:  spare1, spare2, brake-voltage, gnd
    // plug 4: I_MOT+, I_MOT-, VA_BIAS, brake-voltage
    vctDoubleVec m_voltages_extra;

    // kinematics
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

    // clutch data
    unsigned int m_clutched = 0;
    double m_brake_desired_current = 0.0;
    double m_brake_release_current = 0.0;
    double m_brake_engaged_current = 0.0;
    double m_brake_direction_current;

    struct {
        mtsFunctionWrite current_state;
        mtsFunctionWrite desired_state;
        mtsFunctionWrite operating_state;
    } state_events;
};


mtsIntuitiveResearchKitSUJ::mtsIntuitiveResearchKitSUJ(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    m_state_machine(componentName, "DISABLED"),
    m_voltage_samples_number(ANALOG_SAMPLE_NUMBER)
{
    init();
}


mtsIntuitiveResearchKitSUJ::mtsIntuitiveResearchKitSUJ(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    m_state_machine(arg.Name, "DISABLED"),
    m_voltage_samples_number(ANALOG_SAMPLE_NUMBER)
{
    init();
}


void mtsIntuitiveResearchKitSUJ::init(void)
{
    // initialize arm pointers
    m_sarms.SetAll(nullptr);

    // configure state machine common to all arms (ECM/MTM/PSM)
    // possible states
    m_state_machine.AddState("POWERING");
    m_state_machine.AddState("ENABLED");

    // possible desired states
    m_state_machine.AddAllowedDesiredState("DISABLED");
    m_state_machine.AddAllowedDesiredState("ENABLED");

    // state change, to convert to string events for users (Qt, ROS)
    m_state_machine.SetStateChangedCallback(&mtsIntuitiveResearchKitSUJ::state_changed,
                                            this);

    // run for all states
    m_state_machine.SetRunCallback(&mtsIntuitiveResearchKitSUJ::run_all_states,
                                   this);

    // disabled
    m_state_machine.SetEnterCallback("DISABLED",
                                     &mtsIntuitiveResearchKitSUJ::enter_DISABLED,
                                     this);

    m_state_machine.SetTransitionCallback("DISABLED",
                                          &mtsIntuitiveResearchKitSUJ::transition_DISABLED,
                                          this);

    // power
    m_state_machine.SetEnterCallback("POWERING",
                                     &mtsIntuitiveResearchKitSUJ::enter_POWERING,
                                     this);

    m_state_machine.SetTransitionCallback("POWERING",
                                          &mtsIntuitiveResearchKitSUJ::transition_POWERING,
                                          this);

    // powered
    m_state_machine.SetEnterCallback("ENABLED",
                                     &mtsIntuitiveResearchKitSUJ::enter_ENABLED,
                                     this);

    m_state_machine.SetRunCallback("ENABLED",
                                   &mtsIntuitiveResearchKitSUJ::run_ENABLED,
                                   this);

    m_state_machine.SetTransitionCallback("ENABLED",
                                          &mtsIntuitiveResearchKitSUJ::transition_ENABLED,
                                          this);

    m_operating_state.SetValid(true);
    m_operating_state.SetState(prmOperatingState::DISABLED);
    m_operating_state.SetIsHomed(true);
    StateTable.AddData(m_operating_state, "operating_state");

    // default values
    m_mux_timer = 0.0;
    m_mux_state.SetSize(4);
    m_voltages.SetSize(4);
    m_brake_currents.SetSize(4);
    m_voltage_samples.SetSize(m_voltage_samples_number);
    m_voltage_samples_counter = 0;

    // Arm IO
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("RobotIO");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("PowerOnSequence", RobotIO.PowerOnSequence);
        interfaceRequired->AddFunction("PowerOffSequence", RobotIO.PowerOffSequence);
        interfaceRequired->AddFunction("GetEncoderChannelA", RobotIO.GetEncoderChannelA);
        interfaceRequired->AddFunction("GetActuatorAmpStatus", RobotIO.GetActuatorAmpStatus);
        interfaceRequired->AddFunction("SetActuatorCurrent", RobotIO.SetActuatorCurrent);
        interfaceRequired->AddFunction("GetAnalogInputVolts", RobotIO.GetAnalogInputVolts);
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitSUJ::error_event_handler, this, "error");
    }
    interfaceRequired = AddInterfaceRequired("NoMuxReset");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetValue", no_mux_reset.GetValue);
        interfaceRequired->AddFunction("SetValue", no_mux_reset.SetValue);
    }
    interfaceRequired = AddInterfaceRequired("MuxIncrement");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetValue", mux_increment.GetValue);
        interfaceRequired->AddFunction("SetValue", mux_increment.SetValue);
    }
    interfaceRequired = AddInterfaceRequired("ControlPWM");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("SetPWMDutyCycle", PWM.SetPWMDutyCycle);
    }
    interfaceRequired = AddInterfaceRequired("DisablePWM");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("SetValue", PWM.DisablePWM);
    }
    interfaceRequired = AddInterfaceRequired("MotorUp");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitSUJ::motor_up_event_handler, this, "Button");
    }
    interfaceRequired = AddInterfaceRequired("MotorDown");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitSUJ::motor_down_event_handler, this, "Button");
    }

    m_interface = AddInterfaceProvided("Arm");
    if (m_interface) {
        // Arm State
        m_interface->AddCommandWrite(&mtsIntuitiveResearchKitSUJ::state_command,
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


void mtsIntuitiveResearchKitSUJ::Configure(const std::string & filename)
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
    const Json::Value jsonReferenceArm = jsonConfig["reference_arm"];
    if (!jsonReferenceArm.isNull()) {
        reference_sarm_name = jsonReferenceArm.asString();
        CMN_LOG_CLASS_INIT_WARNING << "Configure: \"reference_arm\" is user defined.  This should only happen if you are using a PSM to hold a camera.  Most users shouldn't define \"reference_arm\".  If undefined, all arm cartesian positions will be defined with respect to the ECM" << std::endl;
    }

    // find all arms, there should be 4 of them
    const Json::Value jsonArms = jsonConfig["arms"];
    if (jsonArms.size() != 4) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to find 4 SUJ arms" << std::endl;
        exit(EXIT_FAILURE);
    }

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
        std::string typeString = jsonArm["type"].asString();
        mtsIntuitiveResearchKitSUJArmData::SujType type;
        if (typeString == "ECM") {
            type = mtsIntuitiveResearchKitSUJArmData::SUJ_ECM;
        }
        else if (typeString == "PSM") {
            type = mtsIntuitiveResearchKitSUJArmData::SUJ_PSM;
        }
        else if (typeString == "Motorized PSM") {
            type = mtsIntuitiveResearchKitSUJArmData::SUJ_MOTORIZED_PSM;
        }
        else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: incorrect arm type for SUJ \""
                                     << name << "\", must be one of \"PSM\", \"ECM\" or \"Motorized PSM\""
                                     << std::endl;
            exit(EXIT_FAILURE);
        }

        if (!jsonArm["plug_number"]) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: plug_number is missing for SUJ \""
                                     << name << "\", must be an integer between 1 and 4"
                                     << std::endl;
            exit(EXIT_FAILURE);
        }
        unsigned int plugNumber = jsonArm["plug_number"].asInt();
        unsigned int armIndex = plugNumber - 1;

        // add interfaces, one is provided so users can find the SUJ
        // info, the other is required to the SUJ can get position of
        // ECM and change base frame on attached arms
        mtsInterfaceProvided * interfaceProvided = this->AddInterfaceProvided(name);
        mtsInterfaceRequired * interfaceRequired = this->AddInterfaceRequired(name, MTS_OPTIONAL);
        auto sarm = new mtsIntuitiveResearchKitSUJArmData(name, type, plugNumber, m_simulated,
                                                          interfaceProvided, interfaceRequired);
        m_sarms[armIndex] = sarm;
        AddStateTable(&(sarm->m_state_table));
        AddStateTable(&(sarm->m_state_table_brake_current));

        // save which arm is the Reference Arm
        if (name == reference_sarm_name) {
            m_reference_arm_index = armIndex;
        }

        // Arm State so GUI widget for each arm can set/get state
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitSUJ::state_command,
                                           this, "state_command", std::string(""));

        // Add motor up/down for the motorized arm
        if (type == mtsIntuitiveResearchKitSUJArmData::SUJ_MOTORIZED_PSM) {
            interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitSUJ::set_lift_velocity, this,
                                               "set_lift_velocity", 0.0);
        }

        // create a required interface for each arm to handle clutch button
        if (!m_simulated) {
            std::stringstream interfaceName;
            interfaceName << "SUJ_clutch_" << plugNumber;
            mtsInterfaceRequired * requiredInterface = this->AddInterfaceRequired(interfaceName.str());
            if (requiredInterface) {
                requiredInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitSUJArmData::clutch_callback, sarm,
                                                        "Button");
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: can't add required interface for SUJ \""
                                         << name << "\" clutch button because this interface already exists: \""
                                         << interfaceName.str() << "\".  Make sure all arms have a different plug number."
                                         << std::endl;
                exit(EXIT_FAILURE);
            }
        } else {
            // look for hard coded position if available - users can always push new joint values using ROS
            Json::Value jsonPosition = jsonArm["simulated_position"];
            if (!jsonPosition.empty()) {
                vctDoubleVec position;
                cmnDataJSON<vctDoubleVec>::DeSerializeText(position, jsonPosition);
                if (position.size() == sarm->m_measured_js.Position().size()) {
                    sarm->m_measured_js.Position().Assign(position);
                    sarm->m_measured_js.SetValid(true);
                    sarm->m_need_update_forward_kinemactics = true;
                } else {
                    CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to load \"position_simulated\" for \""
                                             << name << "\", expected vector size is "
                                             << sarm->m_measured_js.Position().size() << " but vector in configuration file has "
                                             << position.size() << " element(s)"
                                             << std::endl;
                    exit(EXIT_FAILURE);
                }
            }
        }

        // find serial number
        sarm->m_serial_number = jsonArm["serial_number"].asString();

        // read brake current configuration
        // all math for ramping up/down current is done on positive values
        // negate only when applying
        double brakeCurrent = jsonArm["brake_release_current"].asFloat();
        if (brakeCurrent > 0.0) {
            sarm->m_brake_release_current = brakeCurrent;
            sarm->m_brake_direction_current = 1.0;
        } else {
            sarm->m_brake_release_current = -brakeCurrent;
            sarm->m_brake_direction_current = -1.0;
        }

        brakeCurrent = 0.0;
        if (!jsonArm["brake_engaged_current"].isNull()) {
            brakeCurrent = jsonArm["brake_engaged_current"].asFloat();
        }
        sarm->m_brake_engaged_current = brakeCurrent;

        // read pot settings
        sarm->m_state_table_configuration.Start();
        cmnDataJSON<vctDoubleVec>::DeSerializeText(sarm->m_voltage_to_position_offsets[0], jsonArm["primary_offsets"]);
        cmnDataJSON<vctDoubleVec>::DeSerializeText(sarm->m_voltage_to_position_offsets[1], jsonArm["secondary_offsets"]);
        cmnDataJSON<vctDoubleVec>::DeSerializeText(sarm->m_voltage_to_position_scales[0], jsonArm["primary_scales"]);
        cmnDataJSON<vctDoubleVec>::DeSerializeText(sarm->m_voltage_to_position_scales[1], jsonArm["secondary_scales"]);
        sarm->m_state_table_configuration.Advance();

        // look for DH
        sarm->m_manipulator.LoadRobot(jsonArm["DH"]);

        // Read setup transforms
        vctFrm3 transform;
        cmnDataJSON<vctFrm3>::DeSerializeText(transform, jsonArm["world_origin_to_SUJ"]);
        sarm->m_world_to_SUJ.From(transform);
        cmnDataJSON<vctFrm3>::DeSerializeText(transform, jsonArm["SUJ_tip_to_tool_origin"]);
        sarm->m_SUJ_to_arm_base.From(transform);
    }
}


void mtsIntuitiveResearchKitSUJ::update_operating_state_and_busy(const prmOperatingState::StateType & state,
                                                                 const bool isBusy)
{
    m_operating_state.State() = state;
    m_operating_state.IsBusy() = isBusy;
    dispatch_operating_state();
}


void mtsIntuitiveResearchKitSUJ::state_changed(void)
{
    dispatch_status("current state " + m_state_machine.CurrentState());
    dispatch_operating_state();
}


void mtsIntuitiveResearchKitSUJ::run_all_states(void)
{
    // get robot data, i.e. process mux/pots
    get_robot_data();
    // update all forward kinematics
    update_forward_kinematics();
}


void mtsIntuitiveResearchKitSUJ::reset_mux(void)
{
    m_mux_timer = this->StateTable.GetTic();
    mux_increment.SetValue(false);
    no_mux_reset.SetValue(false);
    Sleep(30.0 * cmn_ms);
    m_mux_index_expected = 0;
}


void mtsIntuitiveResearchKitSUJ::enter_DISABLED(void)
{
    update_operating_state_and_busy(prmOperatingState::DISABLED, false);

    // power off brakes
    RobotIO.SetActuatorCurrent(vctDoubleVec(4, 0.0));
    RobotIO.PowerOffSequence(true); // for the SUJ controller, we need to close the safety relay

    // disable power on PWM
    PWM.DisablePWM(true);
    // set lift velocity
    set_lift_velocity(0.0);

    // reset mux
    reset_mux();

    m_powered = false;
}


void mtsIntuitiveResearchKitSUJ::transition_DISABLED(void)
{
    if (m_state_machine.DesiredStateIsNotCurrent()) {
        m_state_machine.SetCurrentState("POWERING");
    }
}


void mtsIntuitiveResearchKitSUJ::enter_POWERING(void)
{
    update_operating_state_and_busy(prmOperatingState::DISABLED, true);
    m_powered = false;

    if (m_simulated) {
        m_powered = true;
        return;
    }

    const double currentTime = this->StateTable.GetTic();
    m_homing_timer = currentTime;
    // pre-load the boards with zero current
    RobotIO.SetActuatorCurrent(vctDoubleVec(4, 0.0));
    // enable power and set a flags to move to next step
    RobotIO.PowerOnSequence();

    dispatch_status("power requested");
}


void mtsIntuitiveResearchKitSUJ::transition_POWERING(void)
{
    if (m_simulated) {
        m_state_machine.SetCurrentState("ENABLED");
        return;
    }

    const double currentTime = this->StateTable.GetTic();

    // check status
    if ((currentTime - m_homing_timer) > mtsIntuitiveResearchKit::TimeToPower) {
        // check power status
        vctBoolVec actuatorAmplifiersStatus(4);
        RobotIO.GetActuatorAmpStatus(actuatorAmplifiersStatus);
        if (actuatorAmplifiersStatus.All()) {
            dispatch_status("power on");
            m_powered = true;
            m_state_machine.SetCurrentState("ENABLED");
        } else {
            dispatch_error("failed to enable power");
            set_desired_state("DISABLED");
        }
    }
}


void mtsIntuitiveResearchKitSUJ::enter_ENABLED(void)
{
    update_operating_state_and_busy(prmOperatingState::ENABLED, false);

    if (m_simulated) {
        set_homed(true);
        return;
    }

    // enable power on PWM
    PWM.DisablePWM(false);

    // make sure motor current is zero (brakes)
    RobotIO.SetActuatorCurrent(vctDoubleVec(4, 0.0));

    // when returning from manual mode, make sure brakes are not released
    for (auto sarm : m_sarms) {
        sarm->m_clutched = 0;
        sarm->m_brake_desired_current = 0.0;
        m_previous_tic = 0.0;
    }
}


void mtsIntuitiveResearchKitSUJ::transition_ENABLED(void)
{
    // move to next stage if desired state is different
    if (m_state_machine.DesiredStateIsNotCurrent()) {
        m_state_machine.SetCurrentState(m_state_machine.DesiredState());
    }
}


void mtsIntuitiveResearchKitSUJ::Startup(void)
{
    set_desired_state("DISABLED");
}


void mtsIntuitiveResearchKitSUJ::Run(void)
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


void mtsIntuitiveResearchKitSUJ::Cleanup(void)
{
    // Disable PWM
    set_lift_velocity(0.0);
    PWM.DisablePWM(true);
    // make sure requested current is back to 0
    RobotIO.SetActuatorCurrent(vctDoubleVec(4, 0.0));
    // turn off amplifiers
    RobotIO.PowerOffSequence(true); // also opens safety relays
}


void mtsIntuitiveResearchKitSUJ::set_simulated(void)
{
    m_simulated = true;
    // set all arms simulated
    for (auto sarm : m_sarms) {
        if (sarm != nullptr) {
            sarm->m_simulated = true;
        }
    }
    // in simulation mode, we don't need IOs
    RemoveInterfaceRequired("RobotIO");
    RemoveInterfaceRequired("NoMuxReset");
    RemoveInterfaceRequired("MuxIncrement");
    RemoveInterfaceRequired("ControlPWM");
    RemoveInterfaceRequired("DisablePWM");
    RemoveInterfaceRequired("MotorUp");
    RemoveInterfaceRequired("MotorDown");
}


void mtsIntuitiveResearchKitSUJ::get_robot_data(void)
{
    if (m_simulated) {
        return;
    }

    // check that the robot still has power
    if (m_powered) {
        vctBoolVec actuatorAmplifiersStatus(4);
        RobotIO.GetActuatorAmpStatus(actuatorAmplifiersStatus);
        if (!actuatorAmplifiersStatus.All()) {
            m_powered = false;
            dispatch_error("detected power loss");
            set_desired_state("DISABLED");
            return;
        }
    }

    // 30 ms is to make sure A2D stabilizes
    const double muxCycle = 30.0 * cmn_ms;

    // we can start reporting some joint values after the robot is powered
    const double currentTime = this->StateTable.GetTic();

    // we assume the analog in is now stable
    if (currentTime > m_mux_timer) {
        // pot values should be stable by now, get pots values
        get_and_convert_potentiometers();

        // time to toggle
        if (m_voltage_samples_counter == m_voltage_samples_number) {
            // toggle mux
            m_mux_timer = currentTime + muxCycle;
            if (m_mux_index_expected == MUX_MAX_INDEX) {
                no_mux_reset.SetValue(false);
                m_mux_index_expected = 0;
            } else {
                mux_increment.SetValue(true);
                m_mux_index_expected += 1;
            }
            // reset sample counter
            m_voltage_samples_counter = 0;
        }
    }
}


void mtsIntuitiveResearchKitSUJ::get_and_convert_potentiometers(void)
{
    // read encoder channel A to get the mux state
    mtsExecutionResult executionResult = RobotIO.GetEncoderChannelA(m_mux_state);
    // compute pot index
    m_mux_index = (m_mux_state[0]?1:0) + (m_mux_state[1]?2:0) + (m_mux_state[2]?4:0) + (m_mux_state[3]?8:0);
    if (m_mux_index != m_mux_index_expected) {
        dispatch_warning("unexpected multiplexer value");
        CMN_LOG_CLASS_RUN_ERROR << "get_and_convert_potentiometers: mux from IO board, actual: " << m_mux_index << ", expected: " << m_mux_index_expected << std::endl;
        reset_mux();
        set_homed(false);
        return;
    }
    set_homed(true);

    // array index, 0 or 1, primary or secondary pots
    const size_t arrayIndex = m_mux_index / MUX_ARRAY_SIZE; // 0 or 1: mux index 0 to 5 goes to first array of data, 6 to 11 goes to second array
    // 12 to 15 goes to third array (misc. voltages)
    const size_t indexInArray = m_mux_index % MUX_ARRAY_SIZE; // pot index in array, 0 to 5 (0 to 3 for third array)

    executionResult = RobotIO.GetAnalogInputVolts(m_voltages);
    m_voltage_samples[m_voltage_samples_counter].ForceAssign(m_voltages);
    m_voltage_samples_counter++;

    // if we have enough samples
    if (m_voltage_samples_counter == m_voltage_samples_number) {
        // use m_voltages to store average
        m_voltages.Zeros();
        for (size_t index = 0;
             index < m_voltage_samples_number;
             ++index) {
            m_voltages.Add(m_voltage_samples[index]);
        }
        m_voltages.Divide(m_voltage_samples_number);
        // for each arm, i.e. SUJ1, SUJ2, SUJ3, ...
        for (size_t arm_index = 0; arm_index < 4; ++arm_index) {
            auto * sarm = m_sarms[arm_index];
            // start stable when reading 1st joint on all arms
            if (m_mux_index == 0) {
                sarm->m_state_table.Start();
            }
            // all 4 analog inputs are sent to all 4 arm data structures
            if (arrayIndex < 2) {
                sarm->m_voltages[arrayIndex][indexInArray] = m_voltages[arm_index];
            } else {
                if (indexInArray == 2) {
                    if ((arm_index == 0) || (arm_index == 1) || (arm_index == 2)) {
                        m_sarms[3 - arm_index]->m_voltages_extra[indexInArray] = m_voltages[arm_index];
                    } else if (arm_index == 3) {

                    }
                } else if ((indexInArray == 3) && (arm_index == 3)) {
                    m_sarms[0 /* 3 - arm_index */]->m_voltages_extra[2] = m_voltages[arm_index];
                } else {
                    // normal case
                    sarm->m_voltages_extra[indexInArray] = m_voltages[arm_index];
                }
            }
            // advance state table when all joints have been read
            if (m_mux_index == MUX_MAX_INDEX) {
                sarm->m_positions[0].Assign(sarm->m_voltage_to_position_offsets[0]);
                sarm->m_positions[0].AddElementwiseProductOf(sarm->m_voltage_to_position_scales[0], sarm->m_voltages[0]);
                sarm->m_positions[1].Assign(sarm->m_voltage_to_position_offsets[1]);
                sarm->m_positions[1].AddElementwiseProductOf(sarm->m_voltage_to_position_scales[1], sarm->m_voltages[1]);

                // ignore values on ECM arm
                if (sarm->m_type == mtsIntuitiveResearchKitSUJArmData::SUJ_ECM) {
                    // ECM has only 4 joints
                    sarm->m_positions[0][4] = 0.0;
                    sarm->m_positions[0][5] = 0.0;
                    sarm->m_positions[1][4] = 0.0;
                    sarm->m_positions[1][5] = 0.0;
                }

                // if the arm is clutched, we keep resetting mux counter
                if (sarm->m_clutched > 0) {
                    sarm->m_number_of_mux_cycles = 0;
                }

                // check pots when the SUJ is not clutch and if the
                // counter for update cartesian desired position is
                // back to zero (pot values should now be stable).
                if ((sarm->m_clutched == 0) && (sarm->m_number_of_mux_cycles >= NUMBER_OF_MUX_CYCLE_BEFORE_STABLE)) {
                    // compare primary and secondary pots when arm is not clutched
                    const double angleTolerance = 1.0 * cmnPI / 180.0;
                    const double distanceTolerance = 2.0 * cmn_mm;
                    sarm->m_delta_measured_js.DifferenceOf(sarm->m_positions[0], sarm->m_positions[1]);
                    if ((sarm->m_delta_measured_js[0] > distanceTolerance) ||
                        (sarm->m_delta_measured_js.Ref(5, 1).MaxAbsElement() > angleTolerance)) {
                        // send messages if this is new
                        if (sarm->m_pots_agree) {
                            dispatch_warning(sarm->m_name + " primary and secondary potentiometers don't seem to agree");
                            CMN_LOG_CLASS_RUN_WARNING << "get_and_convert_potentiometers, error: " << std::endl
                                                      << " - " << this->GetName() << ": " << sarm->m_name << std::endl
                                                      << " - primary:   " << sarm->m_positions[0] << std::endl
                                                      << " - secondary: " << sarm->m_positions[1] << std::endl;
                            sarm->m_pots_agree = false;
                        }
                    } else {
                        if (!sarm->m_pots_agree) {
                            dispatch_status(sarm->m_name + " primary and secondary potentiometers agree");
                            CMN_LOG_CLASS_RUN_VERBOSE << "get_and_convert_potentiometers recovery" << std::endl
                                                      << " - " << this->GetName() << ": " << sarm->m_name << std::endl;
                            sarm->m_pots_agree = true;
                        }
                    }
                }

                // this mux cycle might have started before brakes where engaged so we can set the valid flag
                if (sarm->m_number_of_mux_cycles < NUMBER_OF_MUX_CYCLE_BEFORE_STABLE) {
                    sarm->m_number_of_mux_cycles++;
                } else {
                    // at that point we know there has been a full mux cycle with brakes engaged
                    // so we treat this as a fixed transformation until the SUJ move again (user clutch)
                    // use average of positions reported by potentiometers
                    sarm->m_measured_js.Position().SumOf(sarm->m_positions[0],
                                                         sarm->m_positions[1]);
                    sarm->m_measured_js.Position().Divide(2.0);
                    sarm->m_measured_js.SetValid(true);
                    if (sarm->m_waiting_for_live) {
                        sarm->m_waiting_for_live = false;
                        sarm->m_need_update_forward_kinemactics = true;
                    }
                }
            }
        }
    }
}


void mtsIntuitiveResearchKitSUJ::update_forward_kinematics(void)
{
    // for real robots first update all the local_measured_cp if all arms are ready
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
                    sarm->m_interface_provided->SendStatus(sarm->m_name + " SUJ: measured_cp updated");
                }
            } else {
                sarm->m_local_measured_cp.SetValid(false);
                sarm->m_measured_cp.SetValid(false);
            }
        }
    }

    // find the reference arm (usually ECM)
    prmPositionCartesianGet reference_arm_local_cp;
    prmPositionCartesianGet reference_arm_to_cart_cp;

    auto * reference_sarm = m_sarms[m_reference_arm_index];
    if (! (reference_sarm->m_get_local_measured_cp(reference_arm_local_cp))) {
        // interface not connected, reporting wrt cart
        reference_arm_to_cart_cp.Position().Assign(vctFrm3::Identity());
        reference_arm_to_cart_cp.SetValid(true);
        reference_arm_to_cart_cp.SetReferenceFrame("Cart");
    } else {
        // get position from reference arm and convert to useful type
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
        auto * sarm = m_sarms[arm_index];
        // update positions with base frame, local positions are only
        // updated from FK when joints are ready
        if (arm_index != m_reference_arm_index) {
            sarm->m_measured_cp.SetReferenceFrame(reference_arm_to_cart_cp.ReferenceFrame());
            local_cp.From(sarm->m_local_measured_cp.Position());
            cp = reference_frame * local_cp;
            // - with base frame
            sarm->m_measured_cp.Position().From(cp);
            sarm->m_measured_cp.SetValid(sarm->m_local_measured_cp.Valid()
                                         && reference_arm_to_cart_cp.Valid());
            sarm->m_measured_cp.SetTimestamp(std::max(sarm->m_local_measured_cp.Timestamp(),
                                                      reference_arm_to_cart_cp.Timestamp()));
        } else {
            // for reference arm, measured_cp is local_measured_cp
            sarm->m_measured_cp = sarm->m_local_measured_cp;
        }
        // convert from prmPositionCartesianGet to prmPositionCartesianSet
        prmPositionCartesianSet base_frame;
        base_frame.Goal().Assign(sarm->m_measured_cp.Position());
        base_frame.SetValid(sarm->m_measured_cp.Valid());
        base_frame.SetTimestamp(sarm->m_measured_cp.Timestamp());
        base_frame.SetReferenceFrame(sarm->m_measured_cp.ReferenceFrame());
        base_frame.SetMovingFrame(sarm->m_measured_cp.MovingFrame());
        // and finally set the base frame
        sarm->m_arm_set_base_frame(base_frame);
    }
}


void mtsIntuitiveResearchKitSUJ::set_desired_state(const std::string & state)
{
    // setting desired state triggers a new event so user nows which state is current
    dispatch_operating_state();
    // try to find the state in state machine
    if (!m_state_machine.StateExists(state)) {
        dispatch_error(this->GetName() + ": unsupported state " + state);
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


void mtsIntuitiveResearchKitSUJ::state_command(const std::string & command)
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


void mtsIntuitiveResearchKitSUJ::run_ENABLED(void)
{
    double currentTic = this->StateTable.GetTic();
    const double timeDelta = currentTic - m_previous_tic;

    const double brakeCurrentRate = 8.0; // rate = 8 A/s, about 1/4 second to get up/down

    for (size_t arm_index = 0; arm_index < 4; ++arm_index) {
        auto sarm = m_sarms[arm_index];
        // brakes
        // increase current for brakes
        if (sarm->m_clutched > 0) {
            if (((brakeCurrentRate * timeDelta) + sarm->m_brake_desired_current) < sarm->m_brake_release_current) {
                sarm->m_brake_desired_current += brakeCurrentRate * timeDelta;
            } else {
                sarm->m_brake_desired_current = sarm->m_brake_release_current;
            }
        }
        // decrease current for brakes
        else {
            if (sarm->m_brake_desired_current != sarm->m_brake_engaged_current) {
                if ((sarm->m_brake_desired_current - (brakeCurrentRate * timeDelta)) >= sarm->m_brake_engaged_current) {
                    sarm->m_brake_desired_current -= brakeCurrentRate * timeDelta;
                    // if by any luck we have reached arm->m_brake_engaged_current, need to update cartesian desired
                    if (sarm->m_brake_desired_current == sarm->m_brake_engaged_current) {
                        sarm->m_number_of_mux_cycles = 0;
                    }
                } else {
                    sarm->m_brake_desired_current = sarm->m_brake_engaged_current;
                    sarm->m_number_of_mux_cycles = 0;
                }
            }
        }
        m_brake_currents[arm_index] = sarm->m_brake_direction_current * sarm->m_brake_desired_current;
    }
    RobotIO.SetActuatorCurrent(m_brake_currents);
    m_previous_tic = currentTic;
}


void mtsIntuitiveResearchKitSUJ::set_homed(const bool homed)
{
    if (homed != m_operating_state.IsHomed()) {
        m_operating_state.IsHomed() = homed;
        dispatch_operating_state();
    }
}


void mtsIntuitiveResearchKitSUJ::set_lift_velocity(const double & velocity)
{
    if ((velocity >= -1.0) && (velocity <= 1.0)) {
        const double dutyCyle = 0.5 + velocity * 0.1;  // 0.1 determines max velocity
        PWM.SetPWMDutyCycle(dutyCyle);
    } else {
        CMN_LOG_CLASS_RUN_ERROR << "set_lift_velocity: value must be between -1.0 and 1.0" << std::endl;
    }
}


void mtsIntuitiveResearchKitSUJ::motor_down_event_handler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        set_lift_velocity(-1.0);
    } else {
        set_lift_velocity(0.0);
    }
}

void mtsIntuitiveResearchKitSUJ::motor_up_event_handler(const prmEventButton & button)
{
    if (button.Type() == prmEventButton::PRESSED) {
        set_lift_velocity(1.0);
    } else {
        set_lift_velocity(0.0);
    }
}


void mtsIntuitiveResearchKitSUJ::error_event_handler(const mtsMessage & message)
{
    RobotIO.PowerOffSequence(false);
    dispatch_error("received [" + message.Message + "]");
    set_desired_state("DISABLED");
}


void mtsIntuitiveResearchKitSUJ::dispatch_error(const std::string & message)
{
    m_interface->SendError(this->GetName() + ": " + message);
    for (auto sarm : m_sarms) {
        sarm->m_interface_provided->SendError(sarm->m_name + " SUJ: " + message);
    }
}


void mtsIntuitiveResearchKitSUJ::dispatch_warning(const std::string & message)
{
    m_interface->SendWarning(this->GetName() + ": " + message);
    for (auto sarm : m_sarms) {
        sarm->m_interface_provided->SendWarning(sarm->m_name + " SUJ: " + message);
    }
}


void mtsIntuitiveResearchKitSUJ::dispatch_status(const std::string & message)
{
    m_interface->SendStatus(this->GetName() + ": " + message);
    for (auto sarm : m_sarms) {
        sarm->m_interface_provided->SendStatus(sarm->m_name + " SUJ: " + message);
    }
}


void mtsIntuitiveResearchKitSUJ::dispatch_operating_state(void)
{
    state_events.operating_state(m_operating_state);
    for (auto sarm : m_sarms) {
        sarm->state_events.operating_state(m_operating_state);
        sarm->state_events.current_state(m_state_machine.CurrentState());
        sarm->state_events.desired_state(m_state_machine.DesiredState());
    }
}
