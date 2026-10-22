/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2022-07-27

  (C) Copyright 2022-2026 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>
#include <memory>
#include <time.h>

// cisst
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitSUJSi.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>
#include <sawIntuitiveResearchKit/robManipulatorGeneric.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmEventButton.h>

static const std::map<std::string, size_t> NB_JOINTS = {{"PSM1", 4}, {"PSM2", 4}, {"PSM3", 5}, {"ECM", 4}};

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitSUJSi, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

class mtsIntuitiveResearchKitSUJSiArmData
{
public:

    typedef enum {SUJ_UNDEFINED, SUJ_PSM, SUJ_ECM, SUJ_MOTORIZED_PSM} SujType;

    inline mtsIntuitiveResearchKitSUJSiArmData(const std::string & name,
                                               const prmSimulationType simulationMode,
                                               mtsInterfaceProvided * interfaceProvided,
                                               mtsInterfaceRequired * interfaceRequired,
                                               mtsInterfaceRequired * interfaceRequiredIO,
                                               mtsInterfaceProvided * interfaceMessage):
        m_name(name),
        m_simulation_mode(simulationMode),
        m_nb_joints(NB_JOINTS.at(name)),
        m_state_table(500, name),
        m_state_table_configuration(100, name + "Configuration"),
        m_interface_message(interfaceMessage)
    {
        // recalibration matrix
        m_recalibration_matrix = Eigen::MatrixXd::Zero(m_nb_joints, m_nb_joints);
        m_new_joint_scales[0] = Eigen::VectorXd::Zero(m_nb_joints);
        m_new_joint_scales[1] = Eigen::VectorXd::Zero(m_nb_joints);

        m_new_joint_offsets[0] = Eigen::VectorXd::Zero(m_nb_joints);
        m_new_joint_offsets[1] = Eigen::VectorXd::Zero(m_nb_joints);

        // configuration table is not added to main component so this
        // has no effect
        m_state_table_configuration.SetAutomaticAdvance(false);

        for (size_t potArray = 0; potArray < 2; ++potArray) {
            m_voltages[potArray] = Eigen::VectorXd::Zero(m_nb_joints);
            m_positions[potArray] = Eigen::VectorXd::Zero(m_nb_joints);
        }
        m_delta_measured_js = Eigen::VectorXd::Zero(m_nb_joints);

        m_measured_js.Position() = Eigen::VectorXd::Zero(m_nb_joints);
        m_measured_js.Name().resize(m_nb_joints);
        m_configuration_js.Name().resize(m_nb_joints);
        std::stringstream jointName;
        for (size_t index = 0; index < m_nb_joints; ++index) {
            jointName.str("");
            jointName << "SUJ_" << name << "_J" << index;
            m_measured_js.Name().at(index) = jointName.str();
            m_configuration_js.Name().at(index) = jointName.str();
        }
        m_live_measured_js.Position() = m_measured_js.Position();
        cmnDataCopy(m_live_measured_js.Name(), m_measured_js.Name());

        m_configuration_js.Type() = std::vector<cmnJointType>(m_nb_joints, CMN_JOINT_REVOLUTE);
        m_configuration_js.Type().at(0) = CMN_JOINT_PRISMATIC;
        // joint limits are only used in simulation mode so we can set
        // arbitrarily wide limits
        m_configuration_js.PositionMin() = -2.0 * cmnPI * Eigen::VectorXd::Ones(m_nb_joints);
        m_configuration_js.PositionMax() = 2.0 * cmnPI * Eigen::VectorXd::Ones(m_nb_joints);
        m_configuration_js.PositionMin() = -2.0 * cmnPI * Eigen::VectorXd::Ones(m_nb_joints);
        m_configuration_js.PositionMax() = 2.0 * cmnPI * Eigen::VectorXd::Ones(m_nb_joints);
        m_configuration_js.PositionMin()[0] = -2.0 * cmn_m;
        m_configuration_js.PositionMax()[0] =  2.0 * cmn_m;

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

        CMN_ASSERT(interfaceProvided);
        m_interface_provided = interfaceProvided;
        // read commands
        m_interface_provided->AddCommandReadState(m_state_table, m_measured_js, "measured_js");
        m_interface_provided->AddCommandReadState(m_state_table, m_live_measured_js, "live/measured_js");
        m_interface_provided->AddCommandReadState(m_state_table, m_configuration_js, "configuration_js");
        // set position is only for simulation, allows both servo and move
        m_interface_provided->AddCommandWrite(&mtsIntuitiveResearchKitSUJSiArmData::servo_jp,
                                              this, "servo_jp");
        m_interface_provided->AddCommandWrite(&mtsIntuitiveResearchKitSUJSiArmData::servo_jp,
                                              this, "move_jp");
        m_interface_provided->AddCommandReadState(m_state_table, m_measured_cp,
                                                  "measured_cp");
        m_interface_provided->AddCommandReadState(m_state_table, m_local_measured_cp,
                                                  "local/measured_cp");
        m_interface_provided->AddCommandReadState(m_state_table, m_voltages[0], "GetVoltagesPrimary");
        m_interface_provided->AddCommandReadState(m_state_table, m_voltages[1], "GetVoltagesSecondary");
        m_interface_provided->AddCommandReadState(m_state_table_configuration, m_name, "GetName");

        // write commands
        m_interface_provided->AddCommandWrite(&mtsIntuitiveResearchKitSUJSiArmData::clutch_command, this,
                                              "clutch", false);
        m_interface_provided->AddCommandWrite(&mtsIntuitiveResearchKitSUJSiArmData::calibrate_potentiometers, this,
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

        CMN_ASSERT(interfaceRequiredIO);
        m_interface_required_IO = interfaceRequiredIO;
        m_interface_required_IO->AddFunction("primary/measured_js", primary_measured_js);
        m_interface_required_IO->AddFunction("secondary/measured_js", secondary_measured_js);
    }


    inline void clutch_callback(const prmEventButton & button)
    {
        if (button.Type() == prmEventButton::PRESSED) {
            // clutch is pressed, arm is moving around and we know the pots are slow, we mark position as invalid
            m_interface_message->SendStatus(m_name + " SUJ: clutched");
            m_measured_js.SetValid(false);
            m_measured_cp.SetTimestamp(m_measured_js.Timestamp());
            m_measured_cp.SetValid(false);
            m_local_measured_cp.SetTimestamp(m_measured_js.Timestamp());
            m_local_measured_cp.SetValid(false);
        } else if (button.Type() == prmEventButton::RELEASED) {
            m_waiting_for_live = true;
            m_interface_message->SendStatus(m_name + " SUJ: not clutched");
        }
    }


    inline void servo_jp(const prmPositionJointSet & newPosition)
    {
        if (m_simulation_mode != prmSimulationType::KINEMATIC) {
            m_interface_message->SendWarning(m_name + " SUJ: servo_jp can't be used unless the SUJs are in simulated mode");
            return;
        }
        // save the desired position
        m_measured_js.Position() = newPosition.Goal();
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


    inline void calibrate_potentiometers(const Eigen::MatrixXd& mat)
    {
        for (size_t col = 0; col < m_nb_joints; col++) {
            // IF:                                      Pi = Offset + Vi * Scale
            // Given P1 / V1 & P2 / V2, THEN:           Scale = (P1 - P2) / (V1 - V2)

            // Delta_P = P1 - P2
            const double deltaJointPosition = mat(0, col) - mat(3, col);

            // Delta_V = V1 - V2 (primary)
            const double deltaPrimaryVoltage = mat(1, col) - mat(4, col);

            // V1 - V2 (secondary)
            const double deltaSecondaryVoltage = mat(2, col) - mat(5, col);

            // Scale = Delta_P / Delta_V
            m_new_joint_scales[0][col] = deltaJointPosition / deltaPrimaryVoltage;
            m_new_joint_scales[1][col] = deltaJointPosition / deltaSecondaryVoltage;

            m_new_joint_offsets[0][col] = mat(0, col) - mat(1, col) * m_new_joint_scales[0][col];
            m_new_joint_offsets[1][col] = mat(0, col) - mat(2, col) * m_new_joint_scales[1][col];
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

    // simulated or not
    prmSimulationType m_simulation_mode;

    // number of joints for this arm
    size_t m_nb_joints;

    // interfaces
    mtsInterfaceProvided * m_interface_provided = nullptr;
    mtsInterfaceRequired * m_interface_required = nullptr;
    mtsInterfaceRequired * m_interface_required_IO = nullptr;

    // state of this SUJ arm
    mtsStateTable m_state_table; // for positions, fairly slow, i.e 12 * delay for a2d
    mtsStateTable m_state_table_configuration; // changes only at config and if recalibrate

    // 2 arrays, one for each set of potentiometers
    Eigen::VectorXd m_voltages[2];
    Eigen::VectorXd m_positions[2];
    Eigen::VectorXd m_delta_measured_js;
    bool m_pots_agree = false;
    bool m_waiting_for_live = true;

    Eigen::VectorXd m_voltage_to_position_scales[2];
    Eigen::VectorXd m_voltage_to_position_offsets[2];
    prmStateJoint m_measured_js, m_live_measured_js;
    prmConfigurationJoint m_configuration_js;

    // kinematics
    bool m_need_update_forward_kinemactics = false;
    prmPositionCartesianGet m_measured_cp;
    prmPositionCartesianGet m_local_measured_cp;

    std::unique_ptr<robManipulatorGeneric> m_manipulator = nullptr;
    Eigen::MatrixXd m_recalibration_matrix;
    Eigen::VectorXd m_new_joint_scales[2];
    Eigen::VectorXd m_new_joint_offsets[2];

    // setup transformations from json file
    Eigen::Isometry3d m_world_to_SUJ;
    Eigen::Isometry3d m_SUJ_to_arm_base;

    // base frame
    mtsFunctionWrite m_arm_set_base_frame;
    // for reference arm only, get current position
    mtsFunctionRead m_get_local_measured_cp;

    mtsFunctionRead primary_measured_js;
    mtsFunctionRead secondary_measured_js;

    // for messages
    mtsInterfaceProvided * m_interface_message = nullptr;

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

// defined here so definition of mtsIntuitiveResearchKitSUJSiArmData is available
mtsIntuitiveResearchKitSUJSi::~mtsIntuitiveResearchKitSUJSi() = default;

void mtsIntuitiveResearchKitSUJSi::init(void)
{
    // initialize arm pointers
    m_sarms = std::array<std::unique_ptr<mtsIntuitiveResearchKitSUJSiArmData>, 4>{};

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
        m_interface->AddCommandWrite(&mtsIntuitiveResearchKitSUJSi::set_reference_arm,
                                     this, "set_reference_arm", std::string(""));
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
    const Json::Value jsonReferenceArm = jsonConfig["reference_arm"];
    if (!jsonReferenceArm.isNull()) {
        reference_sarm_name = jsonReferenceArm.asString();
        CMN_LOG_CLASS_INIT_WARNING << "Configure: \"reference_arm\" is user defined.  This should only happen if you are using a PSM to hold a camera.  Most users shouldn't define \"reference_arm\".  If undefined, all arm cartesian positions will be defined with respect to the ECM" << std::endl;
    }

    if (m_simulation_mode != prmSimulationType::KINEMATIC) {
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

        const size_t nb_joints = NB_JOINTS.at(name);

        // add interfaces, one is provided so users can find the SUJ
        // info, the other is required so the SUJ can get position of
        // ECM and change base frame on attached arms, and the last is
        // required to get joint states from the IO component.
        mtsInterfaceProvided * interfaceProvided = this->AddInterfaceProvided(name);
        mtsInterfaceRequired * interfaceRequired = this->AddInterfaceRequired(name, MTS_OPTIONAL);
        mtsInterfaceRequired * interfaceRequiredIO = this->AddInterfaceRequired(name + "IO", MTS_OPTIONAL);
        auto sarm = std::make_unique<mtsIntuitiveResearchKitSUJSiArmData>(name, m_simulation_mode,
                                                            interfaceProvided,
                                                            interfaceRequired,
                                                            interfaceRequiredIO,
                                                            m_interface);
        m_sarms[index] = std::move(sarm);
        AddStateTable(&(m_sarms[index]->m_state_table));

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
            requiredInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitSUJSiArmData::clutch_callback, m_sarms[index].get(),
                                                    "Button");
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: can't add required interface \"" << itf
                                     << "\" for SUJ \"" << name << "\" clutch button"
                                     << std::endl;
            exit(EXIT_FAILURE);
        }

        if (m_simulation_mode != prmSimulationType::KINEMATIC) {
            // read pot settings
            sarm->m_state_table_configuration.Start();
        } else {
            // look for hard coded position if available - users can always push new joint values using ROS
            Json::Value jsonPosition = jsonArm["simulated_position"];
            if (!jsonPosition.empty()) {
                Eigen::VectorXd position;
                cmnDataJSON<Eigen::VectorXd>::DeSerializeText(position, jsonPosition);
                if (position.size() == (Eigen::Index)nb_joints) {
                    sarm->m_measured_js.Position() = position;
                    sarm->m_measured_js.SetValid(true);
                    sarm->m_need_update_forward_kinemactics = true;
                } else {
                    CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to load \"position-simulated\" for \""
                                             << name << "\", expected vector size is "
                                             << nb_joints << " but vector in configuration file has "
                                             << position.size() << " element(s)"
                                             << std::endl;
                    exit(EXIT_FAILURE);
                }
            }
        }
        sarm->m_state_table_configuration.Advance();

        // DH and transforms should be loaded from the share/kinematic
        // directory bit if DH is defined in this scope, assumes user
        // want to provide DH and base/tip transforms
        Json::Value jsonDH = jsonArm["DH"];
        if (!jsonDH.empty()) {
            // look for DH
            const auto kdl_chain = loadKDLChain(jsonDH);
            if (!kdl_chain.has_value()) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure " << name
                                   << ": failed to load kinematics parameters from file \""
                                   << filename << "\"\n";
                exit(EXIT_FAILURE);
            }

            const auto joint_config = loadJointConfig(jsonDH);
            if (!joint_config.has_value()) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure " << name
                                         << ": failed to load joint configuration parameters from file \""
                                         << filename << std::endl;
                exit(EXIT_FAILURE);
            }

            sarm->m_manipulator = std::make_unique<robManipulatorGeneric>(*joint_config, *kdl_chain);
            if (kdl_chain->getNrOfJoints() != nb_joints) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure: incorrect kinematic chain length for \""
                                         << name << "\", expected " << nb_joints
                                         << " active joints but found " << kdl_chain->getNrOfJoints() << std::endl;
                exit(EXIT_FAILURE);

            }
            // read setup transforms
            cmnDataJSON<Eigen::Isometry3d>::DeSerializeText(sarm->m_world_to_SUJ , jsonArm["world_origin_to_SUJ"]);
            cmnDataJSON<Eigen::Isometry3d>::DeSerializeText(sarm->m_SUJ_to_arm_base, jsonArm["SUJ_tip_to_tool_origin"]);
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
    // get robot data, i.e. process mux/pots
    get_robot_data();
    // update all forward kinematics
    update_forward_kinematics();
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


void mtsIntuitiveResearchKitSUJSi::set_simulation_mode(const prmSimulationType & mode)
{
    m_simulation_mode = mode;

    switch (mode) {
    case prmSimulationType::NONE:
    case prmSimulationType::KINEMATIC:
        break;
    default:
        cmnThrow("mtsIntuitiveResearchKitSUJSi::set_simulation_mode received unsupported mode");
    }

    // set all arms simulated
    for (auto& sarm : m_sarms) {
        if (sarm != nullptr) {
            sarm->m_simulation_mode = mode;
        }
    }
}


void mtsIntuitiveResearchKitSUJSi::get_robot_data(void)
{
    if (m_simulation_mode == prmSimulationType::KINEMATIC) {
        return;
    }

    for (auto& sarm : m_sarms) {
        if (sarm != nullptr) {
            prmStateJoint primary_js, secondary_js;
            if (sarm->primary_measured_js(primary_js) && sarm->secondary_measured_js(secondary_js)) {
                if (primary_js.Valid() && secondary_js.Valid()) {
                    sarm->m_positions[0] = primary_js.Position();
                    sarm->m_positions[1] = secondary_js.Position();

                    // compare primary and secondary pots when arm is not clutched
                    const double angleTolerance = 2.0 * cmnPI_180;
                    const double distanceTolerance = 3.0 * cmn_mm;
                    sarm->m_delta_measured_js = sarm->m_positions[0] - sarm->m_positions[1];
                    if ((sarm->m_delta_measured_js[0] > distanceTolerance) ||
                        (sarm->m_delta_measured_js.tail(sarm->m_nb_joints - 1).array().abs() > angleTolerance).any()) {
                        // send messages if this is new
                        if (sarm->m_pots_agree) {
                            dispatch_warning(sarm->m_name + " primary and secondary potentiometers don't seem to agree");
                            CMN_LOG_CLASS_RUN_WARNING << "get_robot_data, error: " << std::endl
                                                      << " - " << this->GetName() << ": " << sarm->m_name << std::endl
                                                      << " - primary:   " << sarm->m_positions[0] << std::endl
                                                      << " - secondary: " << sarm->m_positions[1] << std::endl;
                            sarm->m_pots_agree = false;
                        }
                    } else {
                        if (!sarm->m_pots_agree) {
                            dispatch_status(sarm->m_name + " primary and secondary potentiometers agree");
                            CMN_LOG_CLASS_RUN_VERBOSE << "get_robot_data recovery" << std::endl
                                                      << " - " << this->GetName() << ": " << sarm->m_name << std::endl;
                            sarm->m_pots_agree = true;
                        }
                    }

                    // use average of positions reported by potentiometers
                    sarm->m_live_measured_js.Position() = (sarm->m_positions[0] + sarm->m_positions[1]) / 2.0;
                    sarm->m_live_measured_js.SetValid(true);

                    if (sarm->m_waiting_for_live) {
                        // copy live jp
                        sarm->m_measured_js.Position() = sarm->m_live_measured_js.Position();
                        sarm->m_measured_js.SetValid(true);
                        sarm->m_measured_js.SetTimestamp(mtsComponentManager::GetInstance()->GetTimeServer().GetRelativeTime());
                        sarm->m_waiting_for_live = false;
                        sarm->m_need_update_forward_kinemactics = true;
                    }
                } else {
                    sarm->m_live_measured_js.SetValid(false);
                    sarm->m_measured_js.SetValid(false);
                }
            } else {
                sarm->m_live_measured_js.SetValid(false);
                sarm->m_measured_js.SetValid(false);
            }
        }
    }
}


void mtsIntuitiveResearchKitSUJSi::update_forward_kinematics(void)
{
    // first update all the local_measured_cp if all arms are ready
    for (auto& sarm : m_sarms) {
        if (sarm != nullptr) {
            if (sarm->m_measured_js.Valid()) {
                if (sarm->m_need_update_forward_kinemactics) {
                    sarm->m_need_update_forward_kinemactics = false;
                    // forward kinematic
                    Eigen::Isometry3d dh_cp = sarm->m_manipulator->forwardKinematics(sarm->m_measured_js.Position());
                    // pre and post transformations loaded from JSON file, base frame updated using events
                    Eigen::Isometry3d local_cp = sarm->m_world_to_SUJ * dh_cp * sarm->m_SUJ_to_arm_base;
                    // update local only
                    sarm->m_local_measured_cp.Position() = local_cp;
                    sarm->m_local_measured_cp.SetTimestamp(sarm->m_measured_js.Timestamp());
                    sarm->m_local_measured_cp.SetValid(true);
                    sarm->m_interface_message->SendStatus(sarm->m_name + " SUJ: measured_cp updated");
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

    auto* reference_sarm = m_sarms[m_reference_arm_index].get();
    if (! (reference_sarm->m_get_local_measured_cp(reference_arm_local_cp))) {
        // interface not connected, reporting wrt cart
        reference_arm_to_cart_cp.Position() = Eigen::Isometry3d();
        reference_arm_to_cart_cp.SetValid(true);
        reference_arm_to_cart_cp.SetReferenceFrame("Cart");
    } else {
        // get position from reference arm and convert to useful type
        Eigen::Isometry3d cart_to_reference_arm_cp = reference_sarm->m_local_measured_cp.Position() * reference_arm_local_cp.Position();
        // compute and send new base frame for all SUJs (SUJ will handle BaseFrameArm differently)
        reference_arm_to_cart_cp.Position() = cart_to_reference_arm_cp.inverse();
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
    reference_sarm->m_measured_cp.Position() = reference_sarm->m_local_measured_cp.Position();
    reference_sarm->m_measured_cp.SetValid(reference_sarm->m_local_measured_cp.Valid());
    reference_sarm->m_measured_cp.SetTimestamp(reference_sarm->m_local_measured_cp.Timestamp());

    // update other arms
    Eigen::Isometry3d reference_frame(reference_arm_to_cart_cp.Position());
    Eigen::Isometry3d local_cp, cp;
    for (size_t arm_index = 0; arm_index < 4; ++arm_index) {
        auto* sarm = m_sarms[arm_index].get();
        // update positions with base frame, local positions are only
        // updated from FK when joints are ready
        if (arm_index != m_reference_arm_index) {
            sarm->m_measured_cp.SetReferenceFrame(reference_arm_to_cart_cp.ReferenceFrame());
            local_cp = sarm->m_local_measured_cp.Position();
            cp = reference_frame * local_cp;
            // - with base frame
            sarm->m_measured_cp.Position() = cp;
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
        base_frame.Goal() = sarm->m_measured_cp.Position();
        base_frame.SetValid(sarm->m_measured_cp.Valid());
        base_frame.SetTimestamp(sarm->m_measured_cp.Timestamp());
        base_frame.SetReferenceFrame(sarm->m_measured_cp.ReferenceFrame());
        base_frame.SetMovingFrame(sarm->m_measured_cp.MovingFrame());
        // and finally set the base frame
        sarm->m_arm_set_base_frame(base_frame);
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


void mtsIntuitiveResearchKitSUJSi::set_reference_arm(const std::string & arm_name)
{
    for (size_t arm_index = 0; arm_index < 4; ++arm_index) {
        if (m_sarms[arm_index]->m_name == arm_name) {
            m_reference_arm_index = arm_index;
            return;
        }
    }
    m_interface->SendError(this->GetName() + ": set_reference_arm, " + arm_name + " not found");
}


void mtsIntuitiveResearchKitSUJSi::dispatch_error(const std::string & message)
{
    m_interface->SendError(this->GetName() + ": " + message);
    for (auto& sarm : m_sarms) {
        sarm->m_interface_provided->SendError(sarm->m_name + " SUJ: " + message);
    }
}


void mtsIntuitiveResearchKitSUJSi::dispatch_warning(const std::string & message)
{
    m_interface->SendWarning(this->GetName() + ": " + message);
    for (auto& sarm : m_sarms) {
        sarm->m_interface_provided->SendWarning(sarm->m_name + " SUJ: " + message);
    }
}


void mtsIntuitiveResearchKitSUJSi::dispatch_status(const std::string & message)
{
    m_interface->SendStatus(this->GetName() + ": " + message);
    for (auto& sarm : m_sarms) {
        sarm->m_interface_provided->SendStatus(sarm->m_name + " SUJ: " + message);
    }
}


void mtsIntuitiveResearchKitSUJSi::dispatch_operating_state(void)
{
    state_events.operating_state(m_operating_state);
    for (auto& sarm : m_sarms) {
        sarm->state_events.operating_state(m_operating_state);
        sarm->state_events.current_state(m_state_machine.CurrentState());
        sarm->state_events.desired_state(m_state_machine.DesiredState());
    }
}

