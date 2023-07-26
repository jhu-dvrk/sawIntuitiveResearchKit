/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2023-06-16

  (C) Copyright 2023 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitSUJFixed.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitSUJFixed, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

class mtsIntuitiveResearchKitSUJFixedArmData
{
public:

    typedef enum {SUJ_UNDEFINED, SUJ_PSM, SUJ_ECM, SUJ_MOTORIZED_PSM} SujType;

    inline mtsIntuitiveResearchKitSUJFixedArmData(const std::string & name,
                                                  mtsInterfaceProvided * interface_provided,
                                                  mtsInterfaceRequired * interface_required):
        m_name(name),
        m_state_table(500, name)
    {
        // base frame
        m_base_frame_valid = true;

        m_measured_cp.SetReferenceFrame("Cart");
        m_measured_cp.SetMovingFrame(name + "_base");
        m_state_table.AddData(m_measured_cp, "measured_cp");

        m_local_measured_cp.SetReferenceFrame("Cart");
        m_local_measured_cp.SetMovingFrame(name + "_base");
        m_state_table.AddData(m_local_measured_cp, "local/measured_cp");

        m_state_table.AddData(m_base_frame, "base_frame");

        CMN_ASSERT(interface_provided);
        m_interface_provided = interface_provided;

        // set position
        m_interface_provided->AddCommandWrite(&mtsIntuitiveResearchKitSUJFixedArmData::servo_cp,
                                              this, "local/servo_cp");
        m_interface_provided->AddCommandReadState(m_state_table, m_measured_cp,
                                                  "measured_cp");
        m_interface_provided->AddCommandReadState(m_state_table, m_local_measured_cp,
                                                  "local/measured_cp");
        m_interface_provided->AddCommandReadState(m_state_table, m_base_frame, "base_frame");

        // cartesian position events
        // m_base_frame is send everytime the mux has found all joint values
        m_interface_provided->AddEventWrite(EventPositionCartesian, "measured_cp", prmPositionCartesianGet());
        m_interface_provided->AddEventWrite(EventPositionCartesianLocal, "local/measured_cp", prmPositionCartesianGet());

        // Events
        m_interface_provided->AddEventWrite(state_events.operating_state, "operating_state", prmOperatingState());
        m_interface_provided->AddMessageEvents();
        // Stats
        m_interface_provided->AddCommandReadState(m_state_table, m_state_table.PeriodStats,
                                                  "period_statistics");

        CMN_ASSERT(interface_required);
        m_interface_required = interface_required;
        m_interface_required->AddFunction("set_base_frame", m_arm_set_base_frame);
        m_interface_required->AddFunction("local/measured_cp", m_get_local_measured_cp);
    }

    inline void servo_cp(const prmPositionCartesianSet & cp) {
        m_measured_cp.Position().Assign(cp.Goal());
        m_measured_cp.Timestamp() = cp.Timestamp();
        m_measured_cp.Valid() = cp.Valid();
    }

    // name of this SUJ arm (ECM, PSM1, ...)
    std::string m_name;

    // interfaces
    mtsInterfaceProvided * m_interface_provided = nullptr;
    mtsInterfaceRequired * m_interface_required = nullptr;

    // state of this SUJ arm
    mtsStateTable m_state_table; // for positions, fairly slow, i.e 12 * delay for a2d

    prmPositionCartesianGet m_measured_cp;
    prmPositionCartesianGet m_local_measured_cp;

    mtsFunctionWrite m_arm_set_base_frame;
    vctFrame4x4<double> m_base_frame;
    bool m_base_frame_valid;
    // for ECM only, get current position
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


mtsIntuitiveResearchKitSUJFixed::mtsIntuitiveResearchKitSUJFixed(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    m_state_machine(componentName, "DISABLED")

{
    init();
}

mtsIntuitiveResearchKitSUJFixed::mtsIntuitiveResearchKitSUJFixed(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    m_state_machine(arg.Name, "DISABLED")
{
    init();
}

void mtsIntuitiveResearchKitSUJFixed::init(void)
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
    m_state_machine.SetStateChangedCallback(&mtsIntuitiveResearchKitSUJFixed::state_changed,
                                            this);

    // run for all states
    m_state_machine.SetRunCallback(&mtsIntuitiveResearchKitSUJFixed::run_all_states,
                                   this);

    // disabled
    m_state_machine.SetEnterCallback("DISABLED",
                                     &mtsIntuitiveResearchKitSUJFixed::enter_DISABLED,
                                     this);

    // enabled
    m_state_machine.SetEnterCallback("ENABLED",
                                     &mtsIntuitiveResearchKitSUJFixed::enter_ENABLED,
                                     this);

    m_operating_state.SetValid(true);
    m_operating_state.SetState(prmOperatingState::DISABLED);
    m_operating_state.SetIsHomed(true);
    StateTable.AddData(m_operating_state, "operating_state");

    m_interface_provided = AddInterfaceProvided("Arm");
    if (m_interface_provided) {
        // Arm State
        m_interface_provided->AddCommandWrite(&mtsIntuitiveResearchKitSUJFixed::state_command,
                                              this, "state_command", std::string(""));
        m_interface_provided->AddCommandReadState(StateTable,
                                                  m_operating_state, "operating_state");
        // Events
        m_interface_provided->AddMessageEvents();
        m_interface_provided->AddEventWrite(state_events.operating_state, "operating_state", prmOperatingState());
        // Stats
        m_interface_provided->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                                  "period_statistics");
    }
}


void mtsIntuitiveResearchKitSUJFixed::Configure(const std::string & filename)
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

    // find all arms, there should be 4 of them
    const Json::Value jsonArms = jsonConfig["arms"];
    if (jsonArms.size() != 4) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to find 4 SUJ arms" << std::endl;
        exit(EXIT_FAILURE);
    }

    mtsIntuitiveResearchKitSUJFixedArmData * sarm;
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

        // add interfaces, one is provided so users can find the SUJ
        // info, the other is required to the SUJ can get position of
        // ECM and change base frame on attached arms
        mtsInterfaceProvided * interfaceProvided = this->AddInterfaceProvided(name);
        mtsInterfaceRequired * interfaceRequired = this->AddInterfaceRequired(name, MTS_OPTIONAL);
        sarm = new mtsIntuitiveResearchKitSUJFixedArmData(name,
                                                          interfaceProvided,
                                                          interfaceRequired);
        m_sarms[index] = sarm;

        // save which arm is the Reference Arm
        if (name == reference_sarm_name) {
            m_reference_arm_index = index;
        }

        // Arm State so GUI widget for each arm can set/get state
        interfaceProvided->AddCommandWrite(&mtsIntuitiveResearchKitSUJFixed::state_command,
                                           this, "state_command", std::string(""));

        // look for hard coded position if available - users can always push new joint values using ROS
        sarm->m_state_table.Start();
        Json::Value json_cp = jsonArm["measured_cp"];
        std::cerr << json_cp << std::endl;
        if (!json_cp.empty()) {
            vctFrm4x4 transform;
            try {
                cmnDataJSON<vctFrm4x4>::DeSerializeText(transform, json_cp);
            } catch (std::exception & e) {
                CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                         << ": caught exception \"" << e.what() << " for "
                                         << name << " measured_cp" << std::endl;
                exit(EXIT_FAILURE);
            }
            sarm->m_measured_cp.Position().From(transform);
            sarm->m_measured_cp.SetValid(true);
        } else {
            CMN_LOG_CLASS_INIT_WARNING << "Configure: failed to load \"measured_cp\" for \""
                                       << name << "\"" << std::endl;
            sarm->m_measured_cp.SetValid(false);
        }
        sarm->m_state_table.Advance();
    }
}


void mtsIntuitiveResearchKitSUJFixed::update_operating_state_and_busy(const prmOperatingState::StateType & state,
                                                                      const bool isBusy)
{
    m_operating_state.State() = state;
    m_operating_state.IsBusy() = isBusy;
    dispatch_operating_state();
}


void mtsIntuitiveResearchKitSUJFixed::state_changed(void)
{
    dispatch_status(this->GetName() + ": current state " + m_state_machine.CurrentState());
    dispatch_operating_state();
}


void mtsIntuitiveResearchKitSUJFixed::run_all_states(void)
{
    // states are fairly meaningless so just change to desired state
    // is different
    if (m_state_machine.DesiredStateIsNotCurrent()) {
        m_state_machine.SetCurrentState(m_state_machine.DesiredState());
    }

    // get robot data, i.e. process mux/pots
    get_robot_data();
    // update all forward kinematics
    update_forward_kinematics();
}


void mtsIntuitiveResearchKitSUJFixed::enter_DISABLED(void)
{
    update_operating_state_and_busy(prmOperatingState::DISABLED, false);
}


void mtsIntuitiveResearchKitSUJFixed::enter_ENABLED(void)
{
    update_operating_state_and_busy(prmOperatingState::ENABLED, false);
}


void mtsIntuitiveResearchKitSUJFixed::Startup(void)
{
    set_desired_state("DISABLED");
}


void mtsIntuitiveResearchKitSUJFixed::Run(void)
{
    // collect data from required interfaces
    ProcessQueuedEvents();
    try {
        m_state_machine.Run();
    } catch (std::exception & e) {
        dispatch_error(this->GetName() + ": in state " + m_state_machine.CurrentState()
                       + ", caught exception \"" + e.what() + "\"");
        set_desired_state("DISABLED");
    }
    // trigger ExecOut event
    RunEvent();
    ProcessQueuedCommands();
}


void mtsIntuitiveResearchKitSUJFixed::Cleanup(void)
{
}


void mtsIntuitiveResearchKitSUJFixed::set_simulated(void)
{
    dispatch_warning(this->GetName() + ": simulated mode doesn't make sense on fixed SUJ");
}


void mtsIntuitiveResearchKitSUJFixed::get_robot_data(void)
{
    // for fixed arm, no data from robot
}

void mtsIntuitiveResearchKitSUJFixed::update_forward_kinematics(void)
{
    // find the reference arm (usually ECM)
    prmPositionCartesianGet reference_arm_local_cp;
    prmPositionCartesianGet reference_arm_to_cart_cp;

    mtsIntuitiveResearchKitSUJFixedArmData * reference_sarm = m_sarms[m_reference_arm_index];
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
        reference_arm_to_cart_cp.SetTimestamp(reference_arm_local_cp.Timestamp());
    }

    for (size_t arm_index = 0; arm_index < 4; ++arm_index) {
        mtsIntuitiveResearchKitSUJFixedArmData * sarm = m_sarms[arm_index];
        // update positions with base frame, local positions are only
        // updated from FK when joints are ready
        if (arm_index != m_reference_arm_index) {
            sarm->m_base_frame.From(reference_arm_to_cart_cp.Position());
            sarm->m_base_frame_valid = reference_arm_to_cart_cp.Valid();
            sarm->m_measured_cp.SetReferenceFrame(reference_arm_to_cart_cp.ReferenceFrame());
        }
        vctFrm4x4 local_cp(sarm->m_local_measured_cp.Position());
        vctFrm4x4 cp = sarm->m_base_frame * local_cp;
        // - with base frame
        sarm->m_measured_cp.Position().From(cp);
        std::cerr << CMN_LOG_DETAILS << " --- need to set a timestamp " << std::endl;
        // sarm->m_measured_cp.SetTimestamp(arm->m_measured_js.Timestamp());
        sarm->EventPositionCartesian(sarm->m_measured_cp);
        // - set base frame for the arm
        prmPositionCartesianSet setpoint_cp;
        setpoint_cp.Goal().Assign(sarm->m_measured_cp.Position());
        setpoint_cp.Valid() = sarm->m_measured_cp.Valid();
        setpoint_cp.Timestamp() = sarm->m_measured_cp.Timestamp();
        setpoint_cp.ReferenceFrame() = sarm->m_measured_cp.ReferenceFrame();
        setpoint_cp.MovingFrame() = sarm->m_measured_cp.MovingFrame();
        sarm->m_arm_set_base_frame(setpoint_cp);
    }
}


void mtsIntuitiveResearchKitSUJFixed::set_desired_state(const std::string & state)
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
        dispatch_error(this->GetName() + ": " + state + " is not an allowed desired state");
        return;
    }

    dispatch_operating_state();
    dispatch_status(this->GetName() + ": desired state " + state);

    // state transitions with direct transitions
    if (state == "DISABLED") {
        m_state_machine.SetCurrentState(state);
    }
}


void mtsIntuitiveResearchKitSUJFixed::state_command(const std::string & command)
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
            dispatch_warning(this->GetName() + ": " + humanReadableMessage);
        }
    } catch (std::runtime_error & e) {
        dispatch_warning(this->GetName() + ": " + command + " doesn't seem to be a valid state_command (" + e.what() + ")");
    }
}


void mtsIntuitiveResearchKitSUJFixed::set_homed(const bool homed)
{
    if (homed != m_operating_state.IsHomed()) {
        m_operating_state.IsHomed() = homed;
        dispatch_operating_state();
    }
}


void mtsIntuitiveResearchKitSUJFixed::dispatch_error(const std::string & message)
{
    m_interface_provided->SendError(message);
    for (auto sarm : m_sarms) {
        sarm->m_interface_provided->SendError(sarm->m_name + " " + message);
    }
}


void mtsIntuitiveResearchKitSUJFixed::dispatch_warning(const std::string & message)
{
    m_interface_provided->SendWarning(message);
    for (auto sarm : m_sarms) {
        sarm->m_interface_provided->SendWarning(sarm->m_name + " " + message);
    }
}


void mtsIntuitiveResearchKitSUJFixed::dispatch_status(const std::string & message)
{
    m_interface_provided->SendStatus(message);
    for (auto sarm : m_sarms) {
        sarm->m_interface_provided->SendStatus(sarm->m_name + " " + message);
    }
}


void mtsIntuitiveResearchKitSUJFixed::dispatch_operating_state(void)
{
    state_events.operating_state(m_operating_state);
    for (auto sarm : m_sarms) {
        sarm->state_events.operating_state(m_operating_state);
        sarm->state_events.current_state(m_state_machine.CurrentState());
        sarm->state_events.desired_state(m_state_machine.DesiredState());
    }
}
