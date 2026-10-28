/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2026 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/rigidbodyinertia.hpp>
#include <time.h>

// cisst
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstRobot/kdl_utilities.h>

#include <sawIntuitiveResearchKit/robManipulator.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitECM.h>
#include <sawIntuitiveResearchKit/robManipulatorECM.h>

// ECM-specific GC - accesses ECM's Manipulator so knows current endoscope mass
class GravityCompensationECM : public robGravityCompensation
{
public:
    static std::unique_ptr<GravityCompensationECM> configure(
        std::string physical_dh_file
    );
    std::string error(void);

    void set_endoscope_mass(double mass);
    void setGravity(const Eigen::Vector3d& gravity) override;
    Eigen::VectorXd compute(const prmStateJoint& state) override;

private:
    GravityCompensationECM(const KDL::Chain& chain);

    KDL::Chain physical_model;
    std::unique_ptr<KDL::ChainIdSolver_RNE> id_solver;
    std::string error_message;

    KDL::JntArray q_in;
    KDL::JntArray qdot_in;
    KDL::Wrenches f_ext;
    KDL::JntArray effort_out;
};

GravityCompensationECM::GravityCompensationECM(const KDL::Chain& chain)
    : physical_model(chain),
      id_solver(std::make_unique<KDL::ChainIdSolver_RNE>(
          physical_model, KDL::Vector::Zero()
      ))
{
    qdot_in.data.setZero(physical_model.getNrOfJoints());
    f_ext = std::vector<KDL::Wrench>(
        physical_model.getNrOfSegments(), KDL::Wrench::Zero()
    );
    effort_out = KDL::JntArray(physical_model.getNrOfJoints());
}

void GravityCompensationECM::setGravity(const Eigen::Vector3d& gravity)
{
    // Can't update ID solver's gravity vector, so we re-instantiate instead
    id_solver = std::make_unique<KDL::ChainIdSolver_RNE>(
        physical_model, -toKDL(gravity)
    );
}

std::unique_ptr<GravityCompensationECM> GravityCompensationECM::configure(
    std::string physical_dh_file
)
{
    std::ifstream json_stream(physical_dh_file);

    Json::Reader json_reader;
    Json::Value json_config;

    if (!json_reader.parse(json_stream, json_config)) {
        CMN_LOG_INIT_ERROR << "Configure GravityCompensationECM"
                           << ": failed to parse configuration file \""
                           << physical_dh_file << "\"\n"
                           << json_reader.getFormattedErrorMessages();
        return nullptr;
    }

    const Json::Value json_dh = json_config["DH"];
    if (json_dh.isNull() || !json_dh.isObject()) {
        CMN_LOG_INIT_ERROR << "Configure GravityCompensationECM"
                           << ": failed to load find DH parameters in \""
                           << physical_dh_file << "\"\n";
        return nullptr;
    }

    const auto kdl_chain = loadKDLChain(json_dh);
    if (!kdl_chain.has_value()) {
        CMN_LOG_INIT_ERROR
            << "Configure GravityCompensationECM"
            << ": failed to load kinematics parameters from file \""
            << physical_dh_file << "\"\n";
        return nullptr;
    }

    // make sure we have expected number of links so we know which link
    // to put the variable endoscope/camera mass
    const int num_segments = kdl_chain->getNrOfSegments();
    if (num_segments != 5 && num_segments != 4) {
        CMN_LOG_INIT_ERROR << "Configure GravityCompensationECM"
                           << ": does not match Classic or Si number of links, "
                           << physical_dh_file << "\"\n";
        return nullptr;
    }

    return std::unique_ptr<GravityCompensationECM>(
        new GravityCompensationECM(*kdl_chain)
    );
}

std::string GravityCompensationECM::error(void) { return error_message; }

void GravityCompensationECM::set_endoscope_mass(double mass)
{
    // Si vs Classic
    if (physical_model.getNrOfSegments() == 5) {
        // Si
        auto inertia_data = physical_model.segments.at(4).getInertia();
        KDL::RigidBodyInertia updated_inertia(
            mass, inertia_data.getCOG(), inertia_data.getRotationalInertia()
        );
        physical_model.segments.at(4).setInertia(updated_inertia);
    } else {
        // Classic
        auto inertia_data = physical_model.segments.at(2).getInertia();
        KDL::RigidBodyInertia updated_inertia(
            mass, inertia_data.getCOG(), inertia_data.getRotationalInertia()
        );
        physical_model.segments.at(2).setInertia(updated_inertia);
    }
}

Eigen::VectorXd GravityCompensationECM::compute(const prmStateJoint& state)
{
    size_t n_joints = physical_model.getNrOfJoints();
    const auto& j = state.Position();

    Eigen::VectorXd efforts = Eigen::VectorXd::Zero(state.Position().size());

    if (n_joints == 5) {
        // convert virtual joint positions to physical
        q_in.data = Eigen::VectorXd({{j[0], j[1], -j[1], j[1], j[2]}});
        id_solver->CartToJnt(q_in, qdot_in, qdot_in, f_ext, effort_out);
        efforts[0] = effort_out(0);
        efforts[1] = effort_out(1) - effort_out(2) + effort_out(3);
        efforts[2] = effort_out(4);
    } else if (n_joints == 4) {
        q_in.data = Eigen::VectorXd({{j[0], j[1], j[2], 0.0}});
        id_solver->CartToJnt(q_in, qdot_in, qdot_in, f_ext, effort_out);
        efforts.head<3>() = effort_out.data.head<3>();
    }

    return efforts;
}

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitECM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsIntuitiveResearchKitECM::mtsIntuitiveResearchKitECM(const std::string & componentName, const double periodInSeconds):
    mtsIntuitiveResearchKitArm(componentName, periodInSeconds)
{
    Init();
}

mtsIntuitiveResearchKitECM::mtsIntuitiveResearchKitECM(const mtsTaskPeriodicConstructorArg & arg):
    mtsIntuitiveResearchKitArm(arg)
{
    Init();
}

// need to define destructor after definition of GravityCompensationECM is available
mtsIntuitiveResearchKitECM::~mtsIntuitiveResearchKitECM() = default;

void mtsIntuitiveResearchKitECM::set_simulation_mode(const prmSimulationType & mode)
{
    // forward the simulation mode setup to the base class
    mtsIntuitiveResearchKitArm::set_simulation_mode(mode);

    // for KINEMATIC simulation mode we don't need IO
    if (m_simulation_mode == prmSimulationType::KINEMATIC) {
        // in simulation mode, we don't need clutch IO
        RemoveInterfaceRequired("arm_clutch");
        // for Si systems, remove a few more interfaces
        if (m_generation == dvrk::generation::Si) {
            RemoveInterfaceRequired("SUJ_clutch");
            RemoveInterfaceRequired("SUJ_clutch_2");
            RemoveInterfaceRequired("SUJ_brake");
        }
    }
}

void mtsIntuitiveResearchKitECM::set_generation(const dvrk::generation generation)
{
    mtsIntuitiveResearchKitArm::set_generation(generation);
    // for S/si, add SUJClutch interface
    if ((generation == dvrk::generation::Si)
        && (m_simulation_mode != prmSimulationType::KINEMATIC)) {
        auto interfaceRequired = AddInterfaceRequired("SUJ_clutch");
        if (interfaceRequired) {
            interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitECM::EventHandlerSUJClutch, this, "Button");
        }
        interfaceRequired = AddInterfaceRequired("SUJ_clutch_2");
        if (interfaceRequired) {
            interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitECM::EventHandlerSUJClutch, this, "Button");
        }
        interfaceRequired = AddInterfaceRequired("SUJ_brake");
        if (interfaceRequired) {
            interfaceRequired->AddFunction("SetValue", SUJClutch.Brake);
        }
    } else {
        if (GetInterfaceProvided("SUJ_clutch")) {
            RemoveInterfaceRequired("SUJ_clutch");
        }
    }
}

void mtsIntuitiveResearchKitECM::PostConfigure(const Json::Value & jsonConfig,
                                               const cmnPath & CMN_UNUSED(configPath),
                                               const std::string & filename)
{
    // load tool tip transform if any (for up/down endoscopes)
    // future work: add UP/DOWN, _HD and set mass for GC, also create separate method with ROS topic + GUI to set the endoscope
    const Json::Value jsonEndoscope = jsonConfig["endoscope"];
    if (!jsonEndoscope.isNull()) {
        std::string endoscope = jsonEndoscope.asString();
        set_endoscope_type(endoscope);
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                 << ": \"endoscope\" must be defined (from file \""
                                 << filename << "\")" << std::endl;
        exit(EXIT_FAILURE);
    }

    if (!m_endoscope_configured) {
        exit(EXIT_FAILURE);
    }

    // ask to set pitch if not already defined
    if (m_mounting_pitch > std::numeric_limits<double>::max()) {
        if (generation() == dvrk::generation::Classic) {
            m_mounting_pitch = -45.0 * cmnPI_180;
        } else {
            m_mounting_pitch = -70.0 * cmnPI_180;
        }
        CMN_LOG_CLASS_INIT_ERROR << "Configure: " << this->GetName() << std::endl
                                 << "\"mounting_pitch\" was not defined in \"" << filename << "\"" << std::endl
                                 << "If your ECM is mounted on the SUJ you should add it using: " << std::endl
                                 << " \"mounting_pitch\": " << std::to_string(m_mounting_pitch)
                                 << " // " << std::to_string(m_mounting_pitch * cmn180_PI) << " degrees" << std::endl;
        exit(EXIT_FAILURE);
    }
}

void mtsIntuitiveResearchKitECM::ConfigureGC(const Json::Value & jsonConfig,
                                               const cmnPath & configPath,
                                               const std::string & filename)
{
    std::string physical_dh_name;
    const auto jsonPhysicalDH = jsonConfig["kinematic_gc"];
    if (!jsonPhysicalDH.isNull()) {
        physical_dh_name = jsonPhysicalDH.asString();
    } else if (m_generation == dvrk::generation::Si) {
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure" << GetName() << ": no GC kinematics specified, using default for ECM Si" << std::endl;
        physical_dh_name = "kinematic/ECM_Si_physical.json";
    } else if (m_generation == dvrk::generation::Classic) {
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure" << GetName() << ": no GC kinematics specified, using default for ECM Classic" << std::endl;
        physical_dh_name = "kinematic/ECM.json";
    } else {
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure" << GetName() << ": no GC kinematics specified, so gravity compensation is not available" << std::endl;
        return;
    }

    const auto physical_dh = configPath.Find(physical_dh_name);
    if (physical_dh == "") {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: " << this->GetName()
                                 << " using file \"" << filename << "\", can't find GC kinematics file \""
                                 << physical_dh << "\"" << std::endl;
        exit(EXIT_FAILURE);
    }

    m_gc = GravityCompensationECM::configure(physical_dh);
    if (m_gc) {
        m_rob_gravity_compensation = m_gc.get();
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureGC: failed to configure using GC kinematics file \"" << physical_dh << "\"";
        exit(EXIT_FAILURE);
    }
}

void mtsIntuitiveResearchKitECM::initializeKinematics(const prmConfigurationJoint& joint_config, const KDL::Chain& chain)
{
    Manipulator = std::make_unique<robManipulatorECM>(joint_config, chain);
}

void mtsIntuitiveResearchKitECM::Init(void)
{
    // main initialization from base type
    mtsIntuitiveResearchKitArm::Init();

    ToolOffset = 0;

    // state machine specific to ECM, see base class for other states
    mArmState.AddState("MANUAL");

    // after arm homed
    mArmState.SetEnterCallback("HOMED",
                               &mtsIntuitiveResearchKitECM::EnterHomed,
                               this);
    mArmState.SetEnterCallback("MANUAL",
                               &mtsIntuitiveResearchKitECM::EnterManual,
                               this);
    mArmState.SetRunCallback("MANUAL",
                             &mtsIntuitiveResearchKitECM::RunManual,
                             this);
    mArmState.SetLeaveCallback("MANUAL",
                               &mtsIntuitiveResearchKitECM::LeaveManual,
                               this);

    // initialize trajectory data
    m_trajectory_j.v_max = Eigen::VectorXd({{30.0 * cmnPI_180, // degrees per second
                                30.0 * cmnPI_180,
                                60.0 * cmn_mm,    // mm per second
                                30.0 * cmnPI_180}});
    m_trajectory_j.a_max = Eigen::VectorXd({{90.0 * cmnPI_180,
                                90.0 * cmnPI_180,
                                60.0 * cmn_mm,
                                90.0 * cmnPI_180}});
    m_trajectory_j.goal_tolerance.setConstant(3.0 * cmnPI / 180.0); // hard coded to 3 degrees

    mtsInterfaceRequired * interfaceRequired;

    // Main interface should have been created by base class init
    CMN_ASSERT(m_arm_interface);
    m_arm_interface->AddEventWrite(ClutchEvents.ManipClutch, "arm_clutch", prmEventButton());

    // Unit gravity direction at the lenses; zero when unavailable.
    m_gravity_direction.setZero();
    StateTable.AddData(m_gravity_direction, "gravity_direction");
    m_arm_interface->AddCommandReadState(StateTable, m_gravity_direction, "gravity_direction");

    // endoscope commands and events
    m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitECM::set_endoscope_type, this, "set_endoscope_type");
    m_arm_interface->AddEventWrite(EndoscopeEvents.endoscope_type, "endoscope_type", std::string());

    // ManipClutch: digital input button event from ECM
    interfaceRequired = AddInterfaceRequired("arm_clutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitECM::EventHandlerManipClutch, this, "Button");
    }
}

bool mtsIntuitiveResearchKitECM::is_homed(void) const
{
    return m_powered && m_encoders_biased_from_pots;
}

void mtsIntuitiveResearchKitECM::unhome(void)
{
    m_encoders_biased_from_pots = false;
}

bool mtsIntuitiveResearchKitECM::is_joint_ready(void) const
{
    return m_powered && m_encoders_biased_from_pots;
}

bool mtsIntuitiveResearchKitECM::is_cartesian_ready(void) const
{
    return m_powered && m_encoders_biased_from_pots;
}

Eigen::Isometry3d mtsIntuitiveResearchKitECM::safe_cartesian_goal(const Eigen::Isometry3d& raw_goal) const {
    Eigen::Isometry3d safe_goal = raw_goal;

    // Make sure we're at least 1mm away from RCM to prevent numerical instability in next step
    if (raw_goal.translation().norm() < 1.0 * cmn_mm) {
        safe_goal.translation().z() = -1.0 * cmn_mm;
    }

    // Enforce a 40mm minimum insertion
    if (safe_goal.translation().norm() < 40.0 * cmn_mm) {
        safe_goal.translation() = 40.0 * cmn_mm * safe_goal.translation().normalized();
    }

    return safe_goal;
}

Eigen::Isometry3d mtsIntuitiveResearchKitECM::safe_cartesian_goal(const Eigen::Isometry3d& raw_goal, const Eigen::Isometry3d& current_pose) const {
    Eigen::Isometry3d safe_goal = raw_goal;
    
    // Make sure we're at least 1mm away from RCM to prevent numerical instability in next step
    if (raw_goal.translation().norm() < 1.0 * cmn_mm) {
        safe_goal.translation().z() = -1.0 * cmn_mm;
    }

    // Enforce a 40mm minimum insertion
    const double goal_rcm_distance = raw_goal.translation().norm();
    if (goal_rcm_distance < 40.0 * cmn_mm) {
        const double current_rcm_distance = current_pose.translation().norm();
        const double safe_distance = std::max(current_rcm_distance, goal_rcm_distance);
        safe_goal.translation() = safe_distance * raw_goal.translation().normalized();
    }

    return safe_goal;
}

void mtsIntuitiveResearchKitECM::SetGoalHomingArm(void)
{
    // if simulated, start at zero but insert endoscope so it can be used in cartesian mode
    if (m_simulation_mode == prmSimulationType::KINEMATIC) {
        m_trajectory_j.goal.setZero();
        m_trajectory_j.goal(2) = 12.0 * cmn_cm;
        return;
    }

    // make sure tracking error is set
    PID.enable_measured_setpoint_check(true);

    // compute joint goal position
    m_trajectory_j.goal = Eigen::VectorXd(number_of_joints());
    if (m_homing_goes_to_zero) {
        // move to zero position
        m_trajectory_j.goal.setZero();
    } else {
        // stay at current position by default
        m_trajectory_j.goal = m_pid_setpoint_js.Position().head(number_of_joints());
    }
}

void mtsIntuitiveResearchKitECM::EnterHomed(void)
{
    mtsIntuitiveResearchKitArm::EnterHomed();

    // event to propagate endoscope type based on configuration file
    EndoscopeEvents.endoscope_type(mtsIntuitiveResearchKitEndoscopeTypes::TypeToString(m_endoscope_type));
}

void mtsIntuitiveResearchKitECM::EnterManual(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, true);
    free();
}

void mtsIntuitiveResearchKitECM::RunManual(void)
{
    if (mControlCallback) {
        mControlCallback->Execute();
    }
}

void mtsIntuitiveResearchKitECM::LeaveManual(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, false);
    hold();
}

void mtsIntuitiveResearchKitECM::EventHandlerTrackingError(void)
{
    m_arm_interface->SendError(this->GetName() + ": PID tracking error");
    SetDesiredState("FAULT");
}

void mtsIntuitiveResearchKitECM::EventHandlerManipClutch(const prmEventButton & button)
{
    // Pass events
    ClutchEvents.ManipClutch(button);

    // Start manual mode but save the previous state
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        if (ArmIsReady("Clutch", mtsIntuitiveResearchKitControlTypes::JOINT_SPACE)) {
            ClutchEvents.ManipClutchPreviousState = mArmState.CurrentState();
            mArmState.SetCurrentState("MANUAL");
            set_LED_pattern(mtsIntuitiveResearchKit::Blue200,
                            mtsIntuitiveResearchKit::Green200,
                            true, false);
        } else {
            m_arm_interface->SendWarning(this->GetName() + ": arm not ready yet, manipulator clutch ignored");
        }
        break;
    case prmEventButton::RELEASED:
        if (mArmState.CurrentState() == "MANUAL") {
            set_LED_pattern(mtsIntuitiveResearchKit::Green200,
                            mtsIntuitiveResearchKit::Green200,
                            false, false);
            // go back to state before clutching
            mArmState.SetCurrentState(ClutchEvents.ManipClutchPreviousState);
        }
        break;
    default:
        break;
    }
}

void mtsIntuitiveResearchKitECM::EventHandlerSUJClutch(const prmEventButton & button)
{
    bool value = (button.Type() == prmEventButton::PRESSED);
    if (value
        && (m_operating_state.State() != prmOperatingState::ENABLED)) {
        m_arm_interface->SendWarning(this->GetName() + ": arm needs to be enabled to release the SUJ brakes");
    } else {
        SUJClutch.Brake(value);
    }
}

void mtsIntuitiveResearchKitECM::get_robot_data(void)
{
    mtsIntuitiveResearchKitArm::get_robot_data();
    m_gravity_direction.setZero();
    if (!m_local_measured_cp.Valid() || !m_endoscope_configured
        || !std::isfinite(m_mounting_pitch)) {
        return;
    }
    // RNE gravity compensation uses the upward base acceleration.  Negate
    // it to report physical down, independently of gravity compensation.
    const Eigen::Vector3d down_in_base(0.0, -sin(m_mounting_pitch), -cos(m_mounting_pitch));
    // Local FK already includes the configured endoscope's lens rotation.
    const Eigen::Vector3d down_in_lens = m_local_measured_cp.Position().rotation().inverse() * down_in_base;
    m_gravity_direction = down_in_lens;
}

void mtsIntuitiveResearchKitECM::set_endoscope_type(const std::string & endoscopeType)
{
    // initialize configured flag
    m_endoscope_configured = false;
    m_gravity_direction.setZero();

    m_arm_interface->SendStatus(this->GetName() + ": setting up for endoscope type \"" + endoscopeType + "\"");
    // check if the endoscope is in the supported list
    auto found =
        std::find(mtsIntuitiveResearchKitEndoscopeTypes::TypeVectorString().begin(),
                  mtsIntuitiveResearchKitEndoscopeTypes::TypeVectorString().end(),
                  endoscopeType);
    if (found == mtsIntuitiveResearchKitEndoscopeTypes::TypeVectorString().end()) {
        m_arm_interface->SendError(this->GetName() + ": endoscope type \"" + endoscopeType + "\" is not supported");
        EndoscopeEvents.endoscope_type(std::string("ERROR"));
        return;
    }
    // supported endoscopes
    m_endoscope_type = mtsIntuitiveResearchKitEndoscopeTypes::TypeFromString(endoscopeType);

    // update tool tip offset
    switch (m_endoscope_type) {
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Custom_STRAIGHT:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Custom_UP:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Custom_DOWN:
        ToolOffsetTransformation = Eigen::Isometry3d(
            Eigen::Matrix4d({
                { 0.0,  1.0,  0.0,  0.0 },
                {-1.0,  0.0,  0.0,  0.0 },
                { 0.0,  0.0,  1.0,  0.18 },
                { 0.0,  0.0,  0.0,  1.0 }
            })
        );
        break;
    default:
        ToolOffsetTransformation = Eigen::Isometry3d(
            Eigen::Matrix4d({
            { 0.0,  1.0,  0.0,  0.0 },
            {-1.0,  0.0,  0.0,  0.0 },
            { 0.0,  0.0,  1.0,  0.0 },
            { 0.0,  0.0,  0.0,  1.0 }
            })
        );
        break;
    }

    Eigen::Isometry3d tip;
    tip.translation() = Eigen::Vector3d(0.0, 0.0, 0.0);
    switch (m_endoscope_type) {
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Classic_SD_UP:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Classic_HD_UP:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Si_HD_UP:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Custom_UP:
        // -30 degree rotation along X axis
        tip.linear() = Eigen::AngleAxisd( -30.0 * cmnPI_180, Eigen::Vector3d(1.0, 0.0, 0.0)).matrix();
        ToolOffsetTransformation = ToolOffsetTransformation * tip;
        break;
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Classic_SD_DOWN:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Classic_HD_DOWN:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Si_HD_DOWN:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Custom_DOWN:
        // 30 degree rotation along X axis
        tip.linear() = Eigen::AngleAxisd(30.0 * cmnPI_180, Eigen::Vector3d(1.0, 0.0, 0.0)).matrix();
        ToolOffsetTransformation = ToolOffsetTransformation * tip;
        break;
    default:
        break;
    }

    Manipulator->setEndEffectorOffset(ToolOffsetTransformation);

    // update estimated mass for gravity compensation
    double mass;
    switch (m_endoscope_type) {
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Classic_SD_STRAIGHT:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Classic_SD_UP:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Classic_SD_DOWN:
        mass = mtsIntuitiveResearchKit::ECM::ClassicSDMass;
        break;
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Classic_HD_STRAIGHT:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Classic_HD_UP:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Classic_HD_DOWN:
        mass = mtsIntuitiveResearchKit::ECM::ClassicHDMass;
        break;
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Si_HD_STRAIGHT:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Si_HD_UP:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Si_HD_DOWN:
        mass = mtsIntuitiveResearchKit::ECM::SiHDMass;
        break;
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Custom_STRAIGHT:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Custom_UP:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Custom_DOWN:
        mass = mtsIntuitiveResearchKit::ECM::CustomMass;
        break;
    default:
        mass = mtsIntuitiveResearchKit::ECM::EmptyMass;
        break;
    }

    if (!m_skip_gravity_compensation && m_gc) {
        m_gc->set_endoscope_mass(mass);
    }

    // set configured flag
    m_endoscope_configured = true;

    // event to inform other components (GUI/ROS)
    EndoscopeEvents.endoscope_type(endoscopeType);
}
