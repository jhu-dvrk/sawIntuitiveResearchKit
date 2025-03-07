/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2025 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstCommon/cmnPath.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitECM.h>
#include <sawIntuitiveResearchKit/robManipulatorECM.h>

// ECM-specific GC - accesses ECM's Manipulator so knows current endoscope mass
class GravityCompensationECM : public robGravityCompensation {
public:
    bool configure(std::string physical_dh_file);
    std::string error(void);

    void set_endoscope_mass(double mass);
    vctVec compute(const prmStateJoint& state, vct3 gravity) override;
private:
    robManipulator physical_model;
    std::string error_message;
};

bool GravityCompensationECM::configure(std::string physical_dh_file)
{
    robManipulator::Errno err = physical_model.LoadRobot(physical_dh_file);
    if (err != robManipulator::ESUCCESS) {
        error_message = physical_model.mLastError;
        return false;
    }

    // make sure we have expected number of links so we know which link
    // to put the variable endoscope/camera mass
    if (physical_model.links.size() == 6) {
        return true; // 6 links, ECM Si physical DH
    } else if (physical_model.links.size() == 4) {
        return true; // 4 links, ECM Classic virtual DH used as physical DH
    } else {
        error_message = "ECM GC kinematics does not match Classic or Si number of links";
        return false; // unknown physical DH
    }
}

std::string GravityCompensationECM::error(void)
{
    return error_message;
}

void GravityCompensationECM::set_endoscope_mass(double mass)
{
    if (physical_model.links.size() == 6) {
        physical_model.links.at(5).MassData().Mass() = mass;
    } else {
        physical_model.links.at(2).MassData().Mass() = mass;
    }
}

vctVec GravityCompensationECM::compute(const prmStateJoint& state, vct3 gravity)
{
    size_t n_joints = physical_model.links.size();
    auto j = state.Position();

    vctDoubleVec qd(n_joints, 0.0);
    vctDoubleVec efforts(state.Position().size(), 0.0);

    if (n_joints == 6) {
        // convert virtual joint positions to physical
        vctDoubleVec q(6, j[0], 0.0, j[1], -j[1], j[1], j[2]);
        vctDoubleVec predicted_efforts = physical_model.CCG_MDH(q, qd, gravity);
        efforts[0] = predicted_efforts[0];
        efforts[1] = predicted_efforts[2] - predicted_efforts[3] + predicted_efforts[4];
        efforts[2] = predicted_efforts[5];
    } else if (n_joints == 4) {
        vctDoubleVec q(4, j[0], j[1], j[2], 0.0);
        efforts = physical_model.CCG_MDH(j, qd, gravity);
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

void mtsIntuitiveResearchKitECM::set_simulated(void)
{
    mtsIntuitiveResearchKitArm::set_simulated();
    // in simulation mode, we don't need clutch IO
    RemoveInterfaceRequired("ManipClutch");
    // for Si systems, remove a few more interfaces
    if (m_generation == GENERATION_Si) {
        RemoveInterfaceRequired("SUJClutch");
        RemoveInterfaceRequired("SUJClutch2");
        RemoveInterfaceRequired("SUJBrake");
    }
}

void mtsIntuitiveResearchKitECM::set_generation(const GenerationType generation)
{
    mtsIntuitiveResearchKitArm::set_generation(generation);
    // for S/si, add SUJClutch interface
    if ((generation == GENERATION_Si)
        && !m_simulated) {
        auto interfaceRequired = AddInterfaceRequired("SUJClutch");
        if (interfaceRequired) {
            interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitECM::EventHandlerSUJClutch, this, "Button");
        }
        interfaceRequired = AddInterfaceRequired("SUJClutch2");
        if (interfaceRequired) {
            interfaceRequired->AddEventHandlerWrite(&mtsIntuitiveResearchKitECM::EventHandlerSUJClutch, this, "Button");
        }
        interfaceRequired = AddInterfaceRequired("SUJBrake");
        if (interfaceRequired) {
            interfaceRequired->AddFunction("SetValue", SUJClutch.Brake);
        }
    } else {
        if (GetInterfaceProvided("SUJClutch")) {
            RemoveInterfaceRequired("SUJClutch");
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
        if (generation() == GENERATION_Classic) {
            m_mounting_pitch = -45.0 * cmnPI_180;
        } else {
            m_mounting_pitch = -70.0 * cmnPI_180;
        }
        CMN_LOG_CLASS_INIT_ERROR << "Configure: " << this->GetName() << std::endl
                                 << "\"mounting-pitch\" was not defined in \"" << filename << "\"" << std::endl
                                 << "If your ECM is mounted on the SUJ you should add it using: " << std::endl
                                 << " \"mounting-pitch\": " << std::to_string(m_mounting_pitch)
                                 << " // " << std::to_string(m_mounting_pitch * cmn180_PI) << " degrees" << std::endl;
        exit(EXIT_FAILURE);
    }
}

void mtsIntuitiveResearchKitECM::ConfigureGC(const Json::Value & jsonConfig,
                                               const cmnPath & configPath,
                                               const std::string & filename)
{
    std::string physical_dh_name;
    const auto jsonPhysicalDH = jsonConfig["kinematic-gc"];
    if (!jsonPhysicalDH.isNull()) {
        physical_dh_name = jsonPhysicalDH.asString();
    } else if (m_generation == GENERATION_Si) {
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure" << GetName() << ": no GC kinematics specified, using default for ECM Si" << std::endl;
        physical_dh_name = "kinematic/ecm-si-physical.json";
    } else if (m_generation == GENERATION_Classic) {
        CMN_LOG_CLASS_INIT_VERBOSE << "Configure" << GetName() << ": no GC kinematics specified, using default for ECM Classic" << std::endl;
        physical_dh_name = "kinematic/ecm.json";
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

    m_gc = std::make_unique<GravityCompensationECM>();
    bool ok = m_gc->configure(physical_dh);
    if (ok) {
        gravity_compensation = m_gc.get();
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureGC: " << this->GetName()
                                 << " using GC kinematics file \"" << physical_dh << "\", got error \""
                                 << m_gc->error() << "\"" << std::endl;
        exit(EXIT_FAILURE);
    }
}

robManipulator::Errno mtsIntuitiveResearchKitECM::InverseKinematics(vctDoubleVec & jointSet,
                                                                    const vctFrm4x4 & cartesianGoal) const
{
    // solve IK
    if (Manipulator->InverseKinematics(jointSet, cartesianGoal) == robManipulator::ESUCCESS) {
        // find closest solution mod 2 pi
        const double difference = m_kin_measured_js.Position()[3] - jointSet[3];
        const double differenceInTurns = nearbyint(difference / (2.0 * cmnPI));
        jointSet[3] = jointSet[3] + differenceInTurns * 2.0 * cmnPI;
        // make sure we are away from RCM point, this test is
        // simplistic
        if (jointSet[2] < 40.0 * cmn_mm) {
            jointSet[2] = 40.0 * cmn_mm;
        }
#if 0
        vctFrm4x4 forward = Manipulator->ForwardKinematics(jointSet);
        vctDouble3 diff;
        diff.DifferenceOf(forward.Translation(), cartesianGoal.Translation());
        std::cerr << cmnInternalTo_mm(diff.Norm()) << "mm ";
#endif
        return robManipulator::ESUCCESS;
    }
    return robManipulator::EFAILURE;
}

void mtsIntuitiveResearchKitECM::CreateManipulator(void)
{
    if (Manipulator) {
        delete Manipulator;
    }
    Manipulator = new robManipulatorECM();
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
    m_trajectory_j.v_max.Assign(30.0 * cmnPI_180, // degrees per second
                                30.0 * cmnPI_180,
                                60.0 * cmn_mm,    // mm per second
                                30.0 * cmnPI_180);
    m_trajectory_j.a_max.Assign(90.0 * cmnPI_180,
                                90.0 * cmnPI_180,
                                60.0 * cmn_mm,
                                90.0 * cmnPI_180);
    m_trajectory_j.goal_tolerance.SetAll(3.0 * cmnPI / 180.0); // hard coded to 3 degrees

    // default PID tracking errors
    PID.measured_setpoint_tolerance.SetSize(number_of_joints());
    PID.measured_setpoint_tolerance.SetAll(7.0 * cmnPI_180); // 7 degrees on angles
    PID.measured_setpoint_tolerance.Element(2) = 10.0 * cmn_mm; // 10 mm

    mtsInterfaceRequired * interfaceRequired;

    // Main interface should have been created by base class init
    CMN_ASSERT(m_arm_interface);
    m_arm_interface->AddEventWrite(ClutchEvents.ManipClutch, "ManipClutch", prmEventButton());

    // endoscope commands and events
    m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitECM::set_endoscope_type, this, "set_endoscope_type");
    m_arm_interface->AddEventWrite(EndoscopeEvents.endoscope_type, "endoscope_type", std::string());

    // ManipClutch: digital input button event from ECM
    interfaceRequired = AddInterfaceRequired("ManipClutch");
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

void mtsIntuitiveResearchKitECM::SetGoalHomingArm(void)
{
    // if simulated, start at zero but insert endoscope so it can be used in cartesian mode
    if (m_simulated) {
        m_trajectory_j.goal.SetAll(0.0);
        m_trajectory_j.goal.at(2) = 12.0 * cmn_cm;
        return;
    }

    // make sure tracking error is set
    PID.set_measured_setpoint_tolerance(PID.measured_setpoint_tolerance);
    PID.enable_measured_setpoint_check(true);

    // compute joint goal position
    m_trajectory_j.goal.SetSize(number_of_joints());
    if (m_homing_goes_to_zero) {
        // move to zero position
        m_trajectory_j.goal.SetAll(0.0);
    } else {
        // stay at current position by default
        m_trajectory_j.goal.Assign(m_pid_setpoint_js.Position(), number_of_joints());
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
        if (ArmIsReady("Clutch", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
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

void mtsIntuitiveResearchKitECM::set_endoscope_type(const std::string & endoscopeType)
{
    // initialize configured flag
    m_endoscope_configured = false;

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
        ToolOffsetTransformation.Assign( 0.0,  1.0,  0.0,  0.0,
                                         -1.0,  0.0,  0.0, 0.0,
                                         0.0,  0.0,  1.0,  0.18,
                                         0.0,  0.0,  0.0,  1.0);
        break;
    default:
        ToolOffsetTransformation.Assign( 0.0,  1.0,  0.0,  0.0,
                                         -1.0,  0.0,  0.0,  0.0,
                                         0.0,  0.0,  1.0,  0.0,
                                         0.0,  0.0,  0.0,  1.0);
        break;
    }

    vctFrm4x4 tip;
    tip.Translation().Assign(vct3(0.0, 0.0, 0.0));
    switch (m_endoscope_type) {
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Classic_SD_UP:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Classic_HD_UP:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Si_HD_UP:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Custom_UP:
        // -30 degree rotation along X axis
        tip.Rotation().From(vctAxAnRot3(vct3(1.0, 0.0, 0.0), -30.0 * cmnPI_180));
        ToolOffsetTransformation = ToolOffsetTransformation * tip;
        break;
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Classic_SD_DOWN:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Classic_HD_DOWN:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Si_HD_DOWN:
    case mtsIntuitiveResearchKitEndoscopeTypes::SCOPE_Custom_DOWN:
        // 30 degree rotation along X axis
        tip.Rotation().From(vctAxAnRot3(vct3(1.0, 0.0, 0.0), 30.0 * cmnPI_180));
        ToolOffsetTransformation = ToolOffsetTransformation * tip;
        break;
    default:
        break;
    }
    // remove old tip and replace by new one
    Manipulator->DeleteTools();
    ToolOffset = new robManipulator(ToolOffsetTransformation);
    Manipulator->Attach(ToolOffset);

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

    m_gc->set_endoscope_mass(mass);

    // set configured flag
    m_endoscope_configured = true;

    // event to inform other components (GUI/ROS)
    EndoscopeEvents.endoscope_type(endoscopeType);
}
