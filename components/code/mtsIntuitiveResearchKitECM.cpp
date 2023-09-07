/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen
  Created on: 2013-05-15

  (C) Copyright 2013-2023 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitECM.h>
#include <sawIntuitiveResearchKit/robManipulatorECM.h>

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

    // this is used for GC on classic systems only
    if (m_generation == GENERATION_Classic) {
        // check that Rtw0 is not set
        if (Manipulator->Rtw0 != vctFrm4x4::Identity()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": you can't define the base-offset for the ECM, it is hard coded so gravity compensation works properly.  We always assume the ECM is mounted at 45 degrees! (from file \""
                                     << filename << "\")" << std::endl;
            exit(EXIT_FAILURE);
        }

        // 45 degrees rotation to make sure Z points up, this helps with
        // cisstRobot::robManipulator gravity compensation
        vctFrame4x4<double> Rt(vctMatRot3(1.0,            0.0,            0.0,
                                          0.0,  sqrt(2.0)/2.0,  sqrt(2.0)/2.0,
                                          0.0, -sqrt(2.0)/2.0,  sqrt(2.0)/2.0),
                               vct3(0.0, 0.0, 0.0));
        Manipulator->Rtw0 = Rt;
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
    PID.DefaultTrackingErrorTolerance.SetSize(number_of_joints());
    PID.DefaultTrackingErrorTolerance.SetAll(7.0 * cmnPI_180); // 7 degrees on angles
    PID.DefaultTrackingErrorTolerance.Element(2) = 10.0 * cmn_mm; // 10 mm

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
    PID.SetTrackingErrorTolerance(PID.DefaultTrackingErrorTolerance);
    PID.EnableTrackingError(true);

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

    // set gravity compensation based on generation
    m_gravity_compensation = (generation() == GENERATION_Classic);

    // event to propagate endoscope type based on configuration file
    EndoscopeEvents.endoscope_type(mtsIntuitiveResearchKitEndoscopeTypes::TypeToString(m_endoscope_type));
}

void mtsIntuitiveResearchKitECM::EnterManual(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, true);
    PID.EnableTrackingError(false);
    SetControlEffortActiveJoints();
}

void mtsIntuitiveResearchKitECM::RunManual(void)
{
    // zero efforts
    m_servo_jf_vector.SetAll(0.0);
    if (m_gravity_compensation) {
        m_servo_jf_vector.Add(m_gravity_compensation_setpoint_js.Effort());
    }
    servo_jf_internal(m_servo_jf_vector);
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
        if (is_joint_ready()) {
            ClutchEvents.ManipClutchPreviousState = mArmState.CurrentState();
            mArmState.SetCurrentState("MANUAL");
        } else {
            m_arm_interface->SendWarning(this->GetName() + ": arm not ready yet, manipulator clutch ignored");
        }
        break;
    case prmEventButton::RELEASED:
        if (mArmState.CurrentState() == "MANUAL") {
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

void mtsIntuitiveResearchKitECM::update_feed_forward(vctDoubleVec & feedForward)
{
    feedForward.SetAll(0.0);
    if (!m_simulated) {
        feedForward.Add(m_gravity_compensation_setpoint_js.Effort());
    }
}

void mtsIntuitiveResearchKitECM::gravity_compensation(vctDoubleVec & efforts)
{
    vctDoubleVec qd(this->number_of_joints_kinematics(), 0.0);
    efforts.ForceAssign(Manipulator->CCG_MDH(m_kin_measured_js.Position(), qd, 9.81));
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
    ToolOffsetTransformation.Assign( 0.0,  1.0,  0.0,  0.0,
                                    -1.0,  0.0,  0.0,  0.0,
                                     0.0,  0.0,  1.0,  0.0,
                                     0.0,  0.0,  0.0,  1.0);
    vctFrm4x4 tip;
    tip.Translation().Assign(vct3(0.0, 0.0, 0.0));
    switch (m_endoscope_type) {
    case mtsIntuitiveResearchKitEndoscopeTypes::SD_UP:
    case mtsIntuitiveResearchKitEndoscopeTypes::HD_UP:
        // -30 degree rotation along X axis
        tip.Rotation().From(vctAxAnRot3(vct3(1.0, 0.0, 0.0), -30.0 * cmnPI_180));
        ToolOffsetTransformation = ToolOffsetTransformation * tip;
        break;
    case mtsIntuitiveResearchKitEndoscopeTypes::SD_DOWN:
    case mtsIntuitiveResearchKitEndoscopeTypes::HD_DOWN:
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
    case mtsIntuitiveResearchKitEndoscopeTypes::SD_STRAIGHT:
    case mtsIntuitiveResearchKitEndoscopeTypes::SD_UP:
    case mtsIntuitiveResearchKitEndoscopeTypes::SD_DOWN:
        mass = mtsIntuitiveResearchKit::ECM::SDMass;
        break;
    case mtsIntuitiveResearchKitEndoscopeTypes::HD_STRAIGHT:
    case mtsIntuitiveResearchKitEndoscopeTypes::HD_UP:
    case mtsIntuitiveResearchKitEndoscopeTypes::HD_DOWN:
        mass = mtsIntuitiveResearchKit::ECM::HDMass;
        break;
    default:
        mass = mtsIntuitiveResearchKit::ECM::EmptyMass;
        break;
    }

    // make sure we have enough joints in the kinematic chain
    CMN_ASSERT(Manipulator->links.size() == 4);
    Manipulator->links.at(3).MassData().Mass() = mass;

    // set configured flag
    m_endoscope_configured = true;

    // event to inform other components (GUI/ROS)
    EndoscopeEvents.endoscope_type(endoscopeType);
}
