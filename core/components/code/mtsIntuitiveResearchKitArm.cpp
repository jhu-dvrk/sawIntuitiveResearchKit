/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen, Zerui Wang
  Created on: 2016-02-24

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
#include <cisstNumerical/nmrIsOrthonormal.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmInputData.h>
#include <sawControllers/osaCartesianImpedanceController.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitRevision.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitConfig.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArm.h>
#include <sawIntuitiveResearchKit/prmActuatorJointCouplingCheck.h>
#include <sawIntuitiveResearchKit/prmConfigurationJointFromManipulator.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsIntuitiveResearchKitArm, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsIntuitiveResearchKitArm::mtsIntuitiveResearchKitArm(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    mArmState(componentName, "DISABLED"),
    mStateTableState(100, "State"),
    mStateTableConfiguration(100, "Configuration"),
    mControlCallback(0)
{
    mCartesianImpedanceController = new osaCartesianImpedanceController();
}

mtsIntuitiveResearchKitArm::mtsIntuitiveResearchKitArm(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    mArmState(arg.Name, "DISABLED"),
    mStateTableState(100, "State"),
    mStateTableConfiguration(100, "Configuration"),
    mControlCallback(0)
{
    mCartesianImpedanceController = new osaCartesianImpedanceController();
}

mtsIntuitiveResearchKitArm::~mtsIntuitiveResearchKitArm()
{
    if (Manipulator) {
        delete Manipulator;
    }
    if (mCartesianImpedanceController) {
        delete mCartesianImpedanceController;
    }
}

void mtsIntuitiveResearchKitArm::CreateManipulator(void)
{
    if (Manipulator) {
        delete Manipulator;
    }
    Manipulator = new robManipulator();
}

void mtsIntuitiveResearchKitArm::Init(void)
{
    // configure state machine common to all arms (ECM/MTM/PSM)
    // possible states
    mArmState.AddState("POWERING");
    mArmState.AddState("ENABLED");
    mArmState.AddState("CALIBRATING_ENCODERS_FROM_POTS");
    mArmState.AddState("ENCODERS_BIASED");
    mArmState.AddState("HOMING");
    mArmState.AddState("HOMED");
    mArmState.AddState("PAUSED");
    mArmState.AddState("FAULT");

    // possible desired states
    mArmState.AddAllowedDesiredState("DISABLED");
    mArmState.AddAllowedDesiredState("ENABLED");
    mArmState.AddAllowedDesiredState("ENCODERS_BIASED");
    mArmState.AddAllowedDesiredState("HOMED");
    mArmState.AddAllowedDesiredState("PAUSED");
    mArmState.AddAllowedDesiredState("FAULT");

    // state change, to convert to string events for users (Qt, ROS)
    mArmState.SetStateChangedCallback(&mtsIntuitiveResearchKitArm::StateChanged,
                                      this);

    // run for all states
    mArmState.SetRunCallback(&mtsIntuitiveResearchKitArm::RunAllStates,
                             this);

    // disabled
    mArmState.SetEnterCallback("DISABLED",
                               &mtsIntuitiveResearchKitArm::EnterDisabled,
                               this);

    mArmState.SetTransitionCallback("DISABLED",
                                    &mtsIntuitiveResearchKitArm::TransitionDisabled,
                                    this);

    // power
    mArmState.SetEnterCallback("POWERING",
                               &mtsIntuitiveResearchKitArm::EnterPowering,
                               this);

    mArmState.SetTransitionCallback("POWERING",
                                    &mtsIntuitiveResearchKitArm::TransitionPowering,
                                    this);

    mArmState.SetEnterCallback("ENABLED",
                               &mtsIntuitiveResearchKitArm::EnterEnabled,
                               this);

    mArmState.SetTransitionCallback("ENABLED",
                                    &mtsIntuitiveResearchKitArm::TransitionEnabled,
                                    this);

    // bias encoders
    mArmState.SetEnterCallback("CALIBRATING_ENCODERS_FROM_POTS",
                               &mtsIntuitiveResearchKitArm::EnterCalibratingEncodersFromPots,
                               this);

    mArmState.SetTransitionCallback("CALIBRATING_ENCODERS_FROM_POTS",
                                    &mtsIntuitiveResearchKitArm::TransitionCalibratingEncodersFromPots,
                                    this);

    mArmState.SetEnterCallback("ENCODERS_BIASED",
                               &mtsIntuitiveResearchKitArm::EnterEncodersBiased,
                               this);

    mArmState.SetTransitionCallback("ENCODERS_BIASED",
                                    &mtsIntuitiveResearchKitArm::TransitionEncodersBiased,
                                    this);

    // arm homing
    mArmState.SetEnterCallback("HOMING",
                               &mtsIntuitiveResearchKitArm::EnterHoming,
                               this);

    mArmState.SetRunCallback("HOMING",
                             &mtsIntuitiveResearchKitArm::RunHoming,
                             this);

    // state HOMED depends on the arm type, see
    // derived classes
    mArmState.SetEnterCallback("HOMED",
                               &mtsIntuitiveResearchKitArm::EnterHomed,
                               this);

    mArmState.SetRunCallback("HOMED",
                             &mtsIntuitiveResearchKitArm::RunHomed,
                             this);

    mArmState.SetLeaveCallback("HOMED",
                               &mtsIntuitiveResearchKitArm::LeaveHomed,
                               this);

    // paused
    mArmState.SetEnterCallback("PAUSED",
                               &mtsIntuitiveResearchKitArm::EnterPaused,
                               this);

    // fault
    mArmState.SetEnterCallback("FAULT",
                               &mtsIntuitiveResearchKitArm::EnterFault,
                               this);

    // state table to maintain state :-)
    mStateTableState.AddData(mStateTableStateDesired, "desired_state");
    m_operating_state.SetValid(true);
    mStateTableState.AddData(m_operating_state, "operating_state");
    AddStateTable(&mStateTableState);
    mStateTableState.SetAutomaticAdvance(false);

    // state table for configuration
    mStateTableConfiguration.AddData(m_configuration_js, "configuration_js");
    AddStateTable(&mStateTableConfiguration);
    mStateTableConfiguration.SetAutomaticAdvance(false);

    m_control_space = mtsIntuitiveResearchKitControlTypes::UNDEFINED_SPACE;
    m_control_mode = mtsIntuitiveResearchKitControlTypes::UNDEFINED_MODE;

    m_safe_for_cartesian_control_counter = 0;
    mArmNotReadyCounter = 0;
    mArmNotReadyTimeLastMessage = 0.0;

    // initialize trajectory data
    m_servo_jp.SetSize(number_of_joints());
    m_servo_jv.SetSize(number_of_joints());
    m_servo_jp_param.Goal().SetSize(number_of_joints());
    m_trajectory_j.v_max.SetSize(number_of_joints());
    m_trajectory_j.v.SetSize(number_of_joints());
    m_trajectory_j.a_max.SetSize(number_of_joints());
    m_trajectory_j.a.SetSize(number_of_joints());
    m_trajectory_j.goal.SetSize(number_of_joints());
    m_trajectory_j.goal_v.SetSize(number_of_joints());
    m_trajectory_j.goal_error.SetSize(number_of_joints());
    m_trajectory_j.goal_tolerance.SetSize(number_of_joints());
    m_trajectory_j.is_active = false;

    // initialize velocity
    m_local_measured_cv.SetVelocityLinear(vct3(0.0));
    m_local_measured_cv.SetVelocityAngular(vct3(0.0));
    m_local_measured_cv.SetValid(false);
    m_measured_cv.SetVelocityLinear(vct3(0.0));
    m_measured_cv.SetVelocityAngular(vct3(0.0));
    m_measured_cv.SetValid(false);

    m_local_setpoint_cv.SetVelocityLinear(vct3(0.0));
    m_local_setpoint_cv.SetVelocityAngular(vct3(0.0));
    m_local_setpoint_cv.SetValid(false);
    m_setpoint_cv.SetVelocityLinear(vct3(0.0));
    m_setpoint_cv.SetVelocityAngular(vct3(0.0));
    m_setpoint_cv.SetValid(false);

    // base manipulator class used by most arms (except PSM with snake like tool)
    CreateManipulator();

    // jacobian
    ResizeKinematicsData();
    this->StateTable.AddData(m_body_jacobian, "body_jacobian");
    this->StateTable.AddData(m_spatial_jacobian, "spatial_jacobian");

    // efforts for kinematics
    m_servo_jf.ForceTorque().SetSize(number_of_joints_kinematics());
    m_servo_jf.ForceTorque().Zeros();
    m_body_measured_cf.SetValid(false);
    m_spatial_measured_cf.SetValid(false);

    // efforts computed by gravity compensation
    m_feed_forward_jf_param.ForceTorque().SetSize(number_of_joints());
    m_feed_forward_jf_param.ForceTorque().Zeros();
    m_gravity_compensation_setpoint_js.Effort().SetSize(number_of_joints_kinematics());
    m_gravity_compensation_setpoint_js.Effort().Zeros();
    m_gravity_compensation_setpoint_js.SetAutomaticTimestamp(false);
    m_gravity_compensation_setpoint_js.SetValid(false);
    this->StateTable.AddData(m_gravity_compensation_setpoint_js, "gravity_compensation/setpoint_js");

    // base frame, mostly for cases where no base frame is set by user
    m_base_frame = vctFrm4x4::Identity();
    m_base_frame_valid = true;

    m_measured_cp.SetAutomaticTimestamp(false); // based on PID timestamp
    m_measured_cp.SetReferenceFrame(GetName() + "_base");
    m_measured_cp.SetMovingFrame(GetName());
    this->StateTable.AddData(m_measured_cp, "measured_cp");

    m_setpoint_cp.SetAutomaticTimestamp(false); // based on PID timestamp
    m_setpoint_cp.SetReferenceFrame(GetName() + "_base");
    m_setpoint_cp.SetMovingFrame(GetName() + "_setpoint");
    this->StateTable.AddData(m_setpoint_cp, "setpoint_cp");

    m_local_measured_cp.SetAutomaticTimestamp(false); // based on PID timestamp
    m_local_measured_cp.SetReferenceFrame(GetName() + "_base");
    m_local_measured_cp.SetMovingFrame(GetName());
    this->StateTable.AddData(m_local_measured_cp, "local/measured_cp");

    m_local_setpoint_cp.SetAutomaticTimestamp(false); // based on PID timestamp
    m_local_setpoint_cp.SetReferenceFrame(GetName() + "_base");
    m_local_setpoint_cp.SetMovingFrame(GetName() + "_setpoint");
    this->StateTable.AddData(m_local_setpoint_cp, "local/setpoint_cp");

    this->StateTable.AddData(m_base_frame, "base_frame");

    m_local_measured_cv.SetAutomaticTimestamp(false); // keep PID timestamp
    m_local_measured_cv.SetMovingFrame(GetName());
    m_local_measured_cv.SetReferenceFrame(GetName() + "_base");
    this->StateTable.AddData(m_local_measured_cv, "local/measured_cv");

    m_measured_cv.SetAutomaticTimestamp(false); // keep PID timestamp
    m_measured_cv.SetMovingFrame(GetName());
    m_measured_cv.SetReferenceFrame(GetName() + "_base");
    this->StateTable.AddData(m_measured_cv, "measured_cv");

    m_local_setpoint_cv.SetAutomaticTimestamp(false); // keep PID timestamp
    m_local_setpoint_cv.SetMovingFrame(GetName());
    m_local_setpoint_cv.SetReferenceFrame(GetName() + "_base");
    this->StateTable.AddData(m_local_setpoint_cv, "local/setpoint_cv");

    m_setpoint_cv.SetAutomaticTimestamp(false); // keep PID timestamp
    m_setpoint_cv.SetMovingFrame(GetName());
    m_setpoint_cv.SetReferenceFrame(GetName() + "_base");
    this->StateTable.AddData(m_setpoint_cv, "setpoint_cv");

    m_body_measured_cf.SetAutomaticTimestamp(false); // keep PID timestamp
    this->StateTable.AddData(m_body_measured_cf, "body/measured_cf");

    m_spatial_measured_cf.SetAutomaticTimestamp(false); // keep PID timestamp
    this->StateTable.AddData(m_spatial_measured_cf, "spatial/measured_cf");

    m_kin_measured_js.SetAutomaticTimestamp(false); // keep PID timestamp
    this->StateTable.AddData(m_kin_measured_js, "kin/measured_js");

    m_kin_setpoint_js.SetAutomaticTimestamp(false); // keep PID timestamp
    this->StateTable.AddData(m_kin_setpoint_js, "kin/setpoint_js");

    // PID
    PIDInterface = AddInterfaceRequired("PID");
    if (PIDInterface) {
        PIDInterface->AddFunction("enable", PID.enable);
        PIDInterface->AddFunction("enable_joints", PID.enable_joints);
        PIDInterface->AddFunction("enabled", PID.enabled);
        PIDInterface->AddFunction("measured_js", PID.measured_js);
        PIDInterface->AddFunction("setpoint_js", PID.setpoint_js);
        PIDInterface->AddFunction("servo_jp", PID.servo_jp);
        PIDInterface->AddFunction("feed_forward/servo_jf", PID.feed_forward_servo_jf);
        PIDInterface->AddFunction("enforce_position_limits", PID.enforce_position_limits);
        PIDInterface->AddFunction("EnableTorqueMode", PID.EnableTorqueMode);
        PIDInterface->AddFunction("servo_jf", PID.servo_jf);
        PIDInterface->AddFunction("enable_measured_setpoint_check", PID.enable_measured_setpoint_check);
        PIDInterface->AddFunction("set_measured_setpoint_tolerance", PID.set_measured_setpoint_tolerance);
        PIDInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::PositionLimitEventHandler, this, "PositionLimit");
        PIDInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::ErrorEventHandler, this, "error");
    }

    // Arm IO
    IOInterface = AddInterfaceRequired("RobotIO");
    if (IOInterface) {
        IOInterface->AddFunction("GetSerialNumber", IO.GetSerialNumber);
        IOInterface->AddFunction("PowerOnSequence", IO.PowerOnSequence);
        IOInterface->AddFunction("PowerOffSequence", IO.PowerOffSequence);
        IOInterface->AddFunction("Explain", IO.Explain);
        IOInterface->AddFunction("set_LED_pattern", IO.set_LED_pattern);
        IOInterface->AddFunction("GetActuatorAmpStatus", IO.GetActuatorAmpStatus);
        IOInterface->AddFunction("GetBrakeAmpStatus", IO.GetBrakeAmpStatus);
        IOInterface->AddFunction("BiasEncoder", IO.BiasEncoder);
        IOInterface->AddFunction("SetEncoderPosition", IO.SetEncoderPosition);
        IOInterface->AddFunction("SetSomeEncoderPosition", IO.SetSomeEncoderPosition);
        IOInterface->AddFunction("SetActuatorCurrent", IO.SetActuatorCurrent);
        IOInterface->AddFunction("UsePotsForSafetyCheck", IO.UsePotsForSafetyCheck);
        IOInterface->AddFunction("BrakeRelease", IO.BrakeRelease);
        IOInterface->AddFunction("BrakeEngage", IO.BrakeEngage);
        IOInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::BiasEncoderEventHandler, this, "BiasEncoder");
    }

    // Arm
    m_arm_interface = AddInterfaceProvided("Arm");
    if (m_arm_interface) {
        m_arm_interface->AddMessageEvents();

        // Get
        m_arm_interface->AddCommandRead(&mtsIntuitiveResearchKitArm::crtk_version, this, "crtk_version");
        m_arm_interface->AddCommandReadState(this->mStateTableConfiguration, m_configuration_js, "configuration_js");
        m_arm_interface->AddCommandReadState(this->StateTable, m_kin_measured_js, "measured_js");
        m_arm_interface->AddCommandReadState(this->StateTable, m_kin_setpoint_js, "setpoint_js");
        m_arm_interface->AddCommandReadState(this->StateTable, m_gravity_compensation_setpoint_js, "gravity_compensation/setpoint_js");
        m_arm_interface->AddCommandReadState(this->StateTable, m_local_measured_cp, "local/measured_cp");
        m_arm_interface->AddCommandReadState(this->StateTable, m_local_setpoint_cp, "local/setpoint_cp");
        m_arm_interface->AddCommandReadState(this->StateTable, m_measured_cp, "measured_cp");
        m_arm_interface->AddCommandReadState(this->StateTable, m_setpoint_cp, "setpoint_cp");
        m_arm_interface->AddCommandReadState(this->StateTable, m_base_frame, "base_frame");
        m_arm_interface->AddCommandReadState(this->StateTable, m_local_measured_cv, "local/measured_cv");
        m_arm_interface->AddCommandReadState(this->StateTable, m_local_setpoint_cv, "local/setpoint_cv");
        m_arm_interface->AddCommandReadState(this->StateTable, m_measured_cv, "measured_cv");
        m_arm_interface->AddCommandReadState(this->StateTable, m_setpoint_cv, "setpoint_cv");
        m_arm_interface->AddCommandReadState(this->StateTable, m_body_measured_cf, "body/measured_cf");
        m_arm_interface->AddCommandReadState(this->StateTable, m_body_jacobian, "body/jacobian");
        m_arm_interface->AddCommandReadState(this->StateTable, m_spatial_measured_cf, "spatial/measured_cf");
        m_arm_interface->AddCommandReadState(this->StateTable, m_spatial_jacobian, "spatial/jacobian");
        m_arm_interface->AddCommandReadState(this->mStateTableState,
                                             m_operating_state, "operating_state");
        // Set
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::set_base_frame,
                                         this, "set_base_frame");
        m_arm_interface->AddCommandVoid(&mtsIntuitiveResearchKitArm::hold,
                                        this, "hold");
        m_arm_interface->AddCommandVoid(&mtsIntuitiveResearchKitArm::free,
                                        this, "free");
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::servo_jp,
                                         this, "servo_jp");
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::servo_jr,
                                         this, "servo_jr");
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::move_jp,
                                         this, "move_jp");
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::move_jr,
                                         this, "move_jr");
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::servo_cp,
                                         this, "servo_cp");
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::servo_cr,
                                         this, "servo_cr_not_working_yet");
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::move_cp,
                                         this, "move_cp");
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::servo_jf,
                                         this, "servo_jf");
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::body_servo_cf,
                                         this, "body/servo_cf");
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::body_set_cf_orientation_absolute,
                                         this, "body/set_cf_orientation_absolute");
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::spatial_servo_cf,
                                         this, "spatial/servo_cf");
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::use_gravity_compensation,
                                         this, "use_gravity_compensation");
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::servo_ci,
                                         this, "servo_ci");

        // Actuator conversion
        m_arm_interface->AddCommandQualifiedRead(&mtsIntuitiveResearchKitArm::actuator_to_joint_position, this,
                                                 "actuator_to_joint_position", vctDoubleVec(), vctDoubleVec());

        // Kinematic queries
        m_arm_interface->AddCommandQualifiedRead(&mtsIntuitiveResearchKitArm::forward_kinematics,
                                                 this, "forward_kinematics");
        m_arm_interface->AddCommandQualifiedRead(&mtsIntuitiveResearchKitArm::local_forward_kinematics,
                                                 this, "local/forward_kinematics");
        m_arm_interface->AddCommandQualifiedRead(&mtsIntuitiveResearchKitArm::inverse_kinematics,
                                                 this, "inverse_kinematics");

        // Trajectory
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::trajectory_j_set_ratio_v,
                                         this, "trajectory_j/set_ratio_v");
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::trajectory_j_set_ratio_a,
                                         this, "trajectory_j/set_ratio_a");
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::trajectory_j_set_ratio,
                                         this, "trajectory_j/set_ratio");
        m_arm_interface->AddEventWrite(m_trajectory_j.ratio_v_event, "trajectory_j/ratio_v", double());
        m_arm_interface->AddEventWrite(m_trajectory_j.ratio_a_event, "trajectory_j/ratio_a", double());
        m_arm_interface->AddEventWrite(m_trajectory_j.ratio_event, "trajectory_j/ratio", double());
        m_arm_interface->AddEventWrite(m_trajectory_j.goal_reached_event, "goal_reached", bool());
        // Arm State
        m_arm_interface->AddCommandWrite(&mtsIntuitiveResearchKitArm::state_command,
                                         this, "state_command", std::string(""));
        // Human readable messages
        m_arm_interface->AddEventWrite(state_events.desired_state, "desired_state", std::string(""));
        m_arm_interface->AddEventWrite(state_events.current_state, "current_state", std::string(""));
        m_arm_interface->AddEventWrite(state_events.operating_state, "operating_state", prmOperatingState());

        // Stats
        m_arm_interface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                             "period_statistics");
    }

    // SetState will send log events, it needs to happen after the
    // provided interface has been created
    SetDesiredState("DISABLED");
}

void mtsIntuitiveResearchKitArm::SetDesiredState(const std::string & state)
{
    // setting desired state triggers a new event so user nows which state is current
    StateEvents();
    // try to find the state in state machine
    if (!mArmState.StateExists(state)) {
        m_arm_interface->SendError(this->GetName() + ": unsupported state " + state);
        return;
    }
    // try to set the desired state
    try {
        mArmState.SetDesiredState(state);
    } catch (...) {
        m_arm_interface->SendError(this->GetName() + ": " + state + " is not an allowed desired state");
        return;
    }
    // update state table
    mStateTableState.Start();
    mStateTableStateDesired = state;
    mStateTableState.Advance();

    StateEvents();
    m_arm_interface->SendStatus(this->GetName() + ": desired state " + state);

    // state transitions with direct transitions
    if ((state == "DISABLED")
        || (state == "PAUSED")
        || (state == "FAULT")) {
        mArmState.SetCurrentState(state);
    }
}

void mtsIntuitiveResearchKitArm::state_command(const std::string & command)
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
                SetDesiredState("HOMED");
                return;
            }
            if (command == "unhome") {
                unhome();
                UpdateHomed(false);
                return;
            }
            if (command == "pause") {
                SetDesiredState("PAUSED");
                return;
            }
            if (command == "resume") {
                mArmState.SetDesiredState(m_resume_desired_state);
                mArmState.SetCurrentState(m_resume_current_state);
                return;
            }
        } else {
            m_arm_interface->SendWarning(this->GetName() + ": " + humanReadableMessage);
        }
    } catch (std::runtime_error & e) {
        m_arm_interface->SendWarning(this->GetName() + ": " + command + " doesn't seem to be a valid state_command (" + e.what() + ")");
    }
}

void mtsIntuitiveResearchKitArm::update_configuration_js(void)
{
    // get names, types and joint limits for kinematics config from the manipulator
    // name and types need conversion
    mStateTableConfiguration.Start();
    prmConfigurationJointFromManipulator(*(this->Manipulator),
                                         number_of_joints_kinematics(),
                                         m_configuration_js);
    cmnDataCopy(m_gravity_compensation_setpoint_js.Name(),
                m_configuration_js.Name());
    mStateTableConfiguration.Advance();
}

void mtsIntuitiveResearchKitArm::actuator_to_joint_position(const vctDoubleVec & actuator,
                                                            vctDoubleVec & joint) const
{
    if (m_has_coupling) {
        joint = m_coupling.ActuatorToJointPosition() * actuator;
    } else {
        joint.ForceAssign(actuator);
    }
}

void mtsIntuitiveResearchKitArm::ResizeKinematicsData(void)
{
    // jacobians
    m_body_jacobian.SetSize(6, number_of_joints_kinematics());
    m_spatial_jacobian.SetSize(6, number_of_joints_kinematics());
    m_jacobian_pinverse_data.Allocate(m_body_jacobian);
    // transposes
    m_body_jacobian_transpose.ForceAssign(m_body_jacobian.Transpose());
    m_spatial_jacobian_transpose.ForceAssign(m_spatial_jacobian.Transpose());
    m_jacobian_transpose_pinverse_data.Allocate(m_body_jacobian_transpose);
    // servo
    m_servo_jf.ForceTorque().SetSize(number_of_joints_kinematics());
    m_servo_jf.ForceTorque().Zeros();
    m_servo_jf_vector.SetSize(number_of_joints_kinematics());
    m_servo_jf_vector.Zeros();
}

void mtsIntuitiveResearchKitArm::Configure(const std::string & filename)
{
    try {
        std::ifstream jsonStream;
        Json::Value jsonConfig;
        Json::Reader jsonReader;

        jsonStream.open(filename.c_str());
        if (!jsonReader.parse(jsonStream, jsonConfig)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
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

        // detect if we're using 1.8 and up with two fields, kinematic and tool-detection
        const auto jsonKinematic = jsonConfig["kinematic"];
        if (jsonKinematic.isNull()) {
            std::stringstream message;
            message << "Configure " << this->GetName() << ":" << std::endl
                    << "----------------------------------------------------" << std::endl
                    << " ERROR:" << std::endl
                    << "   You should have a \"arm\" file for each arm in the console" << std::endl
                    << "   file.  The arm file should contain the field" << std::endl
                    << "   \"kinematic\" and options specific to each arm type." << std::endl
                    << "----------------------------------------------------";
            std::cerr << "mtsIntuitiveResearchKitArm::" << message.str() << std::endl;
            CMN_LOG_CLASS_INIT_ERROR << message.str() << std::endl;
            exit(EXIT_FAILURE);
        }

        // extract path of main json config file to search other files relative to it
        cmnPath configPath(cmnPath::GetWorkingDirectory());
        std::string fullname = configPath.Find(filename);
        std::string configDir = fullname.substr(0, fullname.find_last_of('/'));
        // for user files first
        configPath.Add(configDir, cmnPath::TAIL);
        // for standard files using io/xyz.json, arm/xyz.json
        configPath.Add(std::string(sawIntuitiveResearchKit_SOURCE_CONFIG_DIR), cmnPath::TAIL);
        // for tool definition files
        configPath.Add(std::string(sawIntuitiveResearchKit_SOURCE_CONFIG_DIR) + "/tool", cmnPath::TAIL);

        // arm specific configuration
        PreConfigure(jsonConfig, configPath, filename);

        // kinematic
        const auto fileKinematic = configPath.Find(jsonKinematic.asString());
        if (fileKinematic == "") {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: " << this->GetName()
                                     << " using file \"" << filename << "\" can't find kinematic file \""
                                     << jsonKinematic.asString() << "\"" << std::endl;
            exit(EXIT_FAILURE);
        } else {
            ConfigureDH(fileKinematic);
        }

        // detect generation
        const auto jsonGeneration = jsonConfig["generation"];
        if (!jsonGeneration.isNull()) {
            const auto generation = jsonGeneration.asString();
            if (generation == "Classic") {
                set_generation(dvrk::generation::Classic);
            } else if (generation == "Si") {
                set_generation(dvrk::generation::Si);
            } else {
                CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                         << ": \"generation\" must be either \"Classic\" or \"Si\", found: "
                                         << generation << " in " << filename << std::endl;
                exit(EXIT_FAILURE);
            }
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName()
                                     << ": missing \"generation\"" << " in " << filename << std::endl;
            exit(EXIT_FAILURE);
        }

        // should arm go to zero position when homing, default set in Init method
        const Json::Value jsonHomingGoesToZero = jsonConfig["homing-zero-position"];
        if (!jsonHomingGoesToZero.isNull()) {
            m_homing_goes_to_zero = jsonHomingGoesToZero.asBool();
        }

        // mount pitch in radians!, default assumes angles match SUJ (e.g. ECM Classic is -45, ECM Si is -70).
        const Json::Value jsonMountingPitch = jsonConfig["mounting_pitch"];
        if (!jsonMountingPitch.isNull()) {
            m_mounting_pitch = jsonMountingPitch.asDouble();
        }

        // should ignore preloaded encoders and force homing
        const Json::Value jsonAlwaysHome = jsonConfig["re-home"];
        if (!jsonAlwaysHome.isNull()) {
            m_re_home = jsonAlwaysHome.asBool();
        }

        // arm specific configuration for gravity compensation
        ConfigureGC(jsonConfig, configPath, filename);

        // arm specific configuration
        PostConfigure(jsonConfig, configPath, filename);

    } catch (std::exception & e) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() << ": parsing file \""
                                 << filename << "\", got error: " << e.what() << std::endl;
        exit(EXIT_FAILURE);
    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure " << this->GetName() << ": make sure the file \""
                                 << filename << "\" is in JSON format" << std::endl;
        exit(EXIT_FAILURE);
    }
}

void mtsIntuitiveResearchKitArm::ConfigureDH(const Json::Value & jsonConfig,
                                             const std::string & filename,
                                             const bool ignoreCoupling)
{
    // load base offset transform if any (without warning)
    const Json::Value jsonBase = jsonConfig["base-offset"];
    if (!jsonBase.isNull()) {
        // save the transform as Manipulator Rtw0
        cmnDataJSON<vctFrm4x4>::DeSerializeText(Manipulator->Rtw0, jsonBase);
        if (!nmrIsOrthonormal(Manipulator->Rtw0.Rotation())) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureDH " << this->GetName()
                                     << ": the base offset rotation doesn't seem to be orthonormal"
                                     << " in " << filename << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    // load DH parameters
    const Json::Value jsonDH = jsonConfig["DH"];
    if (jsonDH.isNull()) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureDH " << this->GetName()
                                 << ": can find \"DH\" data in configuration file \""
                                 << filename << "\"" << std::endl;
        exit(EXIT_FAILURE);
    }
    if (this->Manipulator->LoadRobot(jsonDH) != robManipulator::ESUCCESS) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureDH " << this->GetName()
                                 << ": failed to load \"DH\" parameters from file \""
                                 << filename << "\", error is "
                                 << this->Manipulator->LastError() << std::endl;
        exit(EXIT_FAILURE);
    }
    std::stringstream dhResult;
    this->Manipulator->PrintKinematics(dhResult);
    CMN_LOG_CLASS_INIT_VERBOSE << "ConfigureDH " << this->GetName()
                               << ": loaded kinematics" << std::endl << dhResult.str() << std::endl;
    // save the base arm configuration file, this is useful for PSM
    // when changing tool and we need to reload the base arm
    // configuration
    if (mConfigurationFile == "") {
        mConfigurationFile = filename;
        CMN_LOG_CLASS_INIT_VERBOSE << "ConfigureDH " << this->GetName()
                                   << ": saved base configuration file name: "
                                   << mConfigurationFile << std::endl;
    }

    // update configuration joint from manipulator
    update_configuration_js();

    // resize data members using kinematics (jacobians and effort vectors)
    ResizeKinematicsData();

    // coupling matrix
    if (!ignoreCoupling) {
        const Json::Value jsonCoupling = jsonConfig["coupling"];
        if (!jsonCoupling.isNull()) {
            prmActuatorJointCoupling coupling;
            cmnDataJSON<prmActuatorJointCoupling>::DeSerializeText(coupling,
                                                                   jsonCoupling);
            try {
                prmActuatorJointCouplingCheck(number_of_joints(),
                                              number_of_joints(),
                                              coupling, m_coupling);
                m_has_coupling = true;
            } catch (std::exception & e) {
                CMN_LOG_CLASS_INIT_ERROR << "ConfigureDH " << this->GetName()
                                         << ": caught exception \"" << e.what() << "\" for coupling from \""
                                         << filename << "\"" << std::endl;
                exit(EXIT_FAILURE);
            }
        }
    }
}

void mtsIntuitiveResearchKitArm::ConfigureDH(const std::string & filename)
{
    try {
        std::ifstream jsonStream;
        Json::Value jsonConfig;
        Json::Reader jsonReader;

        jsonStream.open(filename.c_str());
        if (!jsonReader.parse(jsonStream, jsonConfig)) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureDH " << this->GetName()
                                     << ": failed to parse kinematic (DH) configuration file \""
                                     << filename << "\"\n"
                                     << jsonReader.getFormattedErrorMessages();
            exit(EXIT_FAILURE);
        }

        CMN_LOG_CLASS_INIT_VERBOSE << "ConfigureDH: " << this->GetName()
                                   << " using file \"" << filename << "\"" << std::endl
                                   << "----> content of kinematic (GC) configuration file: " << std::endl
                                   << jsonConfig << std::endl
                                   << "<----" << std::endl;

        if (!jsonConfig.isNull()) {
            ConfigureDH(jsonConfig, filename);
        }

    } catch (std::exception & e) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureDH " << this->GetName() << ": parsing file \""
                                 << filename << "\", got error: " << e.what() << std::endl;
        exit(EXIT_FAILURE);
    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "ConfigureDH " << this->GetName() << ": make sure the file \""
                                 << filename << "\" is in JSON format" << std::endl;
        exit(EXIT_FAILURE);
    }
}

void mtsIntuitiveResearchKitArm::Startup(void)
{
    SetDesiredState("DISABLED");
    trajectory_j_set_ratio(mtsIntuitiveResearchKit::JointTrajectory::ratio);
}

void mtsIntuitiveResearchKitArm::Run(void)
{
    // collect data from required interfaces
    ProcessQueuedEvents();
    try {
        mArmState.Run();
    } catch (std::exception & e) {
        m_arm_interface->SendError(this->GetName() + ": in state " + mArmState.CurrentState()
                                   + ", caught exception \"" + e.what() + "\"");
        SetDesiredState("DISABLED");
    }
    // trigger ExecOut event
    RunEvent();
    ProcessQueuedCommands();
}

void mtsIntuitiveResearchKitArm::Cleanup(void)
{
    // engage brakes
    if (has_brakes()) {
        IO.BrakeEngage();
    }
    // turn off power
    IO.PowerOffSequence(false);
    // if in calibration mode, reset encoders to 0
    if (m_calibration_mode) {
        vctDoubleVec values(number_of_joints());
        values.Zeros();
        IO.SetEncoderPosition(values);
    }
    // leave the arm blinking
    set_LED_pattern(mtsIntuitiveResearchKit::Red200,
                    mtsIntuitiveResearchKit::Green200,
                    false, true);

    CMN_LOG_CLASS_INIT_VERBOSE << GetName() << ": Cleanup" << std::endl;
}

void mtsIntuitiveResearchKitArm::set_simulated(void)
{
    m_simulated = true;
    // in simulation mode, we don't need IO
    RemoveInterfaceRequired("RobotIO");
}

void mtsIntuitiveResearchKitArm::get_robot_data(void)
{
    // check that the robot still has power
    if (m_powered && !m_simulated) {
        vctBoolVec actuatorAmplifiersStatus(number_of_joints());
        IO.GetActuatorAmpStatus(actuatorAmplifiersStatus);
        vctBoolVec brakeAmplifiersStatus(number_of_brakes());
        if (has_brakes()) {
            IO.GetBrakeAmpStatus(brakeAmplifiersStatus);
        }
        if (!(actuatorAmplifiersStatus.All())) {
            m_powered = false;
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": get_robot_data:\n - Actuator amp status: "
                                    << actuatorAmplifiersStatus << std::endl;
            m_arm_interface->SendError(this->GetName() + ": detected power loss (actuators)");
            SetDesiredState("FAULT");
            return;
        }
        if (!(brakeAmplifiersStatus.All())) {
            m_powered = false;
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": get_robot_data:\n - Brake amp status: "
                                    << brakeAmplifiersStatus << std::endl;
            m_arm_interface->SendError(this->GetName() + ": detected power loss (brakes)");
            SetDesiredState("FAULT");
            return;
        }
    }

    // we can start reporting some joint values after the robot is powered
    if (is_joint_ready()) {
        mtsExecutionResult executionResult;
        // joint state
        executionResult = PID.measured_js(m_pid_measured_js);
        if (executionResult.IsOK()) {
            m_pid_measured_js.SetValid(true);
        } else {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": get_robot_data: call to PID.measured_js failed \""
                                    << executionResult << "\"" << std::endl;
            m_pid_measured_js.SetValid(false);
        }

        // desired joint state
        executionResult = PID.setpoint_js(m_pid_setpoint_js);
        if (executionResult.IsOK()) {
            m_pid_setpoint_js.SetValid(true);
        } else {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": get_robot_data: call to PID.setpoint_js failed \""
                                    << executionResult << "\"" << std::endl;
            m_pid_setpoint_js.SetValid(false);
        }

        // apply coupling if needed
        if (m_has_coupling) {
            m_pid_measured_js.Position() = m_coupling.ActuatorToJointPosition() * m_pid_measured_js.Position();
            m_pid_measured_js.Velocity() = m_coupling.ActuatorToJointPosition() * m_pid_measured_js.Velocity();
            m_pid_measured_js.Effort() = m_coupling.ActuatorToJointEffort() * m_pid_measured_js.Effort();
            m_pid_setpoint_js.Position() = m_coupling.ActuatorToJointPosition() * m_pid_setpoint_js.Position();
            m_pid_setpoint_js.Velocity() = m_coupling.ActuatorToJointPosition() * m_pid_setpoint_js.Velocity();
            m_pid_setpoint_js.Effort() = m_coupling.ActuatorToJointEffort() * m_pid_setpoint_js.Effort();
        }

        // update joint states used for kinematics
        UpdateStateJointKinematics();

        // updated joint efforts for gravity compensation
        if (m_gravity_compensation && gravity_compensation) {
            double pitch = (m_mounting_pitch < std::numeric_limits<double>::max()) ? m_mounting_pitch : 0.0;
            vct3 gravity(0.0, sin(pitch) * 9.81, cos(pitch) * 9.81);
            m_gravity_compensation_setpoint_js.Effort() = gravity_compensation->compute(m_kin_measured_js, gravity);
        } else {
            m_gravity_compensation_setpoint_js.Effort().Zeros();
        }
        m_gravity_compensation_setpoint_js.SetTimestamp(m_kin_measured_js.Timestamp());
        m_gravity_compensation_setpoint_js.SetValid(true);
    } else {
        // set joint to zeros
        m_pid_measured_js.Position().Zeros();
        m_pid_measured_js.Velocity().Zeros();
        m_pid_measured_js.Effort().Zeros();
        m_pid_measured_js.SetValid(false);

        m_kin_measured_js.Position().Zeros();
        m_kin_measured_js.Velocity().Zeros();
        m_kin_measured_js.Effort().Zeros();
        m_kin_measured_js.SetValid(false);

        m_gravity_compensation_setpoint_js.SetValid(false);
    }

    // when the robot is ready, we can compute cartesian position
    if (is_cartesian_ready()) {
        CMN_ASSERT(is_joint_ready());
        // update cartesian position
        m_local_measured_cp_frame = Manipulator->ForwardKinematics(m_kin_measured_js.Position());
        m_measured_cp_frame = m_base_frame * m_local_measured_cp_frame;
        // normalize
        m_local_measured_cp_frame.Rotation().NormalizedSelf();
        m_measured_cp_frame.Rotation().NormalizedSelf();
        // prm types
        m_local_measured_cp.Position().From(m_local_measured_cp_frame);
        m_local_measured_cp.SetTimestamp(m_kin_measured_js.Timestamp());
        m_local_measured_cp.SetValid(true);
        m_measured_cp.Position().From(m_measured_cp_frame);
        m_measured_cp.SetTimestamp(m_kin_measured_js.Timestamp());
        m_measured_cp.SetValid(m_base_frame_valid);

        // update jacobians
        Manipulator->JacobianSpatial(m_kin_measured_js.Position(), m_spatial_jacobian);
        Manipulator->JacobianBody(m_kin_measured_js.Position(), m_body_jacobian);

        // update cartesian velocity using the jacobian and joint
        // velocities.
        vctDoubleVec cartesianVelocity(6);
        cartesianVelocity.ProductOf(m_body_jacobian, m_kin_measured_js.Velocity());
        vct3 relative, absolute;
        // linear
        relative.Assign(cartesianVelocity.Ref(3, 0));
        m_local_measured_cv.SetVelocityLinear(relative);
        m_measured_cp_frame.Rotation().ApplyTo(relative, absolute);
        m_measured_cv.SetVelocityLinear(absolute);
        // angular
        relative.Assign(cartesianVelocity.Ref(3, 3));
        m_local_measured_cv.SetVelocityAngular(relative);
        m_measured_cp_frame.Rotation().ApplyTo(relative, absolute);
        m_measured_cv.SetVelocityAngular(absolute);
        // valid/timestamp
        m_local_measured_cv.SetValid(true);
        m_local_measured_cv.SetTimestamp(m_kin_measured_js.Timestamp());
        m_measured_cv.SetValid(true);
        m_measured_cv.SetTimestamp(m_kin_measured_js.Timestamp());

        // update wrench based on measured joint current efforts
        m_body_jacobian_transpose.Assign(m_body_jacobian.Transpose());
        nmrPInverse(m_body_jacobian_transpose, m_jacobian_transpose_pinverse_data);
        vctDoubleVec wrench(6);
        wrench.ProductOf(m_jacobian_transpose_pinverse_data.PInverse(), m_kin_measured_js.Effort());
        if (m_body_cf_orientation_absolute) {
            // forces
            relative.Assign(wrench.Ref(3, 0));
            m_measured_cp_frame.Rotation().ApplyTo(relative, absolute);
            m_body_measured_cf.Force().Ref<3>(0).Assign(absolute);
            // torques
            relative.Assign(wrench.Ref(3, 3));
            m_measured_cp_frame.Rotation().ApplyTo(relative, absolute);
            m_body_measured_cf.Force().Ref<3>(3).Assign(absolute);
        } else {
            m_body_measured_cf.Force().Assign(wrench);
        }
        // valid/timestamp
        m_body_measured_cf.SetValid(true);
        m_body_measured_cf.SetTimestamp(m_kin_measured_js.Timestamp());

        m_spatial_jacobian_transpose.Assign(m_spatial_jacobian.Transpose());
        nmrPInverse(m_spatial_jacobian_transpose, m_jacobian_transpose_pinverse_data);
        wrench.ProductOf(m_jacobian_transpose_pinverse_data.PInverse(), m_kin_measured_js.Effort());
        m_spatial_measured_cf.Force().Assign(wrench);
        // valid/timestamp
        m_spatial_measured_cf.SetValid(true);
        m_spatial_measured_cf.SetTimestamp(m_kin_measured_js.Timestamp());

        // update cartesian position desired based on joint desired
        m_local_setpoint_cp_frame = Manipulator->ForwardKinematics(m_kin_setpoint_js.Position());
        m_setpoint_cp_frame = m_base_frame * m_local_setpoint_cp_frame;
        // normalize
        m_local_setpoint_cp_frame.Rotation().NormalizedSelf();
        m_setpoint_cp_frame.Rotation().NormalizedSelf();
        // prm type
        m_local_setpoint_cp.Position().From(m_local_setpoint_cp_frame);
        m_local_setpoint_cp.SetTimestamp(m_kin_setpoint_js.Timestamp());
        m_local_setpoint_cp.SetValid(true);
        m_setpoint_cp.Position().From(m_setpoint_cp_frame);
        m_setpoint_cp.SetTimestamp(m_kin_setpoint_js.Timestamp());
        m_setpoint_cp.SetValid(m_base_frame_valid);

        // update cartesian velocity using the jacobian and joint
        // velocities.
        cartesianVelocity.ProductOf(m_body_jacobian, m_kin_setpoint_js.Velocity());
        // linear
        relative.Assign(cartesianVelocity.Ref(3, 0));
        m_local_setpoint_cv.SetVelocityLinear(relative);
        m_setpoint_cp_frame.Rotation().ApplyTo(relative, absolute);
        m_setpoint_cv.SetVelocityLinear(absolute);
        // angular
        relative.Assign(cartesianVelocity.Ref(3, 3));
        m_local_setpoint_cv.SetVelocityAngular(relative);
        m_setpoint_cp_frame.Rotation().ApplyTo(relative, absolute);
        m_setpoint_cv.SetVelocityAngular(absolute);
        // valid/timestamp
        m_local_setpoint_cv.SetValid(true);
        m_local_setpoint_cv.SetTimestamp(m_kin_setpoint_js.Timestamp());
        m_setpoint_cv.SetValid(true);
        m_setpoint_cv.SetTimestamp(m_kin_setpoint_js.Timestamp());
    } else {
        // set cartesian data to "zero"
        m_local_measured_cp_frame.Assign(vctFrm4x4::Identity());
        m_measured_cp_frame.Assign(vctFrm4x4::Identity());
        m_local_measured_cp.SetValid(false);
        m_measured_cp.SetValid(false);
        // velocities and wrench
        m_local_measured_cv.SetValid(false);
        m_measured_cv.SetValid(false);
        m_body_measured_cf.SetValid(false);
        m_spatial_measured_cf.SetValid(false);
        // update cartesian position desired
        m_local_setpoint_cp_frame.Assign(vctFrm4x4::Identity());
        m_setpoint_cp_frame.Assign(vctFrm4x4::Identity());
        m_local_setpoint_cp.SetValid(false);
        m_setpoint_cp.SetValid(false);
        // velocities and wrench
        m_local_setpoint_cv.SetValid(false);
        m_setpoint_cv.SetValid(false);
    }
}

void mtsIntuitiveResearchKitArm::UpdateStateJointKinematics(void)
{
    m_kin_measured_js = m_pid_measured_js;
    m_kin_setpoint_js = m_pid_setpoint_js;
}

void mtsIntuitiveResearchKitArm::ToJointsPID(const vctDoubleVec & jointsKinematics, vctDoubleVec & jointsPID)
{
    jointsPID.Assign(jointsKinematics);
}

void mtsIntuitiveResearchKitArm::UpdateOperatingStateAndBusy(const prmOperatingState::StateType & state,
                                                             const bool isBusy)
{
    mStateTableState.Start();
    m_operating_state.State() = state;
    m_operating_state.IsHomed() = is_homed();
    m_operating_state.IsBusy() = isBusy;
    m_operating_state.SubState() = mArmState.CurrentState();
    mStateTableState.Advance();
    // push only operating_state since it's the only one changing
    state_events.operating_state(m_operating_state);
}

void mtsIntuitiveResearchKitArm::UpdateHomed(const bool isHomed)
{
    mStateTableState.Start();
    m_operating_state.IsHomed() = isHomed;
    m_operating_state.SubState() = mArmState.CurrentState();
    mStateTableState.Advance();
    // push only operating_state since it's the only one changing
    state_events.operating_state(m_operating_state);
}

void mtsIntuitiveResearchKitArm::UpdateIsBusy(const bool isBusy)
{
    mStateTableState.Start();
    m_operating_state.IsBusy() = isBusy;
    m_operating_state.SubState() = mArmState.CurrentState();
    mStateTableState.Advance();
    // push only operating_state since it's the only one changing
    state_events.operating_state(m_operating_state);
}

void mtsIntuitiveResearchKitArm::StateEvents(void)
{
    // push all state related events
    state_events.current_state(mArmState.CurrentState());
    state_events.desired_state(mArmState.DesiredState());
    state_events.operating_state(m_operating_state);
}

void mtsIntuitiveResearchKitArm::StateChanged(void)
{
    const std::string & state = mArmState.CurrentState();
    // update state table
    mStateTableState.Start();
    mStateTableStateCurrent = state;
    mStateTableState.Advance();
    // push all state events
    StateEvents();
    m_arm_interface->SendStatus(this->GetName() + ": current state " + state);
}

void mtsIntuitiveResearchKitArm::RunAllStates(void)
{
    get_robot_data();
}

void mtsIntuitiveResearchKitArm::EnterDisabled(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::DISABLED, false);

    if (has_brakes()) {
        IO.BrakeEngage();
    }

    IO.UsePotsForSafetyCheck(false);
    IO.SetActuatorCurrent(vctDoubleVec(number_of_joints(), 0.0));
    IO.PowerOffSequence(false); // do not open safety relays
    PID.enable(false);
    PID.enforce_position_limits(true);
    m_powered = false;
    set_LED_pattern(mtsIntuitiveResearchKit::Red200,
                    mtsIntuitiveResearchKit::Red200,
                    false, false);
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::UNDEFINED_SPACE,
                           mtsIntuitiveResearchKitControlTypes::UNDEFINED_MODE);
}

void mtsIntuitiveResearchKitArm::TransitionDisabled(void)
{
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState("POWERING");
    }
}

void mtsIntuitiveResearchKitArm::EnterPowering(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::DISABLED, true);

    m_powered = false;

    if (m_simulated) {
        PID.enable_measured_setpoint_check(false);
        PID.enable(true);
        PID.enable_joints(vctBoolVec(number_of_joints(), true));
        vctDoubleVec goal(number_of_joints());
        goal.Zeros();
        mtsIntuitiveResearchKitArm::servo_jp_internal(goal, vctDoubleVec());
        return;
    }

    const double currentTime = this->StateTable.GetTic();

    // in case we still have power but brakes are not engaged
    if (has_brakes()) {
        IO.BrakeEngage();
    }

    m_homing_timer = currentTime;
    // make sure the PID is not sending currents
    PID.enable(false);
    // pre-load the boards with zero current
    IO.SetActuatorCurrent(vctDoubleVec(number_of_joints(), 0.0));
    // enable power
    IO.PowerOnSequence();
    m_arm_interface->SendStatus(this->GetName() + ": power requested");
}

void mtsIntuitiveResearchKitArm::TransitionPowering(void)
{
    if (m_simulated) {
        mArmState.SetCurrentState("ENABLED");
        return;
    }

    const double currentTime = this->StateTable.GetTic();

    // check power status
    vctBoolVec actuatorAmplifiersStatus(number_of_joints());
    IO.GetActuatorAmpStatus(actuatorAmplifiersStatus);
    vctBoolVec brakeAmplifiersStatus(number_of_brakes());
    if (has_brakes()) {
        IO.GetBrakeAmpStatus(brakeAmplifiersStatus);
    }
    if (actuatorAmplifiersStatus.All() && brakeAmplifiersStatus.All()) {
        m_arm_interface->SendStatus(this->GetName() + ": power on");
        mArmState.SetCurrentState("ENABLED");
    } else {
        if ((currentTime - m_homing_timer) > 2.0 * mtsIntuitiveResearchKit::TimeToPower) {
            m_arm_interface->SendError(this->GetName() + ": failed to enable power");
            SetDesiredState("FAULT");
        }
    }
}

void mtsIntuitiveResearchKitArm::EnterEnabled(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, false);

    if (m_simulated) {
        m_powered = true;
        return;
    }

    m_powered = true;
    set_LED_pattern(mtsIntuitiveResearchKit::Blue200,
                    mtsIntuitiveResearchKit::Blue200,
                    false, false);

    // disable PID for fallback
    IO.SetActuatorCurrent(vctDoubleVec(number_of_joints(), 0.0));
    PID.enable(false);
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::UNDEFINED_SPACE,
                           mtsIntuitiveResearchKitControlTypes::UNDEFINED_MODE);

    // engage brakes for fallback
    if (has_brakes()) {
        IO.BrakeEngage();
    }
}

void mtsIntuitiveResearchKitArm::TransitionEnabled(void)
{
    // move to next stage if desired state is anything past power
    // unless user request new pots calibration
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState("CALIBRATING_ENCODERS_FROM_POTS");
    }
}

void mtsIntuitiveResearchKitArm::EnterCalibratingEncodersFromPots(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, true);

    // if simulated, no need to bias encoders
    if (m_simulated) {
        m_arm_interface->SendStatus(this->GetName() + ": simulated mode, no need to calibrate encoders");
        return;
    }
    // for Si, always calibrate from pot
    if (m_encoders_biased_from_pots
        && !m_calibration_mode
        && (m_generation == dvrk::generation::Classic)) {
        m_arm_interface->SendStatus(this->GetName() + ": encoders have already been calibrated, skipping");
        return;
    }

    // request bias encoder
    const double currentTime = this->StateTable.GetTic();
    const int nb_samples = 1970; // birth year, state table contains 1999 elements so anything under that would work
    if (m_re_home
        || m_calibration_mode
        || (m_generation == dvrk::generation::Si)) {
        // positive number to ignore encoder preloads
        IO.BiasEncoder(nb_samples);
    } else {
        // negative numbers means that we first check if encoders have already been preloaded
        IO.BiasEncoder(-nb_samples);
    }
    m_homing_bias_encoder_requested = true;
    m_homing_timer = currentTime;
}

void mtsIntuitiveResearchKitArm::TransitionCalibratingEncodersFromPots(void)
{
    if (m_simulated || m_encoders_biased_from_pots) {
        m_encoders_biased_from_pots = true;
        mArmState.SetCurrentState("ENCODERS_BIASED");
        return;
    }

    const double currentTime = this->StateTable.GetTic();
    const double timeToBias = 30.0 * cmn_s; // large timeout
    if ((currentTime - m_homing_timer) > timeToBias) {
        m_homing_bias_encoder_requested = false;
        m_arm_interface->SendError(this->GetName() + ": failed to bias encoders (timeout)");
        SetDesiredState("FAULT");
    }
}

void mtsIntuitiveResearchKitArm::EnterEncodersBiased(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, false);

    // use pots for redundancy when not in calibration mode and always
    // for the Si
    if (m_calibration_mode
        && (m_generation == dvrk::generation::Classic)) {
        IO.UsePotsForSafetyCheck(false);
    } else {
        IO.UsePotsForSafetyCheck(true);
    }
}

void mtsIntuitiveResearchKitArm::TransitionEncodersBiased(void)
{
    // move to next stage if desired state is anything past post pot
    // calibration
    if (mArmState.DesiredStateIsNotCurrent()) {
        mArmState.SetCurrentState("HOMING");
    }
}

void mtsIntuitiveResearchKitArm::EnterHoming(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, true);

    // disable joint limits, arm might start outside them
    PID.enforce_position_limits(false);
    // enable tracking errors
    PID.set_measured_setpoint_tolerance(PID.measured_setpoint_tolerance);

    // release brakes if any
    if ((has_brakes()) && !m_simulated) {
        IO.BrakeRelease();
    }

    // get robot data to make sure we have latest state
    CMN_ASSERT(is_joint_ready());
    get_robot_data();

    // compute joint goal position
    this->SetGoalHomingArm();
    // initialize trajectory with current position and velocities
    m_servo_jp.Assign(m_pid_setpoint_js.Position());
    m_servo_jv.Assign(m_pid_measured_js.Velocity());
    m_trajectory_j.goal_v.Zeros();
    m_trajectory_j.end_time = 0.0;
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitControlTypes::TRAJECTORY_MODE);

    // enable PID on all joints
    mtsIntuitiveResearchKitArm::servo_jp_internal(m_servo_jp, vctDoubleVec());
    PID.enable(true);
    PID.enable_joints(vctBoolVec(number_of_joints(), true));
}

void mtsIntuitiveResearchKitArm::RunHoming(void)
{
    static const double extraTime = 2.0 * cmn_s;
    const double currentTime = this->StateTable.GetTic();

    m_trajectory_j.Reflexxes.Evaluate(m_servo_jp,
                                      m_servo_jv,
                                      m_trajectory_j.goal,
                                      m_trajectory_j.goal_v);
    mtsIntuitiveResearchKitArm::servo_jp_internal(m_servo_jp, m_servo_jv);

    const robReflexxes::ResultType trajectoryResult = m_trajectory_j.Reflexxes.ResultValue();
    bool isHomed;

    switch (trajectoryResult) {

    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (m_trajectory_j.end_time == 0.0) {
            m_trajectory_j.end_time = currentTime + m_trajectory_j.Reflexxes.Duration();
            m_homing_timer = m_trajectory_j.end_time;
        }
        break;

    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        // check position
        m_trajectory_j.goal_error.DifferenceOf(m_trajectory_j.goal, m_pid_measured_js.Position());
        m_trajectory_j.goal_error.AbsSelf();
        isHomed = !m_trajectory_j.goal_error.ElementwiseGreaterOrEqual(m_trajectory_j.goal_tolerance).Any();
        if (isHomed) {
            m_operating_state.IsHomed() = true;
            PID.enforce_position_limits(true);
            mArmState.SetCurrentState("HOMED");
        } else {
            // time out
            if (currentTime > m_homing_timer + extraTime) {
                CMN_LOG_CLASS_INIT_WARNING << GetName() << ": RunHoming: unable to reach home position, error in degrees is "
                                           << m_trajectory_j.goal_error * cmn180_PI << std::endl;
                m_arm_interface->SendError(this->GetName() + ": unable to reach home position during calibration on pots");
                SetDesiredState("FAULT");
            }
        }
        break;

    default:
        m_arm_interface->SendError(this->GetName() + ": error while evaluating trajectory");
        SetDesiredState("FAULT");
        break;
    }
}

void mtsIntuitiveResearchKitArm::EnterHomed(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, false);
    set_LED_pattern(mtsIntuitiveResearchKit::Green200,
                    mtsIntuitiveResearchKit::Green200,
                    false, false);

    // no control mode defined
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::UNDEFINED_SPACE,
                           mtsIntuitiveResearchKitControlTypes::UNDEFINED_MODE);

    if (m_simulated) {
        return;
    }

    // enable PID and start from current position
    mtsIntuitiveResearchKitArm::servo_jp_internal(m_pid_setpoint_js.Position(), vctDoubleVec());
    PID.enable_measured_setpoint_check(should_use_measured_setpoint_check());
    PID.enable_joints(vctBoolVec(number_of_joints(), true));
    PID.enforce_position_limits(true);
    PID.enable(true);
}

void mtsIntuitiveResearchKitArm::LeaveHomed(void)
{
    // no control mode defined
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::UNDEFINED_SPACE,
                           mtsIntuitiveResearchKitControlTypes::UNDEFINED_MODE);
}

void mtsIntuitiveResearchKitArm::RunHomed(void)
{
    if (mControlCallback) {
        mControlCallback->Execute();
    }
}

void mtsIntuitiveResearchKitArm::EnterPaused(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::PAUSED, false);
    m_resume_current_state = mArmState.PreviousState();
    m_resume_desired_state = mArmState.PreviousDesiredState();
}

void mtsIntuitiveResearchKitArm::EnterFault(void)
{
    set_LED_pattern(mtsIntuitiveResearchKit::Red200,
                    mtsIntuitiveResearchKit::Off,
                    true, false);
    IO.Explain();
    IO.PowerOffSequence(false);
    UpdateOperatingStateAndBusy(prmOperatingState::FAULT, false);
}

void mtsIntuitiveResearchKitArm::set_LED_pattern(uint32_t color1, uint32_t color2,
                                                 bool blink1, bool blink2)
{
    prmInputData pattern;
    pattern.AnalogInputs().SetSize(2);
    pattern.DigitalInputs().SetSize(2);
    pattern.AnalogInputs().at(0) = color1;
    pattern.AnalogInputs().at(1) = color2;
    pattern.DigitalInputs().at(0) = blink1;
    pattern.DigitalInputs().at(1) = blink2;
    IO.set_LED_pattern(pattern);
}

void mtsIntuitiveResearchKitArm::clip_jp(vctDoubleVec & jp) const
{
    auto upper = m_configuration_js.PositionMax().cbegin();
    auto lower = m_configuration_js.PositionMin().cbegin();
    auto desired = jp.begin();
    const auto end = jp.end();
    for (; desired != end; ++desired, ++upper, ++lower) {
        if (*desired > *upper) {
            *desired = *upper;
        } else if (*desired < *lower) {
            *desired = *lower;
        }
    }
}

void mtsIntuitiveResearchKitArm::control_servo_jp(void)
{
    if (m_pid_new_goal) {
        servo_jp_internal(m_servo_jp, m_servo_jv);
        // reset flag
        m_pid_new_goal = false;
    }
}

void mtsIntuitiveResearchKitArm::control_move_jp(void)
{
    // check if there's anything to do
    if (!m_trajectory_j.is_active) {
        return;
    }

    m_trajectory_j.Reflexxes.Evaluate(m_servo_jp,
                                      m_servo_jv,
                                      m_trajectory_j.goal,
                                      m_trajectory_j.goal_v);
    servo_jp_internal(m_servo_jp, m_servo_jv);

    const robReflexxes::ResultType trajectoryResult = m_trajectory_j.Reflexxes.ResultValue();
    const double currentTime = this->StateTable.GetTic();

    switch (trajectoryResult) {
    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (m_trajectory_j.end_time == 0.0) {
            m_trajectory_j.end_time = currentTime + m_trajectory_j.Reflexxes.Duration();
        }
        break;
    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        control_move_jp_on_stop(true); // goal reached
        break;
    default:
        m_arm_interface->SendError(this->GetName() + ": error while evaluating trajectory");
        control_move_jp_on_stop(false); // goal NOT reached
        break;
    }
}

void mtsIntuitiveResearchKitArm::control_servo_cp(void)
{
    if (m_pid_new_goal) {
        // copy current position
        vctDoubleVec jp(m_kin_measured_js.Position());

        // compute desired arm position
        CartesianPositionFrm.From(m_servo_cp.Goal());
        if (this->InverseKinematics(jp, m_base_frame.Inverse() * CartesianPositionFrm) == robManipulator::ESUCCESS) {
            // also compute velocities
            vctDoubleVec jv(number_of_joints_kinematics());
            if (m_servo_cp.Velocity().Any() || m_servo_cp.VelocityAngular().Any()) {
                vctDoubleVec cv(6);
                // make relative
                vct3 relative;
                // linear
                m_measured_cp_frame.Rotation().ApplyInverseTo(m_servo_cp.Velocity(), relative);
                cv.Ref(3).Assign(relative);
                // angular
                m_measured_cp_frame.Rotation().ApplyInverseTo(m_servo_cp.VelocityAngular(), relative);
                cv.Ref(3, 3).Assign(relative);
                // update the body jacobian pseudo inverse
                nmrPInverse(m_body_jacobian, m_jacobian_pinverse_data);
                jv.ProductOf(m_jacobian_pinverse_data.PInverse(), cv);
            } else {
                jv.Zeros();
            }
            // finally send new joint values
            servo_jp_internal(jp, jv);
        } else {
            // shows robManipulator error if used
            if (this->Manipulator) {
                m_arm_interface->SendError(this->GetName()
                                           + ": unable to solve inverse kinematics ("
                                           + this->Manipulator->LastError() + ")");
            } else {
                m_arm_interface->SendError(this->GetName() + ": unable to solve inverse kinematics");
            }
        }
        // reset flag
        m_pid_new_goal = false;
    }
}

void mtsIntuitiveResearchKitArm::control_move_cp(void)
{
    // trajectories are computed in joint space for now
    control_move_jp();
}

bool mtsIntuitiveResearchKitArm::ArmIsReady(const std::string & methodName,
                                            const mtsIntuitiveResearchKitControlTypes::ControlSpace space)
{
    // reset counter if ready
    if (m_operating_state.State() == prmOperatingState::ENABLED) {
        if (((space == mtsIntuitiveResearchKitControlTypes::JOINT_SPACE)
             && is_joint_ready())
            || ((space == mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE)
                && is_cartesian_ready())) {
            mArmNotReadyCounter = 0;
            mArmNotReadyTimeLastMessage = 0.0;
            return true;
        }
    }
    // throttle messages in time
    if ((StateTable.GetTic() - mArmNotReadyTimeLastMessage) > 2.0 * cmn_s) {
        std::stringstream message;
        message << this->GetName() << ": " << methodName << ", arm not ready";
        if (mArmNotReadyCounter > 1) {
            message << " (" << mArmNotReadyCounter << " errors)";
        }
        m_arm_interface->SendWarning(message.str());
        mArmNotReadyTimeLastMessage = StateTable.GetTic();
    }
    mArmNotReadyCounter++;
    return false;
}

void mtsIntuitiveResearchKitArm::trajectory_j_set_ratio_v(const double & ratio)
{
    if ((ratio > 0.0) && (ratio <= 1.0)) {
        m_trajectory_j.ratio_v = ratio;
        m_trajectory_j.ratio_v_event(ratio);
        trajectory_j_update_ratio();
        trajectory_j_update_reflexxes();
    } else {
        std::stringstream message;
        message << this->GetName() << ": trajectory_j_set_ratio_v, ratio must be within ]0;1], received " << ratio;
        m_arm_interface->SendWarning(message.str());
    }
}

void mtsIntuitiveResearchKitArm::trajectory_j_set_ratio_a(const double & ratio)
{
    if ((ratio > 0.0) && (ratio <= 1.0)) {
        m_trajectory_j.ratio_a = ratio;
        m_trajectory_j.ratio_a_event(ratio);
        trajectory_j_update_ratio();
        trajectory_j_update_reflexxes();
    } else {
        std::stringstream message;
        message << this->GetName() << ": trajectory_j_set_ratio_a, ratio must be within ]0;1], received " << ratio;
        m_arm_interface->SendWarning(message.str());
    }
}

void mtsIntuitiveResearchKitArm::trajectory_j_set_ratio(const double & ratio)
{
    if ((ratio > 0.0) && (ratio <= 1.0)) {
        m_trajectory_j.ratio = ratio;
        m_trajectory_j.ratio_v = ratio;
        m_trajectory_j.ratio_a = ratio;
        m_trajectory_j.ratio_event(ratio);
        m_trajectory_j.ratio_v_event(ratio);
        m_trajectory_j.ratio_a_event(ratio);
        trajectory_j_update_reflexxes();
    } else {
        std::stringstream message;
        message << this->GetName() << ": trajectory_j_set_ratio, ratio must be within ]0;1], received " << ratio;
        m_arm_interface->SendWarning(message.str());
    }
}

void mtsIntuitiveResearchKitArm::trajectory_j_update_ratio(void)
{
    // same v and a ratios, main ratio makes sense
    if (m_trajectory_j.ratio_v == m_trajectory_j.ratio_a) {
        m_trajectory_j.ratio = m_trajectory_j.ratio_v;
    } else {
        m_trajectory_j.ratio = 0.0;
    }
    m_trajectory_j.ratio_event(m_trajectory_j.ratio);
}

void mtsIntuitiveResearchKitArm::trajectory_j_update_reflexxes(void)
{
    m_trajectory_j.v.ProductOf(m_trajectory_j.ratio_v,
                               m_trajectory_j.v_max);
    m_trajectory_j.a.ProductOf(m_trajectory_j.ratio_a,
                               m_trajectory_j.a_max);
    m_trajectory_j.Reflexxes.Set(m_trajectory_j.v,
                                 m_trajectory_j.a,
                                 StateTable.PeriodStats.PeriodAvg(),
                                 robReflexxes::Reflexxes_TIME);
}

void mtsIntuitiveResearchKitArm::SetControlSpaceAndMode(const mtsIntuitiveResearchKitControlTypes::ControlSpace space,
                                                        const mtsIntuitiveResearchKitControlTypes::ControlMode mode,
                                                        mtsCallableVoidBase * callback)
{
    // ignore if already in the same space
    if ((space == m_control_space) && (mode == m_control_mode)) {
        if (callback) {
            delete callback;
        }
        return;
    }

    // transitions
    if (space != m_control_space) {
        // check if the arm is ready to use in cartesian space
        if (space == mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE) {
            if (this->is_safe_for_cartesian_control()) {
                // set flag
                m_control_space = space;
                m_safe_for_cartesian_control_counter = 0;
            } else {
                if (m_safe_for_cartesian_control_counter == 0) {
                    // message if needed
                    m_arm_interface->SendWarning(this->GetName() + ": tool/endoscope needs to be inserted past the cannula to switch to cartesian space");
                }
                m_safe_for_cartesian_control_counter = (m_safe_for_cartesian_control_counter + 1) % 1000;
                return;
            }
        } else {
            // all other spaces, just set the current space
            m_effort_orientation_locked = false;
            m_control_space = space;
        }
    }

    if (mode != m_control_mode) {

        if ((m_control_mode == mtsIntuitiveResearchKitControlTypes::TRAJECTORY_MODE)
            &&  m_trajectory_j.is_active) {
            control_move_jp_on_stop(false); // move was active and interrupted so assume goal not reached
        }

        switch (mode) {
        case mtsIntuitiveResearchKitControlTypes::POSITION_MODE:
            // configure PID
            PID.enable_measured_setpoint_check(should_use_measured_setpoint_check());
            PID.EnableTorqueMode(vctBoolVec(number_of_joints(), false));
            m_pid_new_goal = false;
            mCartesianRelative = vctFrm3::Identity();
            m_servo_jp.Assign(m_pid_setpoint_js.Position(), number_of_joints());
            m_servo_jv.Zeros();
            m_effort_orientation_locked = false;
            break;
        case mtsIntuitiveResearchKitControlTypes::TRAJECTORY_MODE:
            // configure PID
            PID.enable_measured_setpoint_check(should_use_measured_setpoint_check());
            PID.EnableTorqueMode(vctBoolVec(number_of_joints(), false));
            m_effort_orientation_locked = false;
            // initialize trajectory
            m_servo_jp.Assign(m_pid_setpoint_js.Position(), number_of_joints());
            if (m_control_mode == mtsIntuitiveResearchKitControlTypes::POSITION_MODE) {
                m_servo_jv.Assign(m_pid_measured_js.Velocity(), number_of_joints());
            } else {
                // we're switching from effort or no mode
                m_servo_jv.SetSize(number_of_joints());
                m_servo_jv.Zeros();
            }
            m_trajectory_j.Reflexxes.Set(m_trajectory_j.v,
                                         m_trajectory_j.a,
                                         StateTable.PeriodStats.PeriodAvg(),
                                         robReflexxes::Reflexxes_TIME);
            break;
        case mtsIntuitiveResearchKitControlTypes::EFFORT_MODE:
            // configure PID
            PID.enable_measured_setpoint_check(false);
            m_servo_jf_vector.Assign(vctDoubleVec(number_of_joints_kinematics(), 0.0));
            SetControlEffortActiveJoints();
            break;
        default:
            break;
        }

        // set flag
        m_control_mode = mode;
    }

    // set control callback for RunHomed
    switch (m_control_mode) {
    case mtsIntuitiveResearchKitControlTypes::POSITION_MODE:
        switch (m_control_space) {
        case mtsIntuitiveResearchKitControlTypes::JOINT_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::control_servo_jp, this);
            break;
        case mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::control_servo_cp, this);
            break;
        default:
            break;
        }
        break;
    case mtsIntuitiveResearchKitControlTypes::TRAJECTORY_MODE:
        switch (m_control_space) {
        case mtsIntuitiveResearchKitControlTypes::JOINT_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::control_move_jp, this);
            break;
        case mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::control_move_cp, this);
            break;
        default:
            break;
        }
        break;
    case mtsIntuitiveResearchKitControlTypes::EFFORT_MODE:
        switch (m_control_space) {
        case mtsIntuitiveResearchKitControlTypes::JOINT_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::control_servo_jf, this);
            break;
        case mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::control_servo_cf, this);
            break;
        default:
            break;
        }
        break;
    default:
        SetControlCallback(0);
        break;
    }

    // use provided callback if the space or mode is user defined
    if ((m_control_mode == mtsIntuitiveResearchKitControlTypes::USER_MODE)
        || (m_control_space == mtsIntuitiveResearchKitControlTypes::USER_SPACE)) {
        mControlCallback = callback;
    }

    // messages
    m_arm_interface->SendStatus(this->GetName() + ": control "
                                + cmnData<mtsIntuitiveResearchKitControlTypes::ControlSpace>::HumanReadable(m_control_space)
                                + '/'
                                + cmnData<mtsIntuitiveResearchKitControlTypes::ControlMode>::HumanReadable(m_control_mode));
}

void mtsIntuitiveResearchKitArm::control_move_jp_on_start(void)
{
    UpdateIsBusy(true);
    m_trajectory_j.is_active = true;
    m_trajectory_j.end_time = 0.0;
}

void mtsIntuitiveResearchKitArm::control_move_jp_on_stop(const bool goal_reached)
{
    m_trajectory_j.goal_reached_event(goal_reached);
    m_trajectory_j.is_active = false;
    UpdateIsBusy(false);
}

void mtsIntuitiveResearchKitArm::control_servo_cf_orientation_locked(void)
{
    CMN_LOG_CLASS_RUN_ERROR << GetName()
                            << ": control_servo_cf_orientation_locked, this method should never be called, MTMs are the only arms able to lock orientation and the derived implementation of this method should be called"
                            << std::endl;
}

void mtsIntuitiveResearchKitArm::SetControlEffortActiveJoints(void)
{
    PID.EnableTorqueMode(vctBoolVec(number_of_joints(), true));
}

void mtsIntuitiveResearchKitArm::control_servo_cf_preload(vctDoubleVec & effortPreload,
                                                          vctDoubleVec & wrenchPreload)
{
    effortPreload.Zeros();
    wrenchPreload.Zeros();
}

void mtsIntuitiveResearchKitArm::control_servo_jf(void)
{
    // effort required
    m_servo_jf_vector.Assign(m_servo_jf.ForceTorque());
    // convert to cisstParameterTypes
    servo_jf_internal(m_servo_jf_vector);
}

void mtsIntuitiveResearchKitArm::control_servo_cf(void)
{
    // update torques based on wrench
    vctDoubleVec wrench(6);

    // get force preload from derived classes, in most cases 0, platform control for MTM
    vctDoubleVec effortPreload(number_of_joints_kinematics());
    vctDoubleVec wrenchPreload(6);

    control_servo_cf_preload(effortPreload, wrenchPreload);

    // body wrench
    if (m_servo_cf_type == WRENCH_BODY) {
        // either using wrench provided by user or cartesian impedance
        if (m_cartesian_impedance) {
            mCartesianImpedanceController->Update(m_measured_cp,
                                                  m_measured_cv,
                                                  m_servo_cf,
                                                  m_body_cf_orientation_absolute);
            wrench.Assign(m_servo_cf.Force());
        } else {
            // user provided wrench
            if (m_body_cf_orientation_absolute) {
                // use forward kinematics orientation to have constant wrench orientation
                vct3 relative, absolute;
                // force
                relative.Assign(m_servo_cf.Force().Ref<3>(0));
                m_measured_cp_frame.Rotation().ApplyInverseTo(relative, absolute);
                wrench.Ref(3, 0).Assign(absolute);
                // torque
                relative.Assign(m_servo_cf.Force().Ref<3>(3));
                m_measured_cp_frame.Rotation().ApplyInverseTo(relative, absolute);
                wrench.Ref(3, 3).Assign(absolute);
            } else {
                wrench.Assign(m_servo_cf.Force());
            }
        }
        m_servo_jf_vector.ProductOf(m_body_jacobian.Transpose(), wrench + wrenchPreload);
        m_servo_jf_vector.Add(effortPreload);
    }
    // spatial wrench
    else if (m_servo_cf_type == WRENCH_SPATIAL) {
        wrench.Assign(m_servo_cf.Force());
        m_servo_jf_vector.ProductOf(m_spatial_jacobian.Transpose(), wrench + wrenchPreload);
        m_servo_jf_vector.Add(effortPreload);
    }

    // send to PID
    servo_jf_internal(m_servo_jf_vector);

    // lock orientation if needed
    if (m_effort_orientation_locked) {
        control_servo_cf_orientation_locked();
    }
}

void mtsIntuitiveResearchKitArm::servo_jf_internal(const vctDoubleVec & jf)
{
    // convert to cisstParameterTypes
    m_servo_jf_param.SetForceTorque(jf);
    m_servo_jf_param.SetTimestamp(StateTable.GetTic());
    if (m_has_coupling) {
        m_servo_jf_param.ForceTorque() = m_coupling.JointToActuatorEffort() * m_servo_jf_param.ForceTorque();
    }
    PID.servo_jf(m_servo_jf_param);

    apply_feed_forward();
}

void mtsIntuitiveResearchKitArm::servo_jp_internal(const vctDoubleVec & jp,
                                                   const vctDoubleVec & jv)
{
    // assign positions and check limits
    m_servo_jp_param.Goal().Assign(jp);
    if (is_homed()) {
        clip_jp(m_servo_jp_param.Goal());
    }

    // assign velocities
    m_servo_jp_param.Velocity().ForceAssign(jv);
    m_servo_jp_param.SetTimestamp(StateTable.GetTic());
    if (m_has_coupling) {
        m_servo_jp_param.Goal() = m_coupling.JointToActuatorPosition() * m_servo_jp_param.Goal();
        if (m_servo_jp_param.Velocity().size() != 0) {
            m_servo_jp_param.Velocity() = m_coupling.JointToActuatorPosition() * m_servo_jp_param.Velocity();
        }
    }
    PID.servo_jp(m_servo_jp_param);

    apply_feed_forward();
}

bool mtsIntuitiveResearchKitArm::should_use_gravity_compensation(void)
{
    return m_gravity_compensation && m_gravity_compensation_setpoint_js.Valid();
}

void mtsIntuitiveResearchKitArm::apply_feed_forward(void)
{
    auto& jf = m_feed_forward_jf_param.ForceTorque();
    jf.Zeros(); // reset feed forward
    if (should_use_gravity_compensation()) {
        const auto& gc_jf = m_gravity_compensation_setpoint_js.Effort();
        jf.Ref(gc_jf.size()).Assign(gc_jf);
    }

    if (m_has_coupling) {
        jf = m_coupling.JointToActuatorEffort() * jf;
    }

    m_feed_forward_jf_param.SetTimestamp(StateTable.GetTic());
    PID.feed_forward_servo_jf(m_feed_forward_jf_param);
}

void mtsIntuitiveResearchKitArm::hold(void)
{
    if (!ArmIsReady("hold", mtsIntuitiveResearchKitControlTypes::JOINT_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitControlTypes::POSITION_MODE);
    // set goal
    m_servo_jp.Assign(m_pid_setpoint_js.Position());
    m_pid_new_goal = true;
}

void mtsIntuitiveResearchKitArm::free(void)
{
    if (!ArmIsReady("free", mtsIntuitiveResearchKitControlTypes::JOINT_SPACE)) {
        return;
    }

    // request zero effort in joint space
    prmForceTorqueJointSet jf;
    jf.ForceTorque().SetSize(number_of_joints_kinematics());
    jf.ForceTorque().Zeros();
    this->servo_jf(jf);
    // for MTMs
    m_effort_orientation_locked = false;
}

void mtsIntuitiveResearchKitArm::servo_jp(const prmPositionJointSet & jp)
{
    if (!ArmIsReady("servo_jp", mtsIntuitiveResearchKitControlTypes::JOINT_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitControlTypes::POSITION_MODE);
    // set goal
    m_servo_jp.Assign(jp.Goal(), number_of_joints_kinematics());
    m_servo_jv.Assign(jp.Velocity(), jp.Velocity().size());
    m_pid_new_goal = true;
}

void mtsIntuitiveResearchKitArm::servo_jr(const prmPositionJointSet & difference)
{
    if (!ArmIsReady("servo_jr", mtsIntuitiveResearchKitControlTypes::JOINT_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitControlTypes::POSITION_MODE);
    // if there's no current goal, reset it
    if (!m_pid_new_goal) {
        m_servo_jp.Assign(m_pid_setpoint_js.Position());
    }
    m_servo_jp.Ref(number_of_joints_kinematics()).Add(difference.Goal());
    m_pid_new_goal = true;
}

void mtsIntuitiveResearchKitArm::move_jp(const prmPositionJointSet & jp)
{
    if (!ArmIsReady("move_jp", mtsIntuitiveResearchKitControlTypes::JOINT_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitControlTypes::TRAJECTORY_MODE);
    // make sure trajectory is reset
    control_move_jp_on_start();
    // new goal
    vctDoubleVec new_jp(jp.Goal());
    clip_jp(new_jp);
    ToJointsPID(new_jp, m_trajectory_j.goal);
    m_trajectory_j.goal_v.Zeros();
}

void mtsIntuitiveResearchKitArm::move_jr(const prmPositionJointSet & jp)
{
    if (!ArmIsReady("move_jr", mtsIntuitiveResearchKitControlTypes::JOINT_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitControlTypes::TRAJECTORY_MODE);
    // make sure trajectory is reset
    UpdateIsBusy(true);
    // if trajectory is active, add to existing goal
    if (m_trajectory_j.is_active) {
        vctDoubleVec relative(m_trajectory_j.goal.size());
        ToJointsPID(jp.Goal(), relative);
        m_trajectory_j.goal.Add(relative);
    } else {
        // new goal, goal + setpoint
        ToJointsPID(jp.Goal(), m_trajectory_j.goal);
        m_trajectory_j.goal.Add(m_pid_setpoint_js.Position());
    }
    // reset trajectory time
    control_move_jp_on_start();
    m_trajectory_j.goal_v.Zeros();
}

void mtsIntuitiveResearchKitArm::servo_cp(const prmPositionCartesianSet & cp)
{
    if (!ArmIsReady("servo_cp", mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitControlTypes::POSITION_MODE);
    // set goal
    m_servo_cp = cp;
    m_pid_new_goal = true;
}

void mtsIntuitiveResearchKitArm::servo_cr(const prmPositionCartesianSet & difference)
{
    if (!ArmIsReady("servo_cr", mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitControlTypes::POSITION_MODE);
    // set goal --- not sure of this math, move relative to base or tool?
    mCartesianRelative = mCartesianRelative * difference.Goal();
    m_pid_new_goal = true;
}

void mtsIntuitiveResearchKitArm::move_cp(const prmPositionCartesianSet & cp)
{
    if (!ArmIsReady("move_cp", mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitControlTypes::TRAJECTORY_MODE);

    // copy current position
    vctDoubleVec jp(m_kin_measured_js.Position());

    // compute desired slave position
    CartesianPositionFrm.From(cp.Goal());

    if (this->InverseKinematics(jp, m_base_frame.Inverse() * CartesianPositionFrm) == robManipulator::ESUCCESS) {
        // make sure trajectory is reset
        control_move_jp_on_start();
        // new goal
        clip_jp(jp);
        ToJointsPID(jp, m_trajectory_j.goal);
        m_trajectory_j.goal_v.Zeros();
    } else {
        // shows robManipulator error if used
        if (this->Manipulator) {
            m_arm_interface->SendError(this->GetName()
                                       + ": unable to solve inverse kinematics ("
                                       + this->Manipulator->LastError() + ")");
        } else {
            m_arm_interface->SendError(this->GetName() + ": unable to solve inverse kinematics");
        }
        m_trajectory_j.goal_reached_event(false);
        UpdateIsBusy(false);
    }
}

void mtsIntuitiveResearchKitArm::set_base_frame(const prmPositionCartesianSet & newBaseFrame)
{
    if (newBaseFrame.Valid()) {
        this->m_base_frame.FromNormalized(newBaseFrame.Goal());
        this->m_base_frame_valid = true;
        this->m_measured_cp.SetReferenceFrame(newBaseFrame.ReferenceFrame());
        this->m_setpoint_cp.SetReferenceFrame(newBaseFrame.ReferenceFrame());
    } else {
        this->m_base_frame_valid = false;
    }
}

void mtsIntuitiveResearchKitArm::ErrorEventHandler(const mtsMessage & message)
{
    m_arm_interface->SendError(this->GetName() + ": received [" + message.Message + "]");
    SetDesiredState("FAULT");
}

void mtsIntuitiveResearchKitArm::PositionLimitEventHandler(const vctBoolVec & CMN_UNUSED(flags))
{
    m_arm_interface->SendWarning(this->GetName() + ": PID position limit");
}

void mtsIntuitiveResearchKitArm::BiasEncoderEventHandler(const int & nbSamples)
{
    // encoders are biased from pots
    m_encoders_biased_from_pots = true;

    if (nbSamples > 0) {
        // some encoders need to be biased not from pots: MTM roll
        m_encoders_biased = false;
        std::stringstream nbSamplesString;
        nbSamplesString << nbSamples;
        m_arm_interface->SendStatus(this->GetName() + ": encoders biased using " + nbSamplesString.str() + " potentiometer values");
    } else {
        // we assume that all encoders were properly biased.  This is
        // mostly for MTM homing to skip roll homing.  This could fail
        // if the homing process was interrupted between bias from
        // pots and MTM search for roll limits.
        m_encoders_biased = true;
        m_arm_interface->SendStatus(this->GetName() + ": encoders seem to be already biased");
    }

    if (m_homing_bias_encoder_requested) {
        m_homing_bias_encoder_requested = false;
        mArmState.SetCurrentState("ENCODERS_BIASED");
    } else {
        m_arm_interface->SendWarning(this->GetName() + ": encoders have been biased by another process");
    }
}

void mtsIntuitiveResearchKitArm::inverse_kinematics(const prmInverseKinematicsRequest & request,
                                                    prmInverseKinematicsResponse & response) const
{
    if (request.jp().size() != number_of_joints_kinematics()) {
        response.result() = false;
        response.message() = "incorrect number of joints";
        return;
    }
    response.jp().ForceAssign(request.jp());
    if (this->InverseKinematics(response.jp(), request.cp()) == robManipulator::ESUCCESS) {
        response.result() = true;
    } else {
        response.result() = false;
        response.message() = "inverse kinematics failed";
    }
}

void mtsIntuitiveResearchKitArm::forward_kinematics(const prmForwardKinematicsRequest & request,
                                                    prmForwardKinematicsResponse & response) const
{
    size_t nb_joints = request.jp().size();
    if (nb_joints > this->number_of_joints_kinematics()) {
        response.result() = false;
        response.message() = "too many joints";
        return;
    }
    response.cp() = m_base_frame * Manipulator->ForwardKinematics(request.jp(), nb_joints);
    response.result() = true;
}

void mtsIntuitiveResearchKitArm::local_forward_kinematics(const prmForwardKinematicsRequest & request,
                                                          prmForwardKinematicsResponse & response) const
{
    size_t nb_joints = request.jp().size();
    if (nb_joints > this->number_of_joints_kinematics()) {
        response.result() = false;
        response.message() = "too many joints";
        return;
    }
    response.cp() = Manipulator->ForwardKinematics(request.jp(), nb_joints);
    response.result() = true;
}

void mtsIntuitiveResearchKitArm::servo_jf(const prmForceTorqueJointSet & effort)
{
    if (!ArmIsReady("servo_jf", mtsIntuitiveResearchKitControlTypes::JOINT_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitControlTypes::EFFORT_MODE);

    // set new effort
    m_servo_jf.ForceTorque().Assign(effort.ForceTorque());
}

void mtsIntuitiveResearchKitArm::body_servo_cf(const prmForceCartesianSet & cf)
{
    if (!ArmIsReady("body_servo_cf", mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitControlTypes::EFFORT_MODE);

    // set new wrench
    m_cartesian_impedance = false;
    m_servo_cf = cf;
    if (m_servo_cf_type != WRENCH_BODY) {
        m_servo_cf_type = WRENCH_BODY;
        m_arm_interface->SendStatus(this->GetName() + ": effort cartesian WRENCH_BODY");
    }
}

void mtsIntuitiveResearchKitArm::spatial_servo_cf(const prmForceCartesianSet & cf)
{
    if (!ArmIsReady("spatial_servo_cf", mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitControlTypes::EFFORT_MODE);

    // set new wrench
    m_cartesian_impedance = false;
    m_servo_cf = cf;
    if (m_servo_cf_type != WRENCH_SPATIAL) {
        m_servo_cf_type = WRENCH_SPATIAL;
        m_arm_interface->SendStatus(this->GetName() + ": effort cartesian WRENCH_SPATIAL");
    }
}

void mtsIntuitiveResearchKitArm::body_set_cf_orientation_absolute(const bool & absolute)
{
    m_body_cf_orientation_absolute = absolute;
}

void mtsIntuitiveResearchKitArm::use_gravity_compensation(const bool & gravityCompensation)
{
    m_gravity_compensation = gravityCompensation;
}

void mtsIntuitiveResearchKitArm::servo_ci(const prmCartesianImpedance & goal)
{
    if (!ArmIsReady("servo_cf_body", mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitControlTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitControlTypes::EFFORT_MODE);

    // set new wrench
    m_cartesian_impedance = true;
    m_body_cf_orientation_absolute = true;
    mCartesianImpedanceController->SetGoal(goal);
    if (m_servo_cf_type != WRENCH_BODY) {
        m_servo_cf_type = WRENCH_BODY;
        m_arm_interface->SendStatus(this->GetName() + ": effort cartesian WRENCH_BODY");
    }
}
