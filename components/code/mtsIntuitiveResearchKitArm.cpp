/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Zihan Chen, Zerui Wang
  Created on: 2016-02-24

  (C) Copyright 2013-2020 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <sawControllers/osaCartesianImpedanceController.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitRevision.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitConfig.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArm.h>

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
    mStateTableState.AddData(mStateTableStateDesired, "DesiredState");
    m_operating_state.SetValid(true);
    mStateTableState.AddData(m_operating_state, "operating_state");
    AddStateTable(&mStateTableState);
    mStateTableState.SetAutomaticAdvance(false);

    // state table for configuration
    mStateTableConfiguration.AddData(m_configuration_js_kin, "m_configuration_js_kin");
    mStateTableConfiguration.AddData(m_configuration_js_pid, "m_configuration_js_pid");
    AddStateTable(&mStateTableConfiguration);
    mStateTableConfiguration.SetAutomaticAdvance(false);

    m_control_space = mtsIntuitiveResearchKitArmTypes::UNDEFINED_SPACE;
    m_control_mode = mtsIntuitiveResearchKitArmTypes::UNDEFINED_MODE;

    m_simulated = false;
    m_encoders_biased_from_pots = false;
    mHomingGoesToZero = false; // MTM ignores this
    mHomingBiasEncoderRequested = false;

    mWrenchBodyOrientationAbsolute = false;
    m_gravity_compensation = false;
    m_cartesianImpedance = false;

    m_new_pid_goal = false;

    m_effort_orientation_locked = false;

    mSafeForCartesianControlCounter = 0;
    mArmNotReadyCounter = 0;
    mArmNotReadyTimeLastMessage = 0.0;

    // initialize trajectory data
    JointSet.SetSize(NumberOfJoints());
    JointVelocitySet.SetSize(NumberOfJoints());
    JointSetParam.Goal().SetSize(NumberOfJoints());
    FeedForwardParam.ForceTorque().SetSize(NumberOfJoints());
    mJointTrajectory.VelocityMaximum.SetSize(NumberOfJoints());
    mJointTrajectory.Velocity.SetSize(NumberOfJoints());
    mJointTrajectory.AccelerationMaximum.SetSize(NumberOfJoints());
    mJointTrajectory.Acceleration.SetSize(NumberOfJoints());
    mJointTrajectory.Goal.SetSize(NumberOfJoints());
    mJointTrajectory.GoalVelocity.SetSize(NumberOfJoints());
    mJointTrajectory.GoalError.SetSize(NumberOfJoints());
    mJointTrajectory.GoalTolerance.SetSize(NumberOfJoints());
    mJointTrajectory.IsActive = false;

    // initialize velocity
    m_measured_cv.SetVelocityLinear(vct3(0.0));
    m_measured_cv.SetVelocityAngular(vct3(0.0));
    m_measured_cv.SetValid(false);

    // base manipulator class used by most arms (except PSM with snake like tool)
    CreateManipulator();

    // jacobian
    ResizeKinematicsData();
    this->StateTable.AddData(m_jacobian_body, "JacobianBody");
    this->StateTable.AddData(m_jacobian_spatial, "JacobianSpatial");

    // efforts for kinematics
    mEffortJointSet.SetSize(NumberOfJointsKinematics());
    mEffortJointSet.ForceTorque().SetAll(0.0);
    m_measured_cf_body.SetValid(false);

    // base frame, mostly for cases where no base frame is set by user
    m_base_frame = vctFrm4x4::Identity();
    BaseFrameValid = true;

    m_measured_cp.SetAutomaticTimestamp(false); // based on PID timestamp
    m_measured_cp.SetReferenceFrame(GetName() + "_base");
    m_measured_cp.SetMovingFrame(GetName());
    this->StateTable.AddData(m_measured_cp, "CartesianPosition");

    m_setpoint_cp.SetAutomaticTimestamp(false); // based on PID timestamp
    m_setpoint_cp.SetReferenceFrame(GetName() + "_base");
    m_setpoint_cp.SetMovingFrame(GetName() + "_setpoint");
    this->StateTable.AddData(m_setpoint_cp, "setpoint_cp");

    m_measured_cp_local.SetAutomaticTimestamp(false); // based on PID timestamp
    m_measured_cp_local.SetReferenceFrame(GetName() + "_base");
    m_measured_cp_local.SetMovingFrame(GetName());
    this->StateTable.AddData(m_measured_cp_local, "measured_cp_local");

    m_setpoint_cp_local.SetAutomaticTimestamp(false); // based on PID timestamp
    m_setpoint_cp_local.SetReferenceFrame(GetName() + "_base");
    m_setpoint_cp_local.SetMovingFrame(GetName() + "_setpoint");
    this->StateTable.AddData(m_setpoint_cp_local, "setpoint_cp_local");

    this->StateTable.AddData(m_base_frame, "m_base_frame");

    m_measured_cv.SetAutomaticTimestamp(false); // keep PID timestamp
    m_measured_cv.SetMovingFrame(GetName());
    m_measured_cv.SetReferenceFrame(GetName() + "_base");
    this->StateTable.AddData(m_measured_cv, "m_measured_cv");

    m_measured_cf_body.SetAutomaticTimestamp(false); // keep PID timestamp
    this->StateTable.AddData(m_measured_cf_body, "measured_cf_body");

    m_measured_js_kin.SetAutomaticTimestamp(false); // keep PID timestamp
    this->StateTable.AddData(m_measured_js_kin, "m_measured_js_kin");

    m_setpoint_js_kin.SetAutomaticTimestamp(false); // keep PID timestamp
    this->StateTable.AddData(m_setpoint_js_kin, "m_setpoint_js_kin");

    // PID
    PIDInterface = AddInterfaceRequired("PID");
    if (PIDInterface) {
        PIDInterface->AddFunction("SetCoupling", PID.SetCoupling);
        PIDInterface->AddFunction("Enable", PID.Enable);
        PIDInterface->AddFunction("EnableJoints", PID.EnableJoints);
        PIDInterface->AddFunction("Enabled", PID.Enabled);
        PIDInterface->AddFunction("measured_js", PID.measured_js);
        PIDInterface->AddFunction("setpoint_js", PID.setpoint_js);
        PIDInterface->AddFunction("servo_jp", PID.servo_jp);
        PIDInterface->AddFunction("SetFeedForwardJoint", PID.SetFeedForwardJoint);
        PIDInterface->AddFunction("SetCheckPositionLimit", PID.SetCheckPositionLimit);
        PIDInterface->AddFunction("configuration_js", PID.configuration_js);
        PIDInterface->AddFunction("configure_js", PID.configure_js);
        PIDInterface->AddFunction("EnableTorqueMode", PID.EnableTorqueMode);
        PIDInterface->AddFunction("servo_jf", PID.servo_jf);
        PIDInterface->AddFunction("EnableTrackingError", PID.EnableTrackingError);
        PIDInterface->AddFunction("SetTrackingErrorTolerances", PID.SetTrackingErrorTolerance);
        PIDInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::PositionLimitEventHandler, this, "PositionLimit");
        PIDInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::ErrorEventHandler, this, "error");
    }

    // Robot IO
    IOInterface = AddInterfaceRequired("RobotIO");
    if (IOInterface) {
        IOInterface->AddFunction("GetSerialNumber", IO.GetSerialNumber);
        IOInterface->AddFunction("EnablePower", IO.EnablePower);
        IOInterface->AddFunction("DisablePower", IO.DisablePower);
        IOInterface->AddFunction("GetActuatorAmpStatus", IO.GetActuatorAmpStatus);
        IOInterface->AddFunction("GetBrakeAmpStatus", IO.GetBrakeAmpStatus);
        IOInterface->AddFunction("BiasEncoder", IO.BiasEncoder);
        IOInterface->AddFunction("SetSomeEncoderPosition", IO.SetSomeEncoderPosition);
        IOInterface->AddFunction("SetActuatorCurrent", IO.SetActuatorCurrent);
        IOInterface->AddFunction("UsePotsForSafetyCheck", IO.UsePotsForSafetyCheck);
        IOInterface->AddFunction("BrakeRelease", IO.BrakeRelease);
        IOInterface->AddFunction("BrakeEngage", IO.BrakeEngage);
        IOInterface->AddEventHandlerWrite(&mtsIntuitiveResearchKitArm::BiasEncoderEventHandler, this, "BiasEncoder");
    }

    // Arm
    RobotInterface = AddInterfaceProvided("Robot");
    if (RobotInterface) {
        RobotInterface->AddMessageEvents();

        // Get
        RobotInterface->AddCommandReadState(this->mStateTableConfiguration, m_configuration_js_kin, "configuration_js");
        RobotInterface->AddCommandReadState(this->StateTable, m_measured_js_kin, "measured_js");
        RobotInterface->AddCommandReadState(this->StateTable, m_setpoint_js_kin, "setpoint_js");
        RobotInterface->AddCommandReadState(this->StateTable, m_measured_cp_local, "measured_cp_local");
        RobotInterface->AddCommandReadState(this->StateTable, m_setpoint_cp_local, "setpoint_cp_local");
        RobotInterface->AddCommandReadState(this->StateTable, m_measured_cp, "measured_cp");
        RobotInterface->AddCommandReadState(this->StateTable, m_setpoint_cp, "setpoint_cp");
        RobotInterface->AddCommandReadState(this->StateTable, m_base_frame, "GetBaseFrame");
        RobotInterface->AddCommandReadState(this->StateTable, m_measured_cv, "measured_cv");
        RobotInterface->AddCommandReadState(this->StateTable, m_measured_cf_body, "measured_cf_body");
        RobotInterface->AddCommandReadState(this->StateTable, m_jacobian_body, "jacobian_body");
        RobotInterface->AddCommandReadState(this->StateTable, m_jacobian_spatial, "jacobian_spatial");
        RobotInterface->AddCommandReadState(this->mStateTableState,
                                            m_operating_state, "operating_state");
        // Set
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetBaseFrame,
                                        this, "SetBaseFrame");
        RobotInterface->AddCommandVoid(&mtsIntuitiveResearchKitArm::Freeze,
                                       this, "Freeze");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::servo_jp,
                                        this, "servo_jp");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::servo_jr,
                                        this, "servo_jr");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::move_jp,
                                        this, "move_jp");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::servo_cp,
                                        this, "servo_cp");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::servo_cr,
                                        this, "servo_cr");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::move_cp,
                                        this, "move_cp");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::servo_jf,
                                        this, "servo_jf");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::servo_cf_body,
                                        this, "servo_cf_body");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetWrenchBodyOrientationAbsolute,
                                        this, "SetWrenchBodyOrientationAbsolute");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::servo_cf_spatial,
                                        this, "servo_cf_spatial");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetGravityCompensation,
                                        this, "SetGravityCompensation");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetCartesianImpedanceGains,
                                        this, "SetCartesianImpedanceGains");

        // Trajectory events
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetJointVelocityRatio,
                                        this, "SetJointVelocityRatio");
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::SetJointAccelerationRatio,
                                        this, "SetJointAccelerationRatio");
        RobotInterface->AddEventWrite(mJointTrajectory.VelocityRatioEvent, "JointVelocityRatio", double());
        RobotInterface->AddEventWrite(mJointTrajectory.AccelerationRatioEvent, "JointAccelerationRatio", double());
        RobotInterface->AddEventWrite(mJointTrajectory.GoalReachedEvent, "GoalReached", bool());
        // Robot State
        RobotInterface->AddCommandWrite(&mtsIntuitiveResearchKitArm::state_command,
                                        this, "state_command", std::string(""));
        // Human readable messages
        RobotInterface->AddEventWrite(state_events.desired_state, "DesiredState", std::string(""));
        RobotInterface->AddEventWrite(state_events.current_state, "CurrentState", std::string(""));
        RobotInterface->AddEventWrite(state_events.operating_state, "operating_state", prmOperatingState());

        // Stats
        RobotInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
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
        RobotInterface->SendError(this->GetName() + ": unsupported state " + state);
        return;
    }
    // try to set the desired state
    try {
        mArmState.SetDesiredState(state);
    } catch (...) {
        RobotInterface->SendError(this->GetName() + ": " + state + " is not an allowed desired state");
        return;
    }
    // update state table
    mStateTableState.Start();
    mStateTableStateDesired = state;
    mStateTableState.Advance();

    StateEvents();
    RobotInterface->SendStatus(this->GetName() + ": desired state " + state);

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
                UnHome();
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
            RobotInterface->SendWarning(this->GetName() + ": " + humanReadableMessage);
        }
    } catch (std::runtime_error & e) {
        RobotInterface->SendWarning(this->GetName() + ": " + command + " doesn't seem to be a valid state_command (" + e.what() + ")");
    }
}

void mtsIntuitiveResearchKitArm::UpdateConfigurationJointKinematic(void)
{
    // get names, types and joint limits for kinematics config from the manipulator
    // name and types need conversion
    mStateTableConfiguration.Start();
    m_configuration_js_kin.Name().SetSize(NumberOfJointsKinematics());
    m_configuration_js_kin.Type().SetSize(NumberOfJointsKinematics());
    const size_t jointsConfiguredSoFar = this->Manipulator->links.size();
    std::vector<std::string> names(jointsConfiguredSoFar);
    std::vector<robJoint::Type> types(jointsConfiguredSoFar);
    this->Manipulator->GetJointNames(names);
    this->Manipulator->GetJointTypes(types);
    for (size_t index = 0; index < jointsConfiguredSoFar; ++index) {
        m_configuration_js_kin.Name().at(index) = names.at(index);
        switch (types.at(index)) {
        case robJoint::HINGE:
            m_configuration_js_kin.Type().at(index) = PRM_JOINT_REVOLUTE;
            break;
        case robJoint::SLIDER:
            m_configuration_js_kin.Type().at(index) = PRM_JOINT_PRISMATIC;
            break;
        default:
            m_configuration_js_kin.Type().at(index) = PRM_JOINT_UNDEFINED;
            break;
        }
    }
    // position limits can be read as is
    m_configuration_js_kin.PositionMin().SetSize(NumberOfJointsKinematics());
    m_configuration_js_kin.PositionMax().SetSize(NumberOfJointsKinematics());
    this->Manipulator->GetJointLimits(m_configuration_js_kin.PositionMin().Ref(jointsConfiguredSoFar),
                                      m_configuration_js_kin.PositionMax().Ref(jointsConfiguredSoFar));
    mStateTableConfiguration.Advance();
}

void mtsIntuitiveResearchKitArm::ResizeKinematicsData(void)
{
    m_jacobian_body.SetSize(6, NumberOfJointsKinematics());
    m_jacobian_spatial.SetSize(6, NumberOfJointsKinematics());
    mJacobianBodyTranspose.ForceAssign(m_jacobian_body.Transpose());
    mJacobianPInverseData.Allocate(mJacobianBodyTranspose);
    mEffortJointSet.SetSize(NumberOfJointsKinematics());
    mEffortJointSet.ForceTorque().SetAll(0.0);
    mEffortJoint.SetSize(NumberOfJointsKinematics());
    mEffortJoint.SetAll(0.0);
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

        // detect if we're using 1.8 and up with two fields, kinematic and tool-detection
        const auto jsonKinematic = jsonConfig["kinematic"];
        if (!jsonKinematic.isNull()) {
            // extract path of main json config file to search other files relative to it
            cmnPath configPath(cmnPath::GetWorkingDirectory());
            std::string fullname = configPath.Find(filename);
            std::string configDir = fullname.substr(0, fullname.find_last_of('/'));
            // for user files first
            configPath.Add(configDir, cmnPath::TAIL);
            // for standard files using io/xyz.json, arm/xyz.json
            configPath.Add(std::string(sawIntuitiveResearchKit_SOURCE_DIR) + "/../share", cmnPath::TAIL);
            // for tool definition files
            configPath.Add(std::string(sawIntuitiveResearchKit_SOURCE_DIR) + "/../share/tool", cmnPath::TAIL);

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

            // arm specific configuration
            PostConfigure(jsonConfig, configPath, filename);

        } else {
            std::stringstream message;
            message << "Configure " << this->GetName() << ":" << std::endl
                    << "----------------------------------------------------" << std::endl
                    << " ERROR:" << std::endl
                    << "   You should have a \"arm\" file for each arm in the console" << std::endl
                    << "   file.  The arm file should contain the fields" << std::endl
                    << "   \"kinematic\" and options specific to each arm type." << std::endl
                    << "----------------------------------------------------";
            std::cerr << "mtsIntuitiveResearchKitConsole::" << message.str() << std::endl;
            CMN_LOG_CLASS_INIT_ERROR << message.str() << std::endl;
            exit(EXIT_FAILURE);
        }

        // should arm go to zero position when homing, default set in Init method
        const Json::Value jsonHomingGoesToZero = jsonConfig["homing-zero-position"];
        if (!jsonHomingGoesToZero.isNull()) {
            mHomingGoesToZero = jsonHomingGoesToZero.asBool();
        }

        // should ignore preloaded encoders and force homing
        const Json::Value jsonAlwaysHome = jsonConfig["re-home"];
        if (!jsonAlwaysHome.isNull()) {
            m_re_home = jsonAlwaysHome.asBool();
        }

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
                                             const std::string & filename)
{
    // load base offset transform if any (without warning)
    const Json::Value jsonBase = jsonConfig["base-offset"];
    if (!jsonBase.isNull()) {
        // save the transform as Manipulator Rtw0
        cmnDataJSON<vctFrm4x4>::DeSerializeText(Manipulator->Rtw0, jsonBase);
        if (!nmrIsOrthonormal(Manipulator->Rtw0.Rotation())) {
            CMN_LOG_CLASS_INIT_ERROR << "ConfigureDH " << this->GetName()
                                     << ": the base offset rotation doesn't seem to be orthonormal"
                                     << std::endl;
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

    // update ConfigurationJointKinematic from manipulator
    UpdateConfigurationJointKinematic();

    // resize data members using kinematics (jacobians and effort vectors)
    ResizeKinematicsData();
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
}

void mtsIntuitiveResearchKitArm::Run(void)
{
    // collect data from required interfaces
    ProcessQueuedEvents();
    try {
        mArmState.Run();
    } catch (std::exception & e) {
        RobotInterface->SendError(this->GetName() + ": in state " + mArmState.CurrentState()
                                  + ", caught exception \"" + e.what() + "\"");
        SetDesiredState("DISABLED");
    }
    // trigger ExecOut event
    RunEvent();
    ProcessQueuedCommands();

    CartesianGetPrevious = m_measured_cp_frame;
}

void mtsIntuitiveResearchKitArm::Cleanup(void)
{
    if (NumberOfBrakes() > 0) {
        IO.BrakeEngage();
    }
    CMN_LOG_CLASS_INIT_VERBOSE << GetName() << ": Cleanup" << std::endl;
}

void mtsIntuitiveResearchKitArm::SetSimulated(void)
{
    m_simulated = true;
    // in simulation mode, we don't need IO
    RemoveInterfaceRequired("RobotIO");
}

void mtsIntuitiveResearchKitArm::GetRobotData(void)
{
    // check that the robot still has power
    if (m_powered && !m_simulated) {
        vctBoolVec actuatorAmplifiersStatus(NumberOfJoints());
        IO.GetActuatorAmpStatus(actuatorAmplifiersStatus);
        vctBoolVec brakeAmplifiersStatus(NumberOfBrakes());
        if (NumberOfBrakes() > 0) {
            IO.GetBrakeAmpStatus(brakeAmplifiersStatus);
        }
        if (!(actuatorAmplifiersStatus.All() && brakeAmplifiersStatus.All())) {
            m_powered = false;
            RobotInterface->SendError(this->GetName() + ": detected power loss");
            mArmState.SetDesiredState("DISABLED");
            return;
        }
    }

    // we can start reporting some joint values after the robot is powered
    if (IsJointReady()) {
        mtsExecutionResult executionResult;
        // joint state
        executionResult = PID.measured_js(m_measured_js_pid);
        if (executionResult.IsOK()) {
            m_measured_js_pid.SetValid(true);
        } else {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to PID.measured_js failed \""
                                    << executionResult << "\"" << std::endl;
            m_measured_js_pid.SetValid(false);
        }

        // desired joint state
        executionResult = PID.setpoint_js(m_setpoint_js_pid);
        if (executionResult.IsOK()) {
            m_setpoint_js_pid.SetValid(true);
        } else {
            CMN_LOG_CLASS_RUN_ERROR << GetName() << ": GetRobotData: call to PID.setpoint_js failed \""
                                    << executionResult << "\"" << std::endl;
            m_setpoint_js_pid.SetValid(false);
        }

        // update joint states used for kinematics
        UpdateStateJointKinematics();

    } else {
        // set joint to zeros
        m_measured_js_pid.Position().Zeros();
        m_measured_js_pid.Velocity().Zeros();
        m_measured_js_pid.Effort().Zeros();
        m_measured_js_pid.SetValid(false);

        m_measured_js_kin.Position().Zeros();
        m_measured_js_kin.Velocity().Zeros();
        m_measured_js_kin.Effort().Zeros();
        m_measured_js_kin.SetValid(false);
    }

    // when the robot is ready, we can compute cartesian position
    if (IsCartesianReady()) {
        CMN_ASSERT(IsJointReady());
        // update cartesian position
        m_measured_cp_local_frame = Manipulator->ForwardKinematics(m_measured_js_kin.Position());
        m_measured_cp_frame = m_base_frame * m_measured_cp_local_frame;
        // normalize
        m_measured_cp_local_frame.Rotation().NormalizedSelf();
        m_measured_cp_frame.Rotation().NormalizedSelf();
        // prm types
        m_measured_cp_local.Position().From(m_measured_cp_local_frame);
        m_measured_cp_local.SetTimestamp(m_measured_js_kin.Timestamp());
        m_measured_cp_local.SetValid(true);
        m_measured_cp.Position().From(m_measured_cp_frame);
        m_measured_cp.SetTimestamp(m_measured_js_kin.Timestamp());
        m_measured_cp.SetValid(BaseFrameValid);

        // update jacobians
        Manipulator->JacobianSpatial(m_measured_js_kin.Position(), m_jacobian_spatial);
        Manipulator->JacobianBody(m_measured_js_kin.Position(), m_jacobian_body);

        // update cartesian velocity using the jacobian and joint
        // velocities.
        vctDoubleVec cartesianVelocity(6);
        cartesianVelocity.ProductOf(m_jacobian_body, m_measured_js_kin.Velocity());
        vct3 relative, absolute;
        // linear
        relative.Assign(cartesianVelocity.Ref(3, 0));
        m_measured_cp_frame.Rotation().ApplyTo(relative, absolute);
        m_measured_cv.SetVelocityLinear(absolute);
        // angular
        relative.Assign(cartesianVelocity.Ref(3, 3));
        m_measured_cp_frame.Rotation().ApplyTo(relative, absolute);
        m_measured_cv.SetVelocityAngular(absolute);
        // valid/timestamp
        m_measured_cv.SetValid(true);
        m_measured_cv.SetTimestamp(m_measured_js_kin.Timestamp());

        // update wrench based on measured joint current efforts
        mJacobianBodyTranspose.Assign(m_jacobian_body.Transpose());
        nmrPInverse(mJacobianBodyTranspose, mJacobianPInverseData);
        vctDoubleVec wrench(6);
        wrench.ProductOf(mJacobianPInverseData.PInverse(), m_measured_js_kin.Effort());
        if (mWrenchBodyOrientationAbsolute) {
            // forces
            relative.Assign(wrench.Ref(3, 0));
            m_measured_cp_frame.Rotation().ApplyTo(relative, absolute);
            m_measured_cf_body.Force().Ref<3>(0).Assign(absolute);
            // torques
            relative.Assign(wrench.Ref(3, 3));
            m_measured_cp_frame.Rotation().ApplyTo(relative, absolute);
            m_measured_cf_body.Force().Ref<3>(3).Assign(absolute);
        } else {
            m_measured_cf_body.Force().Assign(wrench);
        }
        // valid/timestamp
        m_measured_cf_body.SetValid(true);
        m_measured_cf_body.SetTimestamp(m_measured_js_kin.Timestamp());

        // update cartesian position desired based on joint desired
        m_setpoint_cp_local_frame = Manipulator->ForwardKinematics(m_setpoint_js_kin.Position());
        m_setpoint_cp_frame = m_base_frame * m_setpoint_cp_local_frame;
        // normalize
        m_setpoint_cp_local_frame.Rotation().NormalizedSelf();
        m_setpoint_cp_frame.Rotation().NormalizedSelf();
        // prm type
        m_setpoint_cp_local.Position().From(m_setpoint_cp_local_frame);
        m_setpoint_cp_local.SetTimestamp(m_setpoint_js_kin.Timestamp());
        m_setpoint_cp_local.SetValid(true);
        m_setpoint_cp.Position().From(m_setpoint_cp_frame);
        m_setpoint_cp.SetTimestamp(m_setpoint_js_kin.Timestamp());
        m_setpoint_cp.SetValid(BaseFrameValid);

    } else {
        // set cartesian data to "zero"
        m_measured_cp_local_frame.Assign(vctFrm4x4::Identity());
        m_measured_cp_frame.Assign(vctFrm4x4::Identity());
        m_measured_cp_local.SetValid(false);
        m_measured_cp.SetValid(false);
        // velocities and wrench
        m_measured_cv.SetValid(false);
        m_measured_cf_body.SetValid(false);
        // update cartesian position desired
        m_setpoint_cp_local_frame.Assign(vctFrm4x4::Identity());
        m_setpoint_cp_frame.Assign(vctFrm4x4::Identity());
        m_setpoint_cp_local.SetValid(false);
        m_setpoint_cp.SetValid(false);
    }
}

void mtsIntuitiveResearchKitArm::UpdateStateJointKinematics(void)
{
    m_measured_js_kin = m_measured_js_pid;
    m_setpoint_js_kin = m_setpoint_js_pid;
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
    m_operating_state.IsHomed() = IsHomed();
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
    RobotInterface->SendStatus(this->GetName() + ": current state " + state);
}

void mtsIntuitiveResearchKitArm::RunAllStates(void)
{
    GetRobotData();
}

void mtsIntuitiveResearchKitArm::EnterDisabled(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::DISABLED, false);

    if (NumberOfBrakes() > 0) {
        IO.BrakeEngage();
    }

    IO.UsePotsForSafetyCheck(false);
    IO.SetActuatorCurrent(vctDoubleVec(NumberOfJoints(), 0.0));
    IO.DisablePower();
    PID.Enable(false);
    PID.SetCheckPositionLimit(true);
    m_powered = false;
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::UNDEFINED_SPACE,
                           mtsIntuitiveResearchKitArmTypes::UNDEFINED_MODE);
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
        PID.EnableTrackingError(false);
        PID.Enable(true);
        PID.EnableJoints(vctBoolVec(NumberOfJoints(), true));
        vctDoubleVec goal(NumberOfJoints());
        goal.SetAll(0.0);
        mtsIntuitiveResearchKitArm::SetPositionJointLocal(goal);
        return;
    }

    const double currentTime = this->StateTable.GetTic();

    // in case we still have power but brakes are not engaged
    if (NumberOfBrakes() > 0) {
        IO.BrakeEngage();
    }

    mHomingTimer = currentTime;
    // make sure the PID is not sending currents
    PID.Enable(false);
    // pre-load the boards with zero current
    IO.SetActuatorCurrent(vctDoubleVec(NumberOfJoints(), 0.0));
    // enable power and set a flags to move to next step
    IO.EnablePower();
    RobotInterface->SendStatus(this->GetName() + ": power requested");
}

void mtsIntuitiveResearchKitArm::TransitionPowering(void)
{
    if (m_simulated) {
        mArmState.SetCurrentState("ENABLED");
        return;
    }

    const double currentTime = this->StateTable.GetTic();

    // check power status
    vctBoolVec actuatorAmplifiersStatus(NumberOfJoints());
    IO.GetActuatorAmpStatus(actuatorAmplifiersStatus);
    vctBoolVec brakeAmplifiersStatus(NumberOfBrakes());
    if (NumberOfBrakes() > 0) {
        IO.GetBrakeAmpStatus(brakeAmplifiersStatus);
    }
    if (actuatorAmplifiersStatus.All() && brakeAmplifiersStatus.All()) {
        RobotInterface->SendStatus(this->GetName() + ": power on");
        mArmState.SetCurrentState("ENABLED");
    } else {
        if ((currentTime - mHomingTimer) > mtsIntuitiveResearchKit::TimeToPower) {
            RobotInterface->SendError(this->GetName() + ": failed to enable power");
            std::cerr << CMN_LOG_DETAILS << " should something be done here?" << std::endl;
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

    // disable PID for fallback
    IO.SetActuatorCurrent(vctDoubleVec(NumberOfJoints(), 0.0));
    PID.Enable(false);
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::UNDEFINED_SPACE,
                           mtsIntuitiveResearchKitArmTypes::UNDEFINED_MODE);

    // engage brakes for fallback
    if (NumberOfBrakes() > 0) {
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
        RobotInterface->SendStatus(this->GetName() + ": simulated mode, no need to calibrate encoders");
        return;
    }
    if (m_encoders_biased_from_pots) {
        RobotInterface->SendStatus(this->GetName() + ": encoders have already been calibrated, skipping");
        return;
    }

    // request bias encoder
    const double currentTime = this->StateTable.GetTic();
    const int nb_samples = 1970; // birth year, state table contains 1999 elements so anything under that would work
    if (m_re_home) {
        // positive number to ignore encoder preloads
        IO.BiasEncoder(nb_samples);
    } else {
        // negative numbers means that we first check if encoders have already been preloaded
        IO.BiasEncoder(-nb_samples);
    }
    mHomingBiasEncoderRequested = true;
    mHomingTimer = currentTime;
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
    if ((currentTime - mHomingTimer) > timeToBias) {
        mHomingBiasEncoderRequested = false;
        RobotInterface->SendError(this->GetName() + ": failed to bias encoders (timeout)");
        std::cerr << CMN_LOG_DETAILS << " should something be done here?" << std::endl;
        SetDesiredState("FAULT");
    }
}

void mtsIntuitiveResearchKitArm::EnterEncodersBiased(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, false);
    // use pots for redundancy
    IO.UsePotsForSafetyCheck(true);
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
    PID.SetCheckPositionLimit(false);
    // enable tracking errors
    PID.SetTrackingErrorTolerance(PID.DefaultTrackingErrorTolerance);

    // release brakes if any
    if ((NumberOfBrakes() > 0) && !m_simulated) {
        IO.BrakeRelease();
    }

    // get robot data to make sure we have latest state
    CMN_ASSERT(IsJointReady());
    GetRobotData();

    // compute joint goal position
    this->SetGoalHomingArm();
    // initialize trajectory with current position and velocities
    JointSet.Assign(m_setpoint_js_pid.Position());
    JointVelocitySet.Assign(m_measured_js_pid.Velocity());
    mJointTrajectory.GoalVelocity.SetAll(0.0);
    mJointTrajectory.EndTime = 0.0;
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);

    // enable PID on all joints
    mtsIntuitiveResearchKitArm::SetPositionJointLocal(JointSet);
    PID.Enable(true);
    PID.EnableJoints(vctBoolVec(NumberOfJoints(), true));
}

void mtsIntuitiveResearchKitArm::RunHoming(void)
{
    static const double extraTime = 2.0 * cmn_s;
    const double currentTime = this->StateTable.GetTic();

    mJointTrajectory.Reflexxes.Evaluate(JointSet,
                                        JointVelocitySet,
                                        mJointTrajectory.Goal,
                                        mJointTrajectory.GoalVelocity);
    mtsIntuitiveResearchKitArm::SetPositionJointLocal(JointSet);

    const robReflexxes::ResultType trajectoryResult = mJointTrajectory.Reflexxes.ResultValue();
    bool isHomed;

    switch (trajectoryResult) {

    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (mJointTrajectory.EndTime == 0.0) {
            mJointTrajectory.EndTime = currentTime + mJointTrajectory.Reflexxes.Duration();
            mHomingTimer = mJointTrajectory.EndTime;
        }
        break;

    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        // check position
        mJointTrajectory.GoalError.DifferenceOf(mJointTrajectory.Goal, m_measured_js_pid.Position());
        mJointTrajectory.GoalError.AbsSelf();
        isHomed = !mJointTrajectory.GoalError.ElementwiseGreaterOrEqual(mJointTrajectory.GoalTolerance).Any();
        if (isHomed) {
            m_operating_state.IsHomed() = true;
            PID.SetCheckPositionLimit(true);
            mArmState.SetCurrentState("HOMED");
        } else {
            // time out
            if (currentTime > mHomingTimer + extraTime) {
                CMN_LOG_CLASS_INIT_WARNING << GetName() << ": RunHoming: unable to reach home position, error in degrees is "
                                           << mJointTrajectory.GoalError * (180.0 / cmnPI) << std::endl;
                RobotInterface->SendError(this->GetName() + ": unable to reach home position during calibration on pots");
                std::cerr << CMN_LOG_DETAILS << " should something be done here?" << std::endl;
                SetDesiredState("FAULT");
            }
        }
        break;

    default:
        RobotInterface->SendError(this->GetName() + ": error while evaluating trajectory");
        std::cerr << CMN_LOG_DETAILS << " should something be done here?" << std::endl;
        SetDesiredState("FAULT");
        break;
    }
}

void mtsIntuitiveResearchKitArm::EnterHomed(void)
{
    UpdateOperatingStateAndBusy(prmOperatingState::ENABLED, false);

    // no control mode defined
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::UNDEFINED_SPACE,
                           mtsIntuitiveResearchKitArmTypes::UNDEFINED_MODE);

    if (m_simulated) {
        return;
    }

    // enable PID and start from current position
    mtsIntuitiveResearchKitArm::SetPositionJointLocal(m_setpoint_js_pid.Position());
    PID.EnableTrackingError(UsePIDTrackingError());
    PID.EnableJoints(vctBoolVec(NumberOfJoints(), true));
    PID.SetCheckPositionLimit(true);
    PID.Enable(true);
    PID.EnableJoints(vctBoolVec(NumberOfJoints(), true));
}

void mtsIntuitiveResearchKitArm::LeaveHomed(void)
{
    // no control mode defined
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::UNDEFINED_SPACE,
                           mtsIntuitiveResearchKitArmTypes::UNDEFINED_MODE);
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
    UpdateOperatingStateAndBusy(prmOperatingState::FAULT, false);
}

void mtsIntuitiveResearchKitArm::ControlPositionJoint(void)
{
    if (m_new_pid_goal) {
        SetPositionJointLocal(JointSet);
        // reset flag
        m_new_pid_goal = false;
    }
}

void mtsIntuitiveResearchKitArm::ControlPositionGoalJoint(void)
{
    // check if there's anything to do
    if (!mJointTrajectory.IsActive) {
        return;
    }

    mJointTrajectory.Reflexxes.Evaluate(JointSet,
                                        JointVelocitySet,
                                        mJointTrajectory.Goal,
                                        mJointTrajectory.GoalVelocity);
    SetPositionJointLocal(JointSet);

    const robReflexxes::ResultType trajectoryResult = mJointTrajectory.Reflexxes.ResultValue();
    const double currentTime = this->StateTable.GetTic();

    switch (trajectoryResult) {
    case robReflexxes::Reflexxes_WORKING:
        // if this is the first evaluation, we can't calculate expected completion time
        if (mJointTrajectory.EndTime == 0.0) {
            mJointTrajectory.EndTime = currentTime + mJointTrajectory.Reflexxes.Duration();
        }
        break;
    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        mJointTrajectory.GoalReachedEvent(true);
        mJointTrajectory.IsActive = false;
        UpdateIsBusy(false);
        break;
    default:
        RobotInterface->SendError(this->GetName() + ": error while evaluating trajectory");
        mJointTrajectory.GoalReachedEvent(false);
        mJointTrajectory.IsActive = false;
        UpdateIsBusy(false);
        break;
    }
}

void mtsIntuitiveResearchKitArm::ControlPositionCartesian(void)
{
    if (m_new_pid_goal) {
        // copy current position
        vctDoubleVec jointSet(m_measured_js_kin.Position());

        // compute desired arm position
        CartesianPositionFrm.From(CartesianSetParam.Goal());
        if (this->InverseKinematics(jointSet, m_base_frame.Inverse() * CartesianPositionFrm) == robManipulator::ESUCCESS) {
            // finally send new joint values
            SetPositionJointLocal(jointSet);
        } else {
            // shows robManipulator error if used
            if (this->Manipulator) {
                RobotInterface->SendError(this->GetName()
                                          + ": unable to solve inverse kinematics ("
                                          + this->Manipulator->LastError() + ")");
            } else {
                RobotInterface->SendError(this->GetName() + ": unable to solve inverse kinematics");
            }
        }
        // reset flag
        m_new_pid_goal = false;
    }
}

void mtsIntuitiveResearchKitArm::ControlPositionGoalCartesian(void)
{
    // trajectories are computed in joint space for now
    ControlPositionGoalJoint();
}

bool mtsIntuitiveResearchKitArm::ArmIsReady(const std::string & methodName,
                                            const mtsIntuitiveResearchKitArmTypes::ControlSpace space)
{
    // reset counter if ready
    if (m_operating_state.State() == prmOperatingState::ENABLED) {
        if (((space == mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)
             && IsJointReady())
            || ((space == mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)
                && IsCartesianReady())) {
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
        RobotInterface->SendWarning(message.str());
        mArmNotReadyTimeLastMessage = StateTable.GetTic();
    }
    mArmNotReadyCounter++;
    return false;
}

void mtsIntuitiveResearchKitArm::SetJointVelocityRatio(const double & ratio)
{
    if (ratio > 0.0 && ratio <= 1.0) {
        mJointTrajectory.Velocity.ProductOf(ratio, mJointTrajectory.VelocityMaximum);
        mJointTrajectory.VelocityRatio = ratio;
        mJointTrajectory.VelocityRatioEvent(ratio);
    } else {
        std::stringstream message;
        message << this->GetName() << ": SetJointVelocityRatio, ratio must be within ]0;1], received " << ratio;
        RobotInterface->SendWarning(message.str());
    }
}

void mtsIntuitiveResearchKitArm::SetJointAccelerationRatio(const double & ratio)
{
    if (ratio > 0.0 && ratio <= 1.0) {
        mJointTrajectory.Acceleration.ProductOf(ratio, mJointTrajectory.AccelerationMaximum);
        mJointTrajectory.AccelerationRatio = ratio;
        mJointTrajectory.AccelerationRatioEvent(ratio);
    } else {
        std::stringstream message;
        message << this->GetName() << ": SetJointAccelerationRatio, ratio must be within ]0;1], received " << ratio;
        RobotInterface->SendWarning(message.str());
    }
}

void mtsIntuitiveResearchKitArm::SetControlSpaceAndMode(const mtsIntuitiveResearchKitArmTypes::ControlSpace space,
                                                        const mtsIntuitiveResearchKitArmTypes::ControlMode mode,
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
        if (space == mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE) {
            if (this->IsSafeForCartesianControl()) {
                // set flag
                m_control_space = space;
                mSafeForCartesianControlCounter = 0;
            } else {
                if (mSafeForCartesianControlCounter == 0) {
                    // message if needed
                    RobotInterface->SendWarning(this->GetName() + ": tool/endoscope needs to be inserted past the cannula to switch to cartesian space");
                }
                mSafeForCartesianControlCounter = (mSafeForCartesianControlCounter + 1) % 1000;
                return;
            }
        } else {
            // all other spaces, just set the current space
            m_effort_orientation_locked = false;
            m_control_space = space;
        }
    }

    if (mode != m_control_mode) {

        if ((m_control_mode == mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE)
            &&  mJointTrajectory.IsActive) {
            mJointTrajectory.IsActive = false;
            UpdateIsBusy(false);
        }

        switch (mode) {
        case mtsIntuitiveResearchKitArmTypes::POSITION_MODE:
        case mtsIntuitiveResearchKitArmTypes::POSITION_INCREMENT_MODE:
            // configure PID
            PID.EnableTrackingError(UsePIDTrackingError());
            PID.EnableTorqueMode(vctBoolVec(NumberOfJoints(), false));
            m_new_pid_goal = false;
            mCartesianRelative = vctFrm3::Identity();
            JointSet.Assign(m_setpoint_js_pid.Position(), NumberOfJoints());
            m_effort_orientation_locked = false;
            break;
        case mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE:
            // configure PID
            PID.EnableTrackingError(UsePIDTrackingError());
            PID.EnableTorqueMode(vctBoolVec(NumberOfJoints(), false));
            m_effort_orientation_locked = false;
            // initialize trajectory
            JointSet.Assign(m_setpoint_js_pid.Position(), NumberOfJoints());
            if ((m_control_mode == mtsIntuitiveResearchKitArmTypes::POSITION_MODE)
                || (m_control_mode == mtsIntuitiveResearchKitArmTypes::POSITION_INCREMENT_MODE)) {
                JointVelocitySet.Assign(m_measured_js_pid.Velocity(), NumberOfJoints());
            } else {
                // we're switching from effort or no mode
                JointVelocitySet.SetSize(NumberOfJoints());
                JointVelocitySet.SetAll(0.0);
            }
            mJointTrajectory.Reflexxes.Set(mJointTrajectory.Velocity,
                                           mJointTrajectory.Acceleration,
                                           StateTable.PeriodStats.PeriodAvg(),
                                           robReflexxes::Reflexxes_TIME);
            break;
        case mtsIntuitiveResearchKitArmTypes::EFFORT_MODE:
            // configure PID
            PID.EnableTrackingError(false);
            mEffortJoint.Assign(vctDoubleVec(NumberOfJointsKinematics(), 0.0));
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
    case mtsIntuitiveResearchKitArmTypes::POSITION_MODE:
    case mtsIntuitiveResearchKitArmTypes::POSITION_INCREMENT_MODE:
        switch (m_control_space) {
        case mtsIntuitiveResearchKitArmTypes::JOINT_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::ControlPositionJoint, this);
            break;
        case mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::ControlPositionCartesian, this);
            break;
        default:
            break;
        }
        break;
    case mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE:
        switch (m_control_space) {
        case mtsIntuitiveResearchKitArmTypes::JOINT_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::ControlPositionGoalJoint, this);
            break;
        case mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::ControlPositionGoalCartesian, this);
            break;
        default:
            break;
        }
        break;
    case mtsIntuitiveResearchKitArmTypes::EFFORT_MODE:
        switch (m_control_space) {
        case mtsIntuitiveResearchKitArmTypes::JOINT_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::ControlEffortJoint, this);
            break;
        case mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE:
            SetControlCallback(&mtsIntuitiveResearchKitArm::ControlEffortCartesian, this);
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
    if ((m_control_mode == mtsIntuitiveResearchKitArmTypes::USER_MODE)
        || (m_control_space == mtsIntuitiveResearchKitArmTypes::USER_SPACE)) {
        mControlCallback = callback;
    }

    // messages
    RobotInterface->SendStatus(this->GetName() + ": control "
                               + cmnData<mtsIntuitiveResearchKitArmTypes::ControlSpace>::HumanReadable(m_control_space)
                               + '/'
                               + cmnData<mtsIntuitiveResearchKitArmTypes::ControlMode>::HumanReadable(m_control_mode));
}

void mtsIntuitiveResearchKitArm::SetControlEffortActiveJoints(void)
{
    PID.EnableTorqueMode(vctBoolVec(NumberOfJoints(), true));
}

void mtsIntuitiveResearchKitArm::ControlEffortCartesianPreload(vctDoubleVec & effortPreload,
                                                               vctDoubleVec & wrenchPreload)
{
    effortPreload.Zeros();
    wrenchPreload.Zeros();
}

void mtsIntuitiveResearchKitArm::ControlEffortJoint(void)
{
    // effort required
    mEffortJoint.Assign(mEffortJointSet.ForceTorque());

    // add gravity compensation if needed
    if (m_gravity_compensation) {
        AddGravityCompensationEfforts(mEffortJoint);
    }

    // add custom efforts
    AddCustomEfforts(mEffortJoint);

    // convert to cisstParameterTypes
    SetEffortJointLocal(mEffortJoint);
}

void mtsIntuitiveResearchKitArm::ControlEffortCartesian(void)
{
    // update torques based on wrench
    vctDoubleVec wrench(6);

    // get force preload from derived classes, in most cases 0, platform control for MTM
    vctDoubleVec effortPreload(NumberOfJointsKinematics());
    vctDoubleVec wrenchPreload(6);

    ControlEffortCartesianPreload(effortPreload, wrenchPreload);

    // body wrench
    if (mWrenchType == WRENCH_BODY) {
        // either using wrench provided by user or cartesian impedance
        if (m_cartesianImpedance) {
            mCartesianImpedanceController->Update(m_measured_cp,
                                                  m_measured_cv,
                                                  mWrenchSet,
                                                  mWrenchBodyOrientationAbsolute);
            wrench.Assign(mWrenchSet.Force());
        } else {
            // user provided wrench
            if (mWrenchBodyOrientationAbsolute) {
                // use forward kinematics orientation to have constant wrench orientation
                vct3 relative, absolute;
                // force
                relative.Assign(mWrenchSet.Force().Ref<3>(0));
                m_measured_cp_frame.Rotation().ApplyInverseTo(relative, absolute);
                wrench.Ref(3, 0).Assign(absolute);
                // torque
                relative.Assign(mWrenchSet.Force().Ref<3>(3));
                m_measured_cp_frame.Rotation().ApplyInverseTo(relative, absolute);
                wrench.Ref(3, 3).Assign(absolute);
            } else {
                wrench.Assign(mWrenchSet.Force());
            }
        }
        mEffortJoint.ProductOf(m_jacobian_body.Transpose(), wrench + wrenchPreload);
        mEffortJoint.Add(effortPreload);
    }
    // spatial wrench
    else if (mWrenchType == WRENCH_SPATIAL) {
        wrench.Assign(mWrenchSet.Force());
        mEffortJoint.ProductOf(m_jacobian_spatial.Transpose(), wrench + wrenchPreload);
        mEffortJoint.Add(effortPreload);
    }

    // add gravity compensation if needed
    if (m_gravity_compensation) {
        AddGravityCompensationEfforts(mEffortJoint);
    }

    // add custom efforts
    AddCustomEfforts(mEffortJoint);

    // send to PID
    SetEffortJointLocal(mEffortJoint);

    // lock orientation if needed
    if (m_effort_orientation_locked) {
        ControlEffortOrientationLocked();
    }
}

void mtsIntuitiveResearchKitArm::ControlEffortOrientationLocked(void)
{
    CMN_LOG_CLASS_RUN_ERROR << GetName()
                            << ": ControlEffortOrientationLocked, this method should never be called, MTMs are the only arms able to lock orientation and the derived implementation of this method should be called"
                            << std::endl;
}

void mtsIntuitiveResearchKitArm::SetEffortJointLocal(const vctDoubleVec & newEffort)
{
    // convert to cisstParameterTypes
    mTorqueSetParam.SetForceTorque(newEffort);
    mTorqueSetParam.SetTimestamp(StateTable.GetTic());
    PID.servo_jf(mTorqueSetParam);
}

void mtsIntuitiveResearchKitArm::SetPositionJointLocal(const vctDoubleVec & newPosition)
{
    // feed forward
    if (UseFeedForward()) {
        UpdateFeedForward(FeedForwardParam.ForceTorque());
        PID.SetFeedForwardJoint(FeedForwardParam);
    }
    // position
    JointSetParam.Goal().Zeros();
    JointSetParam.Goal().Assign(newPosition, NumberOfJoints());
    JointSetParam.SetTimestamp(StateTable.GetTic());
    PID.servo_jp(JointSetParam);
}

void mtsIntuitiveResearchKitArm::Freeze(void)
{
    if (!ArmIsReady("Freeze", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitArmTypes::POSITION_MODE);
    // set goal
    JointSet.Assign(m_setpoint_js_kin.Position(), NumberOfJointsKinematics());
    m_new_pid_goal = true;
}

void mtsIntuitiveResearchKitArm::servo_jp(const prmPositionJointSet & newPosition)
{
    if (!ArmIsReady("servo_jp", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitArmTypes::POSITION_MODE);
    // set goal
    JointSet.Assign(newPosition.Goal(), NumberOfJointsKinematics());
    m_new_pid_goal = true;
}

void mtsIntuitiveResearchKitArm::servo_jr(const prmPositionJointSet & difference)
{
    if (!ArmIsReady("servo_cp", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitArmTypes::POSITION_INCREMENT_MODE);
    // set goal
    JointSet.Add(difference.Goal());
    m_new_pid_goal = true;
}

void mtsIntuitiveResearchKitArm::move_jp(const prmPositionJointSet & newPosition)
{
    if (!ArmIsReady("move_jp", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);
    // make sure trajectory is reset
    UpdateIsBusy(true);
    mJointTrajectory.IsActive = true;
    mJointTrajectory.EndTime = 0.0;
    // new goal
    ToJointsPID(newPosition.Goal(), mJointTrajectory.Goal);
    mJointTrajectory.GoalVelocity.SetAll(0.0);
}

void mtsIntuitiveResearchKitArm::servo_cp(const prmPositionCartesianSet & newPosition)
{
    if (!ArmIsReady("servo_cp", mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitArmTypes::POSITION_MODE);
    // set goal
    CartesianSetParam = newPosition;
    m_new_pid_goal = true;
}

void mtsIntuitiveResearchKitArm::servo_cr(const prmPositionCartesianSet & difference)
{
    if (!ArmIsReady("servo_cp", mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitArmTypes::POSITION_INCREMENT_MODE);
    // set goal
    mCartesianRelative = mCartesianRelative * difference.Goal();
    m_new_pid_goal = true;
}

void mtsIntuitiveResearchKitArm::move_cp(const prmPositionCartesianSet & newPosition)
{
    if (!ArmIsReady("move_cp", mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitArmTypes::TRAJECTORY_MODE);

    // copy current position
    vctDoubleVec jointSet(m_measured_js_kin.Position());

    // compute desired slave position
    CartesianPositionFrm.From(newPosition.Goal());

    if (this->InverseKinematics(jointSet, m_base_frame.Inverse() * CartesianPositionFrm) == robManipulator::ESUCCESS) {
        // make sure trajectory is reset
        UpdateIsBusy(true);
        mJointTrajectory.IsActive = true;
        mJointTrajectory.EndTime = 0.0;
        // new goal
        ToJointsPID(jointSet, mJointTrajectory.Goal);
        mJointTrajectory.GoalVelocity.SetAll(0.0);
    } else {
        // shows robManipulator error if used
        if (this->Manipulator) {
            RobotInterface->SendError(this->GetName()
                                      + ": unable to solve inverse kinematics ("
                                      + this->Manipulator->LastError() + ")");
        } else {
            RobotInterface->SendError(this->GetName() + ": unable to solve inverse kinematics");
        }
        mJointTrajectory.GoalReachedEvent(false);
    }
}

void mtsIntuitiveResearchKitArm::SetBaseFrame(const prmPositionCartesianSet & newBaseFrame)
{
    if (newBaseFrame.Valid()) {
        this->m_base_frame.FromNormalized(newBaseFrame.Goal());
        this->BaseFrameValid = true;
        this->m_measured_cp.SetReferenceFrame(newBaseFrame.ReferenceFrame());
        this->m_setpoint_cp.SetReferenceFrame(newBaseFrame.ReferenceFrame());
    } else {
        this->BaseFrameValid = false;
    }
}

void mtsIntuitiveResearchKitArm::ErrorEventHandler(const mtsMessage & message)
{
    RobotInterface->SendError(this->GetName() + ": received [" + message.Message + "]");
    std::cerr << CMN_LOG_DETAILS << " should something be done here?  YEP!   If PID tracking error do something different" << std::endl;
    SetDesiredState("FAULT");
}

void mtsIntuitiveResearchKitArm::PositionLimitEventHandler(const vctBoolVec & CMN_UNUSED(flags))
{
    RobotInterface->SendWarning(this->GetName() + ": PID position limit");
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
        RobotInterface->SendStatus(this->GetName() + ": encoders biased using " + nbSamplesString.str() + " potentiometer values");
    } else {
        // we assume that all encoders were properly biased.  This is
        // mostly for MTM homing to skip roll homing.  This could fail
        // if the homing process was interrupted between bias from
        // pots and MTM search for roll limits.
        m_encoders_biased = true;
        RobotInterface->SendStatus(this->GetName() + ": encoders seem to be already biased");
    }

    if (mHomingBiasEncoderRequested) {
        mHomingBiasEncoderRequested = false;
        mArmState.SetCurrentState("ENCODERS_BIASED");
    } else {
        RobotInterface->SendWarning(this->GetName() + ": encoders have been biased by another process");
    }
}

void mtsIntuitiveResearchKitArm::servo_jf(const prmForceTorqueJointSet & effort)
{
    if (!ArmIsReady("servo_jf", mtsIntuitiveResearchKitArmTypes::JOINT_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::JOINT_SPACE,
                           mtsIntuitiveResearchKitArmTypes::EFFORT_MODE);

    // set new effort
    mEffortJointSet.ForceTorque().Assign(effort.ForceTorque());
}

void mtsIntuitiveResearchKitArm::servo_cf_body(const prmForceCartesianSet & wrench)
{
    if (!ArmIsReady("servo_cf_body", mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitArmTypes::EFFORT_MODE);

    // set new wrench
    m_cartesianImpedance = false;
    mWrenchSet = wrench;
    if (mWrenchType != WRENCH_BODY) {
        mWrenchType = WRENCH_BODY;
        RobotInterface->SendStatus(this->GetName() + ": effort cartesian WRENCH_BODY");
    }
}

void mtsIntuitiveResearchKitArm::servo_cf_spatial(const prmForceCartesianSet & wrench)
{
    if (!ArmIsReady("servo_cf_spatial", mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitArmTypes::EFFORT_MODE);

    // set new wrench
    m_cartesianImpedance = false;
    mWrenchSet = wrench;
    if (mWrenchType != WRENCH_SPATIAL) {
        mWrenchType = WRENCH_SPATIAL;
        RobotInterface->SendStatus(this->GetName() + ": effort cartesian WRENCH_SPATIAL");
    }
}

void mtsIntuitiveResearchKitArm::SetWrenchBodyOrientationAbsolute(const bool & absolute)
{
    mWrenchBodyOrientationAbsolute = absolute;
}

void mtsIntuitiveResearchKitArm::SetGravityCompensation(const bool & gravityCompensation)
{
    m_gravity_compensation = gravityCompensation;
}

void mtsIntuitiveResearchKitArm::AddGravityCompensationEfforts(vctDoubleVec & efforts)
{
    vctDoubleVec qd(this->NumberOfJointsKinematics(), 0.0);
    vctDoubleVec gravityEfforts;
    gravityEfforts.ForceAssign(Manipulator->CCG(m_measured_js_kin.Position(), qd));  // should this take joint velocities?
    efforts.Add(gravityEfforts);
}

void mtsIntuitiveResearchKitArm::SetCartesianImpedanceGains(const prmCartesianImpedanceGains & gains)
{
    if (!ArmIsReady("servo_cf_body", mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE)) {
        return;
    }

    // set control mode
    SetControlSpaceAndMode(mtsIntuitiveResearchKitArmTypes::CARTESIAN_SPACE,
                           mtsIntuitiveResearchKitArmTypes::EFFORT_MODE);

    // set new wrench
    m_cartesianImpedance = true;
    mWrenchBodyOrientationAbsolute = true;
    mCartesianImpedanceController->SetGains(gains);
    if (mWrenchType != WRENCH_BODY) {
        mWrenchType = WRENCH_BODY;
        RobotInterface->SendStatus(this->GetName() + ": effort cartesian WRENCH_BODY");
    }
}
