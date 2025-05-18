/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Nicole Ortega
  Created on: 2016-01-21

  (C) Copyright 2016-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// cisst
#include <sawIntuitiveResearchKit/mtsTeleOperationECM.h>

#include <cisstCommon/cmnUnits.h>
#include <cisstVector/vctBoundingBox3.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperationECM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsTeleOperationECM::mtsTeleOperationECM(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds),
    mTeleopState(componentName, "DISABLED")
{
    Init();
}

mtsTeleOperationECM::mtsTeleOperationECM(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg),
    mTeleopState(arg.Name, "DISABLED")
{
    Init();
}

mtsTeleOperationECM::~mtsTeleOperationECM()
{
}

void mtsTeleOperationECM::Init(void)
{
    // configure state machine
    mTeleopState.AddState("SETTING_ARMS_STATE");
    mTeleopState.AddState("ENABLED");
    mTeleopState.AddAllowedDesiredState("DISABLED");
    mTeleopState.AddAllowedDesiredState("ENABLED");

    // state change, to convert to string events for users (Qt, ROS)
    mTeleopState.SetStateChangedCallback(&mtsTeleOperationECM::StateChanged,
                                         this);

    // run for all states
    mTeleopState.SetRunCallback(&mtsTeleOperationECM::RunAllStates,
                                this);
    // disabled
    mTeleopState.SetTransitionCallback("DISABLED",
                                       &mtsTeleOperationECM::TransitionDisabled,
                                       this);

    // setting arms state
    mTeleopState.SetEnterCallback("SETTING_ARMS_STATE",
                                  &mtsTeleOperationECM::EnterSettingArmsState,
                                  this);
    mTeleopState.SetTransitionCallback("SETTING_ARMS_STATE",
                                       &mtsTeleOperationECM::TransitionSettingArmsState,
                                       this);

    // enabled
    mTeleopState.SetEnterCallback("ENABLED",
                                  &mtsTeleOperationECM::EnterEnabled,
                                  this);
    mTeleopState.SetRunCallback("ENABLED",
                                &mtsTeleOperationECM::RunEnabled,
                                this);
    mTeleopState.SetTransitionCallback("ENABLED",
                                       &mtsTeleOperationECM::TransitionEnabled,
                                       this);

    m_scale = 0.2;
    m_clutched = false;

    StateTable.AddData(mMTML.m_measured_cp, "MTML/measured_cp");
    StateTable.AddData(mMTMR.m_measured_cp, "MTMR/measured_cp");
    StateTable.AddData(mECM.m_measured_cp, "ECM/measured_cp");

    mConfigurationStateTable = new mtsStateTable(100, "Configuration");
    mConfigurationStateTable->SetAutomaticAdvance(false);
    AddStateTable(mConfigurationStateTable);
    mConfigurationStateTable->AddData(m_scale, "scale");

    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("MTML");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("measured_cp",
                                       mMTML.measured_cp);
        interfaceRequired->AddFunction("measured_cv",
                                       mMTML.measured_cv);
        interfaceRequired->AddFunction("lock_orientation",
                                       mMTML.lock_orientation);
        interfaceRequired->AddFunction("body/servo_cf",
                                       mMTML.body_servo_cf);
        interfaceRequired->AddFunction("body/set_cf_orientation_absolute",
                                       mMTML.body_set_cf_orientation_absolute);
        interfaceRequired->AddFunction("use_gravity_compensation",
                                       mMTML.use_gravity_compensation);
        interfaceRequired->AddFunction("operating_state",
                                       mMTML.operating_state);
        interfaceRequired->AddFunction("state_command",
                                       mMTML.state_command);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::MTMLErrorEventHandler,
                                                this, "error");
    }

    interfaceRequired = AddInterfaceRequired("MTMR");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("measured_cp",
                                       mMTMR.measured_cp);
        interfaceRequired->AddFunction("measured_cv",
                                       mMTMR.measured_cv);
        interfaceRequired->AddFunction("lock_orientation",
                                       mMTMR.lock_orientation);
        interfaceRequired->AddFunction("body/servo_cf",
                                       mMTMR.body_servo_cf);
        interfaceRequired->AddFunction("body/set_cf_orientation_absolute",
                                       mMTMR.body_set_cf_orientation_absolute);
        interfaceRequired->AddFunction("use_gravity_compensation",
                                       mMTMR.use_gravity_compensation);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::MTMRErrorEventHandler,
                                                this, "error");
        interfaceRequired->AddFunction("operating_state",
                                       mMTMR.operating_state);
        interfaceRequired->AddFunction("state_command",
                                       mMTMR.state_command);
    }

    interfaceRequired = AddInterfaceRequired("ECM");
    if (interfaceRequired) {
        // ECM, use PID desired position to make sure there is no jump when engaging
        interfaceRequired->AddFunction("setpoint_cp",
                                       mECM.measured_cp);
        interfaceRequired->AddFunction("setpoint_js",
                                       mECM.setpoint_js);
        interfaceRequired->AddFunction("servo_jp",
                                       mECM.servo_jp);
        interfaceRequired->AddFunction("operating_state",
                                       mECM.operating_state);
        interfaceRequired->AddFunction("state_command",
                                       mECM.state_command);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::ECMErrorEventHandler,
                                                this, "error");
    }

    // footpedal events
    interfaceRequired = AddInterfaceRequired("clutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::ClutchEventHandler, this, "Button");
    }

    mInterface = AddInterfaceProvided("Setting");
    if (mInterface) {
        mInterface->AddMessageEvents();
        // commands
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                        "period_statistics"); // mtsIntervalStatistics

        mInterface->AddCommandWrite(&mtsTeleOperationECM::state_command, this,
                                    "state_command", std::string("DISABLED"));
        mInterface->AddCommandWrite(&mtsTeleOperationECM::set_scale, this,
                                    "set_scale", 0.5);
        mInterface->AddCommandReadState(*mConfigurationStateTable,
                                        m_scale,
                                        "scale");
        mInterface->AddCommandReadState(StateTable,
                                        mMTML.m_measured_cp,
                                        "MTML/measured_cp");
        mInterface->AddCommandReadState(StateTable,
                                        mMTMR.m_measured_cp,
                                        "MTMR/measured_cp");
        mInterface->AddCommandReadState(StateTable,
                                        mECM.m_measured_cp,
                                        "ECM/measured_cp");
        // events
        mInterface->AddEventWrite(MessageEvents.desired_state,
                                  "desired_state", std::string(""));
        mInterface->AddEventWrite(MessageEvents.current_state,
                                  "current_state", std::string(""));
        mInterface->AddEventWrite(MessageEvents.following,
                                  "following", false);
        // configuration
        mInterface->AddEventWrite(ConfigurationEvents.scale,
                                  "scale", 0.5);
    }
}

void mtsTeleOperationECM::Configure(const std::string & filename)
{
    std::ifstream jsonStream;
    Json::Value jsonConfig;
    Json::Reader jsonReader;

    if (filename == "") {
        return;
    }

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

    // JSON part
    mtsTeleOperationECM::Configure(jsonConfig);
}

void mtsTeleOperationECM::Configure(const Json::Value & jsonConfig)
{
    Json::Value jsonValue;

    // base component configuration
    mtsComponent::ConfigureJSON(jsonConfig);

    // read scale if present
    jsonValue = jsonConfig["scale"];
    if (!jsonValue.empty()) {
        m_scale = jsonValue.asDouble();
    }
}

void mtsTeleOperationECM::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
    set_scale(m_scale);
    set_following(false);
}

void mtsTeleOperationECM::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    // run based on state
    mTeleopState.Run();
}

void mtsTeleOperationECM::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsTeleOperationECM::StateChanged(void)
{
    const std::string newState = mTeleopState.CurrentState();
    MessageEvents.current_state(newState);
    mInterface->SendStatus(this->GetName() + ": current state is " + newState);
}

void mtsTeleOperationECM::RunAllStates(void)
{
    mtsExecutionResult executionResult;

    // get MTML Cartesian position/velocity
    executionResult = mMTML.measured_cp(mMTML.m_measured_cp);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTML.measured_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from MTML");
        mTeleopState.SetDesiredState("DISABLED");
    }
    executionResult = mMTML.measured_cv(mMTML.m_measured_cv);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTML.measured_cv failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian velocity from MTML");
        mTeleopState.SetDesiredState("DISABLED");
    }

    // get MTMR Cartesian position
    executionResult = mMTMR.measured_cp(mMTMR.m_measured_cp);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTMR.measured_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from MTMR");
        mTeleopState.SetDesiredState("DISABLED");
    }
    executionResult = mMTMR.measured_cv(mMTMR.m_measured_cv);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTMR.measured_cv failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian velocity from MTMR");
        mTeleopState.SetDesiredState("DISABLED");
    }

    // get ECM Cartesian position for GUI
    executionResult = mECM.measured_cp(mECM.m_measured_cp);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to ECM.measured_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from ECM");
        mTeleopState.SetDesiredState("DISABLED");
    }
    // for motion computation
    executionResult = mECM.setpoint_js(mECM.m_setpoint_js);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to ECM.setpoint_js failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get joint state from ECM");
        mTeleopState.SetDesiredState("DISABLED");
    }

    // check if anyone wanted to disable anyway
    if ((mTeleopState.DesiredState() == "DISABLED")
        && (mTeleopState.CurrentState() != "DISABLED")) {
        set_following(false);
        mTeleopState.SetCurrentState("DISABLED");
        return;
    }

    // monitor state of arms if needed
    if ((mTeleopState.CurrentState() != "DISABLED")
        && (mTeleopState.CurrentState() != "SETTING_ARMS_STATE")) {
        prmOperatingState state;
        mECM.operating_state(state);
        if ((state.State() != prmOperatingState::ENABLED)
            || !state.IsHomed()) {
            mTeleopState.SetDesiredState("DISABLED");
            mInterface->SendError(this->GetName() + ": ECM is not in state \"READY\" anymore");
        }
        mMTML.operating_state(state);
        if ((state.State() != prmOperatingState::ENABLED)
            || !state.IsHomed()) {
            mTeleopState.SetDesiredState("DISABLED");
            mInterface->SendError(this->GetName() + ": MTML is not in state \"READY\" anymore");
        }
        mMTMR.operating_state(state);
        if ((state.State() != prmOperatingState::ENABLED)
            || !state.IsHomed()) {
            mTeleopState.SetDesiredState("DISABLED");
            mInterface->SendError(this->GetName() + ": MTMR is not in state \"READY\" anymore");
        }
    }
}

void mtsTeleOperationECM::TransitionDisabled(void)
{
    if (mTeleopState.DesiredState() == "ENABLED") {
        mTeleopState.SetCurrentState("SETTING_ARMS_STATE");
    }
}

void mtsTeleOperationECM::EnterSettingArmsState(void)
{
    // reset timer
    mInStateTimer = StateTable.GetTic();

    // request state if needed
    prmOperatingState state;
    mECM.operating_state(state);
    if (state.State() != prmOperatingState::ENABLED) {
        mECM.state_command(std::string("enable"));
    }
    if (!state.IsHomed()) {
        mECM.state_command(std::string("home"));
    }

    mMTML.operating_state(state);
    if (state.State() != prmOperatingState::ENABLED) {
        mMTML.state_command(std::string("enable"));
    }
    if (!state.IsHomed()) {
        mMTML.state_command(std::string("home"));
    }

    mMTMR.operating_state(state);
    if (state.State() != prmOperatingState::ENABLED) {
        mMTMR.state_command(std::string("enable"));
    }
    if (!state.IsHomed()) {
        mMTMR.state_command(std::string("home"));
    }
}

void mtsTeleOperationECM::TransitionSettingArmsState(void)
{
    // check state
    prmOperatingState ecmState, mtmlState, mtmrState;
    mECM.operating_state(ecmState);
    mMTML.operating_state(mtmlState);
    mMTMR.operating_state(mtmrState);
    if ((ecmState.State() == prmOperatingState::ENABLED) && ecmState.IsHomed()
        && (mtmlState.State() == prmOperatingState::ENABLED) && mtmlState.IsHomed()
        && (mtmrState.State() == prmOperatingState::ENABLED) && mtmrState.IsHomed()) {
        // make sure the coordinate systems make sense, i.e. did the
        // user set a base-frame that matches ISI coordinate system
        // for the console side
        vctBoundingBox3 workArea(vct3(-400.0 * cmn_mm,
                                      -200.0 * cmn_mm,
                                        10.0 * cmn_mm),
                                 vct3( 400.0 * cmn_mm,
                                       200.0 * cmn_mm,
                                       800.0 * cmn_mm));
        if (!workArea.Includes(mMTML.m_measured_cp.Position().Translation())) {
            mInterface->SendError(this->GetName() + ": MTML position doesn't seem to be in the work area.  Make sure your \"base-frame\" is set correctly in your console JSON configuration file.");
            mTeleopState.SetDesiredState("DISABLED");
            return;
        }
        if (!workArea.Includes(mMTMR.m_measured_cp.Position().Translation())) {
            mInterface->SendError(this->GetName() + ": MTMR position doesn't seem to be in the work area.  Make sure your \"base-frame\" is set correctly in your console JSON configuration file.");
            mTeleopState.SetDesiredState("DISABLED");
            return;
        }
        // we should be good to go
        mTeleopState.SetCurrentState("ENABLED");
        return;
    }
    // check timer
    if ((StateTable.GetTic() - mInStateTimer) > 60.0 * cmn_s) {
        mInterface->SendError(this->GetName() + ": timed out while setting up arms state");
        mTeleopState.SetDesiredState("DISABLED");
    }
}

void mtsTeleOperationECM::EnterEnabled(void)
{
    // set cartesian effort parameters
    mMTML.use_gravity_compensation(true);
    mMTML.body_set_cf_orientation_absolute(true);
    mMTML.lock_orientation(mMTML.m_measured_cp.Position().Rotation());
    mMTMR.use_gravity_compensation(true);
    mMTMR.body_set_cf_orientation_absolute(true);
    mMTMR.lock_orientation(mMTMR.m_measured_cp.Position().Rotation());

    // initial state for MTM force feedback
    // -1- initial distance between MTMs
    vct3 vectorLR;
    vectorLR.DifferenceOf(mMTMR.m_measured_cp.Position().Translation(),
                          mMTML.m_measured_cp.Position().Translation());
    // -2- mid-point, aka center of image
    mInitial.C.SumOf(mMTMR.m_measured_cp.Position().Translation(),
                     mMTML.m_measured_cp.Position().Translation());
    mInitial.C.Multiply(0.5);
    // -3- image up vector
    mInitial.Up.CrossProductOf(vectorLR, mInitial.C);
    mInitial.Up.NormalizedSelf();
    // -4- width of image, depth of arms wrt image plan
    vct3 side;
    side.CrossProductOf(mInitial.C, mInitial.Up);
    side.NormalizedSelf();
    mInitial.w = 0.5 * vctDotProduct(side, vectorLR);
    mInitial.d = 0.5 * vctDotProduct(mInitial.C.Normalized(), vectorLR);

    // projections
    mInitial.Lr.Assign(mInitial.C[0], 0, mInitial.C[2]);
    mInitial.Lr.NormalizedSelf();
    mInitial.Ud.Assign(0, mInitial.C[1], mInitial.C[2]);
    mInitial.Ud.NormalizedSelf();
    mInitial.Cw.Assign(mInitial.Up[0], mInitial.Up[1], 0);
    mInitial.Cw.NormalizedSelf();

    mInitial.ECMPositionJoint = mECM.m_setpoint_js.Position();

    // -5- store current rotation matrix for MTML, MTMR, and ECM
    vctEulerZYXRotation3 eulerAngles;
    eulerAngles.Assign(mInitial.ECMPositionJoint[3], mInitial.ECMPositionJoint[0], mInitial.ECMPositionJoint[1]);
    vctEulerToMatrixRotation3(eulerAngles, mInitial.ECMRotEuler);

    mInitial.MTMLRot = mMTML.m_measured_cp.Position().Rotation();
    mInitial.MTMRRot = mMTMR.m_measured_cp.Position().Rotation();

#if 0
    std::cerr << CMN_LOG_DETAILS << std::endl
              << "L: " << mMTML.m_measured_cp.Position().Translation() << std::endl
              << "R: " << mMTMR.m_measured_cp.Position().Translation() << std::endl
              << "C:  " << mInitial.C << std::endl
              << "Up: " << mInitial.Up << std::endl
              << "w:  " << mInitial.w << std::endl
              << "d:  " << mInitial.d << std::endl
              << "Si: " << side << std::endl;
#endif
    mInitial.ECMPositionJoint = mECM.m_setpoint_js.Position();
    // check if by any chance the clutch pedal is pressed
    if (m_clutched) {
        Clutch(true);
    } else {
        set_following(true);
    }
}

void mtsTeleOperationECM::RunEnabled(void)
{
    if (m_clutched) {
        return;
    }

    /* --- Forces on MTMs --- */
    const vct3 frictionForceCoeff(-10.0, -10.0, -40.0);
    const double distanceForceCoeff = 150.0;

    //-1- vector between MTMs
    vct3 vectorLR;
    vectorLR.DifferenceOf(mMTMR.m_measured_cp.Position().Translation(),
                          mMTML.m_measured_cp.Position().Translation());
    // -2- mid-point, aka center of image
    vct3 c;
    c.SumOf(mMTMR.m_measured_cp.Position().Translation(),
            mMTML.m_measured_cp.Position().Translation());
    c.Multiply(0.5);
    vct3 directionC = c.Normalized();
    // -3- image up vector
    vct3 up;
    up.CrossProductOf(vectorLR, c);
    up.NormalizedSelf();
    // -4- Width of image
    vct3 side;
    side.CrossProductOf(c, up);
    side.NormalizedSelf();
    // -5- find desired position for L and R
    vct3 goalL(c);
    goalL.AddProductOf(-mInitial.w, side);
    goalL.AddProductOf(-mInitial.d, directionC);
    vct3 goalR(c);
    goalR.AddProductOf(mInitial.w, side);
    goalR.AddProductOf(mInitial.d, directionC);


    // compute forces on L and R based on error in position
    vct3 forceFriction;
    vct3 force;
    prmForceCartesianSet wrenchR, wrenchL;

    // MTMR
    // apply force
    force.DifferenceOf(goalR,
                       mMTMR.m_measured_cp.Position().Translation());
    force.Multiply(distanceForceCoeff);
    wrenchR.Force().Ref<3>(0).Assign(force);
    // add friction force
    forceFriction.ElementwiseProductOf(frictionForceCoeff,
                                       mMTMR.m_measured_cv.VelocityLinear());
    wrenchR.Force().Ref<3>(0).Add(forceFriction);
    // apply
    mMTMR.body_servo_cf(wrenchR);

    // MTML
    // apply force
    force.DifferenceOf(goalL,
                       mMTML.m_measured_cp.Position().Translation());
    force.Multiply(distanceForceCoeff);
    wrenchL.Force().Ref<3>(0).Assign(force);
    // add friction force
    forceFriction.ElementwiseProductOf(frictionForceCoeff,
                                       mMTML.m_measured_cv.VelocityLinear());
    wrenchL.Force().Ref<3>(0).Add(forceFriction);
    // apply
    mMTML.body_servo_cf(wrenchL);

    /* --- Joint Control --- */
    static const vct3 normXZ(0.0, 1.0, 0.0);
    static const vct3 normYZ(1.0, 0.0, 0.0);
    static const vct3 normXY(0.0, 0.0, 1.0);
    // Initial ECM joints
    vctVec goalJoints(mInitial.ECMPositionJoint);
    // Change in directions and joints
    vctVec changeJoints(4);
    vctVec changeDir(4);
    vct3 crossN;  // normal to direction of motion

    // - Direction 0 - left/right, movement in the XZ plane
    vct3  lr(c[0], 0.0, c[2]);
    lr.NormalizedSelf();
    if (mInitial.Lr.AlmostEqual(lr)) {
        changeDir[0] = 0.0;
    } else {
        changeDir[0] = -acos(vctDotProduct(mInitial.Lr, lr));
        crossN = vctCrossProduct(mInitial.Lr, lr);
        if (vctDotProduct(normXZ, crossN) < 0.0) {
            changeDir[0] = -changeDir[0];
        }
    }

    // - Direction 1 - up/down, movement in the YZ plane
    vct3  ud(0.0, c[1], c[2]);
    ud.NormalizedSelf();
    if (mInitial.Ud.AlmostEqual(ud)) {
        changeDir[1] = 0.0;
    } else {
        changeDir[1] = acos(vctDotProduct(mInitial.Ud, ud));
        crossN = vctCrossProduct(mInitial.Ud, ud);
        if (vctDotProduct(normYZ, crossN) < 0.0) {
            changeDir[1] = -changeDir[1];
        }
    }

    // - Direction 2 - in/out
    changeDir[2] = m_scale * (mInitial.C.Norm() - c.Norm());

    // - Direction 3 - cc/ccw, movement in the XY plane
    vct3 cw(up[0], up[1], 0);
    cw.NormalizedSelf();
    if (mInitial.Cw.AlmostEqual(cw)) {
        changeDir[3] = 0.0;
    } else {
        changeDir[3] = -acos(vctDotProduct(mInitial.Cw, cw));
        crossN = vctCrossProduct(mInitial.Cw, cw);
        if (vctDotProduct(normXY, crossN) < 0) {
            changeDir[3] = -changeDir[3];
        }
    }

    // adjusting movement for camera orientation
    double totalChangeJoint3 = changeDir[3] + mInitial.ECMPositionJoint[3];
    changeJoints[0] = changeDir[0] * cos(totalChangeJoint3) - changeDir[1] * sin(totalChangeJoint3);
    changeJoints[1] = changeDir[1] * cos(totalChangeJoint3) + changeDir[0] * sin(totalChangeJoint3);
    changeJoints[2] = changeDir[2];
    changeJoints[3] = changeDir[3];

    goalJoints.Add(changeJoints);
    mECM.m_servo_jp.Goal().ForceAssign(goalJoints);
    mECM.servo_jp(mECM.m_servo_jp);

    /* --- Lock Orientation --- */

    //Calculate new rotations of MTMs
    vctMatRot3 currMTMLRot;
    vctMatRot3 currMTMRRot;
    // Current ECM Rotation
    vctEulerZYXRotation3 finalEulerAngles;
    vctMatrixRotation3<double> currECMRot;
    vctMatrixRotation3<double> finalECMRot;

    finalEulerAngles.Assign(goalJoints[3], goalJoints[0], goalJoints[1]);
    vctEulerToMatrixRotation3(finalEulerAngles, finalECMRot);
    currECMRot = finalECMRot * mInitial.ECMRotEuler.Inverse();

    // Set MTM Orientation
    currMTMLRot = currECMRot.Inverse() * mInitial.MTMLRot;
    currMTMRRot = currECMRot.Inverse() * mInitial.MTMRRot;

    // set cartesian effort parameters
    mMTML.body_set_cf_orientation_absolute(true);
    mMTML.lock_orientation(currMTMLRot);
    mMTMR.body_set_cf_orientation_absolute(true);
    mMTMR.lock_orientation(currMTMRRot);
}

void mtsTeleOperationECM::TransitionEnabled(void)
{
    if (mTeleopState.DesiredStateIsNotCurrent()) {
        set_following(false);
        mTeleopState.SetCurrentState(mTeleopState.DesiredState());
    }
}

void mtsTeleOperationECM::set_following(const bool following)
{
    MessageEvents.following(following);
    m_following = following;
}

void mtsTeleOperationECM::MTMLErrorEventHandler(const mtsMessage & message)
{
    mTeleopState.SetDesiredState("DISABLED");
    mInterface->SendError(this->GetName() + ": received from MTML [" + message.Message + "]");
}

void mtsTeleOperationECM::MTMRErrorEventHandler(const mtsMessage & message)
{
    mTeleopState.SetDesiredState("DISABLED");
    mInterface->SendError(this->GetName() + ": received from MTMR [" + message.Message + "]");
}

void mtsTeleOperationECM::ECMErrorEventHandler(const mtsMessage & message)
{
    mTeleopState.SetDesiredState("DISABLED");
    mInterface->SendError(this->GetName() + ": received from ECM [" + message.Message + "]");
}

void mtsTeleOperationECM::ClutchEventHandler(const prmEventButton & button)
{
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        m_clutched = true;
        break;
    case prmEventButton::RELEASED:
        m_clutched = false;
        break;
    default:
        break;
    }

    // if the teleoperation is activated
    if (mTeleopState.DesiredState() == "ENABLED") {
        Clutch(m_clutched);
    }
}

void mtsTeleOperationECM::Clutch(const bool & clutch)
{
    // if the teleoperation is activated
    if (clutch) {
        set_following(false);
        mInterface->SendStatus(this->GetName() + ": console clutch pressed");

        // set MTMs in effort mode, no force applied but gravity and locked orientation
        prmForceCartesianSet wrench;
        mMTML.body_servo_cf(wrench);
        mMTML.use_gravity_compensation(true);
        mMTML.lock_orientation(mMTML.m_measured_cp.Position().Rotation());
        mMTMR.body_servo_cf(wrench);
        mMTMR.use_gravity_compensation(true);
        mMTMR.lock_orientation(mMTMR.m_measured_cp.Position().Rotation());
    } else {
        m_clutched = false;
        mInterface->SendStatus(this->GetName() + ": console clutch released");
        mTeleopState.SetCurrentState("SETTING_ARMS_STATE");
    }
}

void mtsTeleOperationECM::state_command(const std::string & command)
{
    if (command == "enable") {
        SetDesiredState("ENABLED");
        return;
    }
    if (command == "disable") {
        SetDesiredState("DISABLED");
        return;
    }
    mInterface->SendWarning(this->GetName() + ": " + command + " doesn't seem to be a valid state_command");
}

void mtsTeleOperationECM::SetDesiredState(const std::string & state)
{
    // try to find the state in state machine
    if (!mTeleopState.StateExists(state)) {
        mInterface->SendError(this->GetName() + ": unsupported state " + state);
        return;
    }
    // return is already the desired state
    if (mTeleopState.DesiredState() == state) {
        MessageEvents.desired_state(state);
        return;
    }
    // try to set the desired state
    try {
        mTeleopState.SetDesiredState(state);
    } catch (...) {
        mInterface->SendError(this->GetName() + ": " + state + " is not an allowed desired state");
        return;
    }
    // messages and events
    MessageEvents.desired_state(state);
    mInterface->SendStatus(this->GetName() + ": set desired state to " + state);
}

void mtsTeleOperationECM::set_scale(const double & scale)
{
    mConfigurationStateTable->Start();
    m_scale = scale;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.scale(m_scale);
}
