/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Nicole Ortega
  Created on: 2016-01-21

  (C) Copyright 2016-2019 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
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

    mScale = 0.2;
    mIsClutched = false;

    StateTable.AddData(mMTML.PositionCartesianCurrent, "MTMLCartesianPosition");
    StateTable.AddData(mMTMR.PositionCartesianCurrent, "MTMRCartesianPosition");
    StateTable.AddData(mECM.PositionCartesianCurrent, "ECMCartesianPosition");

    mConfigurationStateTable = new mtsStateTable(100, "Configuration");
    mConfigurationStateTable->SetAutomaticAdvance(false);
    AddStateTable(mConfigurationStateTable);
    mConfigurationStateTable->AddData(mScale, "Scale");

    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("MTML");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("measured_cp",
                                       mMTML.measured_cp);
        interfaceRequired->AddFunction("measured_cv",
                                       mMTML.measured_cv);
        interfaceRequired->AddFunction("LockOrientation",
                                       mMTML.LockOrientation);
        interfaceRequired->AddFunction("servo_cf_body",
                                       mMTML.servo_cf_body);
        interfaceRequired->AddFunction("SetWrenchBodyOrientationAbsolute",
                                       mMTML.SetWrenchBodyOrientationAbsolute);
        interfaceRequired->AddFunction("SetGravityCompensation",
                                       mMTML.SetGravityCompensation);
        interfaceRequired->AddFunction("GetCurrentState",
                                       mMTML.GetCurrentState);
        interfaceRequired->AddFunction("GetDesiredState",
                                       mMTML.GetDesiredState);
        interfaceRequired->AddFunction("SetDesiredState",
                                       mMTML.SetDesiredState);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::MTMLErrorEventHandler,
                                                this, "Error");
    }

    interfaceRequired = AddInterfaceRequired("MTMR");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("measured_cp",
                                       mMTMR.measured_cp);
        interfaceRequired->AddFunction("measured_cv",
                                       mMTMR.measured_cv);
        interfaceRequired->AddFunction("LockOrientation",
                                       mMTMR.LockOrientation);
        interfaceRequired->AddFunction("servo_cf_body",
                                       mMTMR.servo_cf_body);
        interfaceRequired->AddFunction("SetWrenchBodyOrientationAbsolute",
                                       mMTMR.SetWrenchBodyOrientationAbsolute);
        interfaceRequired->AddFunction("SetGravityCompensation",
                                       mMTMR.SetGravityCompensation);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::MTMRErrorEventHandler,
                                                this, "Error");
        interfaceRequired->AddFunction("GetCurrentState",
                                       mMTMR.GetCurrentState);
        interfaceRequired->AddFunction("GetDesiredState",
                                       mMTMR.GetDesiredState);
        interfaceRequired->AddFunction("SetDesiredState",
                                       mMTMR.SetDesiredState);
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
        interfaceRequired->AddFunction("GetCurrentState",
                                       mECM.GetCurrentState);
        interfaceRequired->AddFunction("GetDesiredState",
                                       mECM.GetDesiredState);
        interfaceRequired->AddFunction("SetDesiredState",
                                       mECM.SetDesiredState);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::ECMErrorEventHandler,
                                                this, "Error");
    }

    // footpedal events
    interfaceRequired = AddInterfaceRequired("Clutch");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECM::ClutchEventHandler, this, "Button");
    }

    mInterface = AddInterfaceProvided("Setting");
    if (mInterface) {
        mInterface->AddMessageEvents();
        // commands
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                        "period_statistics"); // mtsIntervalStatistics

        mInterface->AddCommandWrite(&mtsTeleOperationECM::SetDesiredState, this,
                                    "SetDesiredState", std::string("DISABLED"));
        mInterface->AddCommandWrite(&mtsTeleOperationECM::SetScale, this,
                                    "SetScale", 0.5);
        mInterface->AddCommandReadState(*mConfigurationStateTable,
                                        mScale,
                                        "GetScale");
        mInterface->AddCommandReadState(StateTable,
                                        mMTML.PositionCartesianCurrent,
                                        "GetPositionCartesianMTML");
        mInterface->AddCommandReadState(StateTable,
                                        mMTMR.PositionCartesianCurrent,
                                        "GetPositionCartesianMTMR");
        mInterface->AddCommandReadState(StateTable,
                                        mECM.PositionCartesianCurrent,
                                        "GetPositionCartesianECM");
        // events
        mInterface->AddEventWrite(MessageEvents.DesiredState,
                                  "DesiredState", std::string(""));
        mInterface->AddEventWrite(MessageEvents.CurrentState,
                                  "CurrentState", std::string(""));
        mInterface->AddEventWrite(MessageEvents.Following,
                                  "Following", false);
        // configuration
        mInterface->AddEventWrite(ConfigurationEvents.Scale,
                                  "Scale", 0.5);
    }
}

void mtsTeleOperationECM::Configure(const std::string & CMN_UNUSED(filename))
{
}

void mtsTeleOperationECM::Configure(const Json::Value & jsonConfig)
{
    Json::Value jsonValue;

    // read scale if present
    jsonValue = jsonConfig["scale"];
    if (!jsonValue.empty()) {
        mScale = jsonValue.asDouble();
    }
}

void mtsTeleOperationECM::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
    SetScale(mScale);
    SetFollowing(false);
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
    MessageEvents.CurrentState(newState);
    mInterface->SendStatus(this->GetName() + ": current state is " + newState);
}

void mtsTeleOperationECM::RunAllStates(void)
{
    mtsExecutionResult executionResult;

    // get MTML Cartesian position/velocity
    executionResult = mMTML.measured_cp(mMTML.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTML.measured_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from MTML");
        this->SetDesiredState("DISABLED");
    }
    executionResult = mMTML.measured_cv(mMTML.VelocityCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTML.measured_cv failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian velocity from MTML");
        this->SetDesiredState("DISABLED");
    }

    // get MTMR Cartesian position
    executionResult = mMTMR.measured_cp(mMTMR.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTMR.measured_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from MTMR");
        this->SetDesiredState("DISABLED");
    }
    executionResult = mMTMR.measured_cv(mMTMR.VelocityCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTMR.measured_cv failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian velocity from MTMR");
        this->SetDesiredState("DISABLED");
    }

    // get ECM Cartesian position for GUI
    executionResult = mECM.measured_cp(mECM.PositionCartesianCurrent);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to ECM.measured_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from ECM");
        this->SetDesiredState("DISABLED");
    }
    // for motion computation
    executionResult = mECM.setpoint_js(mECM.StateJointDesired);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to ECM.setpoint_js failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get joint state from ECM");
        this->SetDesiredState("DISABLED");
    }

    // check if anyone wanted to disable anyway
    if ((mTeleopState.DesiredState() == "DISABLED")
        && (mTeleopState.CurrentState() != "DISABLED")) {
        SetFollowing(false);
        mTeleopState.SetCurrentState("DISABLED");
        return;
    }

    // monitor state of arms if needed
    if ((mTeleopState.CurrentState() != "DISABLED")
        && (mTeleopState.CurrentState() != "SETTING_ARMS_STATE")) {
        std::string armState;
        mECM.GetDesiredState(armState);
        if (armState != "READY") {
            this->SetDesiredState("DISABLED");
            mInterface->SendError(this->GetName() + ": ECM is not in state \"READY\" anymore");
        }
        mMTML.GetDesiredState(armState);
        if (armState != "READY") {
            this->SetDesiredState("DISABLED");
            mInterface->SendError(this->GetName() + ": MTML is not in state \"READY\" anymore");
        }
        mMTMR.GetDesiredState(armState);
        if (armState != "READY") {
            this->SetDesiredState("DISABLED");
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
    std::string armState;
    mECM.GetDesiredState(armState);
    if (armState != "READY") {
        mECM.SetDesiredState(std::string("READY"));
    }
    mMTML.GetDesiredState(armState);
    if (armState != "READY") {
        mMTML.SetDesiredState(std::string("READY"));
    }
    mMTMR.GetDesiredState(armState);
    if (armState != "READY") {
        mMTMR.SetDesiredState(std::string("READY"));
    }
}

void mtsTeleOperationECM::TransitionSettingArmsState(void)
{
    // check if anyone wanted to disable anyway
    if (mTeleopState.DesiredState() == "DISABLED") {
        mTeleopState.SetCurrentState("DISABLED");
        return;
    }
    // check state
    std::string ecmState, leftArmState, rightArmState;
    mECM.GetCurrentState(ecmState);
    mMTML.GetCurrentState(leftArmState);
    mMTMR.GetCurrentState(rightArmState);
    if ((ecmState == "READY") &&
        (leftArmState == "READY") &&
        (rightArmState == "READY")) {
        mTeleopState.SetCurrentState("ENABLED");
        return;
    }
    // check timer
    if ((StateTable.GetTic() - mInStateTimer) > 60.0 * cmn_s) {
        mInterface->SendError(this->GetName() + ": timed out while setting up arms state");
        this->SetDesiredState("DISABLED");
    }
}

void mtsTeleOperationECM::EnterEnabled(void)
{
    // set cartesian effort parameters
    mMTML.SetGravityCompensation(true);
    mMTML.SetWrenchBodyOrientationAbsolute(true);
    mMTML.LockOrientation(mMTML.PositionCartesianCurrent.Position().Rotation());
    mMTMR.SetGravityCompensation(true);
    mMTMR.SetWrenchBodyOrientationAbsolute(true);
    mMTMR.LockOrientation(mMTMR.PositionCartesianCurrent.Position().Rotation());

    // initial state for MTM force feedback
    // -1- initial distance between MTMs
    vct3 vectorLR;
    vectorLR.DifferenceOf(mMTMR.PositionCartesianCurrent.Position().Translation(),
                          mMTML.PositionCartesianCurrent.Position().Translation());
    // -2- mid-point, aka center of image
    mInitial.C.SumOf(mMTMR.PositionCartesianCurrent.Position().Translation(),
                     mMTML.PositionCartesianCurrent.Position().Translation());
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

    mInitial.ECMPositionJoint = mECM.StateJointDesired.Position();

    // -5- store current rotation matrix for MTML, MTMR, and ECM
    vctEulerZYXRotation3 eulerAngles;
    eulerAngles.Assign(mInitial.ECMPositionJoint[3], mInitial.ECMPositionJoint[0], mInitial.ECMPositionJoint[1]);
    vctEulerToMatrixRotation3(eulerAngles, mInitial.ECMRotEuler);

    mInitial.MTMLRot = mMTML.PositionCartesianCurrent.Position().Rotation();
    mInitial.MTMRRot = mMTMR.PositionCartesianCurrent.Position().Rotation();

#if 0
    std::cerr << CMN_LOG_DETAILS << std::endl
              << "L: " << mMTML.PositionCartesianCurrent.Position().Translation() << std::endl
              << "R: " << mMTMR.PositionCartesianCurrent.Position().Translation() << std::endl
              << "C:  " << mInitial.C << std::endl
              << "Up: " << mInitial.Up << std::endl
              << "w:  " << mInitial.w << std::endl
              << "d:  " << mInitial.d << std::endl
              << "Si: " << side << std::endl;
#endif
    mInitial.ECMPositionJoint = mECM.StateJointDesired.Position();
    // check if by any chance the clutch pedal is pressed
    if (mIsClutched) {
        Clutch(true);
    } else {
        SetFollowing(true);
    }
}

void mtsTeleOperationECM::RunEnabled(void)
{
    if (mIsClutched) {
        return;
    }

    /* --- Forces on MTMs --- */
    const vct3 frictionForceCoeff(-10.0, -10.0, -40.0);
    const double distanceForceCoeff = 150.0;

    //-1- vector between MTMs
    vct3 vectorLR;
    vectorLR.DifferenceOf(mMTMR.PositionCartesianCurrent.Position().Translation(),
                          mMTML.PositionCartesianCurrent.Position().Translation());
    // -2- mid-point, aka center of image
    vct3 c;
    c.SumOf(mMTMR.PositionCartesianCurrent.Position().Translation(),
            mMTML.PositionCartesianCurrent.Position().Translation());
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
                       mMTMR.PositionCartesianCurrent.Position().Translation());
    force.Multiply(distanceForceCoeff);
    wrenchR.Force().Ref<3>(0).Assign(force);
    // add friction force
    forceFriction.ElementwiseProductOf(frictionForceCoeff,
                                       mMTMR.VelocityCartesianCurrent.VelocityLinear());
    wrenchR.Force().Ref<3>(0).Add(forceFriction);
    // apply
    mMTMR.servo_cf_body(wrenchR);

    // MTML
    // apply force
    force.DifferenceOf(goalL,
                       mMTML.PositionCartesianCurrent.Position().Translation());
    force.Multiply(distanceForceCoeff);
    wrenchL.Force().Ref<3>(0).Assign(force);
    // add friction force
    forceFriction.ElementwiseProductOf(frictionForceCoeff,
                                       mMTML.VelocityCartesianCurrent.VelocityLinear());
    wrenchL.Force().Ref<3>(0).Add(forceFriction);
    // apply
    mMTML.servo_cf_body(wrenchL);

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
    changeDir[2] = mScale * (mInitial.C.Norm() - c.Norm());

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
    mECM.PositionJointSet.Goal().ForceAssign(goalJoints);
    mECM.servo_jp(mECM.PositionJointSet);

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
    mMTML.SetWrenchBodyOrientationAbsolute(true);
    mMTML.LockOrientation(currMTMLRot);
    mMTMR.SetWrenchBodyOrientationAbsolute(true);
    mMTMR.LockOrientation(currMTMRRot);
}

void mtsTeleOperationECM::TransitionEnabled(void)
{
    std::string armState;

    // check ECM state
    mECM.GetCurrentState(armState);
    if (armState != "READY") {
        mInterface->SendWarning(this->GetName() + ": ECM state has changed to [" + armState + "]");
        mTeleopState.SetDesiredState("DISABLED");
    }

    // check mtml state
    mMTML.GetCurrentState(armState);
    if (armState != "READY") {
        mInterface->SendWarning(this->GetName() + ": MTML state has changed to [" + armState + "]");
        mTeleopState.SetDesiredState("DISABLED");
    }

    // check mtmr state
    mMTMR.GetCurrentState(armState);
    if (armState != "READY") {
        mInterface->SendWarning(this->GetName() + ": MTMR state has changed to [" + armState + "]");
        mTeleopState.SetDesiredState("DISABLED");
    }

    if (mTeleopState.DesiredStateIsNotCurrent()) {
        SetFollowing(false);
        mTeleopState.SetCurrentState(mTeleopState.DesiredState());
    }
}

void mtsTeleOperationECM::SetFollowing(const bool following)
{
    MessageEvents.Following(following);
    mIsFollowing = following;
}

void mtsTeleOperationECM::MTMLErrorEventHandler(const mtsMessage & message)
{
    this->SetDesiredState("DISABLED");
    mInterface->SendError(this->GetName() + ": received from MTML [" + message.Message + "]");
}

void mtsTeleOperationECM::MTMRErrorEventHandler(const mtsMessage & message)
{
    this->SetDesiredState("DISABLED");
    mInterface->SendError(this->GetName() + ": received from MTMR [" + message.Message + "]");
}

void mtsTeleOperationECM::ECMErrorEventHandler(const mtsMessage & message)
{
    this->SetDesiredState("DISABLED");
    mInterface->SendError(this->GetName() + ": received from ECM [" + message.Message + "]");
}

void mtsTeleOperationECM::ClutchEventHandler(const prmEventButton & button)
{
    switch (button.Type()) {
    case prmEventButton::PRESSED:
        mIsClutched = true;
        break;
    case prmEventButton::RELEASED:
        mIsClutched = false;
        break;
    default:
        break;
    }

    // if the teleoperation is activated
    if (mTeleopState.DesiredState() == "ENABLED") {
        Clutch(mIsClutched);
    }
}

void mtsTeleOperationECM::Clutch(const bool & clutch)
{
    // if the teleoperation is activated
    if (clutch) {
        SetFollowing(false);
        mInterface->SendStatus(this->GetName() + ": console clutch pressed");

        // set MTMs in effort mode, no force applied but gravity and locked orientation
        prmForceCartesianSet wrench;
        mMTML.servo_cf_body(wrench);
        mMTML.SetGravityCompensation(true);
        mMTML.LockOrientation(mMTML.PositionCartesianCurrent.Position().Rotation());
        mMTMR.servo_cf_body(wrench);
        mMTMR.SetGravityCompensation(true);
        mMTMR.LockOrientation(mMTMR.PositionCartesianCurrent.Position().Rotation());
    } else {
        mIsClutched = false;
        mInterface->SendStatus(this->GetName() + ": console clutch released");
        mTeleopState.SetCurrentState("SETTING_ARMS_STATE");
    }
}

void mtsTeleOperationECM::SetDesiredState(const std::string & state)
{
    // try to find the state in state machine
    if (!mTeleopState.StateExists(state)) {
        mInterface->SendError(this->GetName() + ": unsupported state " + state);
        return;
    }
    // try to set the desired state
    try {
        mTeleopState.SetDesiredState(state);
    } catch (...) {
        mInterface->SendError(this->GetName() + ": " + state + " is not an allowed desired state");
        return;
    }
    MessageEvents.DesiredState(state);
    mInterface->SendStatus(this->GetName() + ": set desired state to " + state);
}

void mtsTeleOperationECM::SetScale(const double & scale)
{
    mConfigurationStateTable->Start();
    mScale = scale;
    mConfigurationStateTable->Advance();
    ConfigurationEvents.Scale(mScale);
}
