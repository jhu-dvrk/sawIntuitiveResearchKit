/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2021-06-21

  (C) Copyright 2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// cisst
#include <sawIntuitiveResearchKit/mtsAutonomousECM.h>

#include <cisstCommon/cmnUnits.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsAutonomousECM, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

mtsAutonomousECM::mtsAutonomousECM(const std::string & componentName, const double periodInSeconds):
    mtsTaskPeriodic(componentName, periodInSeconds)
{
    Init();
}

mtsAutonomousECM::mtsAutonomousECM(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg)
{
    Init();
}

mtsAutonomousECM::~mtsAutonomousECM()
{
}

void mtsAutonomousECM::Init(void)
{
    StateTable.AddData(mMTML.m_measured_cp, "MTML/measured_cp");
    StateTable.AddData(mMTMR.m_measured_cp, "MTMR/measured_cp");
    StateTable.AddData(mECM.m_measured_cp, "ECM/measured_cp");

    mConfigurationStateTable = new mtsStateTable(100, "Configuration");
    mConfigurationStateTable->SetAutomaticAdvance(false);
    AddStateTable(mConfigurationStateTable);

    mtsInterfaceRequired * interfaceRequired;

#if 0
    interfaceRequired = AddInterfaceRequired("MTML");
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
        interfaceRequired->AddEventHandlerWrite(&mtsAutonomousECM::ArmErrorEventHandler,
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
        interfaceRequired->AddEventHandlerWrite(&mtsAutonomousECM::ArmErrorEventHandler,
                                                this, "error");
        interfaceRequired->AddFunction("operating_state",
                                       mMTMR.operating_state);
        interfaceRequired->AddFunction("state_command",
                                       mMTMR.state_command);
    }
#endif
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
        interfaceRequired->AddEventHandlerWrite(&mtsAutonomousECM::ArmErrorEventHandler,
                                                this, "error");
    }


    mInterface = AddInterfaceProvided("Setting");
    if (mInterface) {
        mInterface->AddMessageEvents();
        // commands
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                        "period_statistics"); // mtsIntervalStatistics

        mInterface->AddCommandWrite(&mtsAutonomousECM::state_command, this,
                                    "state_command", std::string("DISABLED"));

#if 0
        mInterface->AddCommandReadState(StateTable,
                                        mMTML.m_measured_cp,
                                        "MTML/measured_cp");
        mInterface->AddCommandReadState(StateTable,
                                        mMTMR.m_measured_cp,
                                        "MTMR/measured_cp");
        mInterface->AddCommandReadState(StateTable,
                                        mECM.m_measured_cp,
                                        "ECM/measured_cp");
#endif
        // events
        mInterface->AddEventWrite(MessageEvents.state,
                                  "state", std::string(""));
    }
}

void mtsAutonomousECM::ArmErrorEventHandler(const mtsMessage & message)
{
    std::cerr << "got error: " << message << std::endl;
}

void mtsAutonomousECM::Configure(const std::string & filename)
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
    mtsAutonomousECM::Configure(jsonConfig);
}

void mtsAutonomousECM::Configure(const Json::Value & jsonConfig)
{
    Json::Value jsonValue;

    // base component configuration
    mtsComponent::ConfigureJSON(jsonConfig);

    // read scale if present
    // jsonValue = jsonConfig["scale"];
    // if (!jsonValue.empty()) {
    //     m_scale = jsonValue.asDouble();
    // }
}

void mtsAutonomousECM::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
}

void mtsAutonomousECM::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();
}

void mtsAutonomousECM::Cleanup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsAutonomousECM::RunAllStates(void)
{
    mtsExecutionResult executionResult;

    #if 0
    // get MTML Cartesian position/velocity
    executionResult = mMTML.measured_cp(mMTML.m_measured_cp);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTML.measured_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from MTML");
        // mTeleopState.SetDesiredState("DISABLED");
    }
    executionResult = mMTML.measured_cv(mMTML.m_measured_cv);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTML.measured_cv failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian velocity from MTML");
        // mTeleopState.SetDesiredState("DISABLED");
    }

    // get MTMR Cartesian position
    executionResult = mMTMR.measured_cp(mMTMR.m_measured_cp);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTMR.measured_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from MTMR");
        // mTeleopState.SetDesiredState("DISABLED");
    }
    executionResult = mMTMR.measured_cv(mMTMR.m_measured_cv);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTMR.measured_cv failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian velocity from MTMR");
        // mTeleopState.SetDesiredState("DISABLED");
    }
#endif
    // get ECM Cartesian position for GUI
    executionResult = mECM.measured_cp(mECM.m_measured_cp);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to ECM.measured_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from ECM");
        // mTeleopState.SetDesiredState("DISABLED");
    }
    // for motion computation
    executionResult = mECM.setpoint_js(mECM.m_setpoint_js);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to ECM.setpoint_js failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get joint state from ECM");
        // mTeleopState.SetDesiredState("DISABLED");
    }

    // check if anyone wanted to disable anyway
    // if ((mTeleopState.DesiredState() == "DISABLED")
    //     && (mTeleopState.CurrentState() != "DISABLED")) {
    //     set_following(false);
    //     // mTeleopState.SetCurrentState("DISABLED");
    //     return;
    // }

    // // monitor state of arms if needed
    // if ((mTeleopState.CurrentState() != "DISABLED")
    //     && (mTeleopState.CurrentState() != "SETTING_ARMS_STATE")) {
    //     prmOperatingState state;
    //     mECM.operating_state(state);
    //     if ((state.State() != prmOperatingState::ENABLED)
    //         || !state.IsHomed()) {
    //         mTeleopState.SetDesiredState("DISABLED");
    //         mInterface->SendError(this->GetName() + ": ECM is not in state \"READY\" anymore");
    //     }
    //     mMTML.operating_state(state);
    //     if ((state.State() != prmOperatingState::ENABLED)
    //         || !state.IsHomed()) {
    //         mTeleopState.SetDesiredState("DISABLED");
    //         mInterface->SendError(this->GetName() + ": MTML is not in state \"READY\" anymore");
    //     }
    //     mMTMR.operating_state(state);
    //     if ((state.State() != prmOperatingState::ENABLED)
    //         || !state.IsHomed()) {
    //         mTeleopState.SetDesiredState("DISABLED");
    //         mInterface->SendError(this->GetName() + ": MTMR is not in state \"READY\" anymore");
    //     }
    // }
}

// void mtsAutonomousECM::TransitionSettingArmsState(void)
// {
//     // check state
//     prmOperatingState ecmState, mtmlState, mtmrState;
//     mECM.operating_state(ecmState);
//     mMTML.operating_state(mtmlState);
//     mMTMR.operating_state(mtmrState);
//     if ((ecmState.State() == prmOperatingState::ENABLED) && ecmState.IsHomed()
//         && (mtmlState.State() == prmOperatingState::ENABLED) && mtmlState.IsHomed()
//         && (mtmrState.State() == prmOperatingState::ENABLED) && mtmrState.IsHomed()) {
//         mTeleopState.SetCurrentState("ENABLED");
//         return;
//     }
//     // check timer
//     if ((StateTable.GetTic() - mInStateTimer) > 60.0 * cmn_s) {
//         mInterface->SendError(this->GetName() + ": timed out while setting up arms state");
//         mTeleopState.SetDesiredState("DISABLED");
//     }
// }

// void mtsAutonomousECM::EnterEnabled(void)
// {
//     // set cartesian effort parameters
//     mMTML.use_gravity_compensation(true);
//     mMTML.body_set_cf_orientation_absolute(true);
//     mMTML.lock_orientation(mMTML.m_measured_cp.Position().Rotation());
//     mMTMR.use_gravity_compensation(true);
//     mMTMR.body_set_cf_orientation_absolute(true);
//     mMTMR.lock_orientation(mMTMR.m_measured_cp.Position().Rotation());

//     // initial state for MTM force feedback
//     // -1- initial distance between MTMs
//     vct3 vectorLR;
//     vectorLR.DifferenceOf(mMTMR.m_measured_cp.Position().Translation(),
//                           mMTML.m_measured_cp.Position().Translation());
//     // -2- mid-point, aka center of image
//     mInitial.C.SumOf(mMTMR.m_measured_cp.Position().Translation(),
//                      mMTML.m_measured_cp.Position().Translation());
//     mInitial.C.Multiply(0.5);
//     // -3- image up vector
//     mInitial.Up.CrossProductOf(vectorLR, mInitial.C);
//     mInitial.Up.NormalizedSelf();
//     // -4- width of image, depth of arms wrt image plan
//     vct3 side;
//     side.CrossProductOf(mInitial.C, mInitial.Up);
//     side.NormalizedSelf();
//     mInitial.w = 0.5 * vctDotProduct(side, vectorLR);
//     mInitial.d = 0.5 * vctDotProduct(mInitial.C.Normalized(), vectorLR);

//     // projections
//     mInitial.Lr.Assign(mInitial.C[0], 0, mInitial.C[2]);
//     mInitial.Lr.NormalizedSelf();
//     mInitial.Ud.Assign(0, mInitial.C[1], mInitial.C[2]);
//     mInitial.Ud.NormalizedSelf();
//     mInitial.Cw.Assign(mInitial.Up[0], mInitial.Up[1], 0);
//     mInitial.Cw.NormalizedSelf();

//     mInitial.ECMPositionJoint = mECM.m_setpoint_js.Position();

//     // -5- store current rotation matrix for MTML, MTMR, and ECM
//     vctEulerZYXRotation3 eulerAngles;
//     eulerAngles.Assign(mInitial.ECMPositionJoint[3], mInitial.ECMPositionJoint[0], mInitial.ECMPositionJoint[1]);
//     vctEulerToMatrixRotation3(eulerAngles, mInitial.ECMRotEuler);

//     mInitial.MTMLRot = mMTML.m_measured_cp.Position().Rotation();
//     mInitial.MTMRRot = mMTMR.m_measured_cp.Position().Rotation();

// #if 0
//     std::cerr << CMN_LOG_DETAILS << std::endl
//               << "L: " << mMTML.m_measured_cp.Position().Translation() << std::endl
//               << "R: " << mMTMR.m_measured_cp.Position().Translation() << std::endl
//               << "C:  " << mInitial.C << std::endl
//               << "Up: " << mInitial.Up << std::endl
//               << "w:  " << mInitial.w << std::endl
//               << "d:  " << mInitial.d << std::endl
//               << "Si: " << side << std::endl;
// #endif
//     mInitial.ECMPositionJoint = mECM.m_setpoint_js.Position();
//     // check if by any chance the clutch pedal is pressed
//     if (m_clutched) {
//         Clutch(true);
//     } else {
//         set_following(true);
//     }
// }

// void mtsAutonomousECM::RunEnabled(void)
// {
//     if (m_clutched) {
//         return;
//     }

//     /* --- Forces on MTMs --- */
//     const vct3 frictionForceCoeff(-10.0, -10.0, -40.0);
//     const double distanceForceCoeff = 150.0;

//     //-1- vector between MTMs
//     vct3 vectorLR;
//     vectorLR.DifferenceOf(mMTMR.m_measured_cp.Position().Translation(),
//                           mMTML.m_measured_cp.Position().Translation());
//     // -2- mid-point, aka center of image
//     vct3 c;
//     c.SumOf(mMTMR.m_measured_cp.Position().Translation(),
//             mMTML.m_measured_cp.Position().Translation());
//     c.Multiply(0.5);
//     vct3 directionC = c.Normalized();
//     // -3- image up vector
//     vct3 up;
//     up.CrossProductOf(vectorLR, c);
//     up.NormalizedSelf();
//     // -4- Width of image
//     vct3 side;
//     side.CrossProductOf(c, up);
//     side.NormalizedSelf();
//     // -5- find desired position for L and R
//     vct3 goalL(c);
//     goalL.AddProductOf(-mInitial.w, side);
//     goalL.AddProductOf(-mInitial.d, directionC);
//     vct3 goalR(c);
//     goalR.AddProductOf(mInitial.w, side);
//     goalR.AddProductOf(mInitial.d, directionC);


//     // compute forces on L and R based on error in position
//     vct3 forceFriction;
//     vct3 force;
//     prmForceCartesianSet wrenchR, wrenchL;

//     // MTMR
//     // apply force
//     force.DifferenceOf(goalR,
//                        mMTMR.m_measured_cp.Position().Translation());
//     force.Multiply(distanceForceCoeff);
//     wrenchR.Force().Ref<3>(0).Assign(force);
//     // add friction force
//     forceFriction.ElementwiseProductOf(frictionForceCoeff,
//                                        mMTMR.m_measured_cv.VelocityLinear());
//     wrenchR.Force().Ref<3>(0).Add(forceFriction);
//     // apply
//     mMTMR.body_servo_cf(wrenchR);

//     // MTML
//     // apply force
//     force.DifferenceOf(goalL,
//                        mMTML.m_measured_cp.Position().Translation());
//     force.Multiply(distanceForceCoeff);
//     wrenchL.Force().Ref<3>(0).Assign(force);
//     // add friction force
//     forceFriction.ElementwiseProductOf(frictionForceCoeff,
//                                        mMTML.m_measured_cv.VelocityLinear());
//     wrenchL.Force().Ref<3>(0).Add(forceFriction);
//     // apply
//     mMTML.body_servo_cf(wrenchL);

//     /* --- Joint Control --- */
//     static const vct3 normXZ(0.0, 1.0, 0.0);
//     static const vct3 normYZ(1.0, 0.0, 0.0);
//     static const vct3 normXY(0.0, 0.0, 1.0);
//     // Initial ECM joints
//     vctVec goalJoints(mInitial.ECMPositionJoint);
//     // Change in directions and joints
//     vctVec changeJoints(4);
//     vctVec changeDir(4);
//     vct3 crossN;  // normal to direction of motion

//     // - Direction 0 - left/right, movement in the XZ plane
//     vct3  lr(c[0], 0.0, c[2]);
//     lr.NormalizedSelf();
//     if (mInitial.Lr.AlmostEqual(lr)) {
//         changeDir[0] = 0.0;
//     } else {
//         changeDir[0] = -acos(vctDotProduct(mInitial.Lr, lr));
//         crossN = vctCrossProduct(mInitial.Lr, lr);
//         if (vctDotProduct(normXZ, crossN) < 0.0) {
//             changeDir[0] = -changeDir[0];
//         }
//     }

//     // - Direction 1 - up/down, movement in the YZ plane
//     vct3  ud(0.0, c[1], c[2]);
//     ud.NormalizedSelf();
//     if (mInitial.Ud.AlmostEqual(ud)) {
//         changeDir[1] = 0.0;
//     } else {
//         changeDir[1] = acos(vctDotProduct(mInitial.Ud, ud));
//         crossN = vctCrossProduct(mInitial.Ud, ud);
//         if (vctDotProduct(normYZ, crossN) < 0.0) {
//             changeDir[1] = -changeDir[1];
//         }
//     }

//     // - Direction 2 - in/out
//     changeDir[2] = m_scale * (mInitial.C.Norm() - c.Norm());

//     // - Direction 3 - cc/ccw, movement in the XY plane
//     vct3 cw(up[0], up[1], 0);
//     cw.NormalizedSelf();
//     if (mInitial.Cw.AlmostEqual(cw)) {
//         changeDir[3] = 0.0;
//     } else {
//         changeDir[3] = -acos(vctDotProduct(mInitial.Cw, cw));
//         crossN = vctCrossProduct(mInitial.Cw, cw);
//         if (vctDotProduct(normXY, crossN) < 0) {
//             changeDir[3] = -changeDir[3];
//         }
//     }

//     // adjusting movement for camera orientation
//     double totalChangeJoint3 = changeDir[3] + mInitial.ECMPositionJoint[3];
//     changeJoints[0] = changeDir[0] * cos(totalChangeJoint3) - changeDir[1] * sin(totalChangeJoint3);
//     changeJoints[1] = changeDir[1] * cos(totalChangeJoint3) + changeDir[0] * sin(totalChangeJoint3);
//     changeJoints[2] = changeDir[2];
//     changeJoints[3] = changeDir[3];

//     goalJoints.Add(changeJoints);
//     mECM.m_servo_jp.Goal().ForceAssign(goalJoints);
//     mECM.servo_jp(mECM.m_servo_jp);

//     /* --- Lock Orientation --- */

//     //Calculate new rotations of MTMs
//     vctMatRot3 currMTMLRot;
//     vctMatRot3 currMTMRRot;
//     // Current ECM Rotation
//     vctEulerZYXRotation3 finalEulerAngles;
//     vctMatrixRotation3<double> currECMRot;
//     vctMatrixRotation3<double> finalECMRot;

//     finalEulerAngles.Assign(goalJoints[3], goalJoints[0], goalJoints[1]);
//     vctEulerToMatrixRotation3(finalEulerAngles, finalECMRot);
//     currECMRot = finalECMRot * mInitial.ECMRotEuler.Inverse();

//     // Set MTM Orientation
//     currMTMLRot = currECMRot.Inverse() * mInitial.MTMLRot;
//     currMTMRRot = currECMRot.Inverse() * mInitial.MTMRRot;

//     // set cartesian effort parameters
//     mMTML.body_set_cf_orientation_absolute(true);
//     mMTML.lock_orientation(currMTMLRot);
//     mMTMR.body_set_cf_orientation_absolute(true);
//     mMTMR.lock_orientation(currMTMRRot);
// }

// void mtsAutonomousECM::TransitionEnabled(void)
// {
//     if (mTeleopState.DesiredStateIsNotCurrent()) {
//         set_following(false);
//         mTeleopState.SetCurrentState(mTeleopState.DesiredState());
//     }
// }

// void mtsAutonomousECM::MTMLErrorEventHandler(const mtsMessage & message)
// {
//     mTeleopState.SetDesiredState("DISABLED");
//     mInterface->SendError(this->GetName() + ": received from MTML [" + message.Message + "]");
// }

// void mtsAutonomousECM::MTMRErrorEventHandler(const mtsMessage & message)
// {
//     mTeleopState.SetDesiredState("DISABLED");
//     mInterface->SendError(this->GetName() + ": received from MTMR [" + message.Message + "]");
// }

// void mtsAutonomousECM::ECMErrorEventHandler(const mtsMessage & message)
// {
//     mTeleopState.SetDesiredState("DISABLED");
//     mInterface->SendError(this->GetName() + ": received from ECM [" + message.Message + "]");
// }

// void mtsAutonomousECM::SetDesiredState(const std::string & state)
// {
//     // try to find the state in state machine
//     if (!mTeleopState.StateExists(state)) {
//         mInterface->SendError(this->GetName() + ": unsupported state " + state);
//         return;
//     }
//     // return is already the desired state
//     if (mTeleopState.DesiredState() == state) {
//         MessageEvents.desired_state(state);
//         return;
//     }
//     // try to set the desired state
//     try {
//         mTeleopState.SetDesiredState(state);
//     } catch (...) {
//         mInterface->SendError(this->GetName() + ": " + state + " is not an allowed desired state");
//         return;
//     }
//     // messages and events
//     MessageEvents.desired_state(state);
//     mInterface->SendStatus(this->GetName() + ": set desired state to " + state);
// }


void mtsAutonomousECM::state_command(const std::string & command)
{
    if (command == "enable") {
        m_desired_state = prmOperatingState::ENABLED;
        return;
    }
    if (command == "disable") {
        m_desired_state = prmOperatingState::DISABLED; // we might have to end trajectory first...
        return;
    }
    mInterface->SendWarning(this->GetName() + ": " + command + " doesn't seem to be a valid state_command");
}
