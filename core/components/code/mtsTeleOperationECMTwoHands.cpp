/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Nicole Ortega
  Created on: 2016-01-21

  (C) Copyright 2016-2026 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <iostream>

#include <sawIntuitiveResearchKit/mtsTeleOperationECMTwoHands.h>

#include <cisstCommon/cmnUnits.h>
#include <cisstVector/vctBoundingBox3.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperationECMTwoHands, mtsTeleOperationECMBase, mtsTaskPeriodicConstructorArg);

mtsTeleOperationECMTwoHands::mtsTeleOperationECMTwoHands(const std::string & componentName, const double periodInSeconds):
    mtsTeleOperationECMBase(componentName, periodInSeconds)
{
    Init();
}

mtsTeleOperationECMTwoHands::mtsTeleOperationECMTwoHands(const mtsTaskPeriodicConstructorArg & arg):
    mtsTeleOperationECMBase(arg)
{
    Init();
}

mtsTeleOperationECMTwoHands::~mtsTeleOperationECMTwoHands()
{
}

void mtsTeleOperationECMTwoHands::Init(void)
{
    mtsTeleOperationECMBase::Init();

    StateTable.AddData(mMTML.m_measured_cp, "MTML/measured_cp");
    StateTable.AddData(mMTMR.m_measured_cp, "MTMR/measured_cp");

    // Setup cisst interfaces for MTML and MTMR
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("MTML");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("measured_cp", mMTML.measured_cp);
        interfaceRequired->AddFunction("measured_cv", mMTML.measured_cv, MTS_OPTIONAL);
        interfaceRequired->AddFunction("lock_orientation", mMTML.lock_orientation, MTS_OPTIONAL);
        interfaceRequired->AddFunction("body/servo_cf", mMTML.body_servo_cf, MTS_OPTIONAL);
        interfaceRequired->AddFunction("body/set_cf_orientation_absolute", mMTML.body_set_cf_orientation_absolute, MTS_OPTIONAL);
        interfaceRequired->AddFunction("use_gravity_compensation", mMTML.use_gravity_compensation, MTS_OPTIONAL);
        interfaceRequired->AddFunction("operating_state", mMTML.operating_state);
        interfaceRequired->AddFunction("state_command", mMTML.state_command);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECMBase::arm_error_event_handler,
                                                static_cast<mtsTeleOperationECMBase *>(this), "error");
    }

    interfaceRequired = AddInterfaceRequired("MTMR");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("measured_cp", mMTMR.measured_cp);
        interfaceRequired->AddFunction("measured_cv", mMTMR.measured_cv, MTS_OPTIONAL);
        interfaceRequired->AddFunction("lock_orientation", mMTMR.lock_orientation, MTS_OPTIONAL);
        interfaceRequired->AddFunction("body/servo_cf", mMTMR.body_servo_cf, MTS_OPTIONAL);
        interfaceRequired->AddFunction("body/set_cf_orientation_absolute", mMTMR.body_set_cf_orientation_absolute, MTS_OPTIONAL);
        interfaceRequired->AddFunction("use_gravity_compensation", mMTMR.use_gravity_compensation, MTS_OPTIONAL);
        interfaceRequired->AddFunction("operating_state", mMTMR.operating_state);
        interfaceRequired->AddFunction("state_command", mMTMR.state_command);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECMBase::arm_error_event_handler,
                                                static_cast<mtsTeleOperationECMBase *>(this), "error");
    }

    // ECM two-hands joint functions
    interfaceRequired = GetInterfaceRequired("ECM");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("setpoint_js", mECM_TwoHands.setpoint_js);
        interfaceRequired->AddFunction("servo_jp", mECM_TwoHands.servo_jp);
    }

    if (mInterface) {
        mInterface->AddCommandReadState(StateTable,
                                        mMTML.m_measured_cp,
                                        "MTML/measured_cp");
        mInterface->AddCommandReadState(StateTable,
                                        mMTMR.m_measured_cp,
                                        "MTMR/measured_cp");
    }
}

void mtsTeleOperationECMTwoHands::Startup(void)
{
    mtsTeleOperationECMBase::Startup();

    m_mtml_measured_cv_available = mMTML.measured_cv.IsValid();
    if (!m_mtml_measured_cv_available) {
        CMN_LOG_CLASS_INIT_WARNING << "Startup: MTML::measured_cv is not available, will not use velocity for viscous damping"
                                   << std::endl;
    }
    m_mtmr_measured_cv_available = mMTMR.measured_cv.IsValid();
    if (!m_mtmr_measured_cv_available) {
        CMN_LOG_CLASS_INIT_WARNING << "Startup: MTMR::measured_cv is not available, will not use velocity for viscous damping"
                                   << std::endl;
    }
}

void mtsTeleOperationECMTwoHands::RunAllStates(void)
{
    mtsTeleOperationECMBase::RunAllStates();

    mtsExecutionResult executionResult;

    // get MTML Cartesian position/velocity
    executionResult = mMTML.measured_cp(mMTML.m_measured_cp);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTML.measured_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from MTML");
        mTeleopState.SetDesiredState("DISABLED");
    }
    if (m_mtml_measured_cv_available) {
        executionResult = mMTML.measured_cv(mMTML.m_measured_cv);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTML.measured_cv failed \""
                                    << executionResult << "\"" << std::endl;
            mInterface->SendError(this->GetName() + ": unable to get cartesian velocity from MTML");
            mTeleopState.SetDesiredState("DISABLED");
        }
    }

    // get MTMR Cartesian position/velocity
    executionResult = mMTMR.measured_cp(mMTMR.m_measured_cp);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTMR.measured_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from MTMR");
        mTeleopState.SetDesiredState("DISABLED");
    }
    if (m_mtmr_measured_cv_available) {
        executionResult = mMTMR.measured_cv(mMTMR.m_measured_cv);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTMR.measured_cv failed \""
                                    << executionResult << "\"" << std::endl;
            mInterface->SendError(this->GetName() + ": unable to get cartesian velocity from MTMR");
            mTeleopState.SetDesiredState("DISABLED");
        }
    }

    // for motion computation
    executionResult = mECM_TwoHands.setpoint_js(mECM_TwoHands.m_setpoint_js);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to ECM.setpoint_js failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get joint state from ECM");
        mTeleopState.SetDesiredState("DISABLED");
    }

    if (mTeleopState.CurrentState() == "ENABLED") {
        const bool dataValid = mECM.m_measured_cp.Valid()
            && mECM_TwoHands.m_setpoint_js.Valid()
            && (!m_config.MTM_is_haptic
                || (mMTML.m_measured_cp.Valid() && mMTMR.m_measured_cp.Valid()));
        if (!dataValid) {
            if (!m_data_check_warning_sent) {
                mInterface->SendWarning(this->GetName()
                                         + ": MTM or ECM Cartesian data became invalid, disabling teleoperation");
                m_data_check_warning_sent = true;
            }
            mTeleopState.SetDesiredState("DISABLED");
        }
    }

    // monitor state of MTM arms if needed
    if ((mTeleopState.CurrentState() != "DISABLED")
        && (mTeleopState.CurrentState() != "SETTING_ARMS_STATE")) {
        if (m_config.MTM_is_haptic) {
            prmOperatingState state;
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
}

void mtsTeleOperationECMTwoHands::EnterSettingArmsState(void)
{
    mtsTeleOperationECMBase::EnterSettingArmsState();

    if (m_config.MTM_is_haptic) {
        prmOperatingState state;
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
}

void mtsTeleOperationECMTwoHands::TransitionSettingArmsState(void)
{
    prmOperatingState ecmState, mtmlState, mtmrState;
    mECM.operating_state(ecmState);
    if (m_config.MTM_is_haptic) {
        mMTML.operating_state(mtmlState);
        mMTMR.operating_state(mtmrState);
    }
    if ((ecmState.State() == prmOperatingState::ENABLED) && ecmState.IsHomed()
        && (!m_config.MTM_is_haptic
            || ((mtmlState.State() == prmOperatingState::ENABLED) && mtmlState.IsHomed()
                && (mtmrState.State() == prmOperatingState::ENABLED) && mtmrState.IsHomed()))) {
        mTeleopState.SetCurrentState("CHECKING_DATA");
        return;
    }
    // check timer
    if ((StateTable.GetTic() - mInStateTimer) > 60.0 * cmn_s) {
        mInterface->SendError(this->GetName() + ": timed out while setting up arms state");
        mTeleopState.SetDesiredState("DISABLED");
    }
}

void mtsTeleOperationECMTwoHands::EnterCheckingData(void)
{
    mtsTeleOperationECMBase::EnterCheckingData();
}

void mtsTeleOperationECMTwoHands::TransitionCheckingData(void)
{
    const bool dataValid = mECM.m_measured_cp.Valid()
        && mECM_TwoHands.m_setpoint_js.Valid()
        && (!m_config.MTM_is_haptic
            || (mMTML.m_measured_cp.Valid() && mMTMR.m_measured_cp.Valid()));

    if (dataValid) {
        if (m_data_check_warning_sent) {
            mInterface->SendStatus(this->GetName() + ": MTM and ECM Cartesian data is valid");
        }
        if (m_config.check_MTMs_workspace) {
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
        }
        mTeleopState.SetCurrentState("ENABLED");
        return;
    }

    if (!m_data_check_warning_sent) {
        mInterface->SendWarning(this->GetName()
                                + ": waiting for valid MTM and ECM Cartesian data");
        m_data_check_warning_sent = true;
    }
}

void mtsTeleOperationECMTwoHands::EnterEnabled(void)
{
    if (m_config.MTM_is_haptic) {
        mMTML.use_gravity_compensation(true);
        mMTML.body_set_cf_orientation_absolute(true);
        mMTML.lock_orientation(mMTML.m_measured_cp.Position().Rotation());
        mMTMR.use_gravity_compensation(true);
        mMTMR.body_set_cf_orientation_absolute(true);
        mMTMR.lock_orientation(mMTMR.m_measured_cp.Position().Rotation());
    }

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

    mInitial.ECMPositionJoint = mECM_TwoHands.m_setpoint_js.Position();

    // -5- store current rotation matrix for MTML, MTMR, and ECM
    vctEulerZYXRotation3 eulerAngles;
    eulerAngles.Assign(mInitial.ECMPositionJoint[3], mInitial.ECMPositionJoint[0], mInitial.ECMPositionJoint[1]);
    vctEulerToMatrixRotation3(eulerAngles, mInitial.ECMRotEuler);

    mInitial.MTMLRot = mMTML.m_measured_cp.Position().Rotation();
    mInitial.MTMRRot = mMTMR.m_measured_cp.Position().Rotation();

    mInitial.ECMPositionJoint = mECM_TwoHands.m_setpoint_js.Position();
    if (m_clutched) {
        Clutch(true);
    } else {
        set_following(true);
    }
}

void mtsTeleOperationECMTwoHands::RunEnabled(void)
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
    if (m_mtmr_measured_cv_available) {
        forceFriction.ElementwiseProductOf(frictionForceCoeff,
                                           mMTMR.m_measured_cv.VelocityLinear());
        wrenchR.Force().Ref<3>(0).Add(forceFriction);
    }
    wrenchR.Force().Ref<3>(0).Multiply(m_config.haptic_feedback_ratio);
    // apply
    if (m_config.MTM_is_haptic) {
        mMTMR.body_servo_cf(wrenchR);
    }

    // MTML
    // apply force
    force.DifferenceOf(goalL,
                       mMTML.m_measured_cp.Position().Translation());
    force.Multiply(distanceForceCoeff);
    wrenchL.Force().Ref<3>(0).Assign(force);
    // add friction force
    if (m_mtml_measured_cv_available) {
        forceFriction.ElementwiseProductOf(frictionForceCoeff,
                                           mMTML.m_measured_cv.VelocityLinear());
        wrenchL.Force().Ref<3>(0).Add(forceFriction);
    }
    wrenchL.Force().Ref<3>(0).Multiply(m_config.haptic_feedback_ratio);
    // apply
    if (m_config.MTM_is_haptic) {
        mMTML.body_servo_cf(wrenchL);
    }

    /* --- Joint Control --- */
    static const vct3 normXZ(0.0, 1.0, 0.0);
    static const vct3 normYZ(1.0, 0.0, 0.0);
    static const vct3 normXY(0.0, 0.0, 1.0);

    vctVec goalJoints(mInitial.ECMPositionJoint);
    vctVec changeJoints(4);
    vctVec changeDir(4, 0.0);
    vct3 crossN;

    // - Direction 0 - left/right, movement in the XZ plane
    vct3 lr(c[0], 0, c[2]);
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
    vct3 ud(0.0, c[1], c[2]);
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
    changeDir[2] = m_config.scale * (mInitial.C.Norm() - c.Norm());

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
    double totalChangeJoint3 = changeDir[3] + mInitial.ECMPositionJoint[3] + m_config.camera_roll_offset;
    changeJoints[0] = changeDir[0] * cos(totalChangeJoint3) - changeDir[1] * sin(totalChangeJoint3);
    changeJoints[1] = changeDir[1] * cos(totalChangeJoint3) + changeDir[0] * sin(totalChangeJoint3);
    changeJoints[2] = changeDir[2];
    changeJoints[3] = changeDir[3];

    goalJoints.Add(changeJoints);
    mECM_TwoHands.m_servo_jp.Goal().ForceAssign(goalJoints);
    mECM_TwoHands.servo_jp(mECM_TwoHands.m_servo_jp);

    /* --- Lock Orientation --- */
    vctMatRot3 currMTMLRot;
    vctMatRot3 currMTMRRot;
    vctEulerZYXRotation3 finalEulerAngles;
    vctMatrixRotation3<double> currECMRot;
    vctMatrixRotation3<double> finalECMRot;

    finalEulerAngles.Assign(goalJoints[3], goalJoints[0], goalJoints[1]);
    vctEulerToMatrixRotation3(finalEulerAngles, finalECMRot);
    currECMRot = finalECMRot * mInitial.ECMRotEuler.Inverse();

    currMTMLRot = currECMRot.Inverse() * mInitial.MTMLRot;
    currMTMRRot = currECMRot.Inverse() * mInitial.MTMRRot;

    if (m_config.MTM_is_haptic) {
        mMTML.body_set_cf_orientation_absolute(true);
        mMTML.lock_orientation(currMTMLRot);
        mMTMR.body_set_cf_orientation_absolute(true);
        mMTMR.lock_orientation(currMTMRRot);
    }
}

void mtsTeleOperationECMTwoHands::Clutch(const bool & clutch)
{
    mtsTeleOperationECMBase::Clutch(clutch);

    if (clutch) {
        if (m_config.MTM_is_haptic) {
            prmForceCartesianSet wrench;
            mMTML.body_servo_cf(wrench);
            mMTML.use_gravity_compensation(true);
            mMTML.lock_orientation(mMTML.m_measured_cp.Position().Rotation());
            mMTMR.body_servo_cf(wrench);
            mMTMR.use_gravity_compensation(true);
            mMTMR.lock_orientation(mMTMR.m_measured_cp.Position().Rotation());
        }
    }
}
