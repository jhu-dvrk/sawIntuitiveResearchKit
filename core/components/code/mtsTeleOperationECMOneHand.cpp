/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2026-10-09

  (C) Copyright 2026 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/mtsTeleOperationECMOneHand.h>

#include <cisstCommon/cmnUnits.h>
#include <cisstVector/vctBoundingBox3.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmOperatingState.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsTeleOperationECMOneHand, mtsTeleOperationECMBase, mtsTaskPeriodicConstructorArg);

mtsTeleOperationECMOneHand::mtsTeleOperationECMOneHand(const std::string & componentName, const double periodInSeconds):
    mtsTeleOperationECMBase(componentName, periodInSeconds)
{
    Init();
}

mtsTeleOperationECMOneHand::mtsTeleOperationECMOneHand(const mtsTaskPeriodicConstructorArg & arg):
    mtsTeleOperationECMBase(arg)
{
    Init();
}

mtsTeleOperationECMOneHand::~mtsTeleOperationECMOneHand()
{
}

void mtsTeleOperationECMOneHand::Init(void)
{
    mtsTeleOperationECMBase::Init();

    StateTable.AddData(mMTM.m_measured_cp, "MTM/measured_cp");
    StateTable.AddData(mECM_OneHand.m_setpoint_js, "ECM/setpoint_js");

    // MTM interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("MTM");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("measured_cp", mMTM.measured_cp);
        interfaceRequired->AddFunction("measured_cv", mMTM.measured_cv, MTS_OPTIONAL);
        interfaceRequired->AddFunction("use_gravity_compensation", mMTM.use_gravity_compensation, MTS_OPTIONAL);
        interfaceRequired->AddFunction("body/servo_cf", mMTM.body_servo_cf, MTS_OPTIONAL);
        interfaceRequired->AddFunction("operating_state", mMTM.operating_state);
        interfaceRequired->AddFunction("state_command", mMTM.state_command);
        interfaceRequired->AddEventHandlerWrite(&mtsTeleOperationECMBase::arm_error_event_handler,
                                                static_cast<mtsTeleOperationECMBase *>(this), "error");
    }

    // ECM one-hand specific functions (joint space)
    interfaceRequired = GetInterfaceRequired("ECM");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("setpoint_js", mECM_OneHand.setpoint_js);
        interfaceRequired->AddFunction("servo_jp", mECM_OneHand.servo_jp);
    }

    if (mInterface) {
        mInterface->AddCommandReadState(StateTable,
                                        mMTM.m_measured_cp,
                                        "MTM/measured_cp");
        mInterface->AddCommandReadState(StateTable,
                                        mECM_OneHand.m_setpoint_js,
                                        "ECM/setpoint_js");
    }
}

void mtsTeleOperationECMOneHand::Startup(void)
{
    mtsTeleOperationECMBase::Startup();
    m_mtm_measured_cv_available = mMTM.measured_cv.IsValid();
}

void mtsTeleOperationECMOneHand::set_scale(const double & scale)
{
    mtsTeleOperationECMBase::set_scale(scale);

    // prevent jumps if following
    if (m_following) {
        mInitial.MTMPosition = mMTM.m_measured_cp.Position().Translation();
        mInitial.MTMRotation = mMTM.m_measured_cp.Position().Rotation();
        mInitial.ECMPositionJoint = mECM_OneHand.m_setpoint_js.Position();
    }
}

void mtsTeleOperationECMOneHand::RunAllStates(void)
{
    mtsTeleOperationECMBase::RunAllStates();

    mtsExecutionResult executionResult;

    // get MTM Cartesian position/velocity
    executionResult = mMTM.measured_cp(mMTM.m_measured_cp);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTM.measured_cp failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get cartesian position from MTM");
        mTeleopState.SetDesiredState("DISABLED");
    }
    if (m_mtm_measured_cv_available) {
        executionResult = mMTM.measured_cv(mMTM.m_measured_cv);
        if (!executionResult.IsOK()) {
            CMN_LOG_CLASS_RUN_ERROR << "Run: call to MTM.measured_cv failed \""
                                    << executionResult << "\"" << std::endl;
            mInterface->SendError(this->GetName() + ": unable to get cartesian velocity from MTM");
            mTeleopState.SetDesiredState("DISABLED");
        }
    }

    // get ECM joint state
    executionResult = mECM_OneHand.setpoint_js(mECM_OneHand.m_setpoint_js);
    if (!executionResult.IsOK()) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: call to ECM.setpoint_js failed \""
                                << executionResult << "\"" << std::endl;
        mInterface->SendError(this->GetName() + ": unable to get joint state from ECM");
        mTeleopState.SetDesiredState("DISABLED");
    }

    if (mTeleopState.CurrentState() == "ENABLED") {
        const bool dataValid = mECM.m_measured_cp.Valid()
            && mECM_OneHand.m_setpoint_js.Valid()
            && mMTM.m_measured_cp.Valid();
        if (!dataValid) {
            if (!m_data_check_warning_sent) {
                mInterface->SendWarning(this->GetName()
                                         + ": MTM or ECM data became invalid, disabling teleoperation");
                m_data_check_warning_sent = true;
            }
            mTeleopState.SetDesiredState("DISABLED");
        }
    }

    // monitor state of MTM
    if ((mTeleopState.CurrentState() != "DISABLED")
        && (mTeleopState.CurrentState() != "SETTING_ARMS_STATE")) {
        prmOperatingState state;
        mMTM.operating_state(state);
        if ((state.State() != prmOperatingState::ENABLED)
            || !state.IsHomed()) {
            mTeleopState.SetDesiredState("DISABLED");
            mInterface->SendError(this->GetName() + ": MTM is not in state \"READY\" anymore");
        }
    }
}

void mtsTeleOperationECMOneHand::TransitionSettingArmsState(void)
{
    prmOperatingState state;
    mECM.operating_state(state);
    const bool ecmReady = (state.State() == prmOperatingState::ENABLED) && state.IsHomed();
    mMTM.operating_state(state);
    const bool mtmReady = (state.State() == prmOperatingState::ENABLED) && state.IsHomed();
    if (ecmReady && mtmReady) {
        mTeleopState.SetCurrentState("CHECKING_DATA");
        return;
    }
    // check timeout (60 seconds)
    if ((StateTable.GetTic() - mInStateTimer) > 60.0 * cmn_s) {
        mInterface->SendError(this->GetName() + ": timeout while waiting for arms to be ready");
        mTeleopState.SetDesiredState("DISABLED");
    }
}

void mtsTeleOperationECMOneHand::TransitionCheckingData(void)
{
    const bool dataValid = mECM.m_measured_cp.Valid()
        && mECM_OneHand.m_setpoint_js.Valid()
        && (mECM_OneHand.m_setpoint_js.Position().size() >= 4)
        && mMTM.m_measured_cp.Valid();

    if (dataValid) {
        m_data_check_warning_sent = false;
        mTeleopState.SetCurrentState("ENABLED");
    } else {
        if ((StateTable.GetTic() - mInStateTimer) > 2.0 * cmn_s) {
            mInterface->SendError(this->GetName() + ": timeout while waiting for valid data");
            mTeleopState.SetDesiredState("DISABLED");
        }
    }
}

void mtsTeleOperationECMOneHand::EnterEnabled(void)
{
    if (mMTM.use_gravity_compensation.IsValid()) {
        mMTM.use_gravity_compensation(true);
    }
    if (mMTM.body_servo_cf.IsValid()) {
        prmForceCartesianSet wrench;
        mMTM.body_servo_cf(wrench);
    }

    // Nominal handlebar geometry
    mInitial.C.Assign(0.0, 0.0, 0.35);
    mInitial.vectorLR.Assign(1.0, 0.0, 0.0);
    mInitial.Up.CrossProductOf(mInitial.vectorLR, mInitial.C);
    mInitial.Up.NormalizedSelf();

    // Projections
    mInitial.Lr.Assign(mInitial.C[0], 0.0, mInitial.C[2]);
    mInitial.Lr.NormalizedSelf();
    mInitial.Ud.Assign(0.0, mInitial.C[1], mInitial.C[2]);
    mInitial.Ud.NormalizedSelf();
    mInitial.Cw.Assign(mInitial.Up[0], mInitial.Up[1], 0.0);
    mInitial.Cw.NormalizedSelf();

    // Snapshot master pose and ECM joint position
    mInitial.MTMPosition = mMTM.m_measured_cp.Position().Translation();
    mInitial.MTMRotation = mMTM.m_measured_cp.Position().Rotation();
    mInitial.ECMPositionJoint = mECM_OneHand.m_setpoint_js.Position();

    if (m_clutched) {
        Clutch(true);
    } else {
        set_following(true);
    }
}

void mtsTeleOperationECMOneHand::RunEnabled(void)
{
    if (m_clutched) {
        return;
    }

    const vct3 currentMTMPos = mMTM.m_measured_cp.Position().Translation();
    const vctMatRot3 currentMTMRot = mMTM.m_measured_cp.Position().Rotation();

    // 1. Center of virtual handlebar
    vct3 deltaP = (currentMTMPos - mInitial.MTMPosition) * m_config.scale;
    vct3 c = mInitial.C + deltaP;

    // 2. Orientation of virtual handlebar
    vctMatRot3 deltaR = currentMTMRot * mInitial.MTMRotation.Inverse();
    vct3 vLR = deltaR * mInitial.vectorLR;

    // 3. Virtual up vector
    vct3 up;
    vct3 crossLR_C = vctCrossProduct(vLR, c);
    if (crossLR_C.Norm() > 1e-6) {
        up = crossLR_C.Normalized();
    } else {
        up = mInitial.Up;
    }

    // 4. Joint computations
    static const vct3 normXZ(0.0, 1.0, 0.0);
    static const vct3 normYZ(1.0, 0.0, 0.0);
    static const vct3 normXY(0.0, 0.0, 1.0);

    vctVec goalJoints(mInitial.ECMPositionJoint);
    vctVec changeJoints(4);
    vctVec changeDir(4, 0.0);
    vct3 crossN;

    // - Direction 0 - left/right, movement in the XZ plane
    vct3 lr(c[0], 0.0, c[2]);
    lr.NormalizedSelf();
    if (mInitial.Lr.AlmostEqual(lr)) {
        changeDir[0] = 0.0;
    } else {
        double dot = std::max(-1.0, std::min(1.0, vctDotProduct(mInitial.Lr, lr)));
        changeDir[0] = -acos(dot);
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
        double dot = std::max(-1.0, std::min(1.0, vctDotProduct(mInitial.Ud, ud)));
        changeDir[1] = acos(dot);
        crossN = vctCrossProduct(mInitial.Ud, ud);
        if (vctDotProduct(normYZ, crossN) < 0.0) {
            changeDir[1] = -changeDir[1];
        }
    }

    // - Direction 2 - in/out
    changeDir[2] = mInitial.C.Norm() - c.Norm();

    // - Direction 3 - cc/ccw, movement in the XY plane
    vct3 cw(up[0], up[1], 0.0);
    cw.NormalizedSelf();
    if (mInitial.Cw.AlmostEqual(cw)) {
        changeDir[3] = 0.0;
    } else {
        double dot = std::max(-1.0, std::min(1.0, vctDotProduct(mInitial.Cw, cw)));
        changeDir[3] = -acos(dot);
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
    mECM_OneHand.m_servo_jp.Goal().ForceAssign(goalJoints);
    mECM_OneHand.servo_jp(mECM_OneHand.m_servo_jp);
}

void mtsTeleOperationECMOneHand::Clutch(const bool & clutch)
{
    mtsTeleOperationECMBase::Clutch(clutch);

    if (clutch) {
        if (mMTM.body_servo_cf.IsValid()) {
            prmForceCartesianSet wrench;
            mMTM.body_servo_cf(wrench);
        }
        if (mMTM.use_gravity_compensation.IsValid()) {
            mMTM.use_gravity_compensation(true);
        }
    }
}
