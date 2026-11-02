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
        mInitial.MTMPosition = mMTM.m_measured_cp.Position().translation();
        mInitial.MTMRotation = mMTM.m_measured_cp.Position().rotation();
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
    mInitial.C = Eigen::Vector3d(0.0, 0.0, 0.35);
    mInitial.vectorLR = Eigen::Vector3d::UnitX();
    mInitial.Up = mInitial.vectorLR.cross(mInitial.C).normalized();

    // Projections
    mInitial.Lr = Eigen::Vector3d(mInitial.C[0], 0, mInitial.C[2]).normalized();
    mInitial.Ud = Eigen::Vector3d(0, mInitial.C[1], mInitial.C[2]).normalized();
    mInitial.Cw = Eigen::Vector3d(mInitial.Up[0], mInitial.Up[1], 0).normalized();

    // Snapshot master pose and ECM joint position
    mInitial.MTMPosition = mMTM.m_measured_cp.Position().translation();
    mInitial.MTMRotation = mMTM.m_measured_cp.Position().rotation();
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

    const Eigen::Vector3d currentMTMPos = mMTM.m_measured_cp.Position().translation();
    const Eigen::Matrix3d currentMTMRot = mMTM.m_measured_cp.Position().rotation();

    // 1. Center of virtual handlebar
    Eigen::Vector3d deltaP = (currentMTMPos - mInitial.MTMPosition) * m_config.scale;
    Eigen::Vector3d c = mInitial.C + deltaP;

    // 2. Orientation of virtual handlebar
    Eigen::Matrix3d deltaR = currentMTMRot * mInitial.MTMRotation.transpose();
    Eigen::Vector3d vLR = deltaR * mInitial.vectorLR;

    // 3. Virtual up vector
    Eigen::Vector3d up;
    Eigen::Vector3d crossLR_C = vLR.cross(c);
    if (crossLR_C.norm() > 1e-6) {
        up = crossLR_C.normalized();
    } else {
        up = mInitial.Up;
    }

    // 4. Joint computations
    static const Eigen::Vector3d normXZ(0.0, 1.0, 0.0);
    static const Eigen::Vector3d normYZ(1.0, 0.0, 0.0);
    static const Eigen::Vector3d normXY(0.0, 0.0, 1.0);

    Eigen::VectorXd goalJoints = mInitial.ECMPositionJoint;
    Eigen::VectorXd changeJoints = Eigen::VectorXd::Zero(4);
    Eigen::VectorXd changeDir = Eigen::VectorXd::Zero(4);

    // - Direction 0 - left/right, movement in the XZ plane
    Eigen::Vector3d lr = Eigen::Vector3d(c[0], 0.0, c[2]).normalized();
    if (mInitial.Lr.isApprox(lr)) {
        changeDir[0] = 0.0;
    } else {
        double dot = std::max(-1.0, std::min(1.0, mInitial.Lr.dot(lr)));
        changeDir[0] = -acos(dot);
        Eigen::Vector3d crossN = mInitial.Lr.cross(lr);
        if (normXZ.dot(crossN) < 0.0) {
            changeDir[0] = -changeDir[0];
        }
    }

    // - Direction 1 - up/down, movement in the YZ plane
    Eigen::Vector3d ud = Eigen::Vector3d(0.0, c[1], c[2]).normalized();
    if (mInitial.Ud.isApprox(ud)) {
        changeDir[1] = 0.0;
    } else {
        double dot = std::max(-1.0, std::min(1.0, mInitial.Ud.dot(ud)));
        changeDir[1] = acos(dot);
        Eigen::Vector3d crossN = mInitial.Ud.cross(ud);
        if (normYZ.dot(crossN) < 0.0) {
            changeDir[1] = -changeDir[1];
        }
    }

    // - Direction 2 - in/out
    changeDir[2] = mInitial.C.norm() - c.norm();

    // - Direction 3 - cc/ccw, movement in the XY plane
    Eigen::Vector3d cw = Eigen::Vector3d(up[0], up[1], 0.0).normalized();
    if (mInitial.Cw.isApprox(cw)) {
        changeDir[3] = 0.0;
    } else {
        double dot = std::max(-1.0, std::min(1.0, mInitial.Cw.dot(cw)));
        changeDir[3] = -acos(dot);
        Eigen::Vector3d crossN = mInitial.Cw.cross(cw);
        if (normXY.dot(crossN) < 0) {
            changeDir[3] = -changeDir[3];
        }
    }

    // adjusting movement for camera orientation
    double totalChangeJoint3 = changeDir[3] + mInitial.ECMPositionJoint[3] + m_config.camera_roll_offset;
    changeJoints[0] = changeDir[0] * cos(totalChangeJoint3) - changeDir[1] * sin(totalChangeJoint3);
    changeJoints[1] = changeDir[1] * cos(totalChangeJoint3) + changeDir[0] * sin(totalChangeJoint3);
    changeJoints[2] = changeDir[2];
    changeJoints[3] = changeDir[3];

    goalJoints += changeJoints;
    mECM_OneHand.m_servo_jp.Goal() = goalJoints;
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
