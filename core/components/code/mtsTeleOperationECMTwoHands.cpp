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

#include <unsupported/Eigen/EulerAngles>

#include <cisstCommon/cmnUnits.h>
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
            Eigen::AlignedBox3d workArea(Eigen::Vector3d(-400.0 * cmn_mm,
                                          -200.0 * cmn_mm,
                                            10.0 * cmn_mm),
                                     Eigen::Vector3d( 400.0 * cmn_mm,
                                           200.0 * cmn_mm,
                                           800.0 * cmn_mm));
            if (!workArea.contains(mMTML.m_measured_cp.Position().translation())) {
                mInterface->SendError(this->GetName() + ": MTML position doesn't seem to be in the work area.  Make sure your \"base-frame\" is set correctly in your console JSON configuration file.");
                mTeleopState.SetDesiredState("DISABLED");
                return;
            }
            if (!workArea.contains(mMTMR.m_measured_cp.Position().translation())) {
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
        mMTML.lock_orientation(Eigen::Matrix3d(mMTML.m_measured_cp.Position().rotation()));
        mMTMR.use_gravity_compensation(true);
        mMTMR.body_set_cf_orientation_absolute(true);
        mMTMR.lock_orientation(Eigen::Matrix3d(mMTMR.m_measured_cp.Position().rotation()));
    }

    // -1- initial distance between MTMs
    Eigen::Vector3d vectorLR = mMTMR.m_measured_cp.Position().translation() - mMTML.m_measured_cp.Position().translation();
    // -2- mid-point, aka center of image
    mInitial.C = 0.5 * (mMTMR.m_measured_cp.Position().translation() + mMTML.m_measured_cp.Position().translation());
    // -3- image up vector
    mInitial.Up = vectorLR.cross(mInitial.C).normalized();
    // -4- width of image, depth of arms wrt image plan
    Eigen::Vector3d side = mInitial.C.cross(mInitial.Up).normalized();
    mInitial.w = 0.5 * side.dot(vectorLR);
    mInitial.d = 0.5 * mInitial.C.normalized().dot(vectorLR);

    // projections
    mInitial.Lr = Eigen::Vector3d(mInitial.C[0], 0, mInitial.C[2]).normalized();
    mInitial.Ud = Eigen::Vector3d(0, mInitial.C[1], mInitial.C[2]).normalized();
    mInitial.Cw = Eigen::Vector3d(mInitial.Up[0], mInitial.Up[1], 0).normalized();

    mInitial.ECMPositionJoint = mECM_TwoHands.m_setpoint_js.Position();

    // -5- store current rotation matrix for MTML, MTMR, and ECM
    Eigen::EulerAnglesZYXd euler_angles(mInitial.ECMPositionJoint[3], mInitial.ECMPositionJoint[0], mInitial.ECMPositionJoint[1]);
    mInitial.ECMRotEuler = euler_angles.toRotationMatrix();

    mInitial.MTMLRot = mMTML.m_measured_cp.Position().rotation();
    mInitial.MTMRRot = mMTMR.m_measured_cp.Position().rotation();

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
    const Eigen::Vector3d frictionForceCoeff(-10.0, -10.0, -40.0);
    const double distanceForceCoeff = 150.0;

    //-1- vector between MTMs
    const Eigen::Vector3d vectorLR = mMTMR.m_measured_cp.Position().translation() - mMTML.m_measured_cp.Position().translation();
    // -2- mid-point, aka center of image
    const Eigen::Vector3d c = 0.5 * (mMTMR.m_measured_cp.Position().translation() +
                               mMTML.m_measured_cp.Position().translation());
    const Eigen::Vector3d directionC = c.normalized();
    // -3- image up vector
    const Eigen::Vector3d up = vectorLR.cross(c).normalized();
    // -4- Width of image
    const Eigen::Vector3d side = c.cross(up).normalized();
    // -5- find desired position for L and R
    const Eigen::Vector3d offset = mInitial.w * side + mInitial.d * directionC;
    const Eigen::Vector3d goalL = c - offset;
    const Eigen::Vector3d goalR = c + offset;

    // compute forces on L and R based on error in position
    prmForceCartesianSet wrenchR, wrenchL;

    // MTMR
    // apply force
    Eigen::Vector3d force = goalR - mMTMR.m_measured_cp.Position().translation();
    wrenchR.Force().head<3>() = distanceForceCoeff * force;
    // add friction force

    Eigen::Vector3d force_friction;
    if (m_mtmr_measured_cv_available) {
        force_friction = frictionForceCoeff.cwiseProduct(mMTMR.m_measured_cv.VelocityLinear());
        wrenchR.Force().head<3>() += force_friction;
    }
    wrenchR.Force().head<3>() *= m_config.haptic_feedback_ratio;
    // apply
    if (m_config.MTM_is_haptic) {
        mMTMR.body_servo_cf(wrenchR);
    }

    // MTML
    // apply force
    force = goalL - mMTML.m_measured_cp.Position().translation();
    wrenchL.Force().head<3>() = distanceForceCoeff * force;
    // add friction force
    if (m_mtml_measured_cv_available) {
        force_friction = frictionForceCoeff.cwiseProduct(mMTML.m_measured_cv.VelocityLinear());
        wrenchL.Force().head<3>() += force_friction;
    }
    wrenchL.Force().head<3>() *= m_config.haptic_feedback_ratio;
    // apply
    if (m_config.MTM_is_haptic) {
        mMTML.body_servo_cf(wrenchL);
    }

    /* --- Joint Control --- */
    static const Eigen::Vector3d normXZ(0.0, 1.0, 0.0);
    static const Eigen::Vector3d normYZ(1.0, 0.0, 0.0);
    static const Eigen::Vector3d normXY(0.0, 0.0, 1.0);
    // Initial ECM joints
    Eigen::VectorXd goalJoints(mInitial.ECMPositionJoint);
    // Change in directions and joints
    Eigen::VectorXd changeJoints(4);
    Eigen::VectorXd changeDir(4);
    Eigen::Vector3d crossN;  // normal to direction of motion

    // - Direction 0 - left/right, movement in the XZ plane
    Eigen::Vector3d lr = Eigen::Vector3d(c[0], 0.0, c[2]).normalized();
    if (mInitial.Lr.isApprox(lr)) {
        changeDir[0] = 0.0;
    } else {
        changeDir[0] = -std::acos(mInitial.Lr.dot(lr));
        crossN = mInitial.Lr.cross(lr);
        if (normXZ.dot(crossN) < 0.0) {
            changeDir[0] = -changeDir[0];
        }
    }

    // - Direction 1 - up/down, movement in the YZ plane
    Eigen::Vector3d ud = Eigen::Vector3d(0.0, c[1], c[2]).normalized();
    if (mInitial.Ud.isApprox(ud)) {
        changeDir[1] = 0.0;
    } else {
        changeDir[1] = acos(mInitial.Ud.dot(ud));
        crossN = mInitial.Ud.cross(ud);
        if (normYZ.dot(crossN) < 0.0) {
            changeDir[1] = -changeDir[1];
        }
    }

    // - Direction 2 - in/out
    changeDir[2] = m_config.scale * (mInitial.C.norm() - c.norm());

    // - Direction 3 - cc/ccw, movement in the XY plane
    Eigen::Vector3d cw = Eigen::Vector3d(up[0], up[1], 0).normalized();
    if (mInitial.Cw.isApprox(cw)) {
        changeDir[3] = 0.0;
    } else {
        changeDir[3] = -acos(mInitial.Cw.dot(cw));
        crossN = mInitial.Cw.cross(cw);
        if (normXY.dot(crossN) < 0) {
            changeDir[3] = -changeDir[3];
        }
    }

    // adjusting movement for camera orientation (plus offset on camera roll if present between ECM and endoscope)
    double totalChangeJoint3 = changeDir[3] + mInitial.ECMPositionJoint[3] + m_config.camera_roll_offset;
    changeJoints[0] = changeDir[0] * cos(totalChangeJoint3) - changeDir[1] * sin(totalChangeJoint3);
    changeJoints[1] = changeDir[1] * cos(totalChangeJoint3) + changeDir[0] * sin(totalChangeJoint3);
    changeJoints[2] = changeDir[2];
    changeJoints[3] = changeDir[3];

    goalJoints += changeJoints;
    mECM_TwoHands.m_servo_jp.Goal() = goalJoints;
    mECM_TwoHands.servo_jp(mECM_TwoHands.m_servo_jp);

    /* --- Lock Orientation --- */
    Eigen::Matrix3d finalECMRot;

    Eigen::EulerAnglesZYXd finalEulerAngles(goalJoints[3], goalJoints[0], goalJoints[1]);
    const Eigen::Matrix3d currECMRot = finalEulerAngles.toRotationMatrix() * mInitial.ECMRotEuler.inverse();

    // Calculate new rotations of MTMs
    const Eigen::Matrix3d currMTMLRot = currECMRot.inverse() * mInitial.MTMLRot;
    const Eigen::Matrix3d currMTMRRot = currECMRot.inverse() * mInitial.MTMRRot;

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
            mMTML.lock_orientation(Eigen::Matrix3d(mMTML.m_measured_cp.Position().rotation()));
            mMTMR.body_servo_cf(wrench);
            mMTMR.use_gravity_compensation(true);
            mMTMR.lock_orientation(Eigen::Matrix3d(mMTMR.m_measured_cp.Position().rotation()));
        }
    }
}
