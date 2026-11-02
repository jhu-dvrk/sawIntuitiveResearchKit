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

#ifndef _mtsTeleOperationECMOneHand_h
#define _mtsTeleOperationECMOneHand_h

#include <sawIntuitiveResearchKit/mtsTeleOperationECMBase.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>

class CISST_EXPORT mtsTeleOperationECMOneHand: public mtsTeleOperationECMBase
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsTeleOperationECMOneHand(const std::string & componentName, const double periodInSeconds);
    mtsTeleOperationECMOneHand(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsTeleOperationECMOneHand() override;

    void Startup(void) override;
    void set_scale(const double & scale) override;

protected:
    void Init(void) override;

    void Clutch(const bool & clutch) override;

    void RunAllStates(void) override;
    void TransitionSettingArmsState(void) override;
    void TransitionCheckingData(void) override;
    void EnterEnabled(void) override;
    void RunEnabled(void) override;

    struct {
        mtsFunctionRead  measured_cp;
        mtsFunctionRead  measured_cv;
        mtsFunctionWrite use_gravity_compensation;
        mtsFunctionWrite body_servo_cf;
        mtsFunctionRead  operating_state;
        mtsFunctionWrite state_command;

        prmPositionCartesianGet m_measured_cp;
        prmVelocityCartesianGet m_measured_cv;
    } mMTM;

    struct {
        mtsFunctionRead  setpoint_js;
        mtsFunctionWrite servo_jp;

        prmStateJoint m_setpoint_js;
        prmPositionJointSet m_servo_jp;
    } mECM_OneHand;

    bool m_mtm_measured_cv_available = false;

    struct TeleopState {
        Eigen::Vector3d C = Eigen::Vector3d(0.0, 0.0, 0.35);             // nominal center of virtual handlebar
        Eigen::Vector3d vectorLR = Eigen::Vector3d(1.0, 0.0, 0.0);       // nominal LR axis of virtual handlebar
        Eigen::Vector3d Up = Eigen::Vector3d(0.0, -1.0, 0.0);            // up direction
        Eigen::Vector3d Lr = Eigen::Vector3d(0.0, 0.0, 1.0);             // C projected on XZ plane
        Eigen::Vector3d Ud = Eigen::Vector3d(0.0, 0.0, 1.0);             // C projected on YZ plane
        Eigen::Vector3d Cw = Eigen::Vector3d(0.0, -1.0, 0.0);            // Up projected on XY plane
        Eigen::Vector3d MTMPosition = Eigen::Vector3d(0.0, 0.0, 0.0);    // initial MTM position
        Eigen::Matrix3d MTMRotation;                                            // initial MTM rotation
        Eigen::VectorXd ECMPositionJoint;                                       // initial ECM joint positions
    } mInitial;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationECMOneHand);

#endif // _mtsTeleOperationECMOneHand_h
