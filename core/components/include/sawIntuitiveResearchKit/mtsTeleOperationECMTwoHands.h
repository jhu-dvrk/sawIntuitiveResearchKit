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

#ifndef _mtsTeleOperationECMTwoHands_h
#define _mtsTeleOperationECMTwoHands_h

#include <sawIntuitiveResearchKit/mtsTeleOperationECMBase.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionJointSet.h>

class CISST_EXPORT mtsTeleOperationECMTwoHands: public mtsTeleOperationECMBase
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsTeleOperationECMTwoHands(const std::string & componentName, const double periodInSeconds);
    mtsTeleOperationECMTwoHands(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsTeleOperationECMTwoHands() override;

    void Startup(void) override;

protected:

    void Init(void) override;

    void Clutch(const bool & clutch) override;

    void RunAllStates(void) override;
    void EnterSettingArmsState(void) override;
    void TransitionSettingArmsState(void) override;
    void EnterCheckingData(void) override;
    void TransitionCheckingData(void) override;
    void EnterEnabled(void) override;
    void RunEnabled(void) override;

    struct {
        mtsFunctionRead  measured_cp;
        mtsFunctionRead  measured_cv;
        mtsFunctionWrite lock_orientation;
        mtsFunctionWrite body_servo_cf;
        mtsFunctionWrite body_set_cf_orientation_absolute;
        mtsFunctionWrite use_gravity_compensation;

        mtsFunctionRead  operating_state;
        mtsFunctionWrite state_command;

        prmPositionCartesianGet m_measured_cp;
        prmVelocityCartesianGet m_measured_cv;
    } mMTMR, mMTML;

    struct {
        mtsFunctionRead  setpoint_js;
        mtsFunctionWrite servo_jp;

        prmStateJoint m_setpoint_js;
        prmPositionJointSet m_servo_jp;
    } mECM_TwoHands;

    bool m_mtml_measured_cv_available = false;
    bool m_mtmr_measured_cv_available = false;

    struct TeleopState {
        vct3 C;     // center
        vct3 Up;    // up direction
        vct3 Lr;    // left/right movement, ie. c vector projected on the XZ plane
        vct3 Ud;    // up/down movement, ie. c vector projected on the YZ plane
        vct3 Cw;   // cw vector, ie. up vector projected on the XY plane
        double w;   // width of image
        double d;   // depth of R along C, depth of L is opposite
        vctMatRot3 MTMLRot; //initial rotation of MTML
        vctMatRot3 MTMRRot; //initial rotation of MTMR
        vctMatrixRotation3<double> ECMRotEuler; //initial rotation of ECM calc using Euler angles
        vctVec ECMPositionJoint;
    } mInitial;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationECMTwoHands);

#endif // _mtsTeleOperationECMTwoHands_h
