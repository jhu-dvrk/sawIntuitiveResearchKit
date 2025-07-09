/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Nicole Ortega
  Created on: 2016-01-21

  (C) Copyright 2016-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsTeleOperationECM_h
#define _mtsTeleOperationECM_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionJointSet.h>

#include <sawIntuitiveResearchKit/mtsStateMachine.h>
#include <sawIntuitiveResearchKit/teleop_ECM_configuration.h>

// always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class CISST_EXPORT mtsTeleOperationECM: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsTeleOperationECM(const std::string & componentName, const double periodInSeconds);
    mtsTeleOperationECM(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsTeleOperationECM();

    void Configure(const std::string & filename = "");
    virtual void Configure(const Json::Value & jsonConfig);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    void set_scale(const double & scale);

protected:

    virtual void Init(void);

    // Event Handler
    void arm_error_event_handler(const mtsMessage & message);
    void clutch_event_handler(const prmEventButton & button);
    void Clutch(const bool & clutch);

    dvrk::teleop_ECM_configuration m_config;

    // Functions for events
    struct {
        mtsFunctionWrite desired_state;
        mtsFunctionWrite current_state;
        mtsFunctionWrite following;
    } MessageEvents;
    mtsInterfaceProvided * mInterface;

    struct {
        mtsFunctionWrite scale;
    } ConfigurationEvents;

    void SetDesiredState(const std::string & state);
    void state_command(const std::string & command);

    void StateChanged(void);
    void RunAllStates(void);
    void TransitionDisabled(void); // checks for desired state
    void EnterSettingArmsState(void);
    void TransitionSettingArmsState(void);
    void EnterEnabled(void);
    void RunEnabled(void); // performs actual teleoperation
    void TransitionEnabled(void); // performs actual teleoperation


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
        mtsFunctionRead  measured_cp;
        mtsFunctionRead  setpoint_js;
        mtsFunctionWrite servo_jp;

        mtsFunctionRead  operating_state;
        mtsFunctionWrite state_command;

        prmPositionCartesianGet m_measured_cp;
        prmStateJoint m_setpoint_js;
        prmPositionJointSet m_servo_jp;
    } mECM;

    mtsStateTable * mConfigurationStateTable;

    bool m_clutched;

    mtsStateMachine mTeleopState;
    double mInStateTimer;

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

    bool m_following;
    void set_following(const bool following);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationECM);

#endif // _mtsTeleOperationECM_h
