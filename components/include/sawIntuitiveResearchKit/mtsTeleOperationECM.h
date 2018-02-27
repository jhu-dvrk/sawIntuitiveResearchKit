/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-01-21

  (C) Copyright 2016-2018 Johns Hopkins University (JHU), All Rights Reserved.

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
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    void SetScale(const double & scale);
    void SetRegistrationRotation(const vctMatRot3 & rotation);

protected:

    virtual void Init(void);

    // Event Handler
    void MTMLErrorEventHandler(const mtsMessage & message);
    void MTMRErrorEventHandler(const mtsMessage & message);
    void ECMErrorEventHandler(const mtsMessage & message);

    void ClutchEventHandler(const prmEventButton & button);
    void Clutch(const bool & clutch);

    // Functions for events
    struct {
        mtsFunctionWrite DesiredState;
        mtsFunctionWrite CurrentState;
        mtsFunctionWrite Following;
    } MessageEvents;
    mtsInterfaceProvided * mInterface;

    struct {
        mtsFunctionWrite Scale;
    } ConfigurationEvents;

    void SetDesiredState(const std::string & state);

    void StateChanged(void);
    void RunAllStates(void);
    void TransitionDisabled(void); // checks for desired state
    void EnterSettingArmsState(void);
    void TransitionSettingArmsState(void);
    void EnterEnabled(void);
    void RunEnabled(void); // performs actual teleoperation
    void TransitionEnabled(void); // performs actual teleoperation


    class RobotMTM {
    public:
        mtsFunctionRead  GetPositionCartesian;
        // mtsFunctionRead  GetPositionCartesianDesired;
        mtsFunctionRead  GetVelocityCartesian;
        mtsFunctionWrite SetPositionCartesian;
        mtsFunctionRead  GetCurrentState;
        mtsFunctionRead  GetDesiredState;
        mtsFunctionWrite SetDesiredState;
        mtsFunctionWrite LockOrientation;
        mtsFunctionVoid  UnlockOrientation;
        mtsFunctionWrite SetWrenchBody;
        mtsFunctionWrite SetWrenchBodyOrientationAbsolute;
        mtsFunctionWrite SetGravityCompensation;

        prmPositionCartesianGet PositionCartesianCurrent;
        prmVelocityCartesianGet VelocityCartesianCurrent;
        prmPositionCartesianSet PositionCartesianSet;
    };
    RobotMTM * mMTML;
    RobotMTM * mMTMR;

    class RobotECM {
    public:
        mtsFunctionRead  GetPositionCartesian;
        mtsFunctionRead  GetStateJointDesired;
        mtsFunctionWrite SetPositionGoalJoint;
        mtsFunctionRead  GetCurrentState;
        mtsFunctionRead  GetDesiredState;
        mtsFunctionWrite SetDesiredState;

        prmPositionCartesianGet PositionCartesianCurrent;
        prmStateJoint StateJointDesired;
        prmPositionJointSet PositionJointSet;
    };
    RobotECM * mECM;

    double mScale;
    vctMatRot3 mRegistrationRotation;
    mtsStateTable * mConfigurationStateTable;

    bool mIsClutched;

    mtsStateMachine mTeleopState;
    double mInStateTimer;

    struct TeleopState {
        double dLR; // distance
        vct3 C;     // center
        vct3 N;     // normal to image
        vct3 Up;    // up direction
        vct3 Lr;    // left/right movement, ie. c vector projected on the XZ plane
        vct3 Ud;    // up/down movement, ie. c vector projected on the YZ plane
        vct3 Cw;   // cw vector, ie. up vector projected on the XY plane
        double w;   // width of image
        double d;   // depth of R along C, depth of L is opposite
        vctMatRot3 MTMLRot; //initial rotation of MTML
        vctMatRot3 MTMRRot; //initial rotation of MTMR
        vctMatRot3 ECMRot; //initial rotation of ECM
        vctMatrixRotation3<double> ECMRotEuler; //initial rotation of ECM frame calculated using Euler angles
        
        vctVec ECMPositionJoint;
    } mInitial;

    bool mIsFollowing;
    void SetFollowing(const bool following);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationECM);

#endif // _mtsTeleOperationECM_h
