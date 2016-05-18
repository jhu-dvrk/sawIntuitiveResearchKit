/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-01-21

  (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.

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

#include <sawIntuitiveResearchKit/mtsStateMachine.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationECMTypes.h>

// always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class CISST_EXPORT mtsTeleOperationECM: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsTeleOperationECM(const std::string & componentName, const double periodInSeconds);
    mtsTeleOperationECM(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsTeleOperationECM() {}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    void SetScale(const double & scale);
    void SetRegistrationRotation(const vctMatRot3 & rotation);

private:

    void Init(void);

    // Event Handler
    void MTMLErrorEventHandler(const std::string & message);
    void MTMRErrorEventHandler(const std::string & message);
    void ECMErrorEventHandler(const std::string & message);

    void ClutchEventHandler(const prmEventButton & button);

    // Functions for events
    struct {
        mtsFunctionWrite Status;
        mtsFunctionWrite Warning;
        mtsFunctionWrite Error;
        mtsFunctionWrite DesiredState;
        mtsFunctionWrite CurrentState;
    } MessageEvents;

    struct {
        mtsFunctionWrite Scale;
    } ConfigurationEvents;

    void SetDesiredState(const std::string & state);

protected:

    void StateChanged(void);
    void RunAll(void);
    void TransitionDisabled(void); // checks for desired state
    void EnterSettingECMState(void); // request state and set timer
    void TransitionSettingECMState(void); // check current state and timer
    void EnterSettingMTMsState(void);
    void TransitionSettingMTMsState(void);
    void EnterEnabled(void);
    void RunEnabled(void); // performs actual teleoperation
    void TransitionEnabled(void); // performs actual teleoperation


    class RobotMTM {
    public:
        mtsFunctionRead  GetPositionCartesian;
        mtsFunctionRead  GetPositionCartesianDesired;
        mtsFunctionRead  GetVelocityCartesian;
        mtsFunctionWrite SetPositionCartesian;
        mtsFunctionRead  GetRobotControlState;
        mtsFunctionWrite SetRobotControlState;
        mtsFunctionWrite LockOrientation;
        mtsFunctionVoid  UnlockOrientation;
        mtsFunctionWrite SetWrenchBody;
        mtsFunctionWrite SetWrenchBodyOrientationAbsolute;
        mtsFunctionWrite SetGravityCompensation;

        vctFrm3 PositionCartesianInitial;
        prmPositionCartesianGet PositionCartesianCurrent;
        prmPositionCartesianGet PositionCartesianDesired;
        prmVelocityCartesianGet VelocityCartesianCurrent;
        prmPositionCartesianSet PositionCartesianSet;
    };
    RobotMTM mMTML, mMTMR;

    class RobotECM {
    public:
        mtsFunctionRead  GetPositionCartesian;
        mtsFunctionRead  GetPositionCartesianDesired;
        mtsFunctionWrite SetPositionCartesian;
        mtsFunctionRead  GetRobotControlState;
        mtsFunctionWrite SetRobotControlState;

        vctFrm3 PositionCartesianInitial;
        prmPositionCartesianGet PositionCartesianCurrent;
        prmPositionCartesianGet PositionCartesianDesired;
        prmPositionCartesianSet PositionCartesianSet;
    };
    RobotECM mECM;

 private:
    double mScale;
    vctMatRot3 mRegistrationRotation;
    mtsStateTable * mConfigurationStateTable;

    bool mIsClutched;

    mtsStateMachine<mtsTeleOperationECMTypes::StateType> mTeleopState;
    double mInStateTimer;

    double mDistanceLR; // distance between left and right
    double mDistanceL, mDistanceR; // distances to RCM
    vct3 mInitialMTMsPosition;
    double mInitialMTMsAngle;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationECM);

#endif // _mtsTeleOperationECM_h
