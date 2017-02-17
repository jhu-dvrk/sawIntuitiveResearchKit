/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-03-06

  (C) Copyright 2013-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsTeleOperationPSM_h
#define _mtsTeleOperationPSM_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>

#include <sawIntuitiveResearchKit/mtsStateMachine.h>

// always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class CISST_EXPORT mtsTeleOperationPSM: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsTeleOperationPSM(const std::string & componentName, const double periodInSeconds);
    mtsTeleOperationPSM(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsTeleOperationPSM();

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    void SetScale(const double & scale);
    void SetRegistrationRotation(const vctMatRot3 & rotation);
    void LockRotation(const bool & lock);
    void LockTranslation(const bool & lock);

protected:

    void Init(void);

    // Event Handler
    void MTMErrorEventHandler(const mtsMessage & message);
    void PSMErrorEventHandler(const mtsMessage & message);

    void ClutchEventHandler(const prmEventButton & button);
    void Clutch(const bool & clutch);

    // Functions for events
    struct {
        mtsFunctionWrite DesiredState;
        mtsFunctionWrite CurrentState;
    } MessageEvents;
    mtsInterfaceProvided * mInterface;

    struct {
        mtsFunctionWrite Scale;
        mtsFunctionWrite RotationLocked;
        mtsFunctionWrite TranslationLocked;
    } ConfigurationEvents;

    void SetDesiredState(const std::string & state);

    void StateChanged(void);
    void RunAllStates(void); // this should happen for all states
    void TransitionDisabled(void); // checks for desired state
    void EnterSettingPSMState(void); // request state and set timer
    void TransitionSettingPSMState(void); // check current state and timer
    void EnterSettingMTMState(void);
    void TransitionSettingMTMState(void);
    void EnterAligningMTM(void);
    void TransitionAligningMTM(void);
    void EnterEnabled(void); // called when enabling, save initial positions of master and slave
    void RunEnabled(void); // performs actual teleoperation
    void TransitionEnabled(void); // performs actual teleoperation

    class RobotMTM {
    public:
        mtsFunctionRead  GetPositionCartesian;
        mtsFunctionRead  GetPositionCartesianDesired;
        mtsFunctionWrite SetPositionGoalCartesian;
        mtsFunctionRead  GetGripperPosition;

        mtsFunctionRead  GetRobotControlState;
        mtsFunctionWrite SetRobotControlState;
        mtsFunctionWrite LockOrientation;
        mtsFunctionVoid  UnlockOrientation;
        mtsFunctionWrite SetWrenchBody;
        mtsFunctionWrite SetGravityCompensation;

        prmPositionCartesianGet PositionCartesianCurrent;
        prmPositionCartesianGet PositionCartesianDesired;
        prmPositionCartesianSet PositionCartesianSet;
        vctFrm4x4 CartesianPrevious;
    };
    RobotMTM * mMTM;

    class RobotPSM {
    public:
        mtsFunctionRead  GetPositionCartesian;
        mtsFunctionWrite SetPositionCartesian;
        mtsFunctionWrite SetJawPosition;

        mtsFunctionRead  GetRobotControlState;
        mtsFunctionWrite SetRobotControlState;

        prmPositionCartesianGet PositionCartesianCurrent;
        prmPositionCartesianSet PositionCartesianSet;
        vctFrm4x4 CartesianPrevious;
    };
    RobotPSM * mPSM;

    double mScale;
    vctMatRot3 mRegistrationRotation;

    bool mIsClutched;
    bool mRotationLocked;
    bool mTranslationLocked;
    vctMatRot3 mMTMClutchedOrientation;
    mtsStateTable * mConfigurationStateTable;

    mtsStateMachine mTeleopState;
    double mInStateTimer;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationPSM);

#endif // _mtsTeleOperationPSM_h
