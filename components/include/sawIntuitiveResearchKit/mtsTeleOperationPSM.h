/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-03-06

  (C) Copyright 2013-2019 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <cisstParameterTypes/prmPositionJointSet.h>

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
    virtual void Configure(const Json::Value & jsonConfig);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    void SetScale(const double & scale);
    void SetRegistrationRotation(const vctMatRot3 & rotation);
    void LockRotation(const bool & lock);
    void LockTranslation(const bool & lock);
    void SetAlignMTM(const bool & alignMTM);

protected:

    virtual void Init(void);

    // Event Handler
    void MTMErrorEventHandler(const mtsMessage & message);
    void PSMErrorEventHandler(const mtsMessage & message);

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
        mtsFunctionWrite RotationLocked;
        mtsFunctionWrite TranslationLocked;
        mtsFunctionWrite AlignMTM;
    } ConfigurationEvents;

    void SetDesiredState(const std::string & state);

    void StateChanged(void);
    void RunAllStates(void); // this should happen for all states
    void TransitionDisabled(void); // checks for desired state
    void EnterSettingArmsState(void);
    void TransitionSettingArmsState(void);
    void EnterAligningMTM(void);
    void RunAligningMTM(void);
    void TransitionAligningMTM(void);
    void EnterEnabled(void); // called when enabling, save initial positions of master and slave
    void RunEnabled(void); // performs actual teleoperation
    void TransitionEnabled(void); // performs actual teleoperation

    struct {
        mtsFunctionRead  GetPositionCartesian;
        mtsFunctionRead  GetPositionCartesianDesired;
        mtsFunctionWrite SetPositionGoalCartesian;
        mtsFunctionRead  GetStateGripper;
        mtsFunctionWrite LockOrientation;
        mtsFunctionVoid  UnlockOrientation;
        mtsFunctionWrite SetWrenchBody;
        mtsFunctionWrite SetGravityCompensation;
        mtsFunctionRead  GetCurrentState;
        mtsFunctionRead  GetDesiredState;
        mtsFunctionWrite SetDesiredState;

        prmStateJoint StateGripper;
        prmPositionCartesianGet PositionCartesianCurrent;
        prmPositionCartesianGet PositionCartesianDesired;
        prmPositionCartesianSet PositionCartesianSet;
        vctFrm4x4 CartesianInitial;
    } mMTM;

    struct {
        mtsFunctionRead  GetPositionCartesian;
        mtsFunctionWrite SetPositionCartesian;
        mtsFunctionVoid Freeze;
        mtsFunctionRead GetStateJaw;
        mtsFunctionRead GetConfigurationJaw;
        mtsFunctionWrite SetPositionJaw;

        mtsFunctionRead  GetCurrentState;
        mtsFunctionRead  GetDesiredState;
        mtsFunctionWrite SetDesiredState;

        prmStateJoint StateJaw;
        prmConfigurationJoint ConfigurationJaw;
        prmPositionCartesianGet PositionCartesianCurrent;
        prmPositionCartesianSet PositionCartesianSet;
        prmPositionJointSet     PositionJointSet;
        vctFrm4x4 CartesianInitial;
    } mPSM;

    struct {
        mtsFunctionRead  GetPositionCartesian;
        prmPositionCartesianGet PositionCartesianCurrent;
        vctFrm4x4 CartesianInitial;
    } mBaseFrame;

    double mScale = 0.2;
    vctMatRot3 mRegistrationRotation; // optional registration between PSM and MTM orientation
    vctMatRot3 mAlignOffset, mAlignOffsetInitial; // rotation offset between MTM and PSM when tele-operation goes in follow mode

    // initial offset in jaw (PSM) space when teleop starts
    double mJawOffset;

    // conversion from gripper (MTM) to jaw (PSM)
    // j = s * g + o
    // g = (j - o) / s
    struct {
        double Scale;
        double Offset;
        double PositionMin;
    } mGripperToJaw;

    double virtual GripperToJaw(const double & gripperAngle) const;
    double virtual JawToGripper(const double & jawAngle) const;
    void virtual UpdateGripperToJawConfiguration(void);

    bool mIgnoreJaw = false; // flag to tele-op in cartesian position only, don't need or drive the PSM jaws
    int mGripperJawTransitions;
    bool mGripperJawMatchingPrevious;
    bool mIsClutched = false;
    bool mRotationLocked = false;
    bool mTranslationLocked = false;
    bool mAlignMTM = true; // default on da Vinci

    vctMatRot3 mMTMClutchedOrientation;
    mtsStateTable * mConfigurationStateTable;

    mtsStateMachine mTeleopState;
    double mInStateTimer;
    double mTimeSinceLastAlign;

    bool mIsFollowing;
    void SetFollowing(const bool following);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationPSM);

#endif // _mtsTeleOperationPSM_h
