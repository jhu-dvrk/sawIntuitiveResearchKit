/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-03-06

  (C) Copyright 2013-2016 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <sawIntuitiveResearchKit/mtsTeleOperationPSMTypes.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

/**
 * @brief  teleoperation component
 *
 *    position: translation + rotation (vctFrm4x4)
 *    translation: 3D x,y,z (vct3)
 *    rotation: 3x3 rotation (vctMatRot3)
 *
 * \todo
 *
 */
class CISST_EXPORT mtsTeleOperationPSM: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsTeleOperationPSM(const std::string & componentName, const double periodInSeconds);
    mtsTeleOperationPSM(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsTeleOperationPSM(){}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    void SetScale(const double & scale);
    void SetRegistrationRotation(const vctMatRot3 & rotation);
    void LockRotation(const bool & lock);
    void LockTranslation(const bool & lock);

private:

    void Init(void);

    // Event Handler
    void MasterErrorEventHandler(const std::string & message);
    void SlaveErrorEventHandler(const std::string & message);

    void SlaveClutchEventHandler(const prmEventButton & button);
    //    void StartAlignMaster(void);

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
        mtsFunctionWrite RotationLocked;
        mtsFunctionWrite TranslationLocked;
    } ConfigurationEvents;

    void SetDesiredState(const std::string & state);

protected:

    void StateChanged(void);
    void RunAll(void); // this should happen for all states
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

    class RobotMaster {
    public:
        mtsFunctionRead  GetPositionCartesian;
        mtsFunctionWrite SetPositionCartesian;
        mtsFunctionWrite SetPositionGoalCartesian;
        mtsFunctionRead  GetGripperPosition;

        mtsFunctionRead  GetRobotControlState;
        mtsFunctionWrite SetRobotControlState;
        mtsFunctionWrite LockOrientation;
        mtsFunctionVoid  UnlockOrientation;
        mtsFunctionWrite SetWrenchBody;

        prmPositionCartesianGet PositionCartesianCurrent;
        prmPositionCartesianSet PositionCartesianDesired;
        vctFrm4x4 CartesianPrevious;
    };
    RobotMaster mMaster;

    class RobotSlave {
    public:
        mtsFunctionRead  GetPositionCartesian;
        mtsFunctionWrite SetPositionCartesian;
        mtsFunctionWrite SetJawPosition;

        mtsFunctionRead  GetRobotControlState;
        mtsFunctionWrite SetRobotControlState;

        prmPositionCartesianGet PositionCartesianCurrent;
        prmPositionCartesianSet PositionCartesianDesired;
        vctFrm4x4 CartesianPrevious;
    };
    RobotSlave mSlave;

private:
    double mScale;
    vctMatRot3 mRegistrationRotation;
    vctFrm3 mOffset;
    vct3 mMasterLockTranslation;

    bool mIsClutched;
    bool mRotationLocked;
    bool mTranslationLocked;
    vctMatRot3 mMasterClutchedOrientation;
    mtsStateTable * mConfigurationStateTable;

    mtsStateMachine<mtsTeleOperationPSMTypes::StateType> mTeleopState;
    double mInStateTimer;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationPSM);

#endif // _mtsTeleOperationPSM_h
