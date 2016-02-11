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
#include <cisstParameterTypes/prmPositionCartesianSet.h>

#include <sawIntuitiveResearchKit/mtsStateMachine.h>
#include <sawIntuitiveResearchKit/mtsTeleOperationECMTypes.h>

class mtsTeleOperationECM: public mtsTaskPeriodic
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
    void MasterLeftErrorEventHandler(const std::string & message);
    void MasterRightErrorEventHandler(const std::string & message);
    void SlaveErrorEventHandler(const std::string & message);

    void SlaveClutchEventHandler(const prmEventButton & button);

    // Functions for events
    struct {
        mtsFunctionWrite Status;
        mtsFunctionWrite Warning;
        mtsFunctionWrite Error;
        mtsFunctionWrite Enabled;
    } MessageEvents;

    struct {
        mtsFunctionWrite Scale;
    } ConfigurationEvents;

    /*! Convenience method to set desired state to ENABLED/DISABLED */
    void Enable(const bool & enable);

protected:

    void StateChanged(void);
    void TransitionDisabled(void); // checks for desired state
    void EnterSettingECMState(void); // request state and set timer
    void TransitionSettingECMState(void); // check current state and timer
    void EnterSettingMTMsState(void);
    void TransitionSettingMTMsState(void);
    void RunEnabled(void); // performs actual teleoperation
    void TransitionEnabled(void); // performs actual teleoperation
    void EnterEnabledDisabled(void); // mostly send event Enable(true/false)
    

    class RobotMaster {
    public:
        mtsFunctionRead GetPositionCartesian;
        mtsFunctionWrite SetPositionCartesian;
        mtsFunctionRead GetRobotControlState;
        mtsFunctionWrite SetRobotControlState;
        mtsFunctionWrite LockOrientation;
        mtsFunctionVoid UnlockOrientation;
        mtsFunctionWrite SetWrenchBody;
        mtsFunctionWrite SetWrenchBodyOrientationAbsolute;

        vctFrm3 PositionCartesianInitial;
        prmPositionCartesianGet PositionCartesianCurrent;
        prmPositionCartesianSet PositionCartesianDesired;
    };
    RobotMaster mMasterLeft, mMasterRight;

    class RobotSlave {
    public:
        mtsFunctionRead GetPositionCartesian;
        mtsFunctionWrite SetPositionCartesian;
        mtsFunctionRead GetRobotControlState;
        mtsFunctionWrite SetRobotControlState;

        vctFrm3 PositionCartesianInitial;
        prmPositionCartesianGet PositionCartesianCurrent;
        prmPositionCartesianSet PositionCartesianDesired;

        bool IsManipClutched;
        bool IsSUJClutched;
    };
    RobotSlave mSlave;

    double mScale;
    vctMatRot3 mRegistrationRotation;
    mtsStateTable * mConfigurationStateTable;

    mtsStateMachine<mtsTeleOperationECMTypes::StateType,
                    mtsTeleOperationECMTypes> mTeleopState;

    double mSetStateTimer;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationECM);

#endif // _mtsTeleOperationECM_h
