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

#ifndef _mtsTeleOperationBimanual_h
#define _mtsTeleOperationBimanual_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstRobot/robManipulator.h>

class mtsTeleOperationBimanual: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsTeleOperationBimanual(const std::string & componentName, const double periodInSeconds);
    mtsTeleOperationBimanual(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsTeleOperationBimanual() {}

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
    void CameraClutchEventHandler(const prmEventButton & button);
    void OperatorPresentEventHandler(const prmEventButton & button);

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

    void Enable(const bool & enable);

    void SetMasterControlState(void);

protected:

    class RobotMaster {
    public:
        mtsFunctionRead GetPositionCartesian;
        mtsFunctionWrite SetRobotControlState;
        prmPositionCartesianGet PositionCartesianCurrent;
    };
    RobotMaster mMasterLeft, mMasterRight;

    class RobotSlave {
    public:
        mtsFunctionRead GetPositionCartesian;
        mtsFunctionWrite SetPositionCartesian;
        mtsFunctionWrite SetRobotControlState;

        prmPositionCartesianGet PositionCartesianCurrent;
        prmPositionCartesianSet PositionCartesianDesired;
        vctFrm4x4 CartesianPrevious;
        bool IsManipClutched;
        bool IsSUJClutched;
    };
    RobotSlave mSlave;

private:

    double mScale;
    vctMatRot3 mRegistrationRotation;

    bool mIsOperatorPresent;
    bool mIsEnabled;

    mtsStateTable * mConfigurationStateTable;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationBimanual);

#endif // _mtsTeleOperationBimanual_h
