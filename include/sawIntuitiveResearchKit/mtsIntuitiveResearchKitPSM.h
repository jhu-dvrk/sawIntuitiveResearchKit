/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Anton Deguet
  Created on: 2013-05-15

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitPSM_h
#define _mtsIntuitiveResearchKitPSM_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstRobot/robManipulator.h>
#include <cisstOSAbstraction/osaStopwatch.h>





/**
 * \todo Add Enable Power
 * \todo Add HOME
 */


class mtsIntuitiveResearchKitPSM: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitPSM(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitPSM(const mtsTaskPeriodicConstructorArg & arg);
    inline ~mtsIntuitiveResearchKitPSM() {}

    void Configure(const std::string & filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

protected:

    enum PSM_STATE {
        STATE_HOME = 0,
        STATE_TELEOP,
        STATE_ADAPTER,
        STATE_TOOL,
        STATE_IDLE
    };

    void Init(void);
    void EventHandlerHome(void);
    void EventHandlerTeleop(void);
    void EventHandlerAdapter(const prmEventButton & button);
    void EventHandlerTool(const prmEventButton & button);
    void EventHandlerManipClutch(const prmEventButton & button);
    void EventHandlerSUJClutch(const prmEventButton & button);

    void SetPositionCartesian(const prmPositionCartesianSet & newPosition);
    void SetRobotControlState(const mtsStdString & state);

    struct {
        mtsFunctionWrite Enable;
        mtsFunctionRead GetPositionJoint;
        mtsFunctionWrite SetPositionJoint;
        mtsFunctionWrite SetIsCheckJointLimit;
    } PID;

    // Required interface
    struct InterfaceRobotTorque {
        //! Enable Robot Power
        mtsFunctionVoid EnablePower;
        mtsFunctionVoid DisablePower;
        mtsFunctionVoid BiasEncoder;
    } RobotIO;

    struct {
        mtsFunctionRead GetButton;
        bool IsPresent;
    } Adapter;

    struct {
        mtsFunctionRead GetButton;
        bool IsPresent;
    } Tool;

    // Functions for events
    struct {
        mtsFunctionWrite RobotStatusMsg;
        mtsFunctionWrite RobotErrorMsg;
    } EventTriggers;

    prmPositionCartesianGet CartesianCurrent;
    vctFrm4x4 CartesianPrevious;
    prmPositionJointGet JointCurrent;
    prmPositionJointSet JointDesired;
    robManipulator Manipulator;

    PSM_STATE RobotCurrentState;

    // Home Action
    vctDoubleVec HomeJointSet;
    bool IsHomed;

    // Adapter engage
    osaStopwatch AdapterStopwatch;
    vctDoubleVec AdapterJointSet;
    bool IsAdapterEngaged;

    // Tool engage
    osaStopwatch ToolStopwatch;
    vctDoubleVec ToolJointSet;
    bool IsToolEngaged;

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitPSM);

#endif // _mtsIntuitiveResearchKitPSM_h
