/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-15

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsIntuitiveResearchKitArm_h
#define _mtsIntuitiveResearchKitArm_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstRobot/robManipulator.h>
#include <cisstRobot/robLSPB.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmTypes.h>

class mtsIntuitiveResearchKitArm: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitArm(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitArm(const mtsTaskPeriodicConstructorArg & arg);
    virtual inline ~mtsIntuitiveResearchKitArm() {}

    void Configure(const std::string & filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

protected:

    /*! Initialization, including resizing data members and setting up
      cisst/SAW interfaces */
    virtual void Init(void);

    /*! Verify that the state transition is possible, initialize
      global variables for the desired state and finally set the
      state. */
    virtual void SetState(const mtsIntuitiveResearchKitArmTypes::RobotStateType & newState) = 0;
    virtual void SetRobotControlState(const std::string & state) = 0;
    /*! Convert enum to string using function provided by cisstDataGenerator. */
    void GetRobotControlState(std::string & state) const;

    /*! Get data from the PID level based on current state. */
    virtual void GetRobotData(void);

    /*! Homing procedure, power the robot and initial current and encoder calibration. */
    virtual void RunHomingPower(void);

    /*! Homing procedure, home all joints except last one using potentiometers as reference. */
    virtual void RunHomingCalibrateArm(void) = 0;

    /*! Cartesian state. */
    virtual void RunPositionCartesian(void);

    /*! Run method called for all states not handled in base class. */
    inline virtual void RunUserMode(void) {};

    /*! Wrapper to convert vector of joint values to prmPositionJointSet and send to PID */
    virtual void SetPositionJointLocal(const vctDoubleVec & newPosition);

    virtual void SetPositionCartesian(const prmPositionCartesianSet & newPosition);    
    
    /*! Event handler for PID tracking error. */
    virtual void EventHandlerTrackingError(void);

    /*! Configuration methods specific to derived classes. */
    virtual size_t NumberOfJoints(void) const = 0;
    virtual size_t NumberOfBrakes(void) const = 0;
    virtual bool UsePIDTrackingError(void) const = 0;
    inline virtual bool UsePotsForSafetyCheck(void) const {
        return true;
    }

    // Interface to PID component
    mtsInterfaceRequired * PIDInterface;
    struct {
        mtsFunctionWrite Enable;
        mtsFunctionRead  GetPositionJoint;
        mtsFunctionRead  GetPositionJointDesired;
        mtsFunctionWrite SetPositionJoint;
        mtsFunctionWrite SetCheckJointLimit;
        mtsFunctionWrite EnableTrackingError;
        mtsFunctionWrite SetTrackingErrorTolerance;
    } PID;

    // Interface to IO component
    mtsInterfaceRequired * IOInterface;
    struct InterfaceRobotTorque {
        mtsFunctionVoid  EnablePower;
        mtsFunctionVoid  DisablePower;
        mtsFunctionRead  GetActuatorAmpStatus;
        mtsFunctionRead  GetBrakeAmpStatus;
        mtsFunctionVoid  BiasEncoder;
        mtsFunctionWrite SetActuatorCurrent;
        mtsFunctionWrite UsePotsForSafetyCheck;
        mtsFunctionWrite SetPotsToEncodersTolerance;
        mtsFunctionVoid  BrakeRelease;
        mtsFunctionVoid  BrakeEngage;
    } RobotIO;

    // Main provided interface
    mtsInterfaceProvided * RobotInterface;

    // Functions for events
    struct {
        mtsFunctionWrite RobotStatus;
        mtsFunctionWrite RobotError;
    } MessageEvents;

    // Cache cartesian goal position
    prmPositionCartesianSet CartesianSetParam;
    bool IsCartesianGoalSet;

    prmPositionCartesianGet CartesianGetParam;
    vctFrm4x4 CartesianGet;
    prmPositionCartesianGet CartesianGetDesiredParam;
    vctFrm4x4 CartesianGetDesired;
    prmPositionJointGet JointGetParam;
    vctDoubleVec JointGet;
    vctDoubleVec JointGetDesired;
    prmPositionJointSet JointSetParam;
    vctDoubleVec JointSet;
    robManipulator Manipulator;

    vctFrm4x4 CartesianPositionFrm;
    mtsIntuitiveResearchKitArmTypes::RobotStateType RobotState;

    struct {
        robLSPB LSPB;
        vctDoubleVec Velocity;
        vctDoubleVec Acceleration;
        vctDoubleVec Start;
        vctDoubleVec Goal;
        vctDoubleVec GoalError;
        vctDoubleVec GoalTolerance;
        double EndTime;
    } JointTrajectory;

    // Home Action
    double HomingTimer;
    bool HomingPowerRequested;
    bool HomingCalibrateArmStarted;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitArm);

#endif // _mtsIntuitiveResearchKitArm_h
