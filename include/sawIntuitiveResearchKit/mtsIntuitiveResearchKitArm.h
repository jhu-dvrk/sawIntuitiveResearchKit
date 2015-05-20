/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-15

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmVelocityJointGet.h>

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
    bool CurrentStateIs(const mtsIntuitiveResearchKitArmTypes::RobotStateType & state);

    /*! Get data from the PID level based on current state. */
    virtual void GetRobotData(void);

    /*! Homing procedure, power the robot and initial current and encoder calibration. */
    virtual void RunHomingPower(void);

    /*! Homing procedure, home all joints except last one using potentiometers as reference. */
    virtual void RunHomingCalibrateArm(void) = 0;

    /*! Cartesian state. */
    virtual void RunPositionJoint(void);
    virtual void RunPositionGoalJoint(void);
    virtual void RunPositionCartesian(void);
    virtual void RunPositionGoalCartesian(void);

    /*! Run method called for all states not handled in base class. */
    inline virtual void RunArmSpecific(void) {};

    /*! Wrapper to convert vector of joint values to prmPositionJointSet and send to PID */
    void SetPositionJointLocal(const vctDoubleVec & newPosition);

    /*! Methods used for commands */
    virtual void SetPositionJoint(const prmPositionJointSet & newPosition);
    virtual void SetPositionGoalJoint(const prmPositionJointSet & newPosition);
    virtual void SetPositionCartesian(const prmPositionCartesianSet & newPosition);
    virtual void SetPositionGoalCartesian(const prmPositionCartesianSet & newPosition);

    /*! Event handler for PID joint limit. */
    virtual void JointLimitEventHandler(const vctBoolVec & flags);

    /*! Event handler for PID errors. */
    void ErrorEventHandler(const std::string & message);

    /*! Configuration methods specific to derived classes. */
    virtual size_t NumberOfAxes(void) const = 0;           // used IO: ECM 4, PSM 7, MTM 8
    virtual size_t NumberOfJoints(void) const = 0;         // used PID: ECM 4, PSM 7, MTM 7
    virtual size_t NumberOfJointsKinematics(void) const = 0; // used for inverse kinematics: ECM 4, PSM 6, MTM 7
    virtual size_t NumberOfBrakes(void) const = 0;         // ECM 3, PSM 0, MTM 0

    virtual bool UsePIDTrackingError(void) const = 0;      // ECM true, PSM false, MTM false
    inline virtual bool UsePotsForSafetyCheck(void) const {
        return true;
    }

    virtual robManipulator::Errno InverseKinematics(vctDoubleVec & jointSet,
                                                    const vctFrm4x4 & cartesianGoal) = 0;

    // Interface to PID component
    mtsInterfaceRequired * PIDInterface;
    struct {
        mtsFunctionWrite Enable;
        mtsFunctionRead  GetPositionJoint;
        mtsFunctionRead  GetPositionJointDesired;
        mtsFunctionWrite SetPositionJoint;
        mtsFunctionWrite SetCheckJointLimit;
        mtsFunctionRead  GetVelocityJoint;
        mtsFunctionWrite EnableTorqueMode;
        mtsFunctionWrite SetTorqueJoint;
        mtsFunctionWrite SetTorqueOffset;
        mtsFunctionWrite EnableTrackingError;
        mtsFunctionWrite SetTrackingErrorTolerance;
    } PID;

    // Interface to IO component
    mtsInterfaceRequired * IOInterface;
    struct InterfaceRobotTorque {
        mtsFunctionRead  GetSerialNumber;
        mtsFunctionVoid  EnablePower;
        mtsFunctionVoid  DisablePower;
        mtsFunctionRead  GetActuatorAmpStatus;
        mtsFunctionRead  GetBrakeAmpStatus;
        mtsFunctionVoid  BiasEncoder;
        mtsFunctionWrite ResetSingleEncoder;
        mtsFunctionRead  GetAnalogInputPosSI;
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
        mtsFunctionWrite Status;
        mtsFunctionWrite Warning;
        mtsFunctionWrite Error;
        mtsFunctionWrite RobotState;
    } MessageEvents;

    // Cache cartesian goal position
    prmPositionCartesianSet CartesianSetParam;
    bool IsGoalSet;

    prmPositionCartesianGet CartesianGetParam;
    vctFrm4x4 CartesianGet;
    prmPositionCartesianGet CartesianGetDesiredParam;
    vctFrm4x4 CartesianGetDesired;
    prmPositionJointGet JointGetParam;
    vctDoubleVec JointGet;
    vctDoubleVec JointGetDesired;
    prmPositionJointSet JointSetParam;
    vctDoubleVec JointSet;
    //! robot current joint velocity
    prmVelocityJointGet JointVelocityGetParam;
    vctDoubleVec JointVelocityGet;

    // Velocities
    prmPositionCartesianGet CartesianGetPreviousParam;
    prmVelocityCartesianGet CartesianVelocityGetParam;

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
        double EndTime; // time should be set to 0.0 if there is no on-going trajectory
        mtsFunctionWrite GoalReachedEvent; // sends true if goal reached, false otherwise
    } JointTrajectory;

    vctDoubleVec PotsToEncodersTolerance;

    // Home Action
    double HomingTimer;
    bool HomingPowerRequested;
    bool HomingCalibrateArmStarted;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitArm);

#endif // _mtsIntuitiveResearchKitArm_h
