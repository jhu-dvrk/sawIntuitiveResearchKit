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


#ifndef _mtsIntuitiveResearchKitMTM_h
#define _mtsIntuitiveResearchKitMTM_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstRobot/robManipulator.h>
#include <cisstRobot/robQuintic.h>

class mtsIntuitiveResearchKitMTM: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:

    static const size_t NumberOfJoints = 7;
    static const size_t RollIndex = 6;

    mtsIntuitiveResearchKitMTM(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitMTM(const mtsTaskPeriodicConstructorArg & arg);
    inline ~mtsIntuitiveResearchKitMTM() {}

    void Configure(const std::string & filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

protected:

    enum RobotStateType {
        MTM_UNINITIALIZED, /*! State when constructed */
        MTM_HOMING_POWERING, /*! Turn power on, calibrate encoders and current */
        MTM_HOMING_CALIBRATING_ARM, /*! Calibrate using pots and move to zero position for all joints except last one */
        MTM_HOMING_CALIBRATING_ROLL, /*! Calibrate last joint using hardware limit and tracking errors. */
        MTM_READY,
        MTM_POSITION_CARTESIAN
    };

    void Init(void);

    /*! Get data from the PID level based on current state. */
    void GetRobotData(void);

    /*! Verify that the state transition is possible, initialize global
      variables for the desired state and finally set the state. */
    void SetState(const RobotStateType & newState);

    /*! Homing procedure, will check the homing state and call the required method. */
    void RunHoming(void);

    /*! Homing procedure, power the robot and initial current and encoder calibration. */
    void RunHomingPower(void);

    /*! Homing procedure, home all joints except last one using potentiometers as reference. */
    void RunHomingCalibrateArm(void);

    /*! Homing procedure, calibrate last joint based on hardware limits. */
    void RunHomingCalibrateRoll(void);

    /*! Ready state. */
    void RunReady(void);

    /*! Wrapper to convert vector of 7 values to prmPositionJointSet and send to PID */
    void SetPositionJoint(const vctDoubleVec & newPosition);

    void SetPositionCartesian(const prmPositionCartesianSet & newPosition);
    void SetWrench(const prmForceCartesianSet & newForce);
    void SetRobotControlState(const std::string & state);

    struct {
        mtsFunctionWrite Enable;
        mtsFunctionRead GetPositionJoint;
        mtsFunctionWrite SetPositionJoint;
        mtsFunctionWrite SetTorqueJoint;
        mtsFunctionWrite SetCheckJointLimit;
    } PID;

    // Required interface
    struct InterfaceRobotTorque {
        //! Enable Robot Power
        mtsFunctionVoid EnablePower;
        mtsFunctionVoid DisablePower;
        mtsFunctionRead GetAmpStatus;
        mtsFunctionVoid BiasEncoder;
        mtsFunctionWrite BiasCurrent;
        mtsFunctionWrite SetMotorCurrent;
        mtsFunctionWrite ResetSingleEncoder;
        mtsFunctionRead GetAnalogInputPosSI;
    } RobotIO;

    // Functions for events
    struct {
        mtsFunctionWrite RobotStatusMsg;
        mtsFunctionWrite RobotErrorMsg;
    } EventTriggers;

    prmPositionCartesianGet CartesianCurrentParam;
    vctFrm4x4 CartesianCurrent;
    vctFrm4x4 CartesianPrevious;
    prmPositionJointGet JointCurrentParam;
    vctDoubleVec JointCurrent;
    prmPositionJointSet JointDesiredParam;
    vctDoubleVec JointDesired;
    prmForceTorqueJointSet TorqueDesired;
    robManipulator Manipulator;
    vctDoubleVec AnalogInputPosSI;
    double GripperPosition;

    RobotStateType RobotState;

    struct {
        robQuintic Quintic;
        vctDoubleVec Velocity;
        vctDoubleVec Acceleration;
        vctDoubleVec Goal;
        vctDoubleVec GoalError;
        vctDoubleVec GoalTolerance;
        vctDoubleVec Zero;
    } JointTrajectory;

    // Home Action
    double HomingTimer;
    bool HomingPowerRequested, HomingPowerCurrentBiasRequested;
    bool HomingCalibrateArmStarted;
    bool HomingCalibrateRollSeekLower,
         HomingCalibrateRollSeekUpper,
         HomingCalibrateRollSeekCenter;
    double HomingCalibrateRollLower, HomingCalibrateRollUpper;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitMTM);

#endif // _mtsIntuitiveResearchKitMTM_h
