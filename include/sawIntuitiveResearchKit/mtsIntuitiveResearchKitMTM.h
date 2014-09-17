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


#ifndef _mtsIntuitiveResearchKitMTM_h
#define _mtsIntuitiveResearchKitMTM_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstRobot/robManipulator.h>
#include <cisstRobot/robQuintic.h>

class mtsIntuitiveResearchKitMTM: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    enum MTM_TYPE{
        MTM_NULL, MTM_LEFT, MTM_RIGHT
    };

    static const size_t NumberOfJoints = 7;
    static const size_t RollIndex = 6;

    mtsIntuitiveResearchKitMTM(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitMTM(const mtsTaskPeriodicConstructorArg & arg);
    inline ~mtsIntuitiveResearchKitMTM() {}

    void Configure(const std::string & filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    /*!
     \brief Set MTM type, either MTM_LEFT or MTM_RIGHT
     \param autodetect TRUE by default, will set type based on MTM name, otherwise
                       manually sepcify MTM type
     \param type MTM type either MTM_LEFT or MTM_RIGHT
    */
    void SetMTMType(const bool autodetect = true, const MTM_TYPE type = MTM_NULL);

protected:
    enum JointName {
        JNT_OUTER_YAW = 0,
        JNT_OUTER_PITCH_1 = 1,
        JNT_OUTER_PITCH_2 = 2,
        JNT_SETUP_JNT = 3,
        JNT_WRIST_PITCH = 4,
        JNT_WRIST_YAW = 5,
        JNT_WRIST_ROLL = 6
    };

    enum RobotStateType {
        MTM_UNINITIALIZED, /*! State when constructed */
        MTM_HOMING_POWERING, /*! Turn power on, calibrate encoders and current */
        MTM_HOMING_CALIBRATING_ARM, /*! Calibrate using pots and move to zero position for all joints except last one */
        MTM_HOMING_CALIBRATING_ROLL, /*! Calibrate last joint using hardware limit and tracking errors. */
        MTM_READY,
        MTM_POSITION_CARTESIAN,
        MTM_GRAVITY_COMPENSATION,
        MTM_CLUTCH
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

    /*! Position Cartesian. */
    void RunPositionCartesian(void);

    /*! Gravity Compensation. */
    void RunGravityCompensation(void);

    /*! Run Clutch */
    void RunClutch(void);

    /*! Wrapper to convert vector of 7 values to prmPositionJointSet and send to PID */
    void SetPositionJointLocal(const vctDoubleVec & newPosition);

    void SetPositionCartesian(const prmPositionCartesianSet & newPosition);
    void SetWrench(const prmForceCartesianSet & newForce);  // NOTE: in body frame
    void SetRobotControlState(const std::string & state);

    struct {
        mtsFunctionWrite Enable;
        mtsFunctionWrite EnableTorqueMode;
        mtsFunctionRead GetPositionJoint;
        mtsFunctionWrite SetPositionJoint;
        mtsFunctionWrite SetTorqueJoint;
        mtsFunctionWrite SetCheckJointLimit;
        mtsFunctionWrite SetTorqueOffset;
    } PID;

    // Required interface
    struct InterfaceRobotTorque {
        //! Enable Robot Power
        mtsFunctionVoid EnablePower;
        mtsFunctionVoid DisablePower;
        mtsFunctionRead GetActuatorAmpStatus;
        mtsFunctionVoid BiasEncoder;
        mtsFunctionWrite SetActuatorCurrent;
        mtsFunctionWrite ResetSingleEncoder;
        mtsFunctionRead GetAnalogInputPosSI;
    } RobotIO;

    // Functions for events
    struct {
        mtsFunctionVoid GripperPinch;
        mtsFunctionWrite GripperClosed;
        mtsFunctionWrite RobotStatusMsg;
        mtsFunctionWrite RobotErrorMsg;
    } EventTriggers;


    //! robot cartesian position
    prmPositionCartesianGet CartesianCurrentParam;
    vctFrm4x4 CartesianCurrent;
    vctFrm4x4 CartesianPrevious;
    //! robot cartesian velocity
    prmVelocityCartesianGet CartesianVelocityParam;
    vct3 CartesianVelocityLinear;
    vct3 CartesianVelocityAngular;
    //! robot cartesian goal
    prmPositionCartesianSet CartesianGoalSet;
    bool IsCartesianGoalSet;

    //! robot current joint position
    prmPositionJointGet JointCurrentParam;
    vctDoubleVec JointCurrent;
    //! robot desired joint position
    prmPositionJointSet JointDesiredParam;
    vctDoubleVec JointDesired;
    //! robot cartesian position when cluthed
    vctFrm4x4 CartesianClutched;

    //! desired torque for torque mode
    prmForceTorqueJointSet TorqueDesired;

    //! Analog Input from Hardware for Gripper
    vctDoubleVec AnalogInputPosSI;
    //! Gripper angle (deg)
    double GripperPosition;
    bool GripperClosed;

    //! robot control mode
    RobotStateType RobotState;
    //! robot kinematics
    robManipulator Manipulator;
    //! robot type
    MTM_TYPE RobotType;

    struct {
        robQuintic Quintic;
        vctDoubleVec Start;
        vctDoubleVec Velocity;
        vctDoubleVec Acceleration;
        vctDoubleVec Goal;
        vctDoubleVec GoalError;
        vctDoubleVec GoalTolerance;
        vctDoubleVec Zero;
        double GoalToleranceNorm;   // Error norm tolerance, hard coded to 0.1
    } JointTrajectory;

    // Home Action
    double HomingTimer;
    bool HomingPowerRequested;
    bool HomingCalibrateArmStarted;
    bool HomingCalibrateRollSeekLower,
         HomingCalibrateRollSeekUpper,
         HomingCalibrateRollSeekCenter;
    double HomingCalibrateRollLower, HomingCalibrateRollUpper;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitMTM);

#endif // _mtsIntuitiveResearchKitMTM_h
