/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-02-24

  (C) Copyright 2013-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsIntuitiveResearchKitArm_h
#define _mtsIntuitiveResearchKitArm_h

#include <cisstNumerical/nmrPInverse.h>

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmVelocityJointGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>

#include <cisstRobot/robManipulator.h>
#include <cisstRobot/robReflexxes.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmTypes.h>
#include <sawIntuitiveResearchKit/mtsStateMachine.h>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class CISST_EXPORT mtsIntuitiveResearchKitArm: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitArm(const std::string & componentName, const double periodInSeconds = mtsIntuitiveResearchKit::ArmPeriod);
    mtsIntuitiveResearchKitArm(const mtsTaskPeriodicConstructorArg & arg);
    virtual inline ~mtsIntuitiveResearchKitArm() {}

    void Configure(const std::string & filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    virtual void SetSimulated(void);

 protected:

    /*! Define wrench reference frame */
    typedef enum {WRENCH_UNDEFINED, WRENCH_SPATIAL, WRENCH_BODY} WrenchType;

    /*! Load BaseFrame and DH parameters from JSON */
    void ConfigureDH(const Json::Value & jsonConfig);

    /*! Initialization, including resizing data members and setting up
      cisst/SAW interfaces */
    virtual void Init(void);
    void ResizeKinematicsData(void);

    /*! Verify that the state transition is possible, initialize
      global variables for the desired state and finally set the
      state. */
    virtual void SetDesiredState(const std::string & state);

    /*! Get data from the PID level based on current state. */
    virtual void GetRobotData(void);
    virtual void UpdateJointsKinematics(void);
    virtual void ToJointsPID(const vctDoubleVec & jointsKinematics, vctDoubleVec & jointsPID);

    // state machine
    std::string mFallbackState;

    void StateChanged(void);
    void RunAllStates(void); // this should happen for all states

    virtual void EnterUninitialized(void);
    virtual void TransitionUninitialized(void);

    virtual void EnterCalibratingEncodersFromPots(void);
    virtual void TransitionCalibratingEncodersFromPots(void);
    virtual void TransitionEncodersBiased(void);

    virtual void EnterPowering(void);
    virtual void TransitionPowering(void);
    virtual void EnterPowered(void);
    virtual void TransitionPowered(void);

    virtual void EnterHomingArm(void);
    virtual void SetGoalHomingArm(void) = 0;
    virtual void RunHomingArm(void);

    // transitions and states between ARM_HOMED and READY are defined
    // in derived classes.

    virtual void EnterReady(void);
    virtual void LeaveReady(void);
    virtual void RunReady(void);

    /*! Control modes. */
    virtual void ControlPositionJoint(void);
    virtual void ControlPositionGoalJoint(void);
    virtual void ControlPositionCartesian(void);
    virtual void ControlPositionGoalCartesian(void);

    /*! Effort state. */
    virtual void ControlEffortJoint(void);
    virtual void ControlEffortCartesian(void);

    /*! Compute forces/position for PID when orientation is locked in
      effort cartesian mode or gravity compensation. */
    virtual void ControlEffortOrientationLocked(void);

    /*! Wrapper to convert vector of joint values to prmPositionJointSet and send to PID */
    virtual void SetPositionJointLocal(const vctDoubleVec & newPosition);

    /*! Methods used for commands */
    virtual void SetPositionJoint(const prmPositionJointSet & newPosition);
    virtual void SetPositionGoalJoint(const prmPositionJointSet & newPosition);
    virtual void SetPositionCartesian(const prmPositionCartesianSet & newPosition);
    virtual void SetPositionGoalCartesian(const prmPositionCartesianSet & newPosition);
    virtual void SetEffortJoint(const prmForceTorqueJointSet & newEffort);
    virtual void SetWrenchSpatial(const prmForceCartesianSet & newForce);
    virtual void SetWrenchBody(const prmForceCartesianSet & newForce);
    /*! Apply the wrench relative to the body or to reference frame (i.e. absolute). */
    virtual void SetWrenchBodyOrientationAbsolute(const bool & absolute);
    virtual void SetGravityCompensation(const bool & gravityCompensation);

    /*! Set base coordinate frame, this will be added to the kinematics */
    virtual void SetBaseFrameEventHandler(const prmPositionCartesianGet & newBaseFrame);
    virtual void SetBaseFrame(const prmPositionCartesianSet & newBaseFrame);

    /*! Event handler for PID position limit. */
    virtual void PositionLimitEventHandler(const vctBoolVec & flags);

    /*! Event handler for PID errors. */
    void ErrorEventHandler(const mtsMessage & message);

    /*! Event handler for EncoderBias done. */
    void BiasEncoderEventHandler(const int & nbSamples);

    /*! Configuration methods specific to derived classes. */
    virtual size_t NumberOfAxes(void) const = 0;           // used IO: ECM 4, PSM 7, MTM 8
    virtual size_t NumberOfJoints(void) const = 0;         // used PID: ECM 4, PSM 7, MTM 7
    virtual size_t NumberOfJointsKinematics(void) const = 0; // ECM 4, MTM 7, PSM 6 or 8 (snake like tools)
    virtual size_t NumberOfBrakes(void) const = 0;         // ECM 3, PSM 0, MTM 0

    inline virtual bool UsePotsForSafetyCheck(void) const {
        return true;
    }

    inline virtual bool UsePIDTrackingError(void) const {
        return true;
    }

    virtual robManipulator::Errno InverseKinematics(vctDoubleVec & jointSet,
                                                    const vctFrm4x4 & cartesianGoal) = 0;

    // Interface to PID component
    mtsInterfaceRequired * PIDInterface;
    struct {
        mtsFunctionWrite SetCoupling;
        mtsFunctionWrite Enable;
        mtsFunctionWrite EnableJoints;
        mtsFunctionRead  GetStateJoint;
        mtsFunctionRead  GetStateJointDesired;
        mtsFunctionWrite SetPositionJoint;
        mtsFunctionWrite SetCheckPositionLimit;
        mtsFunctionWrite SetPositionLowerLimit;
        mtsFunctionWrite SetPositionUpperLimit;
        mtsFunctionWrite SetTorqueLowerLimit;
        mtsFunctionWrite SetTorqueUpperLimit;
        mtsFunctionWrite EnableTorqueMode;
        mtsFunctionWrite SetTorqueJoint;
        mtsFunctionWrite SetTorqueOffset;
        mtsFunctionWrite EnableTrackingError;
        mtsFunctionWrite SetTrackingErrorTolerance;
        vctDoubleVec DefaultTrackingErrorTolerance;
    } PID;

    // Interface to IO component
    mtsInterfaceRequired * IOInterface;
    struct InterfaceRobotTorque {
        mtsFunctionRead  GetSerialNumber;
        mtsFunctionVoid  EnablePower;
        mtsFunctionVoid  DisablePower;
        mtsFunctionRead  GetActuatorAmpStatus;
        mtsFunctionRead  GetBrakeAmpStatus;
        mtsFunctionWrite BiasEncoder;
        mtsFunctionWrite ResetSingleEncoder;
        mtsFunctionRead  GetAnalogInputPosSI;
        mtsFunctionWrite SetActuatorCurrent;
        mtsFunctionWrite UsePotsForSafetyCheck;
        mtsFunctionWrite SetPotsToEncodersTolerance;
        mtsFunctionVoid  BrakeRelease;
        mtsFunctionVoid  BrakeEngage;
    } RobotIO;

    // Interface to SUJ component
    mtsInterfaceRequired * SUJInterface;

    // Main provided interface
    mtsInterfaceProvided * RobotInterface;

    // Functions for events
    struct {
        mtsFunctionWrite DesiredState;
        mtsFunctionWrite CurrentState;
    } MessageEvents;

    robManipulator *Manipulator;

    // Cache cartesian goal position
    prmPositionCartesianSet CartesianSetParam;
    bool mHasNewPIDGoal;

    // internal kinematics
    prmPositionCartesianGet CartesianGetLocalParam;
    vctFrm4x4 CartesianGetLocal;
    prmPositionCartesianGet CartesianGetLocalDesiredParam;
    vctFrm4x4 CartesianGetLocalDesired;

    // with base frame included
    prmPositionCartesianGet CartesianGetParam, CartesianGetPreviousParam;
    vctFrm4x4 CartesianGet;
    prmPositionCartesianGet CartesianGetDesiredParam;
    vctFrm4x4 CartesianGetDesired;

    // joints
    prmPositionJointSet JointSetParam;
    vctDoubleVec JointSet;
    vctDoubleVec JointVelocitySet;
    prmStateJoint JointsPID, JointsDesiredPID, JointsKinematics, JointsDesiredKinematics;

    // efforts
    vctDoubleMat mJacobianBody, mJacobianBodyTranspose, mJacobianSpatial;
    vctDoubleVec JointExternalEffort;
    WrenchType mWrenchType;
    prmForceCartesianSet mWrenchSet;
    bool mWrenchBodyOrientationAbsolute;
    prmForceTorqueJointSet TorqueSetParam, mEffortJointSet;
    // to estimate wrench from joint efforts
    nmrPInverseDynamicData mJacobianPInverseData;
    prmForceCartesianGet mWrenchGet;

    // used by MTM only
    bool mEffortOrientationLocked;
    vctDoubleVec mEffortOrientationJoint;
    vctMatRot3 mEffortOrientation;
    // gravity compensation
    bool mGravityCompensation;
    void AddGravityCompensationEfforts(vctDoubleVec & efforts);
    // add custom efforts for derived classes
    inline virtual void AddCustomEfforts(vctDoubleVec & CMN_UNUSED(efforts)) {};

    // Velocities
    vctFrm4x4 CartesianGetPrevious;
    prmVelocityCartesianGet CartesianVelocityGetParam;
    vctFrm4x4 CartesianPositionFrm;

    // Base frame
    vctFrm4x4 BaseFrame;
    bool BaseFrameValid;

    mtsStateMachine mArmState;
    bool mPowered;
    bool mReady;
    bool mJointReady;
    bool mCartesianReady;

    mtsIntuitiveResearchKitArmTypes::ControlSpace mControlSpace;
    mtsIntuitiveResearchKitArmTypes::ControlMode mControlMode;
    void SetControlSpaceAndMode(const mtsIntuitiveResearchKitArmTypes::ControlSpace space,
                                const mtsIntuitiveResearchKitArmTypes::ControlMode mode);
    virtual void SetControlEffortActiveJoints(void);

    struct {
        robReflexxes Reflexxes;
        vctDoubleVec Velocity;
        vctDoubleVec Acceleration;
        vctDoubleVec Goal;
        vctDoubleVec GoalVelocity;
        vctDoubleVec GoalError;
        vctDoubleVec GoalTolerance;
        vctDoubleVec MaxJerk;
        bool IsWorking;
        double EndTime;
        mtsFunctionWrite GoalReachedEvent; // sends true if goal reached, false otherwise
    } mJointTrajectory;

    vctDoubleVec PotsToEncodersTolerance;

    // Home Action
    bool mHomedOnce;
    bool mHomingGoesToZero;
    bool mHomingBiasEncoderRequested;
    double mHomingTimer;

    unsigned int mCounter;

    // Flag to determine if this is connected to actual IO/hardware or simulated
    bool mIsSimulated;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitArm);

#endif // _mtsIntuitiveResearchKitArm_h
