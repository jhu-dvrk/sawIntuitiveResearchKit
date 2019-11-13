/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-02-24

  (C) Copyright 2013-2019 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstParameterTypes/prmConfigurationJoint.h>
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

#include <sawControllers/osaCartesianImpedanceController.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmTypes.h>
#include <sawIntuitiveResearchKit/mtsStateMachine.h>

// nearbyint is a C++11 feature; VS2008 does not have it
// (perhaps others as well).
#include <cisstCommon/cmnPortability.h>
#if (CISST_COMPILER == CISST_DOTNET2008)
inline double nearbyint(double x) { return floor(x+0.5); }
#endif

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class CISST_EXPORT mtsIntuitiveResearchKitArm: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

    friend class mtsIntuitiveResearchKitConsole;

public:
    mtsIntuitiveResearchKitArm(const std::string & componentName, const double periodInSeconds = mtsIntuitiveResearchKit::ArmPeriod);
    mtsIntuitiveResearchKitArm(const mtsTaskPeriodicConstructorArg & arg);
    virtual ~mtsIntuitiveResearchKitArm();

    void Configure(const std::string & filename) override;
    void Startup(void) override;
    void Run(void) override;
    void Cleanup(void) override;

    virtual void SetSimulated(void);

 protected:

    /*! Define wrench reference frame */
    typedef enum {WRENCH_UNDEFINED, WRENCH_SPATIAL, WRENCH_BODY} WrenchType;

    /*! Load BaseFrame and DH parameters from JSON */
    void ConfigureDH(const Json::Value & jsonConfig, const std::string & filename);
    void ConfigureDH(const std::string & filename);

    /*! Arm specific configuration for derived classes PSM,
      MTM... Called by Configure method. */
    inline virtual void ConfigureArmSpecific(const Json::Value & CMN_UNUSED(jsonConfig),
                                             const cmnPath & CMN_UNUSED(configPath),
                                             const std::string & CMN_UNUSED(filename)) {};

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
    virtual void UpdateStateJointKinematics(void);
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

    // Arm state machine
    mtsStateMachine mArmState;
    // Just to have read commands to retrieve states
    mtsStateTable mStateTableState;
    mtsStdString mStateTableStateCurrent;
    mtsStdString mStateTableStateDesired;

    /*! Wrapper to convert vector of joint values to prmPositionJointSet and send to PID */
    virtual void SetPositionJointLocal(const vctDoubleVec & newPosition);
    virtual void SetEffortJointLocal(const vctDoubleVec & newEffort);
    inline virtual void UpdateFeedForward(vctDoubleVec & CMN_UNUSED(feedForward)) {};

    /*! Methods used for commands */
    virtual void Freeze(void);
    virtual void SetPositionJoint(const prmPositionJointSet & newPosition);
    virtual void SetPositionRelativeJoint(const prmPositionJointSet & difference);
    virtual void SetPositionGoalJoint(const prmPositionJointSet & newPosition);
    virtual void SetPositionCartesian(const prmPositionCartesianSet & newPosition);
    virtual void SetPositionRelativeCartesian(const prmPositionCartesianSet & difference);
    virtual void SetPositionGoalCartesian(const prmPositionCartesianSet & newPosition);
    virtual void SetEffortJoint(const prmForceTorqueJointSet & newEffort);
    virtual void SetWrenchSpatial(const prmForceCartesianSet & newForce);
    virtual void SetWrenchBody(const prmForceCartesianSet & newForce);
    /*! Apply the wrench relative to the body or to reference frame (i.e. absolute). */
    virtual void SetWrenchBodyOrientationAbsolute(const bool & absolute);
    virtual void SetGravityCompensation(const bool & gravityCompensation);
    virtual void SetCartesianImpedanceGains(const prmCartesianImpedanceGains & gains);

    /*! Set base coordinate frame, this will be added to the kinematics */
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

    inline virtual bool UsePIDTrackingError(void) const {
        return true;
    }

    inline virtual bool UseFeedForward(void) const {
        return false;
    }

    /*! Inverse kinematics must be redefined for each arm type. */
    virtual robManipulator::Errno InverseKinematics(vctDoubleVec & jointSet,
                                                    const vctFrm4x4 & cartesianGoal) = 0;

    /*! Each arm must provide a way to check if the arm is ready to be
      used in cartesian mode.  PSM and ECM need to make sure the
      tool or endoscope is away from the RCM point. */
    virtual bool IsSafeForCartesianControl(void) const = 0;

    /*! Counter to total number of consecutive times the user is
      trying to switch to cartesian control space when it's not
      safe.  Used to throttle error messages. */
    size_t mSafeForCartesianControlCounter;

    // Interface to PID component
    mtsInterfaceRequired * PIDInterface;
    struct {
        mtsFunctionWrite SetCoupling;
        mtsFunctionWrite Enable;
        mtsFunctionWrite EnableJoints;
        mtsFunctionRead  Enabled;
        mtsFunctionRead  GetStateJoint;
        mtsFunctionRead  GetStateJointDesired;
        mtsFunctionWrite SetPositionJoint;
        mtsFunctionWrite SetFeedForwardJoint;
        mtsFunctionWrite SetCheckPositionLimit;
        mtsFunctionRead  GetConfigurationJoint;
        mtsFunctionWrite SetConfigurationJoint;
        mtsFunctionWrite EnableTorqueMode;
        mtsFunctionWrite SetTorqueJoint;
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
        mtsFunctionVoid  BrakeRelease;
        mtsFunctionVoid  BrakeEngage;
    } RobotIO;

    // Main provided interface
    mtsInterfaceProvided * RobotInterface;

    // Functions for events
    struct {
        mtsFunctionWrite DesiredState;
        mtsFunctionWrite CurrentState;
    } MessageEvents;

    robManipulator * Manipulator;
    std::string mConfigurationFile;

    // cache cartesian goal position and increment
    bool mHasNewPIDGoal;
    prmPositionCartesianSet CartesianSetParam;
    vctDoubleVec mJointRelative;
    vctFrm3 mCartesianRelative;

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
    prmForceTorqueJointSet FeedForwardParam;
    prmStateJoint StateJointPID, StateJointDesiredPID, StateJointKinematics, StateJointDesiredKinematics;
    prmConfigurationJoint ConfigurationJointPID, ConfigurationJointKinematics;

    // efforts
    vctDoubleMat mJacobianBody, mJacobianBodyTranspose, mJacobianSpatial;
    WrenchType mWrenchType;
    prmForceCartesianSet mWrenchSet;
    bool mWrenchBodyOrientationAbsolute;
    prmForceTorqueJointSet
        mTorqueSetParam, // number of joints PID, used in SetEffortJointLocal
        mEffortJointSet; // number of joints for kinematics
    vctDoubleVec mEffortJoint; // number of joints for kinematics, more convenient type than prmForceTorqueJointSet
    // to estimate wrench from joint efforts
    nmrPInverseDynamicData mJacobianPInverseData;
    prmForceCartesianGet mWrenchGet;

    // cartesian impendance controller
    osaCartesianImpedanceController mCartesianImpedanceController;
    bool mCartesianImpedance;

    // used by MTM only
    bool mEffortOrientationLocked;
    vctDoubleVec mEffortOrientationJoint;
    vctMatRot3 mEffortOrientation;
    // gravity compensation
    bool mGravityCompensation;
    virtual void AddGravityCompensationEfforts(vctDoubleVec & efforts);
    // add custom efforts for derived classes
    inline virtual void AddCustomEfforts(vctDoubleVec & CMN_UNUSED(efforts)) {};

    // Velocities
    vctFrm4x4 CartesianGetPrevious;
    prmVelocityCartesianGet CartesianVelocityGetParam;
    vctFrm4x4 CartesianPositionFrm;

    // Base frame
    vctFrm4x4 BaseFrame;
    bool BaseFrameValid;

    bool mPowered;
    bool mJointReady;
    bool mCartesianReady;
    bool mJointControlReady;
    bool mCartesianControlReady;

    mtsIntuitiveResearchKitArmTypes::ControlSpace mControlSpace;
    mtsIntuitiveResearchKitArmTypes::ControlMode mControlMode;

    /*! Method used to check if the arm is ready and throttle messages sent. */
    bool ArmIsReady(const std::string & methodName,
                    const mtsIntuitiveResearchKitArmTypes::ControlSpace space);
    size_t mArmNotReadyCounter;
    double mArmNotReadyTimeLastMessage;

    /*! Set joint velocity ratio for trajectory generation.  Computes
      joint velocities based on maximum joint velocities.  Ratio must
      be greater than 0 and lesser or equal to 1. */
    virtual void SetJointVelocityRatio(const double & ratio);

    /*! Set joint acceleration ratio for trajectory generation.  Computes
      joint accelerations based on maximum joint accelerations.  Ratio must
      be greater than 0 and lesser or equal to 1. */
    virtual void SetJointAccelerationRatio(const double & ratio);

    /*! Sets control space and mode.  If none are user defined, the
      callbacks will be using the methods provided in this class.
      If either the space or mode is "USER", a callback must be
      provided. */
    void SetControlSpaceAndMode(const mtsIntuitiveResearchKitArmTypes::ControlSpace space,
                                const mtsIntuitiveResearchKitArmTypes::ControlMode mode,
                                mtsCallableVoidBase * callback = 0);

    /*! Set the control space and mode along with a callback for
      control.  The callback method will be use only if either the
      space or the mode is "USER". */
    template <class __classType>
    inline void SetControlSpaceAndMode(const mtsIntuitiveResearchKitArmTypes::ControlSpace space,
                                       const mtsIntuitiveResearchKitArmTypes::ControlMode mode,
                                       void (__classType::*method)(void),
                                       __classType * classInstantiation) {
        this->SetControlSpaceAndMode(space, mode,
                                     new mtsCallableVoidMethod<__classType>(method,
                                                                            classInstantiation));
    }

    /*! Methods to set the control callback directly.  Users should
      use the SetControlSpaceAndMode method instead. */
    //@{
    template <class __classType>
    inline void SetControlCallback(void (__classType::*method)(void),
                                   __classType * classInstantiation) {
        this->SetControlCallback(new mtsCallableVoidMethod<__classType>(method,
                                                                        classInstantiation));
    }

    inline void SetControlCallback(mtsCallableVoidBase * callback) {
        if (this->mControlCallback != 0) {
            delete this->mControlCallback;
        }
        this->mControlCallback = callback;
    }
    //@}

    mtsCallableVoidBase * mControlCallback;

    virtual void ControlPositionJoint(void);
    virtual void ControlPositionRelativeJoint(void);
    virtual void ControlPositionGoalJoint(void);
    virtual void ControlPositionCartesian(void);
    virtual void ControlPositionRelativeCartesian(void);
    virtual void ControlPositionGoalCartesian(void);
    virtual void ControlEffortJoint(void);
    virtual void ControlEffortCartesian(void);

    /*! Compute forces/position for PID when orientation is locked in
      effort cartesian mode or gravity compensation. */
    virtual void ControlEffortOrientationLocked(void);

    virtual void SetControlEffortActiveJoints(void);

    struct {
        robReflexxes Reflexxes;
        vctDoubleVec VelocityMaximum;
        vctDoubleVec Velocity; // max * ratio
        double VelocityRatio;
        mtsFunctionWrite VelocityRatioEvent;
        vctDoubleVec AccelerationMaximum;
        vctDoubleVec Acceleration; // max * ratio
        double AccelerationRatio;
        mtsFunctionWrite AccelerationRatioEvent;
        vctDoubleVec Goal;
        vctDoubleVec GoalVelocity;
        vctDoubleVec GoalError;
        vctDoubleVec GoalTolerance;
        vctDoubleVec MaxJerk;
        bool IsWorking;
        double EndTime;
        mtsFunctionWrite GoalReachedEvent; // sends true if goal reached, false otherwise
    } mJointTrajectory;

    // Home Action
    bool mEncoderBiased;
    bool mHomingGoesToZero;
    bool mHomingBiasEncoderRequested;
    double mHomingTimer;

    unsigned int mCounter;

    // Flag to determine if this is connected to actual IO/hardware or simulated
    bool mIsSimulated;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitArm);

#endif // _mtsIntuitiveResearchKitArm_h
