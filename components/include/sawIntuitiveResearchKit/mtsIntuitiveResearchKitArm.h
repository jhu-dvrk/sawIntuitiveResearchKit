/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-02-24

  (C) Copyright 2013-2020 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstParameterTypes/prmOperatingState.h>
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
#include <cisstParameterTypes/prmCartesianImpedanceGains.h>

#include <cisstRobot/robManipulator.h>
#include <cisstRobot/robReflexxes.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmTypes.h>
#include <sawIntuitiveResearchKit/mtsStateMachine.h>

// nearbyint is a C++11 feature; VS2008 does not have it
// (perhaps others as well).
#include <cisstCommon/cmnPortability.h>
#if (CISST_COMPILER == CISST_DOTNET2008)
inline double nearbyint(double x) { return floor(x+0.5); }
#endif

// forward declarations
class osaCartesianImpedanceController;

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

    /*! Load m_base_frame and DH parameters from JSON */
    void ConfigureDH(const Json::Value & jsonConfig, const std::string & filename);
    void ConfigureDH(const std::string & filename);

    /*! Arm specific configuration for derived classes PSM,
      MTM... Called by Configure method. */
    inline virtual void PreConfigure(const Json::Value & CMN_UNUSED(jsonConfig),
                                     const cmnPath & CMN_UNUSED(configPath),
                                     const std::string & CMN_UNUSED(filename)) {};
    inline virtual void PostConfigure(const Json::Value & CMN_UNUSED(jsonConfig),
                                      const cmnPath & CMN_UNUSED(configPath),
                                      const std::string & CMN_UNUSED(filename)) {};

    /*! Initialization, including resizing data members and setting up
      cisst/SAW interfaces */
    virtual void CreateManipulator(void);
    virtual void Init(void);

    void UpdateConfigurationJointKinematic(void);
    void ResizeKinematicsData(void);

    /*! Verify that the state transition is possible, initialize
      global variables for the desired state and finally set the
      state. */
    virtual void SetDesiredState(const std::string & state);

    /*! crtk operating state command.  Currently supports "enable" and
      "disable". */
    virtual void state_command(const std::string & command);

    /*! Get data from the PID level based on current state. */
    virtual void GetRobotData(void);
    virtual void UpdateStateJointKinematics(void);
    virtual void ToJointsPID(const vctDoubleVec & jointsKinematics, vctDoubleVec & jointsPID);

    // state machine
    std::string m_resume_current_state, m_resume_desired_state;

    void UpdateOperatingStateAndBusy(const prmOperatingState::StateType & state,
                                     const bool isBusy);
    void UpdateHomed(const bool isHomed);
    void UpdateIsBusy(const bool isBusy);
    void StateEvents(void);

    void StateChanged(void);
    void RunAllStates(void); // this should happen for all states

    virtual void EnterDisabled(void);
    virtual void TransitionDisabled(void);

    virtual void EnterPowering(void);
    virtual void TransitionPowering(void);
    virtual void EnterEnabled(void);
    virtual void TransitionEnabled(void);

    virtual void EnterCalibratingEncodersFromPots(void);
    virtual void TransitionCalibratingEncodersFromPots(void);
    virtual void EnterEncodersBiased(void);
    virtual void TransitionEncodersBiased(void);

    virtual void EnterHoming(void);
    virtual void SetGoalHomingArm(void) = 0;
    virtual void RunHoming(void);

    // transitions to state HOMED are defined
    // in derived classes.

    virtual void EnterHomed(void);
    virtual void LeaveHomed(void);
    virtual void RunHomed(void);

    virtual void EnterPaused(void);
    virtual void EnterFault(void);

    // Arm state machine
    mtsStateMachine mArmState;
    // Just to have read commands to retrieve states
    mtsStateTable mStateTableState;
    mtsStdString mStateTableStateCurrent;
    mtsStdString mStateTableStateDesired;
    prmOperatingState m_operating_state; // crtk operating state

    // state table for configuration parameters
    mtsStateTable mStateTableConfiguration;

    /*! Wrapper to convert vector of joint values to prmPositionJointSet and send to PID */
    virtual void SetPositionJointLocal(const vctDoubleVec & newPosition);
    virtual void SetEffortJointLocal(const vctDoubleVec & newEffort);
    inline virtual void UpdateFeedForward(vctDoubleVec & CMN_UNUSED(feedForward)) {};

    /*! Methods used for commands */
    virtual void Freeze(void);
    virtual void servo_jp(const prmPositionJointSet & newPosition);
    virtual void servo_jr(const prmPositionJointSet & difference);
    virtual void move_jp(const prmPositionJointSet & newPosition);
    virtual void servo_cp(const prmPositionCartesianSet & newPosition);
    virtual void servo_cr(const prmPositionCartesianSet & difference);
    virtual void move_cp(const prmPositionCartesianSet & newPosition);
    virtual void servo_jf(const prmForceTorqueJointSet & newEffort);
    virtual void spatial_servo_cf(const prmForceCartesianSet & newForce);
    virtual void body_servo_cf(const prmForceCartesianSet & newForce);
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
    virtual size_t NumberOfJoints(void) const = 0;         // used PID: ECM 4, PSM 7, MTM 7
    virtual size_t NumberOfJointsKinematics(void) const = 0; // ECM 4, MTM 7, PSM 6 or 8 (snake like tools)
    virtual size_t NumberOfBrakes(void) const = 0;         // ECM 3, PSM 0, MTM 0
    inline bool HasBrakes(void) const {
        return (NumberOfBrakes() > 0);
    }

    inline virtual bool UsePIDTrackingError(void) const {
        return true;
    }

    inline virtual bool UseFeedForward(void) const {
        return false;
    }

    /*! Inverse kinematics must be redefined for each arm type. */
    virtual robManipulator::Errno InverseKinematics(vctDoubleVec & jointSet,
                                                    const vctFrm4x4 & cartesianGoal) = 0;

    /*! Each arm has a different homing procedure. */
    virtual bool IsHomed(void) const = 0;
    virtual void UnHome(void) = 0;
    virtual bool IsJointReady(void) const = 0;
    virtual bool IsCartesianReady(void) const = 0;

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
        mtsFunctionRead  measured_js;
        mtsFunctionRead  setpoint_js;
        mtsFunctionWrite servo_jp;
        mtsFunctionWrite SetFeedForwardJoint;
        mtsFunctionWrite SetCheckPositionLimit;
        mtsFunctionRead  configuration_js;
        mtsFunctionWrite configure_js;
        mtsFunctionWrite EnableTorqueMode;
        mtsFunctionWrite servo_jf;
        mtsFunctionWrite EnableTrackingError;
        mtsFunctionWrite SetTrackingErrorTolerance;
        vctDoubleVec DefaultTrackingErrorTolerance;
    } PID;

    // Interface to IO component
    mtsInterfaceRequired * IOInterface;
    struct {
        mtsFunctionRead  GetSerialNumber;
        mtsFunctionVoid  EnablePower;
        mtsFunctionVoid  DisablePower;
        mtsFunctionRead  GetActuatorAmpStatus;
        mtsFunctionRead  GetBrakeAmpStatus;
        mtsFunctionWrite BiasEncoder;
        mtsFunctionWrite SetSomeEncoderPosition;
        mtsFunctionWrite SetActuatorCurrent;
        mtsFunctionWrite UsePotsForSafetyCheck;
        mtsFunctionVoid  BrakeRelease;
        mtsFunctionVoid  BrakeEngage;
    } IO;

    // Main provided interface
    mtsInterfaceProvided * RobotInterface;

    // Functions for events
    struct {
        mtsFunctionWrite desired_state;
        mtsFunctionWrite current_state;
        mtsFunctionWrite operating_state;
    } state_events;

    robManipulator * Manipulator;
    std::string mConfigurationFile;

    // cache cartesian goal position and increment
    bool m_new_pid_goal;
    prmPositionCartesianSet CartesianSetParam;
    vctFrm3 mCartesianRelative;

    // internal kinematics
    prmPositionCartesianGet m_local_measured_cp;
    vctFrm4x4 m_local_measured_cp_frame;
    prmPositionCartesianGet m_local_setpoint_cp;
    vctFrm4x4 m_local_setpoint_cp_frame;

    // with base frame included
    prmPositionCartesianGet m_measured_cp, CartesianGetPreviousParam;
    vctFrm4x4 m_measured_cp_frame;
    prmPositionCartesianGet m_setpoint_cp;
    vctFrm4x4 m_setpoint_cp_frame;

    // joints
    prmPositionJointSet JointSetParam;
    vctDoubleVec JointSet;
    vctDoubleVec JointVelocitySet;
    prmForceTorqueJointSet FeedForwardParam;
    prmStateJoint m_pid_measured_js, m_pid_setpoint_js, m_kin_measured_js, m_kin_setpoint_js;
    prmConfigurationJoint m_pid_configuration_js, m_kin_configuration_js;

    // efforts
    vctDoubleMat m_body_jacobian, m_body_jacobian_transpose, m_spatial_jacobian;
    WrenchType mWrenchType;
    prmForceCartesianSet mWrenchSet;
    bool mWrenchBodyOrientationAbsolute;
    prmForceTorqueJointSet
        mTorqueSetParam, // number of joints PID, used in SetEffortJointLocal
        mEffortJointSet; // number of joints for kinematics
    vctDoubleVec mEffortJoint; // number of joints for kinematics, more convenient type than prmForceTorqueJointSet
    // to estimate wrench from joint efforts
    nmrPInverseDynamicData mJacobianPInverseData;
    prmForceCartesianGet m_body_measured_cf;

    // cartesian impendance controller
    osaCartesianImpedanceController * mCartesianImpedanceController;
    bool m_cartesianImpedance;

    // used by MTM only
    bool m_effort_orientation_locked;
    vctDoubleVec mEffortOrientationJoint;
    vctMatRot3 mEffortOrientation;
    // gravity compensation
    bool m_gravity_compensation;
    virtual void AddGravityCompensationEfforts(vctDoubleVec & efforts);
    // add custom efforts for derived classes
    inline virtual void AddCustomEfforts(vctDoubleVec & CMN_UNUSED(efforts)) {};

    // Velocities
    vctFrm4x4 CartesianGetPrevious;
    prmVelocityCartesianGet m_measured_cv;
    vctFrm4x4 CartesianPositionFrm;

    // Base frame
    vctFrm4x4 m_base_frame;
    bool BaseFrameValid;

    bool m_powered = false;

    mtsIntuitiveResearchKitArmTypes::ControlSpace m_control_space;
    mtsIntuitiveResearchKitArmTypes::ControlMode m_control_mode;

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
    virtual void ControlPositionGoalJoint(void);
    virtual void ControlPositionCartesian(void);
    virtual void ControlPositionGoalCartesian(void);
    virtual void ControlEffortJoint(void);
    virtual void ControlEffortCartesian(void);

    /*! Compute forces/position for PID when orientation is locked in
      effort cartesian mode or gravity compensation. */
    virtual void ControlEffortOrientationLocked(void);

    /*! Determine which joints should be in effort mode.  MTM will
      redefine this so one can lock orientation using position PID. */
    virtual void SetControlEffortActiveJoints(void);

    /*! Used for derived arms to apply arm specific efforts (joint
      space).  E.g. MTM to control platform orientation.  Derived
      methods must ensure that all elements are set properly, i.e. the
      input vector is not set to zero by default. */
    virtual void ControlEffortCartesianPreload(vctDoubleVec & effortPreload,
                                               vctDoubleVec & wrenchPreload);

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
        bool IsActive;
        double EndTime;
        mtsFunctionWrite GoalReachedEvent; // sends true if goal reached, false otherwise
    } mJointTrajectory;

    // homing
    bool m_encoders_biased_from_pots = false; // encoders biased from pots
    bool m_encoders_biased = false; // encoder might have to be biased on joint limits (MTM roll)
    bool m_re_home = false; // force re-biasing encoder even if values are found on FPGA
    bool mHomingGoesToZero;
    bool mHomingBiasEncoderRequested;
    double mHomingTimer;

    // flag to determine if this is connected to actual IO/hardware or simulated
    bool m_simulated;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitArm);

#endif // _mtsIntuitiveResearchKitArm_h
