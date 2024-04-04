/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-02-24

  (C) Copyright 2013-2024 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <cisstParameterTypes/prmCartesianImpedance.h>
#include <cisstParameterTypes/prmActuatorJointCoupling.h>
#include <cisstParameterTypes/prmInverseKinematicsRequest.h>
#include <cisstParameterTypes/prmInverseKinematicsResponse.h>
#include <cisstParameterTypes/prmForwardKinematicsRequest.h>
#include <cisstParameterTypes/prmForwardKinematicsResponse.h>

#include <cisstRobot/robManipulator.h>
#include <cisstRobot/robReflexxes.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmTypes.h>
#include <sawIntuitiveResearchKit/mtsStateMachine.h>

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

    inline void crtk_version(std::string & placeholder) const {
        placeholder = mtsIntuitiveResearchKit::crtk_version;
    }
    virtual void set_simulated(void);
    virtual inline void set_calibration_mode(const bool mode) {
        m_calibration_mode = mode;
    }

    typedef enum {GENERATION_UNDEFINED, GENERATION_Classic, GENERATION_Si} GenerationType;
    virtual inline GenerationType generation(void) const {
        return m_generation;
    }

 protected:

    /*! Define wrench reference frame */
    typedef enum {WRENCH_UNDEFINED, WRENCH_SPATIAL, WRENCH_BODY} WrenchType;

    /*! Load m_base_frame and DH parameters from JSON */
    void ConfigureDH(const Json::Value & jsonConfig, const std::string & filename, const bool ignoreCoupling = false);
    void ConfigureDH(const std::string & filename);

    /*! Arm specific configuration for derived classes PSM,
      MTM... Called by Configure method. */
    inline virtual void PreConfigure(const Json::Value & CMN_UNUSED(jsonConfig),
                                     const cmnPath & CMN_UNUSED(configPath),
                                     const std::string & CMN_UNUSED(filename)) {};
    inline virtual void PostConfigure(const Json::Value & CMN_UNUSED(jsonConfig),
                                      const cmnPath & CMN_UNUSED(configPath),
                                      const std::string & CMN_UNUSED(filename)) {};
    inline virtual void set_generation(const GenerationType generation) {
        m_generation = generation;
    }

    /*! Initialization, including resizing data members and setting up
      cisst/SAW interfaces */
    virtual void CreateManipulator(void);
    virtual void Init(void);

    virtual void update_configuration_js(void);
    void actuator_to_joint_position(const vctDoubleVec & actuator, vctDoubleVec & joint) const;

    void ResizeKinematicsData(void);

    /*! Verify that the state transition is possible, initialize
      global variables for the desired state and finally set the
      state. */
    virtual void SetDesiredState(const std::string & state);

    /*! crtk operating state command.  Currently supports "enable" and
      "disable". */
    virtual void state_command(const std::string & command);

    /*! Get data from the PID level based on current state. */
    virtual void get_robot_data(void);
    virtual void UpdateStateJointKinematics(void);
    virtual void ToJointsPID(const vctDoubleVec & jointsKinematics, vctDoubleVec & jointsPID);

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

    void set_LED_pattern(uint32_t color1, uint32_t color2, bool blink1, bool blink2);
    virtual void clip_jp(vctDoubleVec & jp) const;

    // Arm state machine
    mtsStateMachine mArmState;
    // Just to have read commands to retrieve states
    mtsStateTable mStateTableState;
    mtsStdString mStateTableStateCurrent;
    mtsStdString mStateTableStateDesired;
    prmOperatingState m_operating_state; // crtk operating state
    // state machine
    std::string m_resume_current_state, m_resume_desired_state;

    // state table for configuration parameters
    mtsStateTable mStateTableConfiguration;

    /*! Wrapper to convert vector of joint values to prmPositionJointSet and send to PID */
    virtual void servo_jp_internal(const vctDoubleVec & jp,
                                   const vctDoubleVec & jv);
    virtual void servo_jf_internal(const vctDoubleVec & jf);
    inline virtual void update_feed_forward(vctDoubleVec & CMN_UNUSED(feedForward)) {};

    /*! Methods used for commands */
    virtual void hold(void);
    virtual void free(void);
    virtual void servo_jp(const prmPositionJointSet & jp);
    virtual void servo_jr(const prmPositionJointSet & difference);
    virtual void move_jp(const prmPositionJointSet & jp);
    virtual void move_jr(const prmPositionJointSet & jp);
    virtual void servo_cp(const prmPositionCartesianSet & cp);
    virtual void servo_cr(const prmPositionCartesianSet & difference);
    virtual void move_cp(const prmPositionCartesianSet & cp);
    virtual void servo_jf(const prmForceTorqueJointSet & jf);
    virtual void pid_feed_forward_servo_jf(const prmForceTorqueJointSet & jf);
    virtual void spatial_servo_cf(const prmForceCartesianSet & cf);
    virtual void body_servo_cf(const prmForceCartesianSet & cf);
    /*! Apply the wrench relative to the body or to reference frame (i.e. absolute). */
    virtual void body_set_cf_orientation_absolute(const bool & absolute);
    virtual void use_gravity_compensation(const bool & gravityCompensation);
    virtual void servo_ci(const prmCartesianImpedance & gains);

    /*! Set base coordinate frame, this will be added to the kinematics */
    virtual void set_base_frame(const prmPositionCartesianSet & newBaseFrame);

    /*! Event handler for PID position limit. */
    virtual void PositionLimitEventHandler(const vctBoolVec & flags);

    /*! Event handler for PID errors. */
    void ErrorEventHandler(const mtsMessage & message);

    /*! Event handler for EncoderBias done. */
    void BiasEncoderEventHandler(const int & nbSamples);

    /*! Configuration methods specific to derived classes. */
    virtual size_t number_of_joints(void) const = 0;         // used PID: ECM 4, PSM 7, MTM 7
    virtual size_t number_of_joints_kinematics(void) const = 0; // ECM 4, MTM 7, PSM 6 or 8 (snake like tools)
    virtual size_t number_of_brakes(void) const = 0;         // ECM 3, PSM 0, MTM 0
    inline bool has_brakes(void) const {
        return (number_of_brakes() > 0);
    }

    inline virtual bool use_PID_tracking_error(void) const {
        return true;
    }

    inline virtual bool use_feed_forward(void) const {
        return false;
    }

    /*! Inverse kinematics must be redefined for each arm type. */
    virtual robManipulator::Errno InverseKinematics(vctDoubleVec & jointSet,
                                                    const vctFrm4x4 & cartesianGoal) const = 0;

    /*! Alternate signature for ROS services. */
    void inverse_kinematics(const prmInverseKinematicsRequest & request,
                            prmInverseKinematicsResponse & response) const ;

    /*! Forward kinematic queries using joint values provided by user.
      The number of joints (size of the vector) determines up to which
      ling the forward kinematic is computed.  If the number of joint
      is invalid, i.e. greater than the number of links, the
      response.result is set to false. */
    //@{{
    virtual void forward_kinematics(const prmForwardKinematicsRequest & request,
                                    prmForwardKinematicsResponse & response) const;
    virtual void local_forward_kinematics(const prmForwardKinematicsRequest & request,
                                          prmForwardKinematicsResponse & response) const;
    //@}

    /*! Each arm has a different homing procedure. */
    virtual bool is_homed(void) const = 0;
    virtual void unhome(void) = 0;
    virtual bool is_joint_ready(void) const = 0;
    virtual bool is_cartesian_ready(void) const = 0;

    /*! Each arm must provide a way to check if the arm is ready to be
      used in cartesian mode.  PSM and ECM need to make sure the
      tool or endoscope is away from the RCM point. */
    virtual bool is_safe_for_cartesian_control(void) const = 0;

    /*! Counter to total number of consecutive times the user is
      trying to switch to cartesian control space when it's not
      safe.  Used to throttle error messages. */
    size_t m_safe_for_cartesian_control_counter = 0;

    // Interface to PID component
    mtsInterfaceRequired * PIDInterface;
    struct {
        mtsFunctionWrite Enable;
        mtsFunctionWrite EnableJoints;
        mtsFunctionRead  Enabled;
        mtsFunctionRead  measured_js;
        mtsFunctionRead  setpoint_js;
        mtsFunctionWrite servo_jp;
        mtsFunctionWrite feed_forward_jf;
        mtsFunctionWrite enforce_position_limits;
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
        mtsFunctionVoid  PowerOnSequence;
        mtsFunctionWrite PowerOffSequence;
        mtsFunctionVoid  Explain;
        mtsFunctionWrite set_LED_pattern;
        mtsFunctionRead  GetActuatorAmpStatus;
        mtsFunctionRead  GetBrakeAmpStatus;
        mtsFunctionWrite BiasEncoder;
        mtsFunctionWrite SetEncoderPosition;
        mtsFunctionWrite SetSomeEncoderPosition;
        mtsFunctionWrite SetActuatorCurrent;
        mtsFunctionWrite UsePotsForSafetyCheck;
        mtsFunctionVoid  BrakeRelease;
        mtsFunctionVoid  BrakeEngage;
    } IO;

    // Main provided interface
    mtsInterfaceProvided * m_arm_interface;

    // Functions for events
    struct {
        mtsFunctionWrite desired_state;
        mtsFunctionWrite current_state;
        mtsFunctionWrite operating_state;
    } state_events;

    robManipulator * Manipulator = nullptr;
    std::string mConfigurationFile;
    bool m_has_coupling = false;
    prmActuatorJointCoupling m_coupling;

    // cache cartesian goal position and increment
    bool m_pid_new_goal = false;
    prmPositionCartesianSet m_servo_cp;
    vctFrm3 mCartesianRelative;

    // internal kinematics
    prmPositionCartesianGet m_local_measured_cp;
    vctFrm4x4 m_local_measured_cp_frame;
    prmPositionCartesianGet m_local_setpoint_cp;
    vctFrm4x4 m_local_setpoint_cp_frame;

    // with base frame included
    prmPositionCartesianGet m_measured_cp;
    vctFrm4x4 m_measured_cp_frame;
    prmPositionCartesianGet m_setpoint_cp;
    vctFrm4x4 m_setpoint_cp_frame;

    // joints
    prmPositionJointSet m_servo_jp_param;
    vctDoubleVec m_servo_jp;
    vctDoubleVec m_servo_jv;
    prmForceTorqueJointSet m_pid_feed_forward_servo_jf;
    prmStateJoint
        m_pid_measured_js,
        m_pid_setpoint_js,
        m_kin_measured_js,
        m_kin_setpoint_js,
        m_gravity_compensation_setpoint_js;
    prmConfigurationJoint m_configuration_js;

    // efforts
    vctDoubleMat m_body_jacobian, m_body_jacobian_transpose, m_spatial_jacobian, m_spatial_jacobian_transpose;
    WrenchType m_servo_cf_type;
    prmForceCartesianSet m_servo_cf;
    bool m_body_cf_orientation_absolute = false;
    prmForceTorqueJointSet
        m_servo_jf_param, // number of joints PID, used in servo_jf_internal
        m_servo_jf; // number of joints for kinematics
    vctDoubleVec m_servo_jf_vector; // number of joints for kinematics, more convenient type than prmForceTorqueJointSet
    // to estimate wrench from joint efforts
    nmrPInverseDynamicData
        m_jacobian_pinverse_data,
        m_jacobian_transpose_pinverse_data;
    prmForceCartesianGet m_body_measured_cf, m_spatial_measured_cf;

    // cartesian impendance controller
    osaCartesianImpedanceController * mCartesianImpedanceController;
    bool m_cartesian_impedance = false;

    // used by MTM only
    bool m_effort_orientation_locked = false;
    vctDoubleVec mEffortOrientationJoint;
    vctMatRot3 mEffortOrientation;
    // use gravity compensation or not
    bool m_gravity_compensation = false;
    double m_gravity_tilt = std::numeric_limits<double>::infinity(); // used for ECMs Classic and Si as well as PSMs Si
    // compute effort for gravity compensation based on current state, called in get_robot_data
    virtual void gravity_compensation(vctDoubleVec & efforts);

    // Velocities
    prmVelocityCartesianGet
        m_local_measured_cv, m_measured_cv,
        m_local_setpoint_cv, m_setpoint_cv;
    vctFrm4x4 CartesianPositionFrm;

    // Base frame
    vctFrm4x4 m_base_frame;
    bool m_base_frame_valid;

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
    virtual void trajectory_j_set_ratio_v(const double & ratio);

    /*! Set joint acceleration ratio for trajectory generation.  Computes
      joint accelerations based on maximum joint accelerations.  Ratio must
      be greater than 0 and lesser or equal to 1. */
    virtual void trajectory_j_set_ratio_a(const double & ratio);

    /*! Set joint velocity and acceleration ratios for trajectory generation.  Computes
      joint accelerations based on maximum joint accelerations.  Ratio must
      be greater than 0 and lesser or equal to 1. */
    virtual void trajectory_j_set_ratio(const double & ratio);

    /*! When setting separate velocity and acceleration ratios for
      trajectory generation, update main ratio.  If both velocity and
      acceleration ratios are the same, sets the ratio to that value.
      Otherwise, sets the ratio to 0, i.e. a meaningless value. */
    virtual void trajectory_j_update_ratio(void);

    /*! Sends new velocities and accelerations to Reflexxes.  This
      needs to be called every time the ratios are changed. */
    virtual void trajectory_j_update_reflexxes(void);

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

    virtual void control_servo_jp(void);
    virtual void control_move_jp(void);
    virtual void control_servo_cp(void);
    virtual void control_move_cp(void);
    virtual void control_servo_jf(void);
    virtual void control_servo_cf(void);

    /* Action on start/stop move commands, can be derived but make
       sure base class method is called in derived methods. */
    virtual void control_move_jp_on_start(void);
    virtual void control_move_jp_on_stop(const bool goal_reached);

    /*! Compute forces/position for PID when orientation is locked in
      effort cartesian mode or gravity compensation. */
    virtual void control_servo_cf_orientation_locked(void);

    /*! Determine which joints should be in effort mode.  MTM will
      redefine this so one can lock orientation using position PID. */
    virtual void SetControlEffortActiveJoints(void);

    /*! Used for derived arms to apply arm specific efforts (joint
      space).  E.g. MTM to control platform orientation.  Derived
      methods must ensure that all elements are set properly, i.e. the
      input vector is not set to zero by default. */
    virtual void control_servo_cf_preload(vctDoubleVec & effortPreload,
                                          vctDoubleVec & wrenchPreload);

    struct {
        robReflexxes Reflexxes;
        vctDoubleVec v_max;
        vctDoubleVec v; // max * ratio
        double ratio_v = mtsIntuitiveResearchKit::JointTrajectory::ratio_v;
        mtsFunctionWrite ratio_v_event;
        vctDoubleVec a_max;
        vctDoubleVec a; // max * ratio
        double ratio_a = mtsIntuitiveResearchKit::JointTrajectory::ratio_a;
        mtsFunctionWrite ratio_a_event;
        // ratio to overwire ratio_v and ratio_a
        double ratio = mtsIntuitiveResearchKit::JointTrajectory::ratio;
        mtsFunctionWrite ratio_event;
        vctDoubleVec goal;
        vctDoubleVec goal_v;
        vctDoubleVec goal_error;
        vctDoubleVec goal_tolerance;
        vctDoubleVec jerk_max;
        bool is_active;
        double end_time;
        mtsFunctionWrite goal_reached_event; // sends true if goal reached, false otherwise
    } m_trajectory_j;

    // homing
    bool m_encoders_biased_from_pots = false; // encoders biased from pots
    bool m_encoders_biased = false; // encoder might have to be biased on joint limits (MTM roll)
    bool m_re_home = false; // force re-biasing encoder even if values are found on FPGA
    bool m_homing_goes_to_zero = false;
    bool m_homing_bias_encoder_requested = false;
    double m_homing_timer;

    // generation
    GenerationType m_generation = GENERATION_UNDEFINED;

    // flag to determine if this is connected to actual IO/hardware or simulated
    bool m_simulated = false;

    // flag to determine if the arm is running in calibration mode, i.e. turn off checks using potentiometers
    bool m_calibration_mode = false;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitArm);

#endif // _mtsIntuitiveResearchKitArm_h
