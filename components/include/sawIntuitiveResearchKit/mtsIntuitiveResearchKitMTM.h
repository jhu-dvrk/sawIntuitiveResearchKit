/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-15

  (C) Copyright 2013-2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitMTM_h
#define _mtsIntuitiveResearchKitMTM_h

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArm.h>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class robGravityCompensationMTM;

class CISST_EXPORT mtsIntuitiveResearchKitMTM: public mtsIntuitiveResearchKitArm
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitMTM(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitMTM(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsIntuitiveResearchKitMTM() override;
    void set_simulated(void) override;

protected:
    enum JointName {
        JNT_OUTER_YAW = 0,
        JNT_OUTER_PITCH_1 = 1,
        JNT_OUTER_PITCH_2 = 2,
        JNT_SETUP_JNT = 3,
        JNT_WRIST_PITCH = 4,
        JNT_WRIST_YAW = 5,
        JNT_WRIST_ROLL = 6,
        JNT_GRIPPER = 7
    };

    void PreConfigure(const Json::Value & jsonConfig,
                      const cmnPath & configPath,
                      const std::string & filename) override;

    /*! Configuration methods */
    inline size_t number_of_joints(void) const override {
        return 7;
    }

    inline size_t number_of_joints_kinematics(void) const override {
        return 7;
    }

    inline size_t number_of_brakes(void) const override {
        return 0;
    }

    inline bool use_PID_tracking_error(void) const override {
        return false;
    }

    void ConfigureGC(const std::string & filename);

    robManipulator::Errno InverseKinematics(vctDoubleVec & jointSet,
                                            const vctFrm4x4 & cartesianGoal) const override;

    inline bool is_safe_for_cartesian_control(void) const override {
        return true;
    };

    enum KinematicType {
        MTM_ITERATIVE,
        MTM_CLOSED
    } mKinematicType = MTM_ITERATIVE;

    virtual void CreateManipulator(void) override;
    virtual void Init(void) override;

    bool is_homed(void) const override;
    void unhome(void) override;
    bool is_joint_ready(void) const override;
    bool is_cartesian_ready(void) const override;

    void UpdateStateJointKinematics(void) override;
    void ToJointsPID(const vctDoubleVec & jointsKinematics, vctDoubleVec & jointsPID) override;
    void servo_jp_internal(const vctDoubleVec & jp, const vctDoubleVec & jv) override;

    // state related methods
    void SetGoalHomingArm(void) override;
    void TransitionEncodersBiased(void) override;
    void EnterCalibratingRoll(void);
    void RunCalibratingRoll(void);
    void TransitionRollCalibrated(void);
    void EnterResettingRollEncoder(void);
    void RunResettingRollEncoder(void);
    void TransitionRollEncoderReset(void);

    /*! Get data specific to the MTM (gripper angle using analog inputs) after
      calling mtsIntuitiveResearchKitArm::GetRobotData. */
    void get_robot_data(void) override;

    // see base class
    void control_servo_cf_orientation_locked(void) override;
    void SetControlEffortActiveJoints(void) override;
    void control_servo_cf_preload(vctDoubleVec & effortPreload,
                                  vctDoubleVec & wrenchPreload) override;

    /*! Lock master orientation when in cartesian effort mode */
    virtual void lock_orientation(const vctMatRot3 & orientation);
    virtual void unlock_orientation(void);

    void gravity_compensation(vctDoubleVec & efforts) override;

    double closest_equivalent_roll(double current_angle, double target_angle) const;
    int m_wrist_roll_virtual_offset = 0;

    // Functions for events
    struct {
        mtsFunctionWrite orientation_locked;
    } mtm_events;

    struct {
        mtsFunctionVoid pinch;
        mtsFunctionWrite closed;
        double zero_angle = 0.0; // in radians
        bool is_closed = false;
        double debounce_threshold = 0.2; // in seconds
        bool debounce_ended;
        double debounce_start;
    } gripper_events;

    mtsInterfaceRequired * GripperIOInterface;
    struct {
        mtsFunctionRead pot_measured_js;
    } GripperIO;

    //! Gripper angle
    prmStateJoint m_gripper_measured_js;
    prmConfigurationJoint m_gripper_configuration_js;

    robGravityCompensationMTM * GravityCompensationMTM = 0;

    double m_platform_gain = mtsIntuitiveResearchKit::MTMPlatform::Gain;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitMTM);

#endif // _mtsIntuitiveResearchKitMTM_h
