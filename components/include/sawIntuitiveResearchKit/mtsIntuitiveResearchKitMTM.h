/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-15

  (C) Copyright 2013-2020 Johns Hopkins University (JHU), All Rights Reserved.

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
    inline size_t NumberOfAxes(void) const override {
        return 8;
    }

    inline size_t NumberOfJoints(void) const override {
        return 7;
    }

    inline size_t NumberOfJointsKinematics(void) const override {
        return 7;
    }

    inline size_t NumberOfBrakes(void) const override {
        return 0;
    }

    inline bool UsePIDTrackingError(void) const override {
        return false;
    }

    void ConfigureGC(const std::string & filename);

    robManipulator::Errno InverseKinematics(vctDoubleVec & jointSet,
                                            const vctFrm4x4 & cartesianGoal) override;

    inline bool IsSafeForCartesianControl(void) const override {
        return true;
    };

    enum KinematicType {
        MTM_ITERATIVE,
        MTM_CLOSED
    } mKinematicType = MTM_ITERATIVE;

    virtual void CreateManipulator(void) override;
    virtual void Init(void) override;

    // state related methods
    void SetGoalHomingArm(void) override;
    void TransitionArmHomed(void);
    void EnterCalibratingRoll(void);
    void RunCalibratingRoll(void);
    void TransitionRollCalibrated(void);
    void EnterHomingRoll(void);
    void RunHomingRoll(void);
    void EnterResettingRollEncoder(void);
    void RunResettingRollEncoder(void);
    void TransitionRollEncoderReset(void);

    /*! Get data specific to the MTM (gripper angle using analog inputs) after
      calling mtsIntuitiveResearchKitArm::GetRobotData. */
    void GetRobotData(void) override;

    // see base class
    void ControlEffortOrientationLocked(void) override;
    void SetControlEffortActiveJoints(void) override;
    void ControlEffortCartesianPreload(vctDoubleVec & effortPreload,
                                       vctDoubleVec & wrenchPreload) override;

    /*! Lock master orientation when in cartesian effort mode */
    virtual void LockOrientation(const vctMatRot3 & orientation);
    virtual void UnlockOrientation(void);

    void AddGravityCompensationEfforts(vctDoubleVec & efforts) override;

    // Functions for events
    struct {
        mtsFunctionVoid GripperPinch;
        mtsFunctionWrite GripperClosed;
    } GripperEvents;

    //! robot cartesian position when cluthed
    vctFrm4x4 CartesianClutched;

    //! Analog Input from Hardware for Gripper
    vctDoubleVec AnalogInputPosSI;
    //! Gripper angle
    prmStateJoint StateGripper;
    prmConfigurationJoint ConfigurationGripper;
    bool GripperClosed;

    // Roll position when hitting lower limit before encoder preload
    bool mHomedOnce;
    double mHomingCalibrateRollLower;
    bool mHomingRollEncoderReset;
    robGravityCompensationMTM * GravityCompensationMTM = 0;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitMTM);

#endif // _mtsIntuitiveResearchKitMTM_h
