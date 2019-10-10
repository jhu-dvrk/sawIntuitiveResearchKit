/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-15

  (C) Copyright 2013-2017 Johns Hopkins University (JHU), All Rights Reserved.

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
    enum MTM_TYPE {
        MTM_NULL, MTM_LEFT, MTM_RIGHT
    };

    mtsIntuitiveResearchKitMTM(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitMTM(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsIntuitiveResearchKitMTM() override;

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
        JNT_WRIST_ROLL = 6,
        JNT_GRIPPER = 7
    };

    void ConfigureArmSpecific(const Json::Value & jsonConfig,
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

    virtual void Init(void);

    // state related methods
    void SetGoalHomingArm(void);
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
    void GetRobotData(void);

    // see base class
    void ControlEffortOrientationLocked(void) override;
    void SetControlEffortActiveJoints(void) override;

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
    prmStateJoint Gripper;
    bool GripperClosed;

    //! robot type
    MTM_TYPE RobotType;

    // Roll position when hitting lower limit before encoder preload
    bool mHomedOnce;
    double mHomingCalibrateRollLower;
    bool mHomingRollEncoderReset;
    robGravityCompensationMTM * GravityCompensationMTM = 0;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitMTM);

#endif // _mtsIntuitiveResearchKitMTM_h
