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


#ifndef _mtsIntuitiveResearchKitMTM_h
#define _mtsIntuitiveResearchKitMTM_h

#include <cisstParameterTypes/prmForceTorqueJointSet.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArm.h>

class mtsIntuitiveResearchKitMTM: public mtsIntuitiveResearchKitArm
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    enum MTM_TYPE {
        MTM_NULL, MTM_LEFT, MTM_RIGHT
    };

    mtsIntuitiveResearchKitMTM(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitMTM(const mtsTaskPeriodicConstructorArg & arg);
    inline ~mtsIntuitiveResearchKitMTM() {}

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

    /*! Configuration methods */
    inline size_t NumberOfJoints(void) const {
        return 7;
    }

    inline size_t NumberOfAxes(void) const {
        return 8;
    }

    inline size_t NumberOfJointsKinematics(void) const {
        return 7;
    }

    inline size_t NumberOfBrakes(void) const {
        return 0;
    }

    inline bool UsePIDTrackingError(void) const {
        return false;
    }

    robManipulator::Errno InverseKinematics(vctDoubleVec & jointSet,
                                            const vctFrm4x4 & cartesianGoal);

    void Init(void);

    /*! Get data specific to the MTM (gripper angle using analog inputs) after
      calling mtsIntuitiveResearchKitArm::GetRobotData. */
    void GetRobotData(void);

    /*! Verify that the state transition is possible, initialize global
      variables for the desired state and finally set the state. */
    void SetState(const mtsIntuitiveResearchKitArmTypes::RobotStateType & newState);

    void SetRobotControlState(const std::string & state);

    /*! Switch case for user mode. */
    void RunArmSpecific(void);

    /*! Homing procedure, home all joints except last one using potentiometers as reference. */
    void RunHomingCalibrateArm(void);

    /*! Homing procedure, calibrate last joint based on hardware limits. */
    void RunHomingCalibrateRoll(void);

    /*! Gravity Compensation. */
    virtual void RunGravityCompensation(void);

    /*! Run Clutch */
    void RunClutch(void);

    virtual void SetWrench(const prmForceCartesianSet & newForce);  // NOTE: in body frame

    // Functions for events
    struct {
        mtsFunctionVoid GripperPinch;
        mtsFunctionWrite GripperClosed;
    } GripperEvents;

    //! robot cartesian position when cluthed
    vctFrm4x4 CartesianClutched;

    //! desired torque for torque mode
    prmForceTorqueJointSet TorqueSet;

    //! Analog Input from Hardware for Gripper
    vctDoubleVec AnalogInputPosSI;
    //! Gripper angle
    double GripperPosition;
    bool GripperClosed;

    //! robot type
    MTM_TYPE RobotType;

    // Home Action
    bool HomingCalibrateRollSeekLower,
         HomingCalibrateRollSeekUpper,
         HomingCalibrateRollSeekCenter;
    double HomingCalibrateRollLower, HomingCalibrateRollUpper;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitMTM);

#endif // _mtsIntuitiveResearchKitMTM_h
