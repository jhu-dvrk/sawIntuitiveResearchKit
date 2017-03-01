/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-15

  (C) Copyright 2013-2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitPSM_h
#define _mtsIntuitiveResearchKitPSM_h

#include <cisstParameterTypes/prmActuatorJointCoupling.h>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArm.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitOptimizer.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class CISST_EXPORT mtsIntuitiveResearchKitPSM: public mtsIntuitiveResearchKitArm
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitPSM(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitPSM(const mtsTaskPeriodicConstructorArg & arg);
    inline ~mtsIntuitiveResearchKitPSM() {}

    void SetSimulated(void);
    void Configure(const std::string & filename);

protected:

    // PSM Optimizer
    mtsIntuitiveResearchKitOptimizer * Optimizer;

    /*! Configuration methods */
    inline size_t NumberOfAxes(void) const {
        return 7;
    }

    inline size_t NumberOfJoints(void) const {
        return 7;
    }

    inline size_t NumberOfJointsKinematics(void) const {
        return 6;
    }

    inline size_t NumberOfBrakes(void) const {
        return 0;
    }

    inline bool UsePIDTrackingError(void) const {
        return false;
    }

    /*! 5mm tools with 8 joints */
    bool mSnakeLike;
    
    inline vctReturnDynamicVector<double> JointsForSnakeLikeKinematics(const vctDoubleVec & joints) const {
        vctDoubleVec result(8);
        result[0] = joints[0];
        result[1] = joints[1];
        result[2] = joints[2];
        result[3] = joints[3];
        result[4] = joints[4] * 0.5;
        result[5] = joints[5] * 0.5;
        result[6] = joints[5] * 0.5;
        result[7] = joints[4] * 0.5;
        return vctReturnDynamicVector<double>(result);
    }
    
    vctFrame4x4<double> ForwardKinematics(const vctDoubleVec & q, const int N) const;

    robManipulator::Errno InverseKinematics(vctDoubleVec & jointSet,
                                            const vctFrm4x4 & cartesianGoal);

    bool JacobianBody(const vctDoubleVec & q, vctDoubleMat & J) const;

    bool JacobianSpatial(const vctDoubleVec & q, vctDoubleMat & J) const;

    void Init(void);

    /*! Verify that the state transition is possible, initialize global
      variables for the desired state and finally set the state. */
    void SetState(const mtsIntuitiveResearchKitArmTypes::RobotStateType & newState);

    void SetRobotControlState(const std::string & state);

    /*! Switch case for user mode. */
    void RunArmSpecific(void);

    /*! Homing procedure, home all joints except last one using potentiometers as reference. */
    void RunHomingCalibrateArm(void);

    /*! Change actuator to joint coupling matrices, needs to power off PID and then reenable. */
    void RunChangingCoupling(void);

    /*! Engaging adapter procedure. */
    void RunEngagingAdapter(void);

    /*! Engaging tool procedure. */
    void RunEngagingTool(void);

    /*! Cartesian constraint controller. */
    virtual void RunConstraintControllerCartesian(void);

    void EventHandlerAdapter(const prmEventButton & button);

    /*! Set tool present.  This should only be used by the tool event
      handler or for custom tools that can't be detected
      automatically. */
    void SetToolPresent(const bool & inserted);

    void EventHandlerTool(const prmEventButton & button);
    void EventHandlerManipClutch(const prmEventButton & button);

    void SetPositionCartesian(const prmPositionCartesianSet & newPosition);
    void SetJawPosition(const double & openAngle);
    void EnableJointsEventHandler(const vctBoolVec & enable);
    void CouplingEventHandler(const prmActuatorJointCoupling & coupling);

    /*! Event handlers for tools */
    //@{
    struct {
        mtsFunctionRead GetButton;
        bool IsPresent;
    } Adapter;

    struct {
        mtsFunctionRead GetButton;
        bool IsPresent;
    } Tool;
    //@}

    struct {
        mtsFunctionRead GetButton;
        bool IsPressed;
    } ManipClutch;

    // Functions for events
    struct {
        mtsFunctionWrite ManipClutch;
        mtsIntuitiveResearchKitArmTypes::RobotStateType ManipClutchPreviousState;
    } ClutchEvents;

    robManipulator * ToolOffset;
    vctFrm4x4 ToolOffsetTransformation;

    // Home Action
    unsigned int EngagingStage; // 0 requested
    unsigned int LastEngagingStage;
    struct {
        bool Started;
        mtsIntuitiveResearchKitArmTypes::RobotStateType PreviousState;
        bool CouplingForTool;
        bool WaitingForEnabledJoints, ReceivedEnabledJoints;
        vctBoolVec LastEnabledJoints, DesiredEnabledJoints;
        bool WaitingForCoupling, ReceivedCoupling;
        prmActuatorJointCoupling LastCoupling, DesiredCoupling, ToolCoupling;
        vctDoubleVec ToolEngageLowerPosition, ToolEngageUpperPosition;
        vctDoubleVec ToolJointLowerLimit, ToolJointUpperLimit;
        vctDoubleVec NoToolJointLowerLimit, NoToolJointUpperLimit;
        vctDoubleVec TorqueLowerLimit, TorqueUpperLimit;
    } CouplingChange;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitPSM);

#endif // _mtsIntuitiveResearchKitPSM_h
