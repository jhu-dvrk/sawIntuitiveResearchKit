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


#ifndef _mtsIntuitiveResearchKitPSM_h
#define _mtsIntuitiveResearchKitPSM_h

#include <cisstParameterTypes/prmActuatorJointCoupling.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArm.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitToolTypes.h>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>


class CISST_EXPORT mtsIntuitiveResearchKitPSM: public mtsIntuitiveResearchKitArm
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitPSM(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitPSM(const mtsTaskPeriodicConstructorArg & arg);
    inline ~mtsIntuitiveResearchKitPSM() {}

    void SetSimulated(void) override;

protected:

    void PostConfigure(const Json::Value & jsonConfig,
                       const cmnPath & configPath,
                       const std::string & filename) override;
    virtual void ConfigureTool(const std::string & filename);

    /*! Configuration methods */
    inline size_t NumberOfAxes(void) const override {
        return 7;
    }

    inline size_t NumberOfJoints(void) const override {
        return 7;
    }

    inline size_t NumberOfJointsKinematics(void) const override {
        return mSnakeLike ? 8 : 6;
    }

    inline size_t NumberOfBrakes(void) const override {
        return 0;
    }

    void UpdateStateJointKinematics(void) override;
    void ToJointsPID(const vctDoubleVec &jointsKinematics, vctDoubleVec &jointsPID) override;

    robManipulator::Errno InverseKinematics(vctDoubleVec & jointSet,
                                            const vctFrm4x4 & cartesianGoal) override;

    // see base class
    inline bool IsSafeForCartesianControl(void) const override {
        return (StateJointKinematics.Position().at(2) >= mtsIntuitiveResearchKit::PSMOutsideCannula);
    }


    void Init(void) override;


    // state related methods
    void SetGoalHomingArm(void) override;
    void EnterArmHomed(void);
    void RunArmHomed(void); // mostly to allow joint control without tool nor adapter
    void LeaveArmHomed(void);
    void TransitionArmHomed(void);

    // methods used in change coupling/engaging
    void RunChangingCoupling(void);
    void UpdateConfigurationJointPID(const bool toolPresent);

    // engaging adapter
    void EnterChangingCouplingAdapter(void);
    inline void RunChangingCouplingAdapter(void) {
        RunChangingCoupling();
    }
    void EnterEngagingAdapter(void);
    void RunEngagingAdapter(void);
    void TransitionAdapterEngaged(void);
    // engaging tool
    void EnterChangingCouplingTool(void);
    inline void RunChangingCouplingTool(void) {
        RunChangingCoupling();
    }
    void EnterEngagingTool(void);
    void RunEngagingTool(void);
    void EnterToolEngaged(void);
    void TransitionToolEngaged(void);

    // manual mode
    void EnterManual(void);
    void EventHandlerAdapter(const prmEventButton & button);

    /*! Set tool present.  This should only be used by the tool event
      handler or for custom tools that can't be detected
      automatically. */
    void SetAdapterPresent(const bool & present);
    void SetToolPresent(const bool & present);

    void EventHandlerTool(const prmEventButton & button);
    void EventHandlerManipClutch(const prmEventButton & button);

    void SetPositionJaw(const prmPositionJointSet & jawPosition);
    void SetPositionGoalJaw(const prmPositionJointSet & jawPosition);
    void SetEffortJaw(const prmForceTorqueJointSet & effort);

    void SetPositionJointLocal(const vctDoubleVec & newPosition) override;
    void SetEffortJointLocal(const vctDoubleVec & newEffort) override;

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

    struct {
        mtsFunctionVoid TriggerRead;
    } Dallas;

    /*! Set tool type.  Uses string as defined in
      mtsIntuitiveResearchKitToolTypes.cdg, upper case with separating
      underscores. */
    void SetToolType(const std::string & toolType);

    /*! Event handler for tool types sent by the IO level based on
      info from Dallas Chip on tools */
    void EventHandlerToolType(const std::string & toolType);

    // Functions for events
    struct {
        mtsFunctionWrite ManipClutch;
        std::string ManipClutchPreviousState;
        bool PIDEnabledPreviousState;
    } ClutchEvents;

    /*! Configuration for tool detection, either using Dallas Chip,
      manual or fixed based on configuration file. */
    mtsIntuitiveResearchKitToolTypes::Detection mToolDetection;
    mtsIntuitiveResearchKitToolTypes::Type mToolType;
    bool mToolConfigured = false;
    bool mToolTypeRequested = false;
    struct {
        mtsFunctionWrite ToolType;
        mtsFunctionVoid ToolTypeRequest;
    } ToolEvents;

    /*! 5mm tools with 8 joints */
    bool mSnakeLike = false;

    robManipulator * ToolOffset = nullptr;
    vctFrm4x4 ToolOffsetTransformation;

    prmStateJoint StateJaw, StateJawDesired;
    double JawGoal;
    double EffortJawSet;

    // Home Action
    unsigned int EngagingStage; // 0 requested
    unsigned int LastEngagingStage;

    bool mAdapterNeedEngage = false;
    bool mToolNeedEngage = false;

    struct {
        bool Started;
        std::string NextState;
        bool CouplingForTool;
        bool WaitingForEnabledJoints, ReceivedEnabledJoints;
        vctBoolVec LastEnabledJoints, DesiredEnabledJoints;
        bool WaitingForCoupling, ReceivedCoupling;
        prmActuatorJointCoupling LastCoupling, DesiredCoupling, ToolCoupling;
        vctDoubleVec ToolEngageLowerPosition, ToolEngageUpperPosition;
        prmConfigurationJoint ToolConfiguration;
        prmConfigurationJoint NoToolConfiguration;
        prmConfigurationJoint JawConfiguration;
    } CouplingChange;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitPSM);

#endif // _mtsIntuitiveResearchKitPSM_h
