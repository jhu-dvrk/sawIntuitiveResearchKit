/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-15

  (C) Copyright 2013-2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitPSM_h
#define _mtsIntuitiveResearchKitPSM_h

#include <memory>

#include <cisstParameterTypes/prmActuatorJointCoupling.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArm.h>
#include <sawIntuitiveResearchKit/mtsToolList.h>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

// forward declaration, definition in mtsIntuitiveResearchKitPSM.cpp
class GravityCompensationPSM;

class CISST_EXPORT mtsIntuitiveResearchKitPSM: public mtsIntuitiveResearchKitArm
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 public:
    mtsIntuitiveResearchKitPSM(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitPSM(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsIntuitiveResearchKitPSM();
    
    void set_simulated(void) override;

 protected:
    void set_generation(const dvrk::generation generation) override;
    void load_tool_list(const cmnPath & path,
                        const std::string & indexFile = "tool/index.json");

    void tool_list_size(size_t & size) const;
    void tool_name(const size_t & index, std::string & name) const;
    void tool_full_description(const size_t & index, std::string & description) const;

    void PostConfigure(const Json::Value & jsonConfig,
                       const cmnPath & configPath,
                       const std::string & filename) override;
    void ConfigureGC(const Json::Value & jsonConfig,
                       const cmnPath & configPath,
                       const std::string & filename) override;
    virtual bool ConfigureTool(const std::string & filename);

    /*! Configuration methods */
    inline size_t number_of_joints(void) const override {
        return 7;
    }

    inline size_t number_of_joints_kinematics(void) const override {
        return m_snake_like ? 8 : 6;
    }

    inline size_t number_of_brakes(void) const override {
        switch (m_generation) {
        case dvrk::generation::Classic:
            return 0;
            break;
        case dvrk::generation::Si:
            return 3;
            break;
        default:
            return 0;
        }
        return 0;
    }

    void UpdateStateJointKinematics(void) override;
    void ToJointsPID(const vctDoubleVec &jointsKinematics, vctDoubleVec &jointsPID) override;


    robManipulator::Errno InverseKinematics(vctDoubleVec & jointSet,
                                            const vctFrm4x4 & cartesianGoal) const override;

    bool is_safe_for_cartesian_control(void) const override;


    void Init(void) override;

    bool is_homed(void) const override;
    void unhome(void) override;
    bool is_joint_ready(void) const override;
    bool is_cartesian_ready(void) const override;

    // state related methods
    void SetGoalHomingArm(void) override;
    void TransitionHomed(void); // for adapter/tool detection

    // methods used in change coupling/engaging
    void update_configuration_js_no_tool(void);
    void update_configuration_js(void) override;

    // engaging adapter
    void EnterEngagingAdapter(void);
    void RunEngagingAdapter(void);

    // engaging tool
    void EnterEngagingTool(void);
    void RunEngagingTool(void);
    void EnterToolEngaged(void);
    void TransitionToolEngaged(void);

    // manual mode
    void EnterManual(void);
    void RunManual(void);
    void LeaveManual(void);

    void EventHandlerAdapter(const prmEventButton & button);

    /*! This should be called to set either m_tool_present or m_tool_configured. */
    void set_tool_present_and_configured(const bool & present, const bool & configured);

    /*! Method called when adapter is detected. */
    void set_adapter_present(const bool & present);

    /*! Emulate tool present.  This can be used for custom instruments
      that don't have the Dallas chip installed.  This will trigger
      the engage motion on the last 4 actuators, moving back and forth
      using the range defined in the instrument definition file
      (.json). */
    void emulate_tool_present(const bool & present);

    void EventHandlerTool(const prmEventButton & button);
    void EventHandlerManipClutch(const prmEventButton & button);
    void EventHandlerSUJClutch(const prmEventButton & button);

    double clip_jaw_jp(const double jp);
    void jaw_servo_jp(const prmPositionJointSet & jp);
    void jaw_move_jp(const prmPositionJointSet & jp);
    void jaw_servo_jf(const prmForceTorqueJointSet & jf);

    void servo_jp_internal(const vctDoubleVec & jp,
                           const vctDoubleVec & jv) override;
    void servo_jf_internal(const vctDoubleVec & jf) override;
    void feed_forward_jf_internal(const vctDoubleVec & jf) override;

    void control_move_jp_on_stop(const bool reached) override;

    /*! Event handlers for tools */
    //@{
    struct {
        mtsFunctionRead GetButton;
        bool IsPresent;
        bool NeedEngage = false;
        bool IsEngaged = false;
        bool IsEmulated = false;
    } Adapter, Tool;
    //@}

    struct {
        mtsFunctionRead GetButton;
        bool IsPressed;
    } ManipClutch;

    struct {
        mtsFunctionWrite Brake;
    } SUJClutch;

    struct {
        mtsFunctionVoid TriggerRead;
    } Dallas;

    /*! Set tool type.  Uses string as defined in
      mtsIntuitiveResearchKitToolTypes.cdg, upper case with separating
      underscores. */
    void set_tool_type(const std::string & toolType);

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
    mtsToolList mToolList;
    size_t mToolIndex;
    mtsIntuitiveResearchKitToolTypes::Detection mToolDetection;
    bool m_tool_present = false;
    bool m_tool_configured = false;
    bool m_tool_type_requested = false;
    struct {
        mtsFunctionWrite tool_type;
        mtsFunctionVoid tool_type_request;
    } ToolEvents;

    /*! 5mm tools with 8 joints */
    bool m_snake_like = false;

    robManipulator * ToolOffset = nullptr;
    vctFrm4x4 ToolOffsetTransformation;

    prmStateJoint m_jaw_measured_js, m_jaw_setpoint_js;
    prmConfigurationJoint m_jaw_configuration_js;
    double m_jaw_servo_jp;
    double m_jaw_servo_jf;

    // Home Action
    unsigned int EngagingStage; // 0 requested
    unsigned int LastEngagingStage;

    vctDoubleVec m_tool_engage_lower_position,
        m_tool_engage_upper_position;

    std::unique_ptr<GravityCompensationPSM> m_gc;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitPSM);

#endif // _mtsIntuitiveResearchKitPSM_h
