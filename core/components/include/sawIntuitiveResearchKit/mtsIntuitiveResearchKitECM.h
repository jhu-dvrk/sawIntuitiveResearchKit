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


#ifndef _mtsIntuitiveResearchKitECM_h
#define _mtsIntuitiveResearchKitECM_h

#include <memory>

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArm.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitEndoscopeTypes.h>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

// forward declaration, definition in mtsIntuitiveResearchKitECM.cpp
class GravityCompensationECM;

class CISST_EXPORT mtsIntuitiveResearchKitECM: public mtsIntuitiveResearchKitArm
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 public:
    mtsIntuitiveResearchKitECM(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitECM(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsIntuitiveResearchKitECM();

    void set_simulated(void) override;

 protected:
    void set_generation(const GenerationType generation) override;
    void PostConfigure(const Json::Value & jsonConfig,
                       const cmnPath & configPath,
                       const std::string & filename) override;
    void ConfigureGC(const Json::Value & jsonConfig,
                       const cmnPath & configPath,
                       const std::string & filename) override;

    /*! Configuration methods */
    inline size_t number_of_joints(void) const override {
        return 4;
    }

    inline size_t number_of_joints_kinematics(void) const override {
        return 4;
    }

    inline size_t number_of_brakes(void) const override {
        return 3;
    }

    robManipulator::Errno InverseKinematics(vctDoubleVec & jointSet,
                                            const vctFrm4x4 & cartesianGoal) const override;

    // see base class
    inline bool is_safe_for_cartesian_control(void) const override {
        return (m_kin_measured_js.Position().at(2) > 50.0 * cmn_mm);
    }

    void CreateManipulator(void) override;
    void Init(void) override;

    bool is_homed(void) const override;
    void unhome(void) override;
    bool is_joint_ready(void) const override;
    bool is_cartesian_ready(void) const override;

    // state related methods
    void SetGoalHomingArm(void) override;
    void EnterHomed(void) override;
    void EnterManual(void);
    void RunManual(void);
    void LeaveManual(void);

    void EventHandlerTrackingError(void);
    void EventHandlerManipClutch(const prmEventButton & button);
    void EventHandlerSUJClutch(const prmEventButton & button);

    struct {
        mtsFunctionRead GetButton;
        bool IsPressed;
    } ManipClutch;

    // Functions for events
    struct {
        mtsFunctionWrite ManipClutch;
        std::string ManipClutchPreviousState;
    } ClutchEvents;

    struct {
        mtsFunctionWrite Brake;
    } SUJClutch;

    /*! Set endoscope type.  Uses string as defined in
      mtsIntuitiveResearchKitEndoscopeTypes.cdg, upper case with separating
      underscores. */
    void set_endoscope_type(const std::string & endoscopeType);

    mtsIntuitiveResearchKitEndoscopeTypes::Type m_endoscope_type;
    bool m_endoscope_configured = false;
    struct {
        mtsFunctionWrite endoscope_type;
    } EndoscopeEvents;

    // tooltip, used for up/down endoscopes
    robManipulator * ToolOffset;
    vctFrm4x4 ToolOffsetTransformation;

    std::unique_ptr<GravityCompensationECM> m_gc;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitECM);

#endif // _mtsIntuitiveResearchKitECM_h
