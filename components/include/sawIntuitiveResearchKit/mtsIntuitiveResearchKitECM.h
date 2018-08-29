/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-15

  (C) Copyright 2013-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitECM_h
#define _mtsIntuitiveResearchKitECM_h

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArm.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class CISST_EXPORT mtsIntuitiveResearchKitECM: public mtsIntuitiveResearchKitArm
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitECM(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitECM(const mtsTaskPeriodicConstructorArg & arg);
    inline ~mtsIntuitiveResearchKitECM() {}

    void SetSimulated(void);
    void Configure(const std::string & filename);

protected:

    /*! Configuration methods */
    inline size_t NumberOfAxes(void) const {
        return 4;
    }

    inline size_t NumberOfJoints(void) const {
        return 4;
    }

    inline size_t NumberOfJointsKinematics(void) const {
        return 4;
    }

    inline size_t NumberOfBrakes(void) const {
        return 3;
    }

    robManipulator::Errno InverseKinematics(vctDoubleVec & jointSet,
                                            const vctFrm4x4 & cartesianGoal);

    // see base class
    inline bool IsSafeForCartesianControl(void) const {
        return (JointsKinematics.Position().at(2) > 50.0 * cmn_mm);
    }

    void Init(void);

    // state related methods
    void SetGoalHomingArm(void);
    void TransitionArmHomed(void);
    void EnterManual(void);
    void RunManual(void);

    void EventHandlerTrackingError(void);
    void EventHandlerManipClutch(const prmEventButton & button);

    struct {
        mtsFunctionRead GetButton;
        bool IsPressed;
    } ManipClutch;

    // Functions for events
    struct {
        mtsFunctionWrite ManipClutch;
        std::string ManipClutchPreviousState;
    } ClutchEvents;

    // tooltip, used for up/down endoscopes
    robManipulator * ToolOffset;
    vctFrm4x4 ToolOffsetTransformation;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitECM);

#endif // _mtsIntuitiveResearchKitECM_h
