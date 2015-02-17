/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-05-15

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitECM_h
#define _mtsIntuitiveResearchKitECM_h

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArm.h>


class mtsIntuitiveResearchKitECM: public mtsIntuitiveResearchKitArm
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsIntuitiveResearchKitECM(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitECM(const mtsTaskPeriodicConstructorArg & arg);
    inline ~mtsIntuitiveResearchKitECM() {}

protected:

    /*! Configuration methods */
    inline size_t NumberOfJoints(void) const {
        return 4;
    }

    inline size_t NumberOfAxes(void) const {
        return 4;
    }

    inline size_t NumberOfBrakes(void) const {
        return 3;
    }

    inline bool UsePIDTrackingError(void) const {
        return true;
    }

    void Init(void);

    /*! Verify that the state transition is possible, initialize global
      variables for the desired state and finally set the state. */
    void SetState(const mtsIntuitiveResearchKitArmTypes::RobotStateType & newState);

    void SetRobotControlState(const std::string & state);

    /*! Homing procedure, home all joints except last one using potentiometers as reference. */
    void RunHomingCalibrateArm(void);

    void EventHandlerTrackingError(void);

    void EventHandlerManipClutch(const prmEventButton & button);

    struct {
        mtsFunctionRead GetButton;
        bool IsPressed;
    } ManipClutch;

    // Functions for events
    struct {
        mtsFunctionWrite ManipClutch;
        mtsFunctionWrite SUJClutch;
        mtsIntuitiveResearchKitArmTypes::RobotStateType ManipClutchPreviousState;
    } ClutchEvents;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitECM);

#endif // _mtsIntuitiveResearchKitECM_h
