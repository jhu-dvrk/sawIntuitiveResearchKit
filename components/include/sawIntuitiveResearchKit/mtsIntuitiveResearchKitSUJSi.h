/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2022-07-27

  (C) Copyright 2022 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsIntuitiveResearchKitSUJSi_h
#define _mtsIntuitiveResearchKitSUJSi_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmOperatingState.h>
#include <sawIntuitiveResearchKit/mtsStateMachine.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmTypes.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

// forward declaration
class mtsIntuitiveResearchKitSUJSiArmData;

class CISST_EXPORT mtsIntuitiveResearchKitSUJSi: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    static const size_t NumberOfJoints = 4;
    static const size_t NumberOfBrakes = 3;

    mtsIntuitiveResearchKitSUJSi(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitSUJSi(const mtsTaskPeriodicConstructorArg & arg);
    inline ~mtsIntuitiveResearchKitSUJSi() {}

    void Configure(const std::string & filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    void set_simulated(void);

protected:

    void Init(void);

    /*! Get data from the PID level based on current state. */
    void GetRobotData(void);

    /*! Logic used to read the potentiometer values and updated the
      appropriate joint values based on the mux state. */
    void GetAndConvertPotentiometerValues(void);

    void UpdateOperatingStateAndBusy(const prmOperatingState::StateType & state,
                                     const bool isBusy);
    void StateChanged(void);
    void RunAllStates(void); // this should happen for all states

    virtual void EnterDisabled(void);
    virtual void TransitionDisabled(void);

    virtual void EnterPowering(void);
    virtual void TransitionPowering(void);

    virtual void EnterEnabled(void);
    virtual void RunEnabled(void);
    virtual void TransitionEnabled(void);

    /*! Verify that the state transition is possible, initialize
      global variables for the desired state and finally set the
      state. */
    void SetDesiredState(const std::string & state);

    /*! crtk operating state command.  Currently supports "enable" and
      "disable". */
    virtual void state_command(const std::string & command);

    // Arm state machine
    mtsStateMachine mArmState;
    bool m_powered = false;

    // Just to have read commands to retrieve states
    mtsStateTable mStateTableState;
    mtsStdString mStateTableStateCurrent;
    mtsStdString mStateTableStateDesired;
    prmOperatingState m_operating_state; // crtk operating state

    void SetHomed(const bool homed);

    // Functions for events
    struct {
        mtsFunctionWrite desired_state;
        mtsFunctionWrite current_state;
        mtsFunctionWrite operating_state;
    } state_events;
    mtsInterfaceProvided * mInterface;

    vctFixedSizeVector<mtsIntuitiveResearchKitSUJSiArmData *, 4> Arms;
    size_t BaseFrameArmIndex; // arm used to provide base frame to all other SUJ arms, traditionally the ECM

    // Flag to determine if this is connected to actual IO/hardware or simulated
    bool m_simulated;
    double mSimulatedTimer;

    void ErrorEventHandler(const mtsMessage & message);
    void DispatchError(const std::string & message);
    void DispatchWarning(const std::string & message);
    void DispatchStatus(const std::string & message);
    void DispatchState(void);
    void DispatchOperatingState(void);
    void EngageBrakes(void);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitSUJSi);

#endif // _mtsIntuitiveResearchKitSUJSi_h
