/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2023-06-16

  (C) Copyright 2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsIntuitiveResearchKitSUJFixed_h
#define _mtsIntuitiveResearchKitSUJFixed_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmOperatingState.h>
#include <sawIntuitiveResearchKit/mtsStateMachine.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmTypes.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

// notes for all SUJ classes
// - snake case: GetRobotData, SetDesiredState
// - mtsStdString -> std::string
// - do we need interface_provided mtsFunction for current_state, desired_state?
// - do we need dispatch_state? or just use operating_state
// - what are the events EventPositionCartesian{,Local} for?
// - renamed main interface Arm -> Cart

// forward declarations
class mtsIntuitiveResearchKitSUJFixedArmData;

class CISST_EXPORT mtsIntuitiveResearchKitSUJFixed: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 public:

    mtsIntuitiveResearchKitSUJFixed(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitSUJFixed(const mtsTaskPeriodicConstructorArg & arg);
    inline ~mtsIntuitiveResearchKitSUJFixed() {}

    void Configure(const std::string & filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    void set_simulated(void);

 protected:

    void init(void);

    /*! Get data from the PID level based on current state.  Does nothing. */
    void get_robot_data(void);

    /*! Update the forward kinematics for all arms.  In this case,
      just preprend inverse of IK for reference arm (ECM). */
    void update_forward_kinematics(void);

    void update_operating_state_and_busy(const prmOperatingState::StateType & state,
                                         const bool isBusy);
    void state_changed(void);
    void run_all_states(void); // this should happen for all states
    virtual void enter_DISABLED(void);
    virtual void enter_ENABLED(void);

    /*! Verify that the state transition is possible, initialize
      global variables for the desired state and finally set the
      state. */
    void set_desired_state(const std::string & state);

    /*! crtk operating state command.  Currently supports "enable" and
      "disable". */
    virtual void state_command(const std::string & command);

    /*! internal method used by state_command. */
    void set_homed(const bool homed);

    // state machine
    mtsStateMachine m_state_machine;
    prmOperatingState m_operating_state;

    /*! cisst interface to control state */
    struct {
        mtsFunctionWrite operating_state;
    } state_events;
    mtsInterfaceProvided * m_interface_provided;

    vctFixedSizeVector<mtsIntuitiveResearchKitSUJFixedArmData *, 4> m_sarms;
    size_t m_reference_arm_index; // arm used to provide base frame to all other SUJ arms, traditionally the ECM

    void error_event_handler(const mtsMessage & message);
    void dispatch_error(const std::string & message);
    void dispatch_warning(const std::string & message);
    void dispatch_status(const std::string & message);
    void dispatch_operating_state(void);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitSUJFixed);

#endif // _mtsIntuitiveResearchKitSUJFixed_h
