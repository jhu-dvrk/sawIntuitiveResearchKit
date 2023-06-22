/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2022-07-27

  (C) Copyright 2022-2023 Johns Hopkins University (JHU), All Rights Reserved.

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

// forward declarations
class mtsIntuitiveResearchKitSUJSiArduino;
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

    void init(void);

    /*! Get data from the PID level based on current state. */
    void get_robot_data(void);

    /*! Logic used to read the potentiometer values and updated the
      appropriate joint values based on the mux state. */
    void get_and_convert_potentiometers(void);

    void update_operating_state_and_busy(const prmOperatingState::StateType & state,
                                         const bool isBusy);
    void state_changed(void);
    void run_all_states(void); // this should happen for all states

    virtual void enter_DISABLED(void);
    virtual void transition_DISABLED(void);

    virtual void enter_ENABLED(void);
    virtual void run_ENABLED(void);
    virtual void transition_ENABLED(void);

    /*! Verify that the state transition is possible, initialize
      global variables for the desired state and finally set the
      state. */
    void set_desired_state(const std::string & state);

    /*! crtk operating state command.  Currently supports "enable" and
      "disable". */
    virtual void state_command(const std::string & command);

    // Arm state machine
    mtsStateMachine m_state_machine;

    // Just to have read commands to retrieve states
    mtsStateTable m_state_table_operating_state;
    mtsStdString m_state_table_state_current;
    mtsStdString m_state_table_state_desired;
    prmOperatingState m_operating_state; // crtk operating state

    void set_homed(const bool homed);

    // Functions for events
    struct {
        mtsFunctionWrite desired_state;
        mtsFunctionWrite current_state;
        mtsFunctionWrite operating_state;
    } state_events;
    mtsInterfaceProvided * m_interface;

    mtsIntuitiveResearchKitSUJSiArduino * m_base_arduino = nullptr;
    vctFixedSizeVector<mtsIntuitiveResearchKitSUJSiArmData *, 4> m_arms;
    size_t m_reference_arm_index; // arm used to provide base frame to all other SUJ arms, traditionally the ECM

    // Flag to determine if this is connected to actual IO/hardware or simulated
    bool m_simulated;
    double m_simulated_timer;

    void dispatch_error(const std::string & message);
    void dispatch_warning(const std::string & message);
    void dispatch_status(const std::string & message);
    void dispatch_state(void);
    void dispatch_operating_state(void);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitSUJSi);

#endif // _mtsIntuitiveResearchKitSUJSi_h
