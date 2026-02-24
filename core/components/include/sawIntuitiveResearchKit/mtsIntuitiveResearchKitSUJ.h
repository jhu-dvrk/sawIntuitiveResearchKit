/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Youri Tan
  Created on: 2014-11-07

  (C) Copyright 2014-2026 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsIntuitiveResearchKitSUJ_h
#define _mtsIntuitiveResearchKitSUJ_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmOperatingState.h>
#include <sawIntuitiveResearchKit/mtsStateMachine.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitControlTypes.h>

#include <cisstParameterTypes/prmSimulationType.h>

#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

// forward declaration
class mtsIntuitiveResearchKitSUJArmData;

class CISST_EXPORT mtsIntuitiveResearchKitSUJ: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 public:
    static const size_t NumberOfJoints = 4;
    static const size_t NumberOfBrakes = 3;

    mtsIntuitiveResearchKitSUJ(const std::string & componentName, const double periodInSeconds);
    mtsIntuitiveResearchKitSUJ(const mtsTaskPeriodicConstructorArg & arg);
    inline ~mtsIntuitiveResearchKitSUJ() {}

    void Configure(const std::string & filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    void set_simulation_mode(const prmSimulationType & mode);

 protected:

    void init(void);

    /*! Get data from the PID level based on current state. */
    void get_robot_data(void);

    /*! Logic used to read the potentiometer values and updated the
      appropriate joint values based on the mux state. */
    void get_and_convert_potentiometers(void);

    /*! Update the forward kinematics for all arms. */
    void update_forward_kinematics(void);

    void update_operating_state_and_busy(const prmOperatingState::StateType & state,
                                         const bool isBusy);
    void state_changed(void);
    void run_all_states(void); // this should happen for all states

    virtual void enter_DISABLED(void);
    virtual void transition_DISABLED(void);

    virtual void enter_POWERING(void);
    virtual void transition_POWERING(void);

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

    prmSimulationType m_simulation_mode;

    // Arm state machine
    mtsStateMachine m_state_machine;
    bool m_powered = false;
    prmOperatingState m_operating_state;

    void set_homed(const bool homed);

    /*! Change the reference arm, ECM by default */
    void set_reference_arm(const std::string & arm_name);

    /*! Set velocity for motorized PSM lift. normalized between -1.0 and 1.0. */
    void set_lift_velocity(const double & velocity);

    /*! Event handler for PID errors. */
    void error_event_handler(const mtsMessage & message);

    /*! Motor down button. */
    void motor_down_event_handler(const prmEventButton & button);

    /*! Motor up button. */
    void motor_up_event_handler(const prmEventButton & button);

    // Required interface
    struct {
        //! Enable Robot Power
        mtsFunctionVoid PowerOnSequence;
        mtsFunctionWrite PowerOffSequence;
        mtsFunctionRead GetEncoderChannelA;
        mtsFunctionRead GetActuatorAmpStatus;
        mtsFunctionWrite SetActuatorCurrent;
        mtsFunctionRead GetAnalogInputVolts;
    } RobotIO;

    // Functions for events
    struct {
        mtsFunctionWrite operating_state;
    } state_events;
    mtsInterfaceProvided * m_interface;

    // Functions to control MUX
    struct {
        mtsFunctionRead GetValue;
        mtsFunctionWrite SetValue;
    } no_mux_reset;

    struct {
        mtsFunctionRead GetValue;
        mtsFunctionWrite SetValue;
    } mux_increment;

    void reset_mux(void);
    double m_mux_timer;
    vctBoolVec m_mux_state;
    size_t m_mux_index, m_mux_index_expected;

    // Functions to control motor on SUJ3
    struct {
        mtsFunctionWrite DisablePWM;
        mtsFunctionWrite SetPWMDutyCycle;
    } PWM;

    // mtsIntuitiveResearchKitArmTypes::RobotStateType mRobotState;

    // Home Action
    double m_homing_timer;

    // Clutch / brake timer
    double m_previous_tic;
    vctDoubleVec m_brake_currents;

    vctDynamicVector<vctDoubleVec> m_voltage_samples;
    const size_t m_voltage_samples_number;
    size_t m_voltage_samples_counter;
    vctDoubleVec m_voltages;
    vctFixedSizeVector<mtsIntuitiveResearchKitSUJArmData *, 4> m_sarms;
    size_t m_reference_arm_index; // arm used to provide base frame to all other SUJ arms, traditionally the ECM

    void dispatch_error(const std::string & message);
    void dispatch_warning(const std::string & message);
    void dispatch_status(const std::string & message);
    void dispatch_operating_state(void);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitSUJ);

#endif // _mtsIntuitiveResearchKitSUJ_h
