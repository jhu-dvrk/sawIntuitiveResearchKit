/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2016-01-21

  (C) Copyright 2016-2026 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsTeleOperationECMBase_h
#define _mtsTeleOperationECMBase_h

#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <sawIntuitiveResearchKit/mtsStateMachine.h>
#include <sawIntuitiveResearchKit/teleop_ECM_configuration.h>

// always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class CISST_EXPORT mtsTeleOperationECMBase: public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsTeleOperationECMBase(const std::string & componentName, const double periodInSeconds);
    mtsTeleOperationECMBase(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsTeleOperationECMBase() override;

    void Configure(const std::string & filename = "") override;
    virtual void Configure(const Json::Value & jsonConfig);
    void Startup(void) override;
    void Run(void) override;
    void Cleanup(void) override;

    virtual void set_scale(const double & scale);

    // Event Handlers
    void arm_error_event_handler(const mtsMessage & message);
    void clutch_event_handler(const prmEventButton & button);

protected:
    virtual void Init(void);
    virtual void Clutch(const bool & clutch);

    dvrk::teleop_ECM_configuration m_config;

    // Functions for events
    struct {
        mtsFunctionWrite desired_state;
        mtsFunctionWrite current_state;
        mtsFunctionWrite following;
    } MessageEvents;
    mtsInterfaceProvided * mInterface = nullptr;

    struct {
        mtsFunctionWrite scale;
    } ConfigurationEvents;

    void SetDesiredState(const std::string & state);
    void state_command(const std::string & command);

    void StateChanged(void);
    virtual void RunAllStates(void);
    virtual void TransitionDisabled(void);
    virtual void EnterSettingArmsState(void);
    virtual void TransitionSettingArmsState(void) {};
    virtual void EnterCheckingData(void);
    virtual void TransitionCheckingData(void) {};
    virtual void EnterEnabled(void) {};
    virtual void RunEnabled(void) {};
    virtual void TransitionEnabled(void);

    struct {
        mtsFunctionRead  measured_cp;
        mtsFunctionRead  operating_state;
        mtsFunctionWrite state_command;

        prmPositionCartesianGet m_measured_cp;
    } mECM;

    mtsStateTable * mConfigurationStateTable = nullptr;

    bool m_clutched = false;
    bool m_data_check_warning_sent = false;

    mtsStateMachine mTeleopState;
    double mInStateTimer = 0.0;

    bool m_following = false;
    void set_following(const bool following);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsTeleOperationECMBase);

#endif // _mtsTeleOperationECMBase_h
