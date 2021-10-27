/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Pretham Chalasani, Anton Deguet
  Created on: 2016-11-04

  (C) Copyright 2016-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsSocketClientPSM_h
#define _mtsSocketClientPSM_h

#include <sawIntuitiveResearchKit/mtsSocketBasePSM.h>
#include <cisstParameterTypes/prmOperatingState.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmStateJoint.h>

class mtsSocketClientPSM: public mtsSocketBasePSM
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public :
    mtsSocketClientPSM(const std::string & componentName, const double periodInSeconds,
                       const std::string & ip, const unsigned int port);
    mtsSocketClientPSM(const mtsTaskPeriodicConstructorArg & arg);

    void Configure(const std::string & fileName = "");
    void Run(void);

protected:
    void state_command(const std::string & state);

    void Freeze(void);
    void servo_cp(const prmPositionCartesianSet & position);
    void jaw_servo_jp(const prmPositionJointSet & position);
    void UpdateApplication(void);
    void ReceivePSMStateData(void);
    void SendPSMCommandData(void);

private:
    prmPositionCartesianGet m_setpoint_cp;
    prmStateJoint m_jaw_setpoint_js;
    mtsInterfaceProvided * mInterface;

    socketMessages::StateType PreviousState;
    prmOperatingState m_operating_state;
    mtsFunctionWrite operating_state_event;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsSocketClientPSM);

#endif // _mtsSocketClientPSM_h
