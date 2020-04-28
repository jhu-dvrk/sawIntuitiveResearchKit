/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Pretham Chalasani, Anton Deguet
  Created on: 2016-11-04

  (C) Copyright 2016-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsSocketClientPSM_h
#define _mtsSocketClientPSM_h

#include <sawIntuitiveResearchKit/mtsSocketBasePSM.h>

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
    void SetDesiredState(const std::string & state);
    void GetDesiredState(std::string & state) const;
    void GetCurrentState(std::string & state) const;

    void Freeze(void);
    void servo_cp(const prmPositionCartesianSet & position);
    void jaw_servo_jp(const prmPositionJointSet & position);
    void UpdateApplication(void);
    void ReceivePSMStateData(void);
    void SendPSMCommandData(void);

private:
    prmPositionCartesianGet PositionCartesianCurrent;
    prmStateJoint m_jaw_measured_js;
    mtsInterfaceProvided * mInterface;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsSocketClientPSM);

#endif // _mtsSocketClientPSM_h
