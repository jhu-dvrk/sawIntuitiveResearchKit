/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Pretham Chalasani, Anton Deguet
  Created on: 2016-11-04

  (C) Copyright 2016-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsSocketBasePSM_h
#define _mtsSocketBasePSM_h

#include <cisstCommon/cmnUnits.h>
#include <cisstOSAbstraction/osaSocket.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstParameterTypes/prmPositionCartesianSet.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmTypes.h>
#include <sawIntuitiveResearchKit/socketMessages.h>

#define VERSION 10000
#define BUFFER_SIZE 1024
#define CLIENT_MSG_SIZE 140
#define SERVER_MSG_SIZE 140

#define TIMEOUT 4.0 * cmn_ms

class mtsSocketBasePSM : public mtsTaskPeriodic
{

public:
    mtsSocketBasePSM(const std::string & componentName, const double periodInSeconds,
                     const std::string & ip, const unsigned int port,
                     bool isServer);
    ~mtsSocketBasePSM() {}

    void Startup(void) {}
    void Cleanup(void);
    void UpdateStatistics(void);

protected:
    // UDP details
    struct {
        socketCommandPSM Data;
        osaSocket * Socket;
        short IpPort;
        char Buffer[BUFFER_SIZE];
    } Command;

    struct {
        socketStatePSM Data;
        osaSocket * Socket;
        short IpPort;
        char Buffer[BUFFER_SIZE];
    } State;

    std::string IpAddress;
    bool mIsServer;
    const osaTimeServer & mTimeServer;
    socketMessages::StateType CurrentState, DesiredState;

private:
    unsigned int mPacketsLost;
    unsigned int mPacketsDelayed;
    double mLoopTime;
};

#endif // _mtsSocketBasePSM_h
