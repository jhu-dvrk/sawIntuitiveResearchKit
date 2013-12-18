/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  $Id$

  Author(s):  Peter Kazanzides
  Created on: 2013-12-02

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsIntuitiveResearchKitUDPStreamer_h
#define _mtsIntuitiveResearchKitUDPSTreamer_h

#include <cisstOSAbstraction/osaSocket.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>

#include <cisstMultiTask/mtsForwardDeclarations.h>
class prmEventButton;

class mtsIntuitiveResearchKitUDPStreamer : public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

 protected:

    osaSocket Socket;
    bool SocketConfigured;

    mtsFunctionRead GetPositionCartesian;
    mtsFunctionRead GetGripperPosition;
    bool Clutch;

    void SetDestination(const std::string &ipPort);
    void EventHandlerManipClutch(const prmEventButton &button);

 public:
    /*! Constructor
        \param name Name of the component
        \param period Period in seconds
        \param ip IP address for streaming UDP packets
        \param port Port for streaming UDP packets
    */
    mtsIntuitiveResearchKitUDPStreamer(const std::string &name, double period, const std::string &ip = "", unsigned short port = 0);

    /*! Destructor */
    virtual ~mtsIntuitiveResearchKitUDPStreamer();

    /*! Configure: set IP address and port number
        \param ipPort IP address and port number, separated by ':'
    */
    void Configure(const std::string &ipPort);

    void Startup(void);

    void Run(void);

    void Cleanup(void);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsIntuitiveResearchKitUDPStreamer)

#endif // _mtsIntuitiveResearchKitUDPStreamer_h
