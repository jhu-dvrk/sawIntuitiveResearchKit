/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Peter Kazanzides
  Created on: 2013-12-02

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitUDPStreamer.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmEventButton.h>

CMN_IMPLEMENT_SERVICES_DERIVED(mtsIntuitiveResearchKitUDPStreamer, mtsTaskPeriodic)

mtsIntuitiveResearchKitUDPStreamer::mtsIntuitiveResearchKitUDPStreamer(const std::string & name,
                                                                       double period, const std::string & ip, unsigned short port) :
    mtsTaskPeriodic(name, period),
    Socket(osaSocket::UDP),
    SocketConfigured(false),
    Clutch(false),
    Coag(false)
{
    mtsInterfaceProvided * provided = AddInterfaceProvided("Configuration");
    if (provided) {
        provided->AddCommandWrite(&mtsIntuitiveResearchKitUDPStreamer::SetDestination, this, "SetDestination");
    }
    mtsInterfaceRequired * required = AddInterfaceRequired("Robot");
    if (required) {
        required->AddFunction("GetPositionCartesian", GetPositionCartesian);
        required->AddFunction("GetGripperPosition", GetGripperPosition);
    }
    required = AddInterfaceRequired("Clutch");
    if (required) {
        required->AddEventHandlerWrite(&mtsIntuitiveResearchKitUDPStreamer::EventHandlerManipClutch, this, "Button");
    }
    required = AddInterfaceRequired("Coag");
    if (required) {
        required->AddEventHandlerWrite(&mtsIntuitiveResearchKitUDPStreamer::EventHandlerCoag, this, "Button");
    }
    if (!ip.empty()) {
        Socket.SetDestination(ip, port);
        SocketConfigured = true;
    }
}

mtsIntuitiveResearchKitUDPStreamer::~mtsIntuitiveResearchKitUDPStreamer()
{
}

void mtsIntuitiveResearchKitUDPStreamer::Configure(const std::string &ipPort)
{
    SetDestination(ipPort);
}

void mtsIntuitiveResearchKitUDPStreamer::Startup(void)
{
}

void mtsIntuitiveResearchKitUDPStreamer::Run(void)
{
    ProcessQueuedCommands();
    ProcessQueuedEvents();

    if (SocketConfigured) {
        // Packet format (9 doubles): buttons (clutch, coag), gripper, x, y, z, q0, qx, qy, qz
        // For the buttons: 0=None, 1=Clutch, 2=Coag, 3=Both
        double packet[9];
        if (Clutch) {
            packet[0] = 1.0;
        } else {
            packet[0] = 0.0;
        }
        if (Coag)
            packet[0] += 2.0;
        GetGripperPosition(packet[1]);
        prmPositionCartesianGet posCart;
        GetPositionCartesian(posCart);
        vct3 pos = posCart.Position().Translation();
        packet[2] = pos.X();
        packet[3] = pos.Y();
        packet[4] = pos.Z();
        vctQuatRot3 qrot(posCart.Position().Rotation());
        packet[5] = qrot.W();
        packet[6] = qrot.X();
        packet[7] = qrot.Y();
        packet[8] = qrot.Z();
        Socket.Send((char *)packet, sizeof(packet));
    }
}

void mtsIntuitiveResearchKitUDPStreamer::Cleanup(void)
{
    Socket.Close();
}

void mtsIntuitiveResearchKitUDPStreamer::SetDestination(const std::string &ipPort)
{
    size_t colon = ipPort.find(':');
    if (colon == std::string::npos) {
        CMN_LOG_CLASS_RUN_ERROR << "SetDestination: invalid address:port " << ipPort << std::endl;
    } else {
        unsigned short port;
        if ((sscanf(ipPort.c_str() + colon + 1, "%hu", &port) != 1)) {
            CMN_LOG_CLASS_RUN_ERROR << "SetDestination: invalid port " << ipPort << std::endl;
        
        } else {
            Socket.SetDestination(ipPort.substr(0, colon), port);
            SocketConfigured = true;
        }
    }
}

void mtsIntuitiveResearchKitUDPStreamer::EventHandlerManipClutch(const prmEventButton &button)
{
    Clutch = (button.Type() == prmEventButton::PRESSED);
}

void mtsIntuitiveResearchKitUDPStreamer::EventHandlerCoag(const prmEventButton &button)
{
    Coag = (button.Type() == prmEventButton::PRESSED);
}
