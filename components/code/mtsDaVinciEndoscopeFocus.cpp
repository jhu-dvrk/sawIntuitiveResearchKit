/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-12-18

  (C) Copyright 2017-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/mtsDaVinciEndoscopeFocus.h>

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsDaVinciEndoscopeFocus, mtsTaskFromSignal, mtsTaskConstructorArg)

mtsDaVinciEndoscopeFocus::mtsDaVinciEndoscopeFocus(const std::string & componentName):
    mtsTaskFromSignal(componentName)
{
    Init();
}

mtsDaVinciEndoscopeFocus::mtsDaVinciEndoscopeFocus(const mtsTaskConstructorArg & arg):
    mtsTaskFromSignal(arg)
{
    Init();
}

void mtsDaVinciEndoscopeFocus::Init(void)
{
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("FocusIn");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsDaVinciEndoscopeFocus::FocusIn,
                                                this, "Button");
    }
    interfaceRequired = AddInterfaceRequired("FocusOut");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsDaVinciEndoscopeFocus::FocusOut,
                                                this, "Button");
    }
    interfaceRequired = AddInterfaceRequired("EndoscopeFocusIn");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("SetValue", RobotIO.FocusIn);
    }
    interfaceRequired = AddInterfaceRequired("EndoscopeFocusOut");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("SetValue", RobotIO.FocusOut);
    }
}


void mtsDaVinciEndoscopeFocus::Configure(const std::string & CMN_UNUSED(filename))
{
}


void mtsDaVinciEndoscopeFocus::Startup(void)
{
}


void mtsDaVinciEndoscopeFocus::Run(void)
{
    ProcessQueuedEvents();
    ProcessQueuedCommands();
}


void mtsDaVinciEndoscopeFocus::Cleanup(void)
{
    RobotIO.FocusOut(true);
    RobotIO.FocusIn(true);
}

void mtsDaVinciEndoscopeFocus::FocusIn(const prmEventButton & event)
{
    switch (event.Type()) {
    case prmEventButton::PRESSED:
        RobotIO.FocusIn(false);
        break;
    case prmEventButton::RELEASED:
        RobotIO.FocusIn(true);
        break;
    default:
        break;
    }
}

void mtsDaVinciEndoscopeFocus::FocusOut(const prmEventButton & event)
{
    switch (event.Type()) {
    case prmEventButton::PRESSED:
        RobotIO.FocusOut(false);
        break;
    case prmEventButton::RELEASED:
        RobotIO.FocusOut(true);
        break;
    default:
        break;
    }
}
