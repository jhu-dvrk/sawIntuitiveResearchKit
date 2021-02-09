/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-12-18

  (C) Copyright 2017-2021 Johns Hopkins University (JHU), All Rights Reserved.

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
    mtsInterfaceProvided * interfaceProvided = AddInterfaceProvided("Control");
    if (interfaceProvided) {
        interfaceProvided->AddCommandWrite(&mtsDaVinciEndoscopeFocus::lock,
                                           this, "lock");
        interfaceProvided->AddCommandWrite(&mtsDaVinciEndoscopeFocus::focus_in,
                                           this, "focus_in");
        interfaceProvided->AddCommandWrite(&mtsDaVinciEndoscopeFocus::focus_out,
                                           this, "focus_out");
        interfaceProvided->AddEventWrite(mEvents.locked, "locked", false);
        interfaceProvided->AddEventWrite(mEvents.focusing_in, "focusing_in", false);
        interfaceProvided->AddEventWrite(mEvents.focusing_out, "focusing_out", false);
    }
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("focus_in");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsDaVinciEndoscopeFocus::FocusInEventHandler,
                                                this, "Button");
    }
    interfaceRequired = AddInterfaceRequired("focus_out");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsDaVinciEndoscopeFocus::FocusOutEventHandler,
                                                this, "Button");
    }
    interfaceRequired = AddInterfaceRequired("EndoscopeFocusIn");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("SetValue", RobotIO.focus_in);
    }
    interfaceRequired = AddInterfaceRequired("EndoscopeFocusOut");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("SetValue", RobotIO.focus_out);
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
    focus_out(false);
    focus_in(false);
}

void mtsDaVinciEndoscopeFocus::FocusInEventHandler(const prmEventButton & event)
{
    switch (event.Type()) {
    case prmEventButton::PRESSED:
        focus_in(true);
        break;
    case prmEventButton::RELEASED:
        focus_in(false);
        break;
    default:
        break;
    }
}

void mtsDaVinciEndoscopeFocus::FocusOutEventHandler(const prmEventButton & event)
{
    switch (event.Type()) {
    case prmEventButton::PRESSED:
        focus_out(true);
        break;
    case prmEventButton::RELEASED:
        focus_out(false);
        break;
    default:
        break;
    }
}

void mtsDaVinciEndoscopeFocus::lock(const bool & lock)
{
    mLocked = lock;
    mEvents.locked(mLocked);
    // turn off any focusing happening
    if (mFocusingIn) {
        focus_in(false);
    } else if (mFocusingOut) {
        focus_out(false);
    }
}

void mtsDaVinciEndoscopeFocus::focus_in(const bool & focus)
{
    // skip if locked
    if (mLocked) {
        return;
    }
    if (focus) {
        // can only focus once at a time
        if (!(mFocusingIn || mFocusingOut)) {
            RobotIO.focus_in(false); // IO is high/low
            mFocusingIn = true;
            mEvents.focusing_in(true);
        }
    } else {
        RobotIO.focus_in(true);
        mFocusingIn = false;
        mEvents.focusing_in(false);
    }
}

void mtsDaVinciEndoscopeFocus::focus_out(const bool & focus)
{
    // skip if locked
    if (mLocked) {
        return;
    }
    if (focus) {
        // can only focus once at a time
        if (!(mFocusingIn || mFocusingOut)) {
            RobotIO.focus_out(false); // IO is high/low
            mFocusingOut = true;
            mEvents.focusing_out(true);
        }
    } else {
        RobotIO.focus_out(true);
        mFocusingOut = false;
        mEvents.focusing_out(false);
    }
}
