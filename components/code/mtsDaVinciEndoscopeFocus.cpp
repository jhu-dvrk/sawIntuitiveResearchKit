/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-12-18

  (C) Copyright 2017-2020 Johns Hopkins University (JHU), All Rights Reserved.

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
        interfaceProvided->AddCommandWrite(&mtsDaVinciEndoscopeFocus::Lock,
                                           this, "Lock");
        interfaceProvided->AddCommandWrite(&mtsDaVinciEndoscopeFocus::FocusIn,
                                           this, "FocusIn");
        interfaceProvided->AddCommandWrite(&mtsDaVinciEndoscopeFocus::FocusOut,
                                           this, "FocusOut");
        interfaceProvided->AddEventWrite(mEvents.Locked, "Locked", false);
        interfaceProvided->AddEventWrite(mEvents.FocusingIn, "FocusingIn", false);
        interfaceProvided->AddEventWrite(mEvents.FocusingOut, "FocusingOut", false);
    }
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("FocusIn");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsDaVinciEndoscopeFocus::FocusInEventHandler,
                                                this, "Button");
    }
    interfaceRequired = AddInterfaceRequired("FocusOut");
    if (interfaceRequired) {
        interfaceRequired->AddEventHandlerWrite(&mtsDaVinciEndoscopeFocus::FocusOutEventHandler,
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
    FocusOut(false);
    FocusIn(false);
}

void mtsDaVinciEndoscopeFocus::FocusInEventHandler(const prmEventButton & event)
{
    switch (event.Type()) {
    case prmEventButton::PRESSED:
        FocusIn(true);
        break;
    case prmEventButton::RELEASED:
        FocusIn(false);
        break;
    default:
        break;
    }
}

void mtsDaVinciEndoscopeFocus::FocusOutEventHandler(const prmEventButton & event)
{
    switch (event.Type()) {
    case prmEventButton::PRESSED:
        FocusOut(true);
        break;
    case prmEventButton::RELEASED:
        FocusOut(false);
        break;
    default:
        break;
    }
}

void mtsDaVinciEndoscopeFocus::Lock(const bool & lock)
{
    mLocked = lock;
    mEvents.Locked(mLocked);
    // turn off any focusing happening
    if (mFocusingIn) {
        FocusIn(false);
    } else if (mFocusingOut) {
        FocusOut(false);
    }
}

void mtsDaVinciEndoscopeFocus::FocusIn(const bool & focus)
{
    // skip if locked
    if (mLocked) {
        return;
    }
    if (focus) {
        // can only focus once at a time
        if (!(mFocusingIn || mFocusingOut)) {
            RobotIO.FocusIn(false); // IO is high/low
            mFocusingIn = true;
            mEvents.FocusingIn(true);
        }
    } else {
        RobotIO.FocusIn(true);
        mFocusingIn = false;
        mEvents.FocusingIn(false);
    }
}

void mtsDaVinciEndoscopeFocus::FocusOut(const bool & focus)
{
    // skip if locked
    if (mLocked) {
        return;
    }
    if (focus) {
        // can only focus once at a time
        if (!(mFocusingIn || mFocusingOut)) {
            RobotIO.FocusOut(false); // IO is high/low
            mFocusingOut = true;
            mEvents.FocusingOut(true);
        }
    } else {
        RobotIO.FocusOut(true);
        mFocusingOut = false;
        mEvents.FocusingOut(false);
    }
}
