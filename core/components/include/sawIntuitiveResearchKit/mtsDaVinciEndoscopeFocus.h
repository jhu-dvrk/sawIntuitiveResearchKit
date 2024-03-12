/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2018-03-14

  (C) Copyright 2018-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsDaVinciEndoscopeFocus_h
#define _mtsDaVinciEndoscopeFocus_h

#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstMultiTask/mtsTaskFromSignal.h>
#include <cisstParameterTypes/prmEventButton.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

class CISST_EXPORT mtsDaVinciEndoscopeFocus: public mtsTaskFromSignal
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 public:
    mtsDaVinciEndoscopeFocus(const std::string & componentName);
    mtsDaVinciEndoscopeFocus(const mtsTaskConstructorArg & arg);
    inline ~mtsDaVinciEndoscopeFocus() {}

    void Configure(const std::string & filename);
    void Startup(void);
    void Run(void);
    void Cleanup(void);

protected:

    void Init(void);

    // required interface
    struct {
        mtsFunctionWrite focus_in;
        mtsFunctionWrite focus_out;
    } RobotIO;

    mtsInterfaceProvided * mInterface;

    // callbacks for each event
    void FocusInEventHandler(const prmEventButton & event);
    void FocusOutEventHandler(const prmEventButton & event);

    // locking focus
    void lock(const bool & lock);
    bool mLocked = false;
    bool mFocusingIn = false;
    bool mFocusingOut = false;

    // direct commands
    void focus_in(const bool & focus);
    void focus_out(const bool & focus);

    // events
    struct {
        mtsFunctionWrite locked;
        mtsFunctionWrite focusing_in;
        mtsFunctionWrite focusing_out;
    } mEvents;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsDaVinciEndoscopeFocus);

#endif // _mtsDaVinciEndoscopeFocus_h
