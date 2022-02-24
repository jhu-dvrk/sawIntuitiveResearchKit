/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-08-09

  (C) Copyright 2017-2021 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsDerivedPSM_h
#define _mtsDerivedPSM_h

#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitPSM.h>

#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>

// derive from the existing PSM class to extend/change its behavior
class mtsDerivedPSM: public mtsIntuitiveResearchKitPSM
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    typedef mtsIntuitiveResearchKitPSM BaseType;

    // We need to redefine the constructors
    mtsDerivedPSM(const std::string & componentName, const double periodInSeconds);
    // this constructor is required for dynamic creation
    mtsDerivedPSM(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsDerivedPSM(){}

    // We know configure it called after constructors but just before
    // connecting so we can use this method to change the tele-op
    // state machine callbacks and add some extra commands to the
    // existing interfaces
    void Configure(const std::string & filename) override;

protected:

    // Custom method to be called when the arm is homed
    void RunHomed(void);

    bool m_activated = false;
    bool m_recently_activated = false;
    double m_gain = 0.5;

    struct {
        mtsFunctionWrite activated;
        mtsFunctionWrite gain;
    } m_events;

    void activate(const bool & _activate);

    void set_gain(const double & _gain);

};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsDerivedPSM);

#endif // _mtsDerivedPSM_h
