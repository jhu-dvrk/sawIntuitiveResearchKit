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

#include <mtsDerivedPSM.h>


#include <cisstMultiTask/mtsInterfaceProvided.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsDerivedPSM, // this class
                                      mtsIntuitiveResearchKitPSM, // base class
                                      mtsTaskPeriodicConstructorArg); // constructor used for dynamic creation

mtsDerivedPSM::mtsDerivedPSM(const std::string & componentName,
                             const double periodInSeconds):
    mtsIntuitiveResearchKitPSM(componentName, periodInSeconds)
{
}

mtsDerivedPSM::mtsDerivedPSM(const mtsTaskPeriodicConstructorArg & arg):
    mtsIntuitiveResearchKitPSM(arg)
{
    // replace the default method called when the arm is homed
    mArmState.SetRunCallback("HOMED",
                             &mtsDerivedPSM::RunHomed,
                             this);

    mStateTableConfiguration.AddData(m_activated, "activated");
    mStateTableConfiguration.AddData(m_gain, "gain");

    mtsInterfaceProvided * new_interface = AddInterfaceProvided("NewInterface");
    new_interface->AddCommandReadState(mStateTableConfiguration, m_activated, "activated");
    new_interface->AddCommandReadState(mStateTableConfiguration, m_gain, "gain");
    new_interface->AddCommandWrite(&mtsDerivedPSM::activate, this, "activate");
    new_interface->AddCommandWrite(&mtsDerivedPSM::set_gain, this, "set_gain");
    new_interface->AddEventWrite(m_events.activated, "activated", false);
    new_interface->AddEventWrite(m_events.gain, "gain", m_gain);
}

// Configure is a virtual method, we can redefine it and have our own
// configuration
void mtsDerivedPSM::Configure(const std::string & filename)
{
    // Call the base class Configure so we don't loose the
    // configuration part of the base class
    if (!filename.empty()) {
        BaseType::Configure(filename);
    }
}

void mtsDerivedPSM::RunHomed(void)
{
    if (m_activated) {
        if (IsCartesianReady()) {
            std::cerr << " " << m_gain;
        }
    } else {
        BaseType::RunHomed();
    }
}

void mtsDerivedPSM::activate(const bool & _activate)
{
    m_activated = _activate;
    m_events.activated(m_activated);
}

void mtsDerivedPSM::set_gain(const double & _gain)
{
    m_gain = _gain;
    if (m_gain > 1.0) {
        m_gain = 1.0;
        m_arm_interface->SendWarning(this->GetName() + ": gain has to be <= 1.0, set to 1.0");
    } else if (m_gain < 0) {
        m_gain = 0.0;
        m_arm_interface->SendWarning(this->GetName() + ": gain has to be >= 0.0, set to 0.0");
    }
    m_events.gain(m_gain);
}
