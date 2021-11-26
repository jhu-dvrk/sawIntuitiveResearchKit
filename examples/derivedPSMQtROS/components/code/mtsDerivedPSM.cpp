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

#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <cmath>

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

    // replace the default method called when the arm is homed
    mArmState.SetRunCallback("HOMED",
                             &mtsDerivedPSM::RunHomed,
                             this);
}

void mtsDerivedPSM::RunHomed(void)
{
    if (IsCartesianReady()) {
        std::cerr << ".";
    }
    BaseType::RunHomed();
}
