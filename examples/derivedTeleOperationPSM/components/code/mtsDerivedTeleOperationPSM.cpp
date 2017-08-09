/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-08-09

  (C) Copyright 2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <mtsDerivedTeleOperationPSM.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsDerivedTeleOperationPSM,
                                      mtsTeleOperationPSM,
                                      mtsTaskPeriodicConstructorArg);

mtsDerivedTeleOperationPSM::mtsDerivedTeleOperationPSM(const std::string & componentName, const double periodInSeconds) :
    mtsTeleOperationPSM(componentName, periodInSeconds)
{
}

mtsDerivedTeleOperationPSM::mtsDerivedTeleOperationPSM(const mtsTaskPeriodicConstructorArg &arg) :
    mtsTeleOperationPSM(arg)
{
}

void mtsDerivedTeleOperationPSM::Configure(const std::string & CMN_UNUSED(filename))
{
    std::cerr << "--------------------- my configure ----------------" << std::endl;
    BaseType::Configure();

    mTeleopState.SetRunCallback("ENABLED",
                                &mtsDerivedTeleOperationPSM::RunEnabled,
                                this);
}

void mtsDerivedTeleOperationPSM::RunEnabled(void)
{
    BaseType::RunEnabled();
    std::cerr << ".";
}
