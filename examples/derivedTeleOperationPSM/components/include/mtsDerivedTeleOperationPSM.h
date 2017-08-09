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

#ifndef _mtsDerivedTeleOperationPSM_h
#define _mtsDerivedTeleOperationPSM_h

#include <sawIntuitiveResearchKit/mtsTeleOperationPSM.h>

#include <cisstParameterTypes/prmForceCartesianGet.h>



class mtsDerivedTeleOperationPSM: public mtsTeleOperationPSM
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    typedef mtsTeleOperationPSM BaseType;

    mtsDerivedTeleOperationPSM(const std::string & componentName, const double periodInSeconds);
    mtsDerivedTeleOperationPSM(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsDerivedTeleOperationPSM(){}

    void Configure(const std::string & CMN_UNUSED(filename));

protected:
    void RunEnabled(void);
    prmPositionCartesianGet mPSMForces;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsDerivedTeleOperationPSM);

#endif // _mtsDerivedTeleOperationPSM_h
