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

#ifndef _mtsDerivedTeleOperationPSM_h
#define _mtsDerivedTeleOperationPSM_h

#include <sawIntuitiveResearchKit/mtsTeleOperationPSM.h>

#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>

class mtsDerivedTeleOperationPSM: public mtsTeleOperationPSM
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    typedef mtsTeleOperationPSM BaseType;

    // We need to redefine the constructors
    mtsDerivedTeleOperationPSM(const std::string & componentName, const double periodInSeconds);
    // this constructor is required for dynamic creation
    mtsDerivedTeleOperationPSM(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsDerivedTeleOperationPSM(){}

    // We know configure it called after constructors but just before
    // connecting so we can use this method to change the tele-op
    // state machine callbacks and add some extra commands to the
    // existing interfaces
    void Configure(const std::string & filename) override;

protected:
    void EnterEnabled(void);
    void RunEnabled(void);

    // Extra functions and data members we need for the behavior in
    // derived tele-op.  We use structs just to organize the new data
    // members.
    struct {
        mtsFunctionRead  body_measured_cf;
        mtsFunctionRead  measured_cv;
        mtsFunctionWrite body_set_cf_orientation_absolute;

        prmForceCartesianGet m_measured_cf;
        prmVelocityCartesianGet m_measured_cv;
    } PSMExtra;

    struct {
        mtsFunctionWrite body_set_cf_orientation_absolute;

        prmForceCartesianSet m_setpoint_cf;
    } MTMExtra;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsDerivedTeleOperationPSM);

#endif // _mtsDerivedTeleOperationPSM_h
