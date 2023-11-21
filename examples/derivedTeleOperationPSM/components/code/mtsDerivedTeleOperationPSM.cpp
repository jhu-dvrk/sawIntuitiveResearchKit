/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2017-08-09

  (C) Copyright 2017-2023 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <mtsDerivedTeleOperationPSM.h>

#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <cmath>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsDerivedTeleOperationPSM,
                                      mtsTeleOperationPSM,
                                      mtsTaskPeriodicConstructorArg);

mtsDerivedTeleOperationPSM::mtsDerivedTeleOperationPSM(const std::string & componentName,
                                                       const double periodInSeconds):
    mtsTeleOperationPSM(componentName, periodInSeconds)
{
}

mtsDerivedTeleOperationPSM::mtsDerivedTeleOperationPSM(const mtsTaskPeriodicConstructorArg & arg):
    mtsTeleOperationPSM(arg)
{
}

// Configure is a virtual method, we can redefine it and have our own
// configuration
void mtsDerivedTeleOperationPSM::Configure(const std::string & filename)
{
    // Call the base class Configure so we don't loose the
    // configuration part of the base class
    BaseType::Configure(filename);

    // We want to replace the method called when the MTM is actually
    // driving the PSM
    mTeleopState.SetEnterCallback("ENABLED",
                                  &mtsDerivedTeleOperationPSM::EnterEnabled,
                                  this);
    mTeleopState.SetRunCallback("ENABLED",
                                &mtsDerivedTeleOperationPSM::RunEnabled,
                                this);

    // We need to add a method to retrieve estimated wrench from the
    // PSM so we first try to retrieve the existing interfaces
    mtsInterfaceRequired * interfacePSM = GetInterfaceRequired("PSM");
    // That interface should exist, abort otherwise
    CMN_ASSERT(interfacePSM);
    // Add some required functions
    interfacePSM->AddFunction("body/measured_cf",
                              PSMExtra.body_measured_cf);
    interfacePSM->AddFunction("body/set_cf_orientation_absolute",
                              PSMExtra.body_set_cf_orientation_absolute);
    interfacePSM->AddFunction("measured_cv",
                              PSMExtra.measured_cv);

    // Same for MTM
    mtsInterfaceRequired * interfaceMTM = GetInterfaceRequired("MTM");
    CMN_ASSERT(interfaceMTM);
    interfaceMTM->AddFunction("body/set_cf_orientation_absolute",
                              MTMExtra.body_set_cf_orientation_absolute,
                              MTS_OPTIONAL);
}

void mtsDerivedTeleOperationPSM::EnterEnabled(void)
{
    // Always remember to call the base class method to make sure we
    // don't loose any of the existing functionalities
    BaseType::EnterEnabled();

    // Then perform actions specific to the derived behavior
    PSMExtra.body_set_cf_orientation_absolute(true);
    // Function body/set_cf_orientation_absolute is optional on MTM,
    // only call if available
    if (MTMExtra.body_set_cf_orientation_absolute.IsValid()) {
        MTMExtra.body_set_cf_orientation_absolute(true);
    }
}

void mtsDerivedTeleOperationPSM::RunEnabled(void)
{
    // We call the base class method for enabled state, it'll handle
    // most of the work, i.e. position control
    BaseType::RunEnabled();

    // Only if the PSM is following
    if (m_following) {
        // Get estimated wrench and velocity from PSM
        PSMExtra.body_measured_cf(PSMExtra.m_measured_cf);
        PSMExtra.measured_cv(PSMExtra.m_measured_cv);

        // Extract only position data, ignore orientation
        vct3 force = PSMExtra.m_measured_cf.Force().Ref<3>(0);
        const vct3 velocity = PSMExtra.m_measured_cv.VelocityLinear();

        // Scale force based on velocity, i.e. at high velocity apply less forces
        for (size_t i = 0; i < 3; i++) {
            double velocityDumping = 1.0 / (20.0 * std::fabs(velocity[i]) + 1.0);
            // Compute force in opposite direction and scale back on master
            force[i] = -0.25 * velocityDumping * force[i];
        }

        // Set wrench for MTM
        MTMExtra.m_setpoint_cf.Force().Ref<3>(0) = force;
        mMTM.body_servo_cf(MTMExtra.m_setpoint_cf);
    }
}
