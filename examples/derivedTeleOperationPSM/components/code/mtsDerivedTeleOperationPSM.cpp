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

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>

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
void mtsDerivedTeleOperationPSM::Configure(const std::string & CMN_UNUSED(filename))
{
    // Call the base class configure, it will do most of the work
    BaseType::Configure();

    // We want to replace the method called when the MTM is actually
    // driving the PSM
    mTeleopState.SetEnterCallback("ENABLED",
                                  &mtsDerivedTeleOperationPSM::EnterEnabled,
                                  this);
    mTeleopState.SetRunCallback("ENABLED",
                                &mtsDerivedTeleOperationPSM::RunEnabled,
                                this);

    // We need to add a method to retrieve estimated wrench from the
    // PSM
    mtsInterfaceRequired * interfacePSM = GetInterfaceRequired("PSM");
    // That interface should exist, abort otherwise
    CMN_ASSERT(interfacePSM);
    // Add a required function
    interfacePSM->AddFunction("body/measured_cf",
                              PSMGetWrenchBody);
    interfacePSM->AddFunction("SetWrenchBodyOrientationAbsolute",
                              PSMSetWrenchBodyOrientationAbsolute);
    interfacePSM->AddFunction("measured_cv",
                              PSMGetVelocityCartesian);

    // Same for MTM
    mtsInterfaceRequired * interfaceMTM = GetInterfaceRequired("MTM");
    CMN_ASSERT(interfaceMTM);
    interfaceMTM->AddFunction("SetWrenchBodyOrientationAbsolute",
                              MTMSetWrenchBodyOrientationAbsolute,
                              MTS_OPTIONAL);
}

void mtsDerivedTeleOperationPSM::EnterEnabled(void)
{
    BaseType::EnterEnabled();
    PSMSetWrenchBodyOrientationAbsolute(true);
    // function SetWrenchBodyOrientationAbsolute is optional on MTM, only call if available
    if (MTMSetWrenchBodyOrientationAbsolute.IsValid()) {
        MTMSetWrenchBodyOrientationAbsolute(true);
    }
}

void mtsDerivedTeleOperationPSM::RunEnabled(void)
{
    // We call the base class method for enabled state, it'll handle most of the work
    BaseType::RunEnabled();

    // Only if the PSM is following
    if (mIsFollowing) {
        prmForceCartesianGet wrenchPSM;
        prmVelocityCartesianGet velocityPSM;
        prmForceCartesianSet wrenchMTM;

        // Get estimated wrench and velocity from PSM
        PSMGetWrenchBody(wrenchPSM);
        PSMGetVelocityCartesian(velocityPSM);

        // Extract only position data, ignore orientation
        const vct3 velocity = velocityPSM.VelocityLinear();
        vct3 force = wrenchPSM.Force().Ref<3>(0);

        // Scale force based on velocity, i.e. at high velocity apply less forces
        for (size_t i = 0; i < 3; i++) {
            double velocityDumping = 1.0 / (20.0 * std::fabs(velocity[i]) + 1.0);
            // Compute force in opposite direction and scale back on master
            force[i] = -0.3 * velocityDumping * force[i];
        }

        // Re-orient based on rotation between MTM and PSM
        force = mRegistrationRotation.Inverse() * force;
        // Set wrench for MTM
        wrenchMTM.Force().Ref<3>(0) = force;
        mMTM.servo_cf_body(wrenchMTM);
    }
}
