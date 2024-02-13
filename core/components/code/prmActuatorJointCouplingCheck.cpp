/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2022-11-18

  (C) Copyright 2022 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

#include <sawIntuitiveResearchKit/prmActuatorJointCouplingCheck.h>
#include <cisstNumerical/nmrInverse.h>

void prmActuatorJointCouplingCheck(const size_t nbJoints,
                                   const size_t nbActuators,
                                   const prmActuatorJointCoupling & input,
                                   prmActuatorJointCoupling & result)
{
  if ((input.ActuatorToJointPosition().rows() != nbJoints) ||
        (input.ActuatorToJointPosition().cols() != nbActuators)) {
        cmnThrow("prmActuatorJointCouplingCheck: invalid size for ActuatorToJointPosition");
    }
    result.ActuatorToJointPosition()
        .ForceAssign(input.ActuatorToJointPosition());

    // if we get an empty matrix, compute the inverse
    if (input.JointToActuatorPosition().size() == 0) {
        result.JointToActuatorPosition()
            .ForceAssign(input.ActuatorToJointPosition());
        nmrInverse(result.JointToActuatorPosition());
    } else {
        if ((input.JointToActuatorPosition().rows() != nbActuators) ||
            (input.JointToActuatorPosition().cols() != nbJoints)) {
            cmnThrow("prmActuatorJointCouplingCheck: invalid size for JointToActuatorPosition");
        }
        result.JointToActuatorPosition()
            .ForceAssign(input.JointToActuatorPosition());
    }

    // if we get an empty matrix, compute the transpose
    if (input.ActuatorToJointEffort().size() == 0) {
        result.ActuatorToJointEffort()
            .ForceAssign(result.JointToActuatorPosition().Transpose());
    } else {
        if ((input.ActuatorToJointEffort().rows() != nbJoints) ||
            (input.ActuatorToJointEffort().cols() != nbActuators)) {
            cmnThrow("prmActuatorJointCouplingCheck: invalid size for ActuatorToJointEffort");
        }
        result.ActuatorToJointEffort()
            .ForceAssign(input.ActuatorToJointEffort());
    }

    // if we get an empty matrix, compute the transpose
    if (input.JointToActuatorEffort().size() == 0) {
        result.JointToActuatorEffort()
            .ForceAssign(result.ActuatorToJointEffort());
        nmrInverse(result.JointToActuatorEffort());
    } else {
        if ((input.JointToActuatorEffort().rows() != nbActuators) ||
            (input.JointToActuatorEffort().cols() != nbJoints)) {
            cmnThrow("prmActuatorJointCouplingCheck: invalid size for JointToActuatorEffort");
        }
        result.JointToActuatorEffort()
            .ForceAssign(input.JointToActuatorEffort());
    }
}
