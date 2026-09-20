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

#include <Eigen/QR>
#include <sawIntuitiveResearchKit/prmActuatorJointCouplingCheck.h>

void prmActuatorJointCouplingCheck(const size_t nbJoints,
                                   const size_t nbActuators,
                                   const prmActuatorJointCoupling & input,
                                   prmActuatorJointCoupling & result)
{
    if ((input.ActuatorToJointPosition().rows() != (Eigen::Index)nbJoints) ||
        (input.ActuatorToJointPosition().cols() != (Eigen::Index)nbActuators)) {
        cmnThrow("prmActuatorJointCouplingCheck: invalid size for ActuatorToJointPosition");
    }

    result.ActuatorToJointPosition() = input.ActuatorToJointPosition();

    // if we get an empty matrix, compute the inverse
    if (input.JointToActuatorPosition().size() == 0) {
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(input.ActuatorToJointPosition());
        result.JointToActuatorPosition() = cod.pseudoInverse();
    } else {
        if ((input.JointToActuatorPosition().rows() != (Eigen::Index)nbActuators) ||
            (input.JointToActuatorPosition().cols() != (Eigen::Index)nbJoints)) {
            cmnThrow("prmActuatorJointCouplingCheck: invalid size for JointToActuatorPosition");
        }
        result.JointToActuatorPosition() = input.JointToActuatorPosition();
    }

    // if we get an empty matrix, compute the transpose
    if (input.ActuatorToJointEffort().size() == 0) {
        result.ActuatorToJointEffort() = result.JointToActuatorPosition().transpose();
    } else {
        if ((input.ActuatorToJointEffort().rows() != (Eigen::Index)nbJoints) ||
            (input.ActuatorToJointEffort().cols() != (Eigen::Index)nbActuators)) {
            cmnThrow("prmActuatorJointCouplingCheck: invalid size for ActuatorToJointEffort");
        }
        result.ActuatorToJointEffort() = input.ActuatorToJointEffort();
    }

    // if we get an empty matrix, compute the inverse
    if (input.JointToActuatorEffort().size() == 0) {
        Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(result.ActuatorToJointEffort());
        result.JointToActuatorEffort() = cod.pseudoInverse();
    } else {
        if ((input.JointToActuatorEffort().rows() != (Eigen::Index)nbActuators) ||
            (input.JointToActuatorEffort().cols() != (Eigen::Index)nbJoints)) {
            cmnThrow("prmActuatorJointCouplingCheck: invalid size for JointToActuatorEffort");
        }
        result.JointToActuatorEffort() = input.JointToActuatorEffort();
    }
}
