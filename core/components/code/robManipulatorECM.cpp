/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:
 */

/*
  Author(s):  Anton Deguet
  Created on: 2019-11-11

  (C) Copyright 2019-2020 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

#include <cisstCommon/cmnUnits.h>
#include <cisstRobot/kdl_utilities.h>
#include <cisstRobot/robJacobian.h>
#include <sawIntuitiveResearchKit/robManipulatorECM.h>

#include <cmath>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "sawIntuitiveResearchKit/robManipulator.h"

double getInsertionOffset(const KDL::Chain& chain)
{
    auto fk = KDL::ChainFkSolverPos_recursive(chain);
    KDL::JntArray q(chain.getNrOfJoints());
    KDL::Frame pose;
    q.data.setZero();
    fk.JntToCart(q, pose);

    return pose.p.z();
}

robManipulatorECM::robManipulatorECM(
    const prmConfigurationJoint& joint_config, const KDL::Chain& chain
)
    : robManipulatorGeneric(joint_config, chain),
      insertion_offset(getInsertionOffset(chain))
{}

robManipulatorECM* robManipulatorECM::raw_clone() const
{
    return new robManipulatorECM(joint_config, chain);
}

bool robManipulatorECM::internalInverseKinematics(
    const Eigen::Isometry3d& goal_pose,
    Eigen::VectorXd& joint_positions,
    std::string& error_message
)
{
    if (joint_positions.size() != chain.getNrOfJoints()) {
        std::stringstream ss;
        ss << "robManipulatorECM::internalInverseKinematics: expected "
           << chain.getNrOfJoints() << " joints values but received "
           << joint_positions.size();
        error_message = ss.str();
        return false;
    }

    // re-align desired frame to 4 axis direction to reduce free space
    Eigen::Isometry3d goal_aligned = goal_pose;
    Eigen::Vector3d shaft = goal_pose.translation();
    const double shaft_norm = shaft.norm();

    // we should not allow anything in the cannula but at least make
    // sure it's numerically stable using 1mm
    if (shaft_norm < 0.1 * cmn_mm) {
        error_message =
            "robManipulatorECM::internalInverseKinematics: cartesian goal is too close "
            "to RCM point";
        return false;
    }

    // normalize
    shaft.normalize();
    const Eigen::Vector3d z_axis = goal_pose.linear().col(2);

    if (!z_axis.isApprox(shaft, 0.0001)) {
        const Eigen::Vector3d axis = z_axis.cross(shaft).normalized();
        const double angle = std::acos(z_axis.dot(shaft));
        Eigen::Matrix3d align = Eigen::AngleAxisd(angle, axis).matrix();
        goal_aligned.linear() = align * goal_pose.linear();
    }

    const double x = goal_aligned.translation().x();
    const double y = goal_aligned.translation().y();
    const double z = goal_aligned.translation().z();

    // hard coded check for dVRK classic, wouldn't work on S/Si system
    // TODO: fix - do we want to allow ECM to go upside down? could it fall out?
    // and if so, would it be better to instead prevent that in
    // mtsIntuitiveReseachKitEcm?
    if (z > cmnTypeTraits<double>::Tolerance()) {
        error_message =
            "robManipulatorECM::internalInverseKinematics: z is positive, out of reach";
        return false;
    }

    // We essentially perform a conversion from Cartesian to spherical
    // coordinates to determine the first three joint values - joint[0] is the
    // azimuthal angle, joint[1] is the polar angle (but measured "up" from the
    // plane instead of "down" from the axis), and joint[2] is the radius.
    // However, the polar axis cooresponds to the y-axis instead of the z-axis
    // as is typical.

    // Solve for first joint / azimuthal angle
    const double azimuthal_radius = std::sqrt(x * x + z * z);
    if (azimuthal_radius < cmnTypeTraits<double>::Tolerance()) {
        joint_positions[0] = 0.0;
    } else {
        joint_positions[0] = std::atan2(x, -z);
    }

    // Solve for second joint / polar angle
    const double radius = goal_pose.translation().norm();
    joint_positions[1] = std::asin(-y / radius);
    joint_positions[2] = radius + insertion_offset;

    // for orientation, we assume the goal is reachable
    Eigen::Isometry3d Rt03;
    robManipulator::forwardKinematics(joint_positions, Rt03, 3);

    // rotation for last link
    Eigen::Isometry3d Rt34 =
        Rt03.inverse() * goal_aligned.inverse();
    Eigen::AngleAxisd axis_angle(Rt34.linear());
    // it's a rotation along z or -z
    if (axis_angle.axis().z() < 0) {
        joint_positions[3] = -axis_angle.angle();
    } else {
        joint_positions[3] = axis_angle.angle();
    }

    return true;
}
