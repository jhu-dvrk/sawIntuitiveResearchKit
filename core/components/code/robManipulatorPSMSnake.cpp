/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Simon Leonard, Anton Deguet
  Created on: 2017-03-07

  (C) Copyright 2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <sawIntuitiveResearchKit/robManipulatorPSMSnake.h>
#include <cisstCommon/cmnAssert.h>
#include <stdexcept>

namespace {
prmConfigurationJoint ActiveConfiguration(const prmConfigurationJoint & full)
{
    if (full.Name().size() != 8 || full.Type().size() != 8) {
        throw std::invalid_argument("A snake tool requires eight kinematic joints");
    }
    prmConfigurationJoint active = full;
    active.Name().resize(6);
    active.Name()[4] = "wrist_pitch";
    active.Name()[5] = "wrist_yaw";
    active.Type().resize(6);
    active.PositionMin() = robManipulatorPSMSnake::ActiveJoints(full.PositionMin().matrix());
    active.PositionMax() = robManipulatorPSMSnake::ActiveJoints(full.PositionMax().matrix());
    active.VelocityMin() = robManipulatorPSMSnake::ActiveJoints(full.VelocityMin().matrix());
    active.VelocityMax() = robManipulatorPSMSnake::ActiveJoints(full.VelocityMax().matrix());
    active.AccelerationMin() = robManipulatorPSMSnake::ActiveJoints(full.AccelerationMin().matrix());
    active.AccelerationMax() = robManipulatorPSMSnake::ActiveJoints(full.AccelerationMax().matrix());
    // Efforts conjugate to the total bend angle are the average of each pair.
    active.EffortMin() = robManipulatorPSMSnake::ActiveJoints(full.EffortMin().matrix());
    active.EffortMax() = robManipulatorPSMSnake::ActiveJoints(full.EffortMax().matrix());
    active.EffortMin().tail(2) *= 0.5;
    active.EffortMax().tail(2) *= 0.5;
    return active;
}

Eigen::Matrix<double, 6, 6> ActiveJacobian(const robJacobian::jacobian_t & full)
{
    Eigen::Matrix<double, 6, 6> active;
    active.leftCols<4>() = full.leftCols<4>();
    active.col(4) = 0.5 * (full.col(4) + full.col(7));
    active.col(5) = 0.5 * (full.col(5) + full.col(6));
    return active;
}
}

robManipulatorPSMSnake::robManipulatorPSMSnake(
    const prmConfigurationJoint & fullConfig, const KDL::Chain & fullChain)
    : robManipulator(ActiveConfiguration(fullConfig)),
      mFullConfig(fullConfig), mFullChain(fullChain),
      mFullManipulator(fullConfig, fullChain)
{
    if (fullChain.getNrOfJoints() != 8) {
        throw std::invalid_argument("A snake tool requires eight kinematic joints");
    }
}

robManipulatorPSMSnake * robManipulatorPSMSnake::raw_clone() const
{
    return new robManipulatorPSMSnake(mFullConfig, mFullChain);
}

Eigen::VectorXd robManipulatorPSMSnake::FullJoints(const Eigen::VectorXd & active)
{
    CMN_ASSERT(active.size() == 6);
    Eigen::VectorXd full(8);
    full.head<4>() = active.head<4>();
    full[4] = full[7] = active[4] / 2.0;
    full[5] = full[6] = active[5] / 2.0;
    return full;
}

Eigen::VectorXd robManipulatorPSMSnake::ActiveJoints(const Eigen::VectorXd & full)
{
    CMN_ASSERT(full.size() == 8);
    Eigen::VectorXd active(6);
    active.head<4>() = full.head<4>();
    active[4] = full[4] + full[7];
    active[5] = full[5] + full[6];
    return active;
}

void robManipulatorPSMSnake::internalForwardKinematics(
    const Eigen::VectorXd & joints, Eigen::Isometry3d & pose, int segments)
{
    // A public request for six independent joints means the complete chain.
    mFullManipulator.forwardKinematics(FullJoints(joints), pose, segments == 6 ? 8 : segments);
}

void robManipulatorPSMSnake::computeJacobians(const Eigen::VectorXd & joints)
{
    mFullManipulator.computeJacobians(FullJoints(joints));
    body_jacobian.update(ActiveJacobian(mFullManipulator.bodyJacobian().matrix()));
    spatial_jacobian.update(ActiveJacobian(mFullManipulator.spatialJacobian().matrix()));
}

bool robManipulatorPSMSnake::internalInverseKinematics(
    const Eigen::Isometry3d & goal, Eigen::VectorXd & joints, std::string & error)
{
    // Solve in independent coordinates so every iteration satisfies the two
    // half-angle constraints, including when the Jacobian is rank deficient.
    for (size_t iteration = 0; iteration < 1000; ++iteration) {
        Eigen::Isometry3d pose;
        internalForwardKinematics(joints, pose, -1);
        Eigen::Matrix<double, 6, 1> delta;
        delta.head<3>() = pose.linear().transpose() * (goal.translation() - pose.translation());
        const Eigen::AngleAxisd rotation(pose.linear().transpose() * goal.linear());
        delta.tail<3>() = rotation.axis() * rotation.angle();
        if (delta.norm() < 1e-10) {
            return true;
        }
        computeJacobians(joints);
        Eigen::VectorXd step = body_jacobian.jointVelocity(delta);
        if (!step.allFinite()) {
            error = "Snake inverse kinematics produced a non-finite step";
            return false;
        }
        if (step.norm() > 0.5) {
            step *= 0.5 / step.norm();
        }
        joints += step;
        joints = joints.array().max(joint_config.PositionMin()).min(joint_config.PositionMax()).matrix();
    }
    error = "Snake inverse kinematics did not converge";
    return false;
}
