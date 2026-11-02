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

#ifndef _robManipulatorMTM_h
#define _robManipulatorMTM_h

#include <sawIntuitiveResearchKit/robManipulatorGeneric.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

class robManipulatorMTM : public robManipulatorGeneric
{
protected:
    robManipulatorMTM* raw_clone() const override;

    bool internalInverseKinematics(
        const Eigen::Isometry3d& goal_pose,
        Eigen::VectorXd& joint_positions,
        std::string& error_message
    ) override;

public:
    robManipulatorMTM(
        const prmConfigurationJoint& joint_config, const KDL::Chain& chain
    );

protected:
    const double upper_arm_length;
    const double elbow_to_wrist_angle;
    const double elbow_to_wrist_length;

    robManipulatorMTM(
        const prmConfigurationJoint& joint_config,
        const KDL::Chain& chain,
        double upper_arm_length,
        double forearm_length,
        double forearm_end_to_wrist
    );

    bool solveShoulderElbowIK(
        Eigen::VectorXd& joint_positions, const Eigen::Vector3d& goal_position
    ) const;

    double findOptimalPlatformAngle(
        const Eigen::VectorXd& joint_positions,
        const Eigen::Isometry3d& goal_pose
    );

    // Cost function for comparing wrist IK solutions
    double wristIKCost(const Eigen::Vector3d& reference_joints, const Eigen::Vector3d& solution_joints) const;

    // Solve for best wrist joint positions to achieve goal pose, given platform
    // angle, wrist joints, and goal pose wrt end of the forearm (link 2).
    Eigen::Vector3d solveWristIK(
        const Eigen::Vector3d& wrist_joints,
        const Eigen::Isometry3d& goal
    ) const;
};

#endif  // _robManipulatorMTM_h
