/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:
 */

/*
  Author(s):  Anton Deguet, Rishibrata Biswas, Adnan Munawar
  Created on: 2019-11-11

  (C) Copyright 2019-2026 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

#include <sawIntuitiveResearchKit/robManipulatorMTM.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <unsupported/Eigen/EulerAngles>

#include <cisstCommon/cmnConstants.h>
#include <sawIntuitiveResearchKit/robManipulatorGeneric.h>

namespace
{

const double TWO_PI = 2.0 * cmnPI;

void AddAngleLifts(
    std::vector<double>& values,
    const double& angle,
    const double& minimum,
    const double& maximum
)
{
    const int first =
        static_cast<int>(std::ceil((minimum - angle) / TWO_PI - 1e-12));
    const int last =
        static_cast<int>(std::floor((maximum - angle) / TWO_PI + 1e-12));
    for (int turn = first; turn <= last; ++turn) {
        values.push_back(angle + turn * TWO_PI);
    }
}

double SquaredLimitPenalty(
    const double& value, const double& minimum, const double& maximum
)
{
    // This is only a tie breaker: continuity with the supplied joint vector
    // remains the primary selection criterion.
    const double range = maximum - minimum;
    const double margin = std::min(value - minimum, maximum - value);
    const double safeMargin = std::max(margin, 1e-6 * range);
    return 1e-6 * range * range / (safeMargin * safeMargin);
}

double SquaredPlatformPreferencePenalty(
    const double& platform, const double& preferredPlatform
)
{
    const double difference = platform - preferredPlatform;
    return difference * difference;
}

double SquaredNeutralJointPenalty(const double& value)
{
    // Joint 5 (wrist pitch) should remain near its neutral position whenever
    // the redundant platform and alternate Euler branch make that possible.
    // This deliberately outweighs continuity with the supplied reference;
    // e.g., it selects the alternate branch instead of retaining a 146 deg
    // wrist-pitch reference solely because it has zero motion cost.
    const double wristPitchPreferenceWeight = 10.0;
    return wristPitchPreferenceWeight * value * value;
}

}  // namespace

// Compute { angle, length } of wrist/gimbal wrt elbow joint
std::pair<double, double> computeElbowToWrist(const KDL::Chain& chain)
{
    const double forearm_length = chain.getSegment(2).getFrameToTip().p.Norm();
    const double forearm_end_to_wrist =
        chain.getSegment(3).getFrameToTip().p.Norm();

    const double elbow_to_wrist_angle =
        std::atan(forearm_end_to_wrist / forearm_length);
    const double elbow_to_wrist_length = std::sqrt(
        forearm_end_to_wrist * forearm_end_to_wrist +
        forearm_length * forearm_length
    );

    return {elbow_to_wrist_angle, elbow_to_wrist_length};
}

robManipulatorMTM::robManipulatorMTM(
    const prmConfigurationJoint& joint_config, const KDL::Chain& chain
)
    : robManipulatorMTM(
          joint_config,
          chain,
          chain.getSegment(1).getFrameToTip().p.Norm(),
          chain.getSegment(2).getFrameToTip().p.Norm(),
          chain.getSegment(3).getFrameToTip().p.Norm()
      )
{
}

robManipulatorMTM::robManipulatorMTM(
    const prmConfigurationJoint& joint_config,
    const KDL::Chain& chain,
    double upper_arm_length,
    double forearm_length,
    double forearm_end_to_wrist
)
    : robManipulatorGeneric(joint_config, chain),
      upper_arm_length(upper_arm_length),
      elbow_to_wrist_angle(std::atan(forearm_end_to_wrist / forearm_length)),
      elbow_to_wrist_length(std::sqrt(
          forearm_end_to_wrist * forearm_end_to_wrist +
          forearm_length * forearm_length
      ))
{
}

robManipulatorMTM* robManipulatorMTM::raw_clone() const
{
    return new robManipulatorMTM(joint_config, chain);
}

// Solve for triangle's interior angle opposite from side c. Returns NaN if no
// such triangle exists.
double solveTriangleInteriorAngle(double side_a, double side_b, double side_c)
{
    // law of cosines: c^2 = a^2 + b^2 - 2ab*cos(gamma)
    // where triangle has sides a, b, c and gamma is the angle opposite side c
    double numerator = side_a * side_a + side_b * side_b - side_c * side_c;
    double denominator = 2 * side_a * side_b;
    double cos_gamma = numerator / denominator;

    return std::acos(cos_gamma);
}

// Given goal position of arm, solve for shoulder yaw/pitch and elbow pitch
bool robManipulatorMTM::solveShoulderElbowIK(
    Eigen::VectorXd& joint_positions, const Eigen::Vector3d& goal_position
) const
{
    // shoulder yaw is just angle in transverse/horizontal plane
    // coordinate system of shoulder is rotated 90 degrees, so we use X/-Y
    // instead of Y/X
    const double shoulder_yaw =
        std::atan2(goal_position.x(), -goal_position.y());
    const double shoulder_to_wrist_distance = goal_position.norm();

    // shoulder-elbow-wrist is a triangle with three known side lengths
    // so the shoulder-elbow-wrist angle is fully determined
    const double theta = solveTriangleInteriorAngle(
        upper_arm_length, elbow_to_wrist_length, shoulder_to_wrist_distance
    );
    if (std::isnan(theta)) { return false; }

    // interior angle formed by shoulder-elbow-forearm
    const double elbow_interior_angle = theta + elbow_to_wrist_angle;
    // elbow joint pitch is supplementary/exterior angle, with 90 degree offset
    const double elbow_pitch = (cmnPI - elbow_interior_angle) - cmnPI_2;

    // angle of ground-shoulder-wrist
    const double alpha =
        std::acos(-goal_position.z() / shoulder_to_wrist_distance);
    if (std::isnan(alpha)) { return false; }

    // angle of elbow-shoulder-wrist
    const double beta = solveTriangleInteriorAngle(
        upper_arm_length, shoulder_to_wrist_distance, elbow_to_wrist_length
    );
    if (std::isnan(beta)) { return false; }

    // shoulder pitch is ground-shoulder-elbow angle
    const double shoulder_pitch = alpha - beta;

    joint_positions[0] = shoulder_yaw;
    joint_positions[1] = shoulder_pitch;
    joint_positions[2] = elbow_pitch;

    return true;
}

double robManipulatorMTM::findOptimalPlatformAngle(
    const Eigen::VectorXd& q, const Eigen::Isometry3d& goal_pose
)
{
    Eigen::Isometry3d wrist_base_pose;
    forwardKinematics(q, wrist_base_pose, 3);

    // Goal pose of wrist/gimbal with respect to platform base
    Eigen::Isometry3d wrist_target_pose = wrist_base_pose.inverse() * goal_pose;

    // Projection of gripper (Z) axis on to the platform plane (XY)
    const Eigen::Vector2d ee_projection =
        wrist_target_pose.rotation().col(2).head<2>();
    // If gripper/ee axis is nearly normal to plane, just keep platform where it
    // is
    if (ee_projection.norm() < 1e-12) { return q[3]; }

    const double theta = -std::atan2(ee_projection.y(), ee_projection.x());
    // Closest equivalent angle (mod 180)
    const double angle = closestEquivalent(q[3], theta, 3, cmnPI);

    const double weight = std::abs(std::cos(q[4]));
    const double platform = angle * weight + q[3] * (1.0 - weight);
    return std::clamp(
        platform, joint_config.PositionMin()[3], joint_config.PositionMax()[3]
    );
}

double robManipulatorMTM::wristIKCost(
    const Eigen::Vector3d& reference_joints, const Eigen::Vector3d& solution_joints
) const
{
    double cost = 0.0;
    cost += SquaredNeutralJointPenalty(solution_joints[0]);

    for (Eigen::Index joint = 0; joint < 3; ++joint) {
        const double delta = solution_joints[joint] - reference_joints[joint];
        cost += delta * delta;
        cost += SquaredLimitPenalty(
            solution_joints[joint], joint_config.PositionMin()[4 + joint], joint_config.PositionMax()[4 + joint]
        );
    }

    return cost;
}

Eigen::Vector3d robManipulatorMTM::solveWristIK(
    const Eigen::Vector3d& wrist_joints,
    const Eigen::Isometry3d& goal
) const
{
    // Convert the MTM tool frame to the frame used by the DH wrist.

    // inverse of frame 4->7 transformation when joints are at zero, used to align EE frame so we can solve for gimbal angles with an Euler angle decomposition
    static const Eigen::Isometry3d Rt78(Eigen::EulerAnglesYZXd(cmnPI_2, 0.0, cmnPI_2));

    const Eigen::Isometry3d aligned_goal = goal * Rt78;

    const Eigen::EulerAnglesZYXd euler(aligned_goal.rotation());

    // ZYX has two non-singular families:
    // (alpha, beta, gamma) and (alpha + pi, pi - beta, gamma + pi).
    // The roll access orientation is opposite from the Euler axis, so roll = -gamma or -(gamma + pi)
    const Eigen::Vector3d angles[2] = {
        Eigen::Vector3d(euler.alpha(), euler.beta(), -euler.gamma()),
        Eigen::Vector3d(euler.alpha() + cmnPI, cmnPI - euler.beta(), -euler.gamma()-cmnPI)
    };

    double best_cost = std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_solution = wrist_joints;

    for (size_t branch = 0; branch < 2; ++branch) {
        // pitch/yaw range is less than 360, at most one equivalent position within the joint limits
        const double pitch = closestEquivalent(angles[branch].x(), angles[branch].x(), 4);
        const double yaw = closestEquivalent(angles[branch].y(), angles[branch].y(), 5);

        if (pitch < joint_config.PositionMin()[4] || pitch > joint_config.PositionMax()[4]) {
            continue;
        }

        if (yaw < joint_config.PositionMin()[5] || yaw > joint_config.PositionMax()[5]) {
            continue;
        }

        // roll has multiple equivalent positions
        std::vector<double> roll_values;
        AddAngleLifts(roll_values, angles[branch].z(), joint_config.PositionMin()[6], joint_config.PositionMax()[6]);

        for (const double roll : roll_values) {
            const Eigen::Vector3d solution(pitch, yaw, roll);
            const double cost = wristIKCost(wrist_joints, solution);

            if (cost < best_cost) {
                best_cost = cost;
                best_solution = solution;
            }
        }
    }

    return best_solution;
}

bool robManipulatorMTM::internalInverseKinematics(
    const Eigen::Isometry3d& goal_pose,
    Eigen::VectorXd& joint_positions,
    std::string& error_message
)
{
    if (joint_positions.size() != chain.getNrOfJoints()) {
        error_message = "robManipulatorMTM::InverseKinematics: expected " +
                        std::to_string(chain.getNrOfJoints()) +
                        " joints values but received " +
                        std::to_string(joint_positions.size());
        CMN_LOG_RUN_ERROR << error_message << std::endl;
        return false;
    }

    bool position_ok =
        solveShoulderElbowIK(joint_positions, goal_pose.translation());
    if (!position_ok) { 
        return false;
    }

    // Joint 3 (the platform) is redundant for a Cartesian pose.  Resolve
    // this one-dimensional redundancy by evaluating the analytic wrist IK at
    // platform angles throughout its physical range.  The caller-provided q
    // is the continuity reference, so include it explicitly before sampling.
    const double preferred_platform =
        findOptimalPlatformAngle(joint_positions, goal_pose);
    const double platform_min = joint_config.PositionMin()[3];
    const double platform_max = joint_config.PositionMax()[3];
    const double platform_step = cmnPI / 90.0;  // two degrees
    std::vector<double> platform_candidates;
    platform_candidates.push_back(
        std::clamp(joint_positions[3], platform_min, platform_max)
    );
    platform_candidates.push_back(platform_min);
    platform_candidates.push_back(platform_max);
    for (double platform = platform_min; platform < platform_max;
         platform += platform_step) {
        platform_candidates.push_back(platform);
    }

    double best_cost = std::numeric_limits<double>::infinity();
    Eigen::VectorXd best_solution = joint_positions;
    Eigen::VectorXd reference = joint_positions;
    bool found_wrist_solution = false;

    const double wristMin[3] = {
        joint_config.PositionMin()[4],
        joint_config.PositionMin()[5],
        joint_config.PositionMin()[6]
    };
    const double wristMax[3] = {
        joint_config.PositionMax()[4],
        joint_config.PositionMax()[5],
        joint_config.PositionMax()[6]
    };

    for (const double platform : platform_candidates) {
        joint_positions[3] = platform;

        // Convert the MTM tool frame to the frame used by the DH wrist.
        const Eigen::EulerAnglesYZXd offset(cmnPI_2, 0.0, -cmnPI_2);
        const Eigen::Isometry3d aligned_goal = goal_pose * offset;

        Eigen::Isometry3d Rt04;
        forwardKinematics(joint_positions, Rt04, 4);
        const Eigen::Isometry3d wrist_goal = Rt04.inverse() * aligned_goal;
        const Eigen::EulerAnglesZYXd euler(wrist_goal.rotation());

        // ZYX has two non-singular families:
        // (alpha, beta, gamma) and (alpha + pi, pi - beta, gamma + pi).
        // The DH offsets map gamma to wrist roll as pi - gamma.
        const double branches[2][3] = {
            {euler.alpha(),             euler.beta(),              cmnPI - euler.gamma()},
            {euler.alpha() + cmnPI,     cmnPI - euler.beta(),      -euler.gamma()}
        };
        for (size_t branch = 0; branch < 2; ++branch) {
            std::vector<double> pitch, yaw, roll;
            AddAngleLifts(pitch, branches[branch][0], wristMin[0], wristMax[0]);
            AddAngleLifts(yaw,   branches[branch][1], wristMin[1], wristMax[1]);
            AddAngleLifts(roll,  branches[branch][2], wristMin[2], wristMax[2]);
            for (size_t pitchIndex = 0; pitchIndex < pitch.size(); ++pitchIndex) {
                for (size_t yawIndex = 0; yawIndex < yaw.size(); ++yawIndex) {
                    for (size_t rollIndex = 0; rollIndex < roll.size(); ++rollIndex) {
                        const double wrist[3] = {pitch[pitchIndex], yaw[yawIndex], roll[rollIndex]};
                        double cost = (platform - reference[3]) * (platform - reference[3]);
                        cost += SquaredPlatformPreferencePenalty(platform, preferred_platform);
                        cost += SquaredNeutralJointPenalty(wrist[0]); // MTM joint 5: wrist pitch
                        for (size_t joint = 0; joint < 3; ++joint) {
                            const double difference = wrist[joint] - joint_positions[joint + 4];
                            cost += difference * difference;
                            cost += SquaredLimitPenalty(wrist[joint], wristMin[joint], wristMax[joint]);
                        }
                        if (cost < best_cost) {
                            best_cost = cost;
                            best_solution = joint_positions;
                            best_solution[4] = wrist[0];
                            best_solution[5] = wrist[1];
                            best_solution[6] = wrist[2];
                            found_wrist_solution = true;
                        }
                    }
                }
            }
        }
    }

    if (!found_wrist_solution) {
        error_message =
            "robManipulatorMTM::InverseKinematics: no platform and wrist "
            "solution within joint limits";
        CMN_LOG_RUN_ERROR << error_message << std::endl;
        return false;
    }

    joint_positions = best_solution;
    bool limits_ok = clampJoints(joint_positions);

    return limits_ok;
}
