/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Rishibrata Biswas, Adnan Munawar
  Created on: 2019-11-11

  (C) Copyright 2019-2020 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

#include <sawIntuitiveResearchKit/robManipulatorMTM.h>

#include <cmath>
#include <string>

using namespace std::literals::string_literals;

// coordinate system of MTM(R/L) link 0 is:
//     z up, y away, x right
// when viewed from front of console

// when all joints are at zero, the
// coordinate system of MTM(R/L) link 4 is:
//     z left, y up, x towards
// when viewed from front of console. The
// rotation from link 0 frame to link 4 frame is:
//  0  0 -1
// -1  0  0
//  0  1  0

// when all joints are at zero, the
// coordinate system of MTM(R/L) link 7 is:
//     z away, y right, x up
// when viewed from front of console. The
// rotation from link 4 frame to link 7 frame is:
//  0  0 -1
//  1  0  0
//  0 -1  0

// joints (directions specified relative to all-zero joint positions):
//   0: shoulder yaw. 0 is centered, positive is counterclockwise when viewed from above
//   1: shoulder pitch. 0 is straight down, positive is rotating forward
//   2: elbow pitch. 0 is straight forward, positive is rotating forward/up
//   3: gimbal platform yaw. 0 is outer side, depending on left/right MTM. positive is counterclockwise from above
//   4: wrist pitch. 0 is upright, positive is rotating forwards
//   5: wrist yaw. 0 is 90 from wrist, positive is counterclockwise from above
//   6: wrist roll. zero is horizontal, positive is *clockwise* from the front

// inverse of frame 4->7 transformation given above
const vctRot3 robManipulatorMTM::rotation_78 = vctRot3(vctEulerYZXRotation3(cmnPI_2, 0.0, cmnPI_2));

robManipulatorMTM::robManipulatorMTM(const std::vector<robKinematics*> linkParms,
                                     const vctFrame4x4<double> &Rtw0)
    : robManipulator(linkParms, Rtw0) {}

robManipulatorMTM::robManipulatorMTM(const std::string &robotfilename,
                                     const vctFrame4x4<double> &Rtw0)
    : robManipulator(robotfilename, Rtw0) {}

robManipulatorMTM::robManipulatorMTM(const vctFrame4x4<double> &Rtw0)
    : robManipulator(Rtw0) {}

#if CISST_HAS_JSON
robManipulator::Errno robManipulatorMTM::LoadRobot(const Json::Value & config)
{
    robManipulator::Errno result = robManipulator::LoadRobot(config);
    if (result != robManipulator::ESUCCESS) {
        return result;
    }

    upper_arm_length = links[1].PStar().Norm();
    double forearm_length = links[2].PStar().Norm();
    double forearm_to_gimbal = links[3].PStar().Norm();

    // pre-compute kinematic constants
    elbow_to_gimbal_angle = std::atan(forearm_to_gimbal / forearm_length);
    elbow_to_gimbal_length = std::sqrt(forearm_to_gimbal * forearm_to_gimbal + forearm_length * forearm_length);

    return robManipulator::ESUCCESS;
}
#endif

robManipulator::Errno
robManipulatorMTM::InverseKinematics(vctDynamicVector<double>& q,
                                     const vctFrame4x4<double>& Rts,
                                     double CMN_UNUSED(tolerance),
                                     size_t CMN_UNUSED(Niterations),
                                     double CMN_UNUSED(LAMBDA))
{
    if (q.size() != links.size()) {
        mLastError = "robManipulatorMTM::InverseKinematics: expected "s + std::to_string(links.size())
            + " joints values but received "s + std::to_string(q.size());
        CMN_LOG_RUN_ERROR << mLastError << std::endl;
        return robManipulator::EFAILURE;
    }

    if (links.size() != 7) {
        mLastError = "robManipulatorMTM::InverseKinematics: manipulator should have 7 links"s
            + " but received "s + std::to_string(links.size());
        CMN_LOG_RUN_ERROR << mLastError << std::endl;
        return robManipulator::EFAILURE;
    }

    // eliminate Rtw0 (link 0 to world transform)
    vctFrm4x4 Rt07 = Rtw0.Inverse() * Rts;

    // shoulder and elbow angles entirely determine position, so we can solve
    // for first three joints separately
    vct3 position_ik = ShoulderElbowIK(Rt07.Translation());
    q.Ref(3, 0).Assign(position_ik);

    // determine remaining transformation once shoulder/elbow is determined
    vctFrm4x4 Rt04 = ForwardKinematics(q, 4);
    vctFrm4x4 Rt47 = Rt04.Inverse() * Rt07;

    // choose platform angle based on minimal movement necessary to solve gimbal IK
    vct3 gripper_axis = -vctRot3(Rt47.Rotation()).Column(2); // negative z-axis points in gripper direction
    q[3] = ChoosePlatformAngle(q[3], gripper_axis);
    // recompute FK to base of gimbal with new platform angle
    Rt04 = ForwardKinematics(q, 4);
    Rt47 = Rt04.Inverse() * Rt07;

    // given platform angle, wrist/gimbal IK is uniquely determined by
    // the remaining portion of the desired overall transformation
    const double current_wrist_roll = q[6];
    vct3 gimbal_ik = WristGimbalIK(vctRot3(Rt47.Rotation()));
    q.Ref(3, 4).Assign(gimbal_ik);
    q[6] = ClosestEquivalentAngle(q[6], current_wrist_roll, links.at(6).PositionMin(), links.at(6).PositionMax());

    // copy prevents ODR-use, which in pre-C++17 requires a definition in addition to declaration
    constexpr double tolerance = joint_limit_tolerance;

    // check+enforce all joint limits
    bool joint_limit_reached = false;
    for (size_t joint = 0; joint < 7; joint++) {
        bool out_of_range = ClampJointValueAndUpdateError(joint, q[joint], tolerance);
        joint_limit_reached = joint_limit_reached || out_of_range;
    }

    // return failure if target pose is not achievable within joint limits
    return joint_limit_reached ? EFAILURE : ESUCCESS;
}

// solve for triangle's interior angle opposite from side c
double robManipulatorMTM::SolveTriangleInteriorAngle(double side_a, double side_b, double side_c)
{
    // law of cosines: c^2 = a^2 + b^2 - 2ab*cos(gamma)
    // where triangle has sides a, b, c and gamma is the angle opposite side c
    double numerator = side_a * side_a + side_b * side_b - side_c * side_c;
    double denominator = 2 * side_a * side_b;
    double cos_gamma = numerator / denominator;
    return std::acos(cos_gamma);
}

vct3 robManipulatorMTM::ShoulderElbowIK(const vct3& position_07) const {
    // shoulder yaw is just angle in transverse/horizontal plane
    // coordinate system of shoulder is rotated 90 degrees, so we use X/-Y instead of Y/X
    const double shoulder_yaw = std::atan2(position_07.X(), -position_07.Y());
    const double shoulder_to_wrist_distance = position_07.Norm();

    // shoulder-elbow-wrist is a triangle with three known side lengths
    // so the shoulder-elbow-wrist angle is fully determined
    const double theta = SolveTriangleInteriorAngle(upper_arm_length, elbow_to_gimbal_length, shoulder_to_wrist_distance);
    // interior angle formed by shoulder-elbow-forearm
    const double elbow_interior_angle = theta + elbow_to_gimbal_angle;
    // elbow joint pitch is supplementary/exterior angle, with 90 degree offset
    const double elbow_pitch = (cmnPI - elbow_interior_angle) - cmnPI_2;

    // angle of ground-shoulder-wrist
    const double alpha = std::acos(-position_07.Z() / shoulder_to_wrist_distance);
    // angle of elbow-shoulder-wrist
    const double beta = SolveTriangleInteriorAngle(upper_arm_length, shoulder_to_wrist_distance, elbow_to_gimbal_length);
    // shoulder pitch is ground-shoulder-elbow angle
    const double shoulder_pitch = alpha - beta;

    return vct3(shoulder_yaw, shoulder_pitch, elbow_pitch);
}

vct3 robManipulatorMTM::WristGimbalIK(const vctRot3& rotation_47) const
{
    // Add virtual frame 8 to align frame 7 with frame 4
    const vctRot3 rotation_48 = rotation_47 * rotation_78;

    // decompose rotation from frame 4 to frame 8 into Euler angles,
    vctEulerZYXRotation3 euler_rotation_decomposition(rotation_48);

    // alignment of frame 4 means pitch/yaw/roll are Z/Y/X rotations
    // roll joint axis is negative of frame's x axis
    const double raw_wrist_pitch = euler_rotation_decomposition.alpha();
    const double raw_wrist_yaw = euler_rotation_decomposition.beta();
    const double raw_wrist_roll = -euler_rotation_decomposition.gamma();

    // finds equivalent angle inside joint limits when possible,
    // however does not actually clamp to joint limits
    const double wrist_pitch = ClosestAngleToJointRange(raw_wrist_pitch, 2 * cmnPI, links[4].PositionMin(), links[4].PositionMax());
    const double wrist_yaw = ClosestAngleToJointRange(raw_wrist_yaw, 2 * cmnPI, links[5].PositionMin(), links[5].PositionMax());
    const double wrist_roll = ClosestAngleToJointRange(raw_wrist_roll, 2 * cmnPI, links[6].PositionMin(), links[6].PositionMax());

    return vct3(wrist_pitch, wrist_yaw, wrist_roll);
}

double robManipulatorMTM::ClosestAngleToJointRange(
    double angle, double modulus, double min, double max
)
{
    const double range_center = 0.5*(min + max);
    return std::remainder(angle - range_center, modulus) + range_center;
}

double robManipulatorMTM::ClosestEquivalentAngle(
    double angle, double target, double min, double max
)
{
    const double value = std::remainder(angle - target, 2 * cmnPI) + target;
    if (min <= value && value <= max) {
        return value;
    } else if (value < min) {
        return value + 2 * cmnPI;
    } else {
        return value - 2 * cmnPI;
    }
}

double robManipulatorMTM::ChoosePlatformAngle(const double current_platform, const vct3& gripper_axis) const
{
    const double z_min = std::sin(links.at(5).PositionMin());
    const double z_max = std::sin(links.at(5).PositionMax());

    const double alpha_min = links.at(4).PositionMin();
    const double alpha_max = links.at(4).PositionMax();

    const double raw_alpha = std::atan2(gripper_axis.Y(), gripper_axis.X());
    const double alpha = ClosestAngleToJointRange(raw_alpha, 2 * cmnPI, alpha_min, alpha_max);
    if (z_min <= gripper_axis.Z() && gripper_axis.Z() <= z_max && alpha_min <= alpha && alpha <= alpha_max) {
        return current_platform;
    }

    std::vector<double> x_options, z_options;

    double radius2 = gripper_axis.Y()*gripper_axis.Y() + z_min*z_min;
    if (radius2 <= 1.0) {
        x_options.push_back( std::sqrt(1.0 - radius2));
        x_options.push_back(-std::sqrt(1.0 - radius2));
        z_options.push_back(z_min);
        z_options.push_back(z_min);
    } else {
        const double x = std::cos(alpha_min) * gripper_axis.Y() / std::sin(alpha_min);
        x_options.push_back(x);
        z_options.push_back(-std::sqrt(1.0 - gripper_axis.Y()*gripper_axis.Y() - x*x));
    }

    radius2 = gripper_axis.Y()*gripper_axis.Y() + z_max*z_max;
    if (radius2 <= 1.0) {
        x_options.push_back( std::sqrt(1.0 - radius2));
        x_options.push_back(-std::sqrt(1.0 - radius2));
        z_options.push_back(z_max);
        z_options.push_back(z_max);
    } else {
        const double x = std::cos(alpha_min) * gripper_axis.Y() / std::sin(alpha_min);
        x_options.push_back(x);
        z_options.push_back(std::sqrt(1.0 - gripper_axis.Y()*gripper_axis.Y() - x*x));
    }

    double best_distance = 2 * cmnPI;
    double best_platform = 0.0;
    for (size_t i = 0; i < x_options.size(); i++) {
        const double x = x_options[i];
        const double y = gripper_axis.Y();
        const double z = z_options[i];

        const double raw_alpha = std::atan2(y, x);
        const double alpha = ClosestAngleToJointRange(raw_alpha, 2 * cmnPI, alpha_min, alpha_max);
        if (alpha < alpha_min || alpha_max < alpha) {
            continue;
        }

        const double phi = std::atan2(gripper_axis.X(), gripper_axis.Z());
        const double psi = std::atan2(x, z);
        const double delta = phi - psi;
        const double platform = ClosestAngleToJointRange(current_platform + delta, 2 * cmnPI, links.at(3).PositionMin(), links.at(3).PositionMax());
        const double movement_distance = std::abs(platform - current_platform);
        const bool feasible_platform = links.at(3).PositionMin() <= platform && platform <= links.at(3).PositionMax();
        if (feasible_platform && movement_distance < best_distance) {
            best_distance = movement_distance;
            best_platform = platform;
        }
    }

    return best_platform;
}
