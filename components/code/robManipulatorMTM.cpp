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

// joints:
//   0: shoulder yaw. 0 is centered, positive is counterclockwise when viewed from above
//   1: shoulder pitch. 0 is straight down, positive is rotating forward
//   2: elbow pitch. 0 is straight forward, positive is rotating forward/up
//   3: gimbal platform yaw. 0 is outer side, depending on left/right MTM. positive is counterclockwise from above
//   4: wrist pitch. 0 is upright, positive is rotating forwards
//   5: wrist yaw. 0 is 90 from wrist, positive is counterclockwise from above
//   6: wrist roll. zero is horizontal, positive is *clockwise* from the front

// inverse of frame 4->7 transformation given above
const vctRot3 robManipulatorMTM::rotation_78 = vctRot3(vctEulerYZXRotation3(cmnPI_2, 0, cmnPI_2));

robManipulatorMTM::robManipulatorMTM(const std::vector<robKinematics *> linkParms,
                                     const vctFrame4x4<double> &Rtw0)
    : robManipulator(linkParms, Rtw0) {}

robManipulatorMTM::robManipulatorMTM(const std::string &robotfilename,
                                     const vctFrame4x4<double> &Rtw0)
    : robManipulator(robotfilename, Rtw0) {}

robManipulatorMTM::robManipulatorMTM(const vctFrame4x4<double> &Rtw0)
    : robManipulator(Rtw0) {}

robManipulator::Errno
robManipulatorMTM::InverseKinematics(vctDynamicVector<double>& q,
                                     const vctFrame4x4<double>& Rts,
                                     double CMN_UNUSED(tolerance),
                                     size_t CMN_UNUSED(Niterations),
                                     double CMN_UNUSED(LAMBDA))
{
    if (q.size() != links.size()) {
        std::stringstream ss;
        ss << "robManipulatorMTM::InverseKinematics: expected " << links.size()
           << " joints values but received " << q.size();
        mLastError = ss.str();
        CMN_LOG_RUN_ERROR << mLastError << std::endl;
        return robManipulator::EFAILURE;
    }

    if (links.size() != 7) {
        std::stringstream ss;
        ss << "robManipulatorMTM::InverseKinematics: manipulator should have 7 links"
           << " but received " << links.size();
        mLastError = ss.str();
        CMN_LOG_RUN_ERROR << mLastError << std::endl;
        return robManipulator::EFAILURE;
    }

    // take Rtw0 (world to frame 0) into account
    vctFrm4x4 Rt07 = Rtw0.Inverse()*Rts;

    // shoulder and elbow entirely determine position, so we can solve
    // for first three joints separately
    vct3 position_ik = ShoulderElbowIK(Rt07.Translation());
    q[0] = position_ik[0];
    q[1] = position_ik[1];
    q[2] = position_ik[2];

    // determine remaining transformation once shoulder/elbow are set
    q[3] = 0.0;
    vctFrm4x4 Rt04 = ForwardKinematics(q, 4);
    vctFrm4x4 Rt47 = Rt04.Inverse()*Rt07;

    // optimized yaw of platform to maximize range of motion
    q[3] = ChoosePlatformYaw(vctRot3(Rt47.Rotation()));

    // once platform yaw has been chosen, need to re-compute 4->7 frame transformation
    Rt04 = ForwardKinematics(q, 4);
    Rt47 = Rt04.Inverse()*Rt07;

    // once platform yaw is set, wrist/gimbal IK is uniquely determined by
    // the remaining portion of the desired overall transformation
    vct3 gimbal_ik = WristGimbalIK(vctRot3(Rt47.Rotation()));
    q[4] = gimbal_ik[0];
    q[5] = gimbal_ik[1];
    q[6] = gimbal_ik[2];

    // copy prevents ODR-use, which in pre-C++17 requires a definition in addition to declaration
    constexpr double tolerance = joint_limit_tolerance;

    // check+enforce all joint limits
    bool joint_limit_reached = false;
    for (size_t joint = 0; joint < 7; joint++) {
        bool out_of_range = ClampJointValueAndUpdateError(joint, q[joint], tolerance);
        joint_limit_reached = joint_limit_reached || out_of_range;
    }

    return joint_limit_reached ? EFAILURE : ESUCCESS;
}

// solve for triangle's interior angle opposite from side c
double robManipulatorMTM::SolveTriangleInteriorAngle(double side_a, double side_b, double side_c) const
{
    // law of cosines: c^2 = a^2 + b^2 - 2ab*cos(gamma)
    // where triangle has sides a, b, c and gamma is the angle opposite side c
    double numerator = side_a*side_a + side_b*side_b - side_c*side_c;
    double denominator = 2*side_a*side_b;
    double cos_gamma = numerator/denominator;
    return std::acos(cos_gamma);
}

vct3 robManipulatorMTM::ShoulderElbowIK(const vct3& position_07) const {
    // shoulder yaw is just angle in transverse/horizontal plane
    // coordinate system of shoulder is rotated 90 degrees, so we use X/-Y instead of Y/X
    const double shoulder_yaw = std::atan2(position_07.X(), -position_07.Y());

    const double shoulder_to_wrist_distance = position_07.Norm();
    const double elbow_to_gimbal_angle = std::atan(forearm_to_gimbal_m/forearm_length_m);
    const double elbow_to_gimbal_length = std::sqrt(forearm_to_gimbal_m*forearm_to_gimbal_m + forearm_length_m*forearm_length_m);

    // shoulder-elbow-wrist is a triangle with three known side lengths
    // so the shoulder-elbow-wrist angle is fully determined
    const double theta = SolveTriangleInteriorAngle(upper_arm_length_m, elbow_to_gimbal_length, shoulder_to_wrist_distance);
    // interior angle formed by shoulder-elbow-forearm
    const double elbow_interior_angle = theta + elbow_to_gimbal_angle;
    // elbow joint pitch is supplementary/exterior angle, with 90 degree offset
    const double elbow_pitch = (cmnPI - elbow_interior_angle) - cmnPI_2;

    // angle of ground-shoulder-wrist
    const double alpha = std::acos(-position_07.Z()/shoulder_to_wrist_distance);
    // angle of elbow-shoulder-wrist
    const double beta = SolveTriangleInteriorAngle(upper_arm_length_m, shoulder_to_wrist_distance, elbow_to_gimbal_length);
    // shoulder pitch is ground-shoulder-elbow angle
    const double shoulder_pitch = alpha - beta;

    return vct3(shoulder_yaw, shoulder_pitch, elbow_pitch);
}

vct3 robManipulatorMTM::WristGimbalIK(const vctRot3& rotation_47) const
{
    // Add virtual frame 8 to align frame 7 with frame 4
    const vctRot3 rotation_48 = rotation_47*rotation_78;

    // decompose rotation from frame 4 to frame 8 into Euler angles,
    vctEulerZYXRotation3 euler_rotation_decomposition(rotation_48);

    // alignment of frame 4 means pitch/yaw/roll are Z/Y/X rotations
    const double wrist_pitch = euler_rotation_decomposition.alpha();
    const double wrist_yaw = euler_rotation_decomposition.beta();
    const double wrist_roll = -euler_rotation_decomposition.gamma();

    return vct3(wrist_pitch, wrist_yaw, wrist_roll);
}

double robManipulatorMTM::ChoosePlatformYaw(const vctRot3& rotation_47) const
{
    // Add virtual frame 8 to align frame 7 with frame 4.
    const vctRot3 rotation_48 = rotation_47*rotation_78;

    // We want platform yaw to account for as muchas possible of the
    // overall yaw rotation of the 4->7 transformation. This maximizes
    // overall range of motion by minimizing the rotation required
    // from the remaining joints.
    vctEulerYZXRotation3 yaw_decomposition(rotation_48);
    const double yaw_angle = yaw_decomposition.alpha();

    // Find yaw (mod 2PI) that is within joint limits,
    // or find closest joint limit (mod 2PI).
    const double max = links[3].GetKinematics()->PositionMax();
    const double min = links[3].GetKinematics()->PositionMin();

    double best_value = yaw_angle;
    double best_distance = 2*cmnPI;

    for (int k = -1; k <= 1; k++) {
        double value = yaw_angle + k*2*cmnPI;
        if (value < min) {
            double distance = min - value;
            if (distance < best_distance) {
                best_distance = distance;
                best_value = min;
            }
        } else if (value > max) {
            double distance = value - max;
            if (distance < best_distance) {
                best_distance = distance;
                best_value = max;
            }
        } else {
            best_value = value;
            best_distance = 0.0;
        }
    }

    return best_value;
}
