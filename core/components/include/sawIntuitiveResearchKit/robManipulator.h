/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:
 */

/*
  Author(s):  Brendan Burkhart
  Created on: 2026-09-27

  (C) Copyright 2026 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

#ifndef _robManipulator_h
#define _robManipulator_h

#include <cisstCommon/cmnConstants.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <cisstRobot/robJacobian.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <mutex>
#include <optional>

// robManipulator encapsulates  Some convienence functions
class robManipulator
{
protected:
    const prmConfigurationJoint joint_config;

    mutable std::mutex offset_mutex;
    Eigen::Isometry3d end_effector_offset = Eigen::Isometry3d::Identity();

    Eigen::Isometry3d input_pose;
    Eigen::VectorXd reference_joints;

    robJacobian body_jacobian;
    robJacobian spatial_jacobian;

    // Covariant thread-safe clone, used to implement non-covariant clone()
    virtual robManipulator* raw_clone() const = 0;

    // Non-thread safe, computes inverse kinematics of goal pose *without*
    // end-effector offset
    virtual bool internalInverseKinematics(
        const Eigen::Isometry3d& goal_pose,
        Eigen::VectorXd& joint_positions,
        std::string& error_message
    ) = 0;

    // Non-thread safe, computes forward kinematics *including* end-effector
    // offset when segments<0
    virtual void internalForwardKinematics(
        const Eigen::VectorXd& joint_positions,
        Eigen::Isometry3d& output_pose,
        int segments
    ) = 0;

    // Only for revolute joints - find alternative joint value that is 1)
    // equivalent to `value` (mod 2pi), 2) closest to `reference` of all
    // possible alternatives, 3) within joint limits if possible
    double closestEquivalent(double reference, double value, Eigen::Index index, double modulus=2 * cmnPI) const;

public:
    robManipulator(const prmConfigurationJoint& joint_config);
    virtual ~robManipulator() = default;

    // Thread safe copy-constructor - all non-thread safe functions can be
    // safely used from a second thread by copy-constructing a new
    // robManipulator, and then calling the method on the new instance.
    std::unique_ptr<robManipulator> clone() const;

    // deleted to reduce risk of slicing
    robManipulator(const robManipulator& manipulator) = delete;
    robManipulator& operator=(const robManipulator& manipulator) = delete;
    robManipulator(robManipulator&& manipulator) = delete;
    robManipulator& operator=(robManipulator&& manipulator) = delete;

    // Thread safe
    virtual int numJoints() const = 0;

    // Thread safe
    Eigen::Isometry3d endEffectorOffset() const;

    // Thread safe
    void setEndEffectorOffset(const Eigen::Isometry3d& offset);

    // Compute end-effector pose with respect to base frame, given joint
    // positions. Leave segments=-1 to compute kinematics of full chain,
    // otherwise will compute kinematics only up to specified number of
    // segments/links. End-effector offset is included when segments<0. Not
    // thread safe, uses internally allocated storage.
    void forwardKinematics(
        const Eigen::VectorXd& joint_positions,
        Eigen::Isometry3d& output_pose,
        int segments = -1
    );

    // Compute end-effector pose with respect to base frame, given joint
    // positions. Leave segments=-1 to compute kinematics of full chain,
    // otherwise will compute kinematics only up to specified number of
    // segments/links. This version is thread safe.
    [[nodiscard]] Eigen::Isometry3d forwardKinematics(
        const Eigen::VectorXd& joint_positions, int segments = -1
    ) const;

    // Compute joint positions to bring end-effector to the goal pose (with
    // respect to base frame), returning success flag. End-effector offset is
    // accounted for. If an error occurs, error_message will be set. Note: NOT
    // thread safe.
    bool inverseKinematics(
        const Eigen::Isometry3d& goal_pose,
        Eigen::VectorXd& joint_positions,
        std::string& error_message
    );

    // Compute body and spatial Jacobians, access via bodyJacobian() and
    // spatialJacobian(). Not thread safe. Note the derived subclasses must
    // implement this method by computing the Jacobian matrices, then updating
    // the body_jacobian/spatial_jacobian data members.
    virtual void computeJacobians(const Eigen::VectorXd& joint_positions) = 0;

    // Access latest body Jacobian compute via computeJacobians(). Not thread
    // safe.
    const robJacobian& bodyJacobian() const;

    // Access latest spatial Jacobian compute via computeJacobians(). Not
    // thread safe.
    const robJacobian& spatialJacobian() const;

    // For each revolute joint in joints, attempt to find valid equivalent joint
    // postition that is closest to the reference joint position.
    void harmonizeJoints(
        const Eigen::VectorXd& reference, Eigen::VectorXd& joints
    );

    // Clamp joint values to valid limits, returning true if joints were already
    // valid, or false if clamping had to be applied.
    bool clampJoints(Eigen::VectorXd& joints);

    const prmConfigurationJoint& jointConfig() { return joint_config; }
};

#if CISST_HAS_JSON
std::optional<prmConfigurationJoint> loadJointConfig(const Json::Value& config);
std::optional<KDL::Chain> loadKDLChain(const Json::Value& config);
#endif  // CISST_HAS_JSON

#endif
