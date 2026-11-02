/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:
 */

/*
  Author(s):  Brendan Burkhart
  Created on: 2026-01-02

  (C) Copyright 2026 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

#ifndef _robManipulatorGeneric_h
#define _robManipulatorGeneric_h

#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <sawIntuitiveResearchKit/robManipulator.h>
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

// Generic/non-specialized implementation of robManipulator via KDL. Can be
// derived from to provide more specialized IK solvers, but if FK/Jacobians also
// need to be specialized the base robManipulator should be derived from
// instead.
class robManipulatorGeneric : public robManipulator
{
protected:
    robManipulatorGeneric* raw_clone() const override;

    bool internalInverseKinematics(
        const Eigen::Isometry3d& goal_pose,
        Eigen::VectorXd& joint_positions,
        std::string& error_message
    ) override;

    void internalForwardKinematics(
        const Eigen::VectorXd& joint_positions,
        Eigen::Isometry3d& output_pose,
        int segments
    ) override;

public:
    robManipulatorGeneric(
        const prmConfigurationJoint& joint_config, const KDL::Chain& chain
    );

    int numJoints() const override;

    void computeJacobians(const Eigen::VectorXd& joint_positions) override;

protected:
    const KDL::Chain chain;

    KDL::ChainFkSolverPos_recursive fk_solver;
    KDL::ChainIkSolverVel_wdls ik_velocity;
    KDL::ChainIkSolverPos_NR_JL ik_solver;

    KDL::ChainJntToJacSolver jac_solver;

    // Working space / storage allocation
    KDL::JntArray q_input, q_output;
    KDL::Frame pose;

    KDL::Jacobian kdl_body_jacobian;
    KDL::Jacobian kdl_spatial_jacobian;
};

#endif
