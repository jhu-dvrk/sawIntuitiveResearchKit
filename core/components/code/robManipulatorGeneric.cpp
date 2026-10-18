#include <cisstCommon/cmnConstants.h>
#include <cisstRobot/kdl_utilities.h>
#include <cisstRobot/robJacobian.h>
#include <sawIntuitiveResearchKit/robManipulatorGeneric.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <limits>

robManipulatorGeneric::robManipulatorGeneric(
    const prmConfigurationJoint& joint_config, const KDL::Chain& chain
)
    : robManipulator(joint_config),
      chain(chain),
      fk_solver(this->chain),
      ik_velocity(this->chain),
      ik_solver(this->chain, fk_solver, ik_velocity),
      jac_solver(this->chain),
      q_input(this->chain.getNrOfJoints()),
      q_output(this->chain.getNrOfJoints()),
      kdl_body_jacobian(this->chain.getNrOfJoints()),
      kdl_spatial_jacobian(this->chain.getNrOfJoints())
{
    KDL::JntArray lower_limits(joint_config.PositionMin().size());
    lower_limits.data = joint_config.PositionMin();

    KDL::JntArray upper_limits(joint_config.PositionMax().size());
    upper_limits.data = joint_config.PositionMax();

    // Remove joint limits from IK solver for revolute joints with at least 360
    // range of motion - we can always post-process an out-of-range value into
    // an equivalent value within the joint's range of motion, and this prevents
    // the IK solver from getting "stuck" against a joint limit artificially.
    for (Eigen::Index joint = 0; joint < lower_limits.data.size(); joint++) {
        const double range = upper_limits(joint) - lower_limits(joint);
        bool is_revolute = joint_config.Type().at(joint) == CMN_JOINT_REVOLUTE;
        if (is_revolute && range > 2 * cmnPI) {
            upper_limits(joint) = std::numeric_limits<double>::max();
            lower_limits(joint) = std::numeric_limits<double>::lowest();
        }
    }

    ik_solver.setJointLimits(lower_limits, upper_limits);
}

robManipulatorGeneric* robManipulatorGeneric::raw_clone() const
{
    return new robManipulatorGeneric(joint_config, chain);
}

void robManipulatorGeneric::internalForwardKinematics(
    const Eigen::VectorXd& joint_positions,
    Eigen::Isometry3d& output_pose,
    int segments
)
{
    fk_solver.updateInternalDataStructures();
    q_input.data = joint_positions;
    int err = fk_solver.JntToCart(q_input, pose, segments);
    if (err < 0) {
        CMN_LOG_RUN_ERROR << "robManipulatorGeneric::internalForwardKinematics: " << std::string(fk_solver.strError(err)) << std::endl;
    }

    output_pose.linear() = fromKDL(pose.M);
    output_pose.translation() = fromKDL(pose.p);
}

bool robManipulatorGeneric::internalInverseKinematics(
    const Eigen::Isometry3d& goal_pose,
    Eigen::VectorXd& joint_positions,
    std::string& error_message
)
{
    // Set up inputs
    toKDL(joint_positions, q_input);
    toKDL(goal_pose, pose);

    const int err = ik_solver.CartToJnt(q_input, pose, q_output);
    if (err < 0) {
        error_message = std::string(ik_solver.strError(err));
        return false;
    }

    fromKDL(q_output, joint_positions);
    return true;
}

int robManipulatorGeneric::numJoints() const {
    return chain.getNrOfJoints();
}

void robManipulatorGeneric::computeJacobians(
    const Eigen::VectorXd& joint_positions
)
{
    q_input.data = joint_positions;
    jac_solver.JntToJac(q_input, kdl_body_jacobian);

    // make a copy before we modify the body Jacobian
    kdl_spatial_jacobian = kdl_body_jacobian;

    fk_solver.JntToCart(q_input, pose);

    // KDL compute Jacobian with respect to chain's base frame, but the
    // reference point is the end-effector. So to get standard body Jacobian, we
    // need to change the reference frame, and to get the standard spatial
    // Jacobian we need to change the reference point to the origin.
    kdl_spatial_jacobian.changeRefPoint(-pose.p);
    spatial_jacobian.update(kdl_spatial_jacobian.data);

    // modify in place so we don't have to allocate a copy
    pose.M.SetInverse();

    kdl_body_jacobian.changeBase(pose.M);
    body_jacobian.update(kdl_body_jacobian.data);
}
