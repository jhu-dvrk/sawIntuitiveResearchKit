#include <cisstCommon/cmnJointType.h>
#include <cisstCommon/cmnLogger.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <cisstRobot/kdl_utilities.h>
#include <cisstRobot/robLinkDynamics.h>
#include <cisstRobot/robLinkKinematics.h>
#include <cisstRobot/robModifiedDH.h>
#include <cisstRobot/robStandardDH.h>
#include <sawIntuitiveResearchKit/robManipulator.h>

#include <memory>

robManipulator::robManipulator(const prmConfigurationJoint& joint_config)
    : joint_config(joint_config)
{
}

std::unique_ptr<robManipulator> robManipulator::clone() const
{
    Eigen::Isometry3d offset = endEffectorOffset();
    auto manipulator = std::unique_ptr<robManipulator>(raw_clone());
    manipulator->setEndEffectorOffset(offset);
    return manipulator;
}

Eigen::Isometry3d robManipulator::endEffectorOffset() const
{
    std::scoped_lock lock(offset_mutex);
    return end_effector_offset;
}

void robManipulator::setEndEffectorOffset(const Eigen::Isometry3d& offset)
{
    std::scoped_lock lock(offset_mutex);
    end_effector_offset = offset;
}

void robManipulator::forwardKinematics(
    const Eigen::VectorXd& joint_positions,
    Eigen::Isometry3d& output_pose,
    int segments
)
{
    internalForwardKinematics(joint_positions, output_pose, segments);
    if (segments < 0) { output_pose = output_pose * end_effector_offset; }
}

Eigen::Isometry3d robManipulator::forwardKinematics(
    const Eigen::VectorXd& joint_positions, int segments
) const
{
    const auto instance = clone();
    Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
    instance->forwardKinematics(joint_positions, output, segments);
    return output;
}

bool robManipulator::inverseKinematics(
    const Eigen::Isometry3d& goal_pose,
    Eigen::VectorXd& joint_positions,
    std::string& error_message
)
{
    // retain copy of initial values for harmonization
    reference_joints = joint_positions;

    input_pose = goal_pose * end_effector_offset.inverse();

    bool ok =
        internalInverseKinematics(input_pose, joint_positions, error_message);
    if (!ok) { return false; }

    harmonizeJoints(reference_joints, joint_positions);

    bool joints_within_limits = clampJoints(joint_positions);
    if (!joints_within_limits) {
        error_message = "joint values out of limits";
        return false;
    }

    return true;
}

const robJacobian& robManipulator::bodyJacobian() const
{
    return body_jacobian;
}

const robJacobian& robManipulator::spatialJacobian() const
{
    return spatial_jacobian;
}

// Only for revolute joints - find alternative joint value (equivalent mod
// 2pi) that is 1) within joint limits, 2) closest to given joint
double robManipulator::closestEquivalent(
    double reference, double value, Eigen::Index index, double modulus
) const
{
    const double min = joint_config.PositionMin()[index];
    const double max = joint_config.PositionMax()[index];

    // If reference joint position is outside valid joint range, use closest
    // limit instead
    const double valid_reference = std::clamp(reference, min, max);

    const auto closest = [=](double reference) {
        const double turns = std::llround((reference - value) / modulus);
        return value + modulus * turns;
    };

    const auto valid = [=](double value) {
        return min <= value && value <= max;
    };

    // Closest joint position to the reference that is equivalent to value
    // If we didn't care about joint limits, this would be the solution
    const double closest_equivalent = closest(valid_reference);

    if (valid(closest_equivalent)) { return closest_equivalent; }

    // If outside joint limits, check if previous/next equivalent position is
    // inside the limits
    if (closest_equivalent > max) {
        const double prev_equivalent = closest_equivalent - modulus;
        if (valid(prev_equivalent)) { return prev_equivalent; }
    } else if (closest_equivalent < min) {
        const double next_equivalent = closest_equivalent + modulus;
        if (valid(next_equivalent)) { return next_equivalent; }
    }

    return closest_equivalent;
}

void robManipulator::harmonizeJoints(
    const Eigen::VectorXd& reference, Eigen::VectorXd& joints
)
{
    for (size_t idx = 0; idx < joint_config.Type().size(); idx++) {
        if (joint_config.Type()[idx] != CMN_JOINT_REVOLUTE) { continue; }

        // number of full rotations to bring joints[idx] as close as possible to
        // the reference
        const double turns =
            std::llround((reference[idx] - joints[idx]) / (2 * cmnPI));

        joints[idx] += 2 * cmnPI * turns;
    }
}

bool robManipulator::clampJoints(Eigen::VectorXd& joints)
{
    bool valid = (joints.array() >= joint_config.PositionMin() &&
                  joints.array() <= joint_config.PositionMax())
                     .all();

    if (valid) { return true; }

    joints.array()
        .cwiseMax(joint_config.PositionMin())
        .cwiseMin(joint_config.PositionMax());

    return false;
}

#if CISST_HAS_JSON

std::optional<prmConfigurationJoint> loadJointConfig(const Json::Value& config)
{
    prmConfigurationJoint joint_config;

    const Json::Value links =
        config["joints"].isNull() ? config["links"] : config["joints"];
    if (links.isNull()) {
        CMN_LOG_INIT_ERROR
            << "Joint config must contain \"links\" or \"joints\"" << std::endl;
        return {};
    }

    joint_config.PositionMin() = Eigen::VectorXd::Zero(links.size());
    joint_config.PositionMax() = Eigen::VectorXd::Zero(links.size());

    joint_config.VelocityMin() = Eigen::VectorXd::Zero(links.size());
    joint_config.VelocityMax() = Eigen::VectorXd::Zero(links.size());
    joint_config.AccelerationMin() = Eigen::VectorXd::Zero(links.size());
    joint_config.AccelerationMax() = Eigen::VectorXd::Zero(links.size());

    joint_config.EffortMin() = Eigen::VectorXd::Zero(links.size());
    joint_config.EffortMax() = Eigen::VectorXd::Zero(links.size());

    for (Json::ArrayIndex idx = 0; idx < links.size(); idx++) {
        const Json::Value link = links[idx];
        if (link.isNull() || !link.isObject()) {
            CMN_LOG_INIT_ERROR << "Joint config: at index" << idx
                               << " is not an object" << std::endl;
            return {};
        }

        const auto joint_type_name = link.get("type", "").asString();
        const auto joint_type =
            robLinkKinematics::jointTypeFromString(joint_type_name);
        if (!joint_type.has_value()) {
            CMN_LOG_INIT_ERROR
                << "Joint config: unknown/unsupported joint type "
                << joint_type_name << std::endl;
            return {};
        }

        joint_config.Name().push_back(link.get("name", "").asString());
        joint_config.Type().push_back(joint_type.value());
        joint_config.PositionMin()[idx] = link.get("qmin", 0.0).asDouble();
        joint_config.PositionMax()[idx] = link.get("qmax", 0.0).asDouble();

        const double max_effort = link.get("ftmax", 0.0).asDouble();
        joint_config.EffortMin()[idx] = -max_effort;
        joint_config.EffortMax()[idx] = max_effort;
    }

    return joint_config;
}

std::optional<KDL::Chain> loadKDLChain(const Json::Value& config)
{
    const std::string convention = config.get("convention", "").asString();
    if (!convention.empty() && convention != "modified" && convention != "standard") {
        CMN_LOG_INIT_ERROR << "Unsupported kinematic convention (must be "
                              "\"modified\" or \"standard\"): \""
                           << convention << "\"" << std::endl;
        return {};
    }

    const Json::Value links =
        config["joints"].isNull() ? config["links"] : config["joints"];
    if (links.isNull()) {
        CMN_LOG_INIT_ERROR
            << "Joint config must contain \"links\" or \"joints\"" << std::endl;
        return {};
    }

    std::vector<std::pair<std::unique_ptr<robLinkKinematics>, robLinkDynamics>>
        kinematics_dynamics_chain;

    for (Json::ArrayIndex idx = 0; idx < links.size(); idx++) {
        const Json::Value link = links[idx];
        if (link.isNull() || !link.isObject()) {
            CMN_LOG_INIT_ERROR << "Kinematic chain: link at index" << idx
                               << " is not an object" << std::endl;
            return {};
        }
        const std::string link_convention = link.get("convention", convention).asString();

        std::unique_ptr<robLinkKinematics> kinematics;
        if (link_convention == "modified") {
            const auto value = robModifiedDH::Read(link);
            if (!value.has_value()) {
                CMN_LOG_INIT_ERROR
                    << "Kinematic chain: error in link kinematics at index"
                    << idx << std::endl;
                return {};
            }

            kinematics = std::make_unique<robModifiedDH>(*value);
        } else if (link_convention == "standard") {
            const auto value = robStandardDH::Read(link);
            if (!value.has_value()) {
                CMN_LOG_INIT_ERROR
                    << "Kinematic chain: error in link kinematics at index"
                    << idx << std::endl;
                return {};
            }

            kinematics = std::make_unique<robStandardDH>(*value);
        } else {
            CMN_LOG_INIT_ERROR << "Unsupported kinematic convention (must be "
                                  "\"modified\" or \"standard\"): \""
                               << link_convention << "\"" << std::endl;
            return {};
        }

        const auto dynamics = robLinkDynamics::Read(link);
        if (!dynamics.has_value()) {
            CMN_LOG_INIT_ERROR
                << "Kinematic chain: error in link dynamics at index" << idx
                << std::endl;
            return {};
        }

        kinematics_dynamics_chain.push_back({std::move(kinematics), *dynamics});
    }

    return loadChain(kinematics_dynamics_chain);
}

#endif  // CISST_HAS_JSON
