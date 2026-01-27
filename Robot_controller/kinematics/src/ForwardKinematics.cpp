#include "kinematics/ForwardKinematics.h"
#include "robot_model/RobotModel.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace kinematics {

ForwardKinematics::ForwardKinematics()
    : joint_indices_(nullptr), joint_names_(nullptr) {}

ForwardKinematics::~ForwardKinematics() {}

bool ForwardKinematics::init(const robot_model::RobotModel &robot_model) {
  model_ = robot_model.getModel();
  if (!model_) {
    std::cerr << "[FK] Error: RobotModel has no URDF loaded" << std::endl;
    return false;
  }

  // Use RobotModel's pre-computed joint map (no duplication!)
  joint_indices_ = &robot_model.getJointIndices();
  joint_names_ = &robot_model.getJointNames();

  std::cout << "[FK] Initialized with " << joint_names_->size()
            << " joints from RobotModel" << std::endl;

  return true;
}

bool ForwardKinematics::computeLinkTransforms(
    const Eigen::VectorXd &q,
    std::map<std::string, Eigen::Isometry3d> &transforms) {

  if (!model_ || !joint_indices_) {
    std::cerr << "FK not initialized" << std::endl;
    return false;
  }

  transforms.clear();

  // Start from root with identity transform
  Eigen::Isometry3d root_transform = Eigen::Isometry3d::Identity();

  if (model_->getRoot()) {
    computeLinkTransformsRecursive(model_->getRoot(), root_transform, q,
                                   transforms);
  }

  return true;
}

void ForwardKinematics::computeLinkTransformsRecursive(
    urdf::LinkConstSharedPtr link, const Eigen::Isometry3d &parent_transform,
    const Eigen::VectorXd &q,
    std::map<std::string, Eigen::Isometry3d> &transforms) {

  // Current link transform starts from parent
  Eigen::Isometry3d link_transform = parent_transform;

  // If this link has a parent joint, apply the joint transform
  if (link->parent_joint) {
    auto joint = link->parent_joint;

    // Get joint origin (fixed part)
    auto &origin = joint->parent_to_joint_origin_transform;
    Eigen::Vector3d translation(origin.position.x, origin.position.y,
                                origin.position.z);

    double roll, pitch, yaw;
    origin.rotation.getRPY(roll, pitch, yaw);

    // RPY to rotation matrix (ZYX convention)
    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);

    Eigen::Matrix3d rotation;
    rotation << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr,
        cp * cr;

    Eigen::Isometry3d joint_origin_transform = Eigen::Isometry3d::Identity();
    joint_origin_transform.linear() = rotation;
    joint_origin_transform.translation() = translation;

    // Apply joint origin
    link_transform = parent_transform * joint_origin_transform;

    // Apply joint angle (variable part)
    if ((joint->type == urdf::Joint::REVOLUTE ||
         joint->type == urdf::Joint::CONTINUOUS ||
         joint->type == urdf::Joint::PRISMATIC) &&
        joint_indices_->count(joint->name) > 0) {

      int joint_idx = joint_indices_->at(joint->name);
      double joint_value = (joint_idx < q.size()) ? q[joint_idx] : 0.0;

      // Handle mimic joints - use the mimic'd joint's value with multiplier and
      // offset
      if (joint->mimic) {
        auto mimic_joint_it = joint_indices_->find(joint->mimic->joint_name);
        if (mimic_joint_it != joint_indices_->end()) {
          int mimic_idx = mimic_joint_it->second;
          double mimic_value = (mimic_idx < q.size()) ? q[mimic_idx] : 0.0;
          joint_value =
              mimic_value * joint->mimic->multiplier + joint->mimic->offset;
        }
      }

      Eigen::Isometry3d joint_variable_transform =
          Eigen::Isometry3d::Identity();

      if (joint->type == urdf::Joint::REVOLUTE ||
          joint->type == urdf::Joint::CONTINUOUS) {
        // Rotation about joint axis
        auto &axis = joint->axis;
        Eigen::Vector3d axis_vec(axis.x, axis.y, axis.z);
        axis_vec.normalize();

        // Rodrigues formula for rotation
        double angle = joint_value;
        Eigen::Matrix3d K;
        K << 0, -axis_vec.z(), axis_vec.y(), axis_vec.z(), 0, -axis_vec.x(),
            -axis_vec.y(), axis_vec.x(), 0;

        Eigen::Matrix3d rot = Eigen::Matrix3d::Identity() + sin(angle) * K +
                              (1 - cos(angle)) * K * K;

        joint_variable_transform.linear() = rot;

      } else if (joint->type == urdf::Joint::PRISMATIC) {
        // Translation along joint axis
        auto &axis = joint->axis;
        Eigen::Vector3d translation_offset(
            axis.x * joint_value, axis.y * joint_value, axis.z * joint_value);
        joint_variable_transform.translation() = translation_offset;
      }

      link_transform = link_transform * joint_variable_transform;
    }
  }

  // Store this link's transform
  transforms[link->name] = link_transform;

  // Recursively process children
  for (const auto &child : link->child_links) {
    computeLinkTransformsRecursive(child, link_transform, q, transforms);
  }
}

Eigen::Isometry3d
ForwardKinematics::getLinkTransform(const std::string &link_name,
                                    const Eigen::VectorXd &q) {

  std::map<std::string, Eigen::Isometry3d> transforms;
  computeLinkTransforms(q, transforms);

  auto it = transforms.find(link_name);
  if (it != transforms.end()) {
    return it->second;
  }

  return Eigen::Isometry3d::Identity();
}

void ForwardKinematics::printRobotStructure() const {
  if (!model_)
    return;
  std::cout << "Robot Name: " << model_->getName() << std::endl;
  std::cout << "Root Link: " << model_->getRoot()->name << std::endl;
  std::cout << "Tree Structure:" << std::endl;
  printLinkRecursive(model_->getRoot(), 0);
}

void ForwardKinematics::printLinkRecursive(urdf::LinkConstSharedPtr link,
                                           int level) const {
  std::string indent(level * 2, ' ');
  std::cout << indent << "- Link: " << link->name << std::endl;

  // Print joint info if it exists (parent joint)
  if (link->parent_joint) {
    std::cout << indent << "  Joint: " << link->parent_joint->name;
    std::cout << " (Type: " << link->parent_joint->type << ")";

    auto origin = link->parent_joint->parent_to_joint_origin_transform.position;
    std::cout << " Origin: [" << origin.x << ", " << origin.y << ", "
              << origin.z << "]";
    std::cout << std::endl;
  }

  // Print inertial info
  if (link->inertial) {
    std::cout << indent << "  Mass: " << link->inertial->mass << std::endl;
  }

  for (const auto &child : link->child_links) {
    printLinkRecursive(child, level + 1);
  }
}

bool ForwardKinematics::getJacobian(
    const Eigen::VectorXd &q, const std::string &ee_link,
    Eigen::Matrix<double, 6, Eigen::Dynamic> &J_out, int numIKJoints) {
  if (!model_) {
    std::cerr << "[FK] Robot model not initialized!" << std::endl;
    return false;
  }

  int total_joints = getNumJoints();

  // Use provided numIKJoints, or default to input size if not specified
  int n_ik = (numIKJoints > 0) ? numIKJoints : static_cast<int>(q.size());
  n_ik = std::min(n_ik, total_joints);

  // Pad q to full joint count if needed (gripper joints default to 0)
  Eigen::VectorXd q_full = Eigen::VectorXd::Zero(total_joints);
  for (int i = 0; i < std::min(static_cast<int>(q.size()), total_joints); ++i) {
    q_full[i] = q[i];
  }

  std::map<std::string, Eigen::Isometry3d> link_transforms;
  computeLinkTransforms(q_full, link_transforms);

  auto ee_it = link_transforms.find(ee_link);
  if (ee_it == link_transforms.end()) {
    std::cerr << "[FK] End-effector link '" << ee_link << "' not found!"
              << std::endl;
    return false;
  }

  Eigen::Vector3d p_e = ee_it->second.translation();
  std::cout << "[FK::getJacobian] Computing Jacobian for " << n_ik
            << " IK joints (total: " << total_joints << ")" << std::endl;
  std::cout << "[FK::getJacobian] EE position: [" << p_e.x() << ", " << p_e.y()
            << ", " << p_e.z() << "]" << std::endl;

  // Output Jacobian only for IK joints (typically 6 for arm, excluding gripper)
  J_out = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, n_ik);

  // Iterate only IK joints (first n_ik joints)
  for (int i = 0; i < n_ik; ++i) {
    std::string joint_name = joint_names_->at(i);
    urdf::JointConstSharedPtr joint = model_->getJoint(joint_name);
    if (!joint) {
      std::cerr << "[FK::getJacobian] Joint[" << i << "] '" << joint_name
                << "' not found!" << std::endl;
      continue;
    }

    // Get joint frame transform (parent link transform + joint origin)
    auto parent_it = link_transforms.find(joint->parent_link_name);
    if (parent_it == link_transforms.end())
      continue;

    Eigen::Isometry3d T_parent = parent_it->second;

    // Apply joint origin offset to get joint frame
    auto &origin = joint->parent_to_joint_origin_transform;
    Eigen::Isometry3d T_offset = Eigen::Isometry3d::Identity();
    T_offset.translation() = Eigen::Vector3d(
        origin.position.x, origin.position.y, origin.position.z);
    double roll, pitch, yaw;
    origin.rotation.getRPY(roll, pitch, yaw);
    T_offset.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                            .toRotationMatrix();

    Eigen::Isometry3d joint_tf = T_parent * T_offset;
    Eigen::Vector3d joint_pos = joint_tf.translation();

    // Joint axis in local frame, transformed to world frame
    Eigen::Vector3d axis_local(joint->axis.x, joint->axis.y, joint->axis.z);
    axis_local.normalize();
    Eigen::Vector3d joint_axis = joint_tf.rotation() * axis_local;

    std::cout << "[FK::getJacobian] Joint[" << i << "] '" << joint_name
              << "' type=" << joint->type << " pos=[" << joint_pos.x() << ","
              << joint_pos.y() << "," << joint_pos.z() << "]"
              << " axis=[" << joint_axis.x() << "," << joint_axis.y() << ","
              << joint_axis.z() << "]" << std::endl;

    if (joint->type == urdf::Joint::REVOLUTE ||
        joint->type == urdf::Joint::CONTINUOUS) {
      // For revolute: J_v = z × (p_ee - p_joint), J_ω = z
      Eigen::Vector3d linear = joint_axis.cross(p_e - joint_pos);
      Eigen::Vector3d angular = joint_axis;
      J_out.block<3, 1>(0, i) = linear;
      J_out.block<3, 1>(3, i) = angular;
    } else if (joint->type == urdf::Joint::PRISMATIC) {
      // For prismatic: J_v = z, J_ω = 0
      Eigen::Vector3d linear = joint_axis;
      Eigen::Vector3d angular = Eigen::Vector3d::Zero();
      J_out.block<3, 1>(0, i) = linear;
      J_out.block<3, 1>(3, i) = angular;
    }
    // Fixed joints contribute zero (not included for IK joints anyway)
  }

  return true;
}

} // namespace kinematics
