#include "kinematics/ForwardKinematics.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace kinematics {

ForwardKinematics::ForwardKinematics() {}

ForwardKinematics::~ForwardKinematics() {}

bool ForwardKinematics::init(urdf::ModelInterfaceSharedPtr model) {
  model_ = model;
  if (model_) {
    buildJointMap();
  }
  return (model_ != nullptr);
}

void ForwardKinematics::buildJointMap() {
  joint_indices_.clear();
  joint_names_.clear();

  int index = 0;
  for (const auto &joint_pair : model_->joints_) {
    const auto &joint = joint_pair.second;
    // Only add movable joints (revolute, prismatic)
    if (joint->type == urdf::Joint::REVOLUTE ||
        joint->type == urdf::Joint::PRISMATIC ||
        joint->type == urdf::Joint::CONTINUOUS) {
      joint_indices_[joint->name] = index;
      joint_names_.push_back(joint->name);
      index++;
    }
  }

  std::cout << "Built joint map: " << joint_names_.size() << " movable joints"
            << std::endl;
}

bool ForwardKinematics::computeLinkTransforms(
    const Eigen::VectorXd &q,
    std::map<std::string, fcl::Transform3d> &transforms) {

  if (!model_) {
    std::cerr << "FK not initialized" << std::endl;
    return false;
  }

  transforms.clear();

  // Start from root with identity transform
  fcl::Transform3d root_transform = fcl::Transform3d::Identity();

  if (model_->getRoot()) {
    computeLinkTransformsRecursive(model_->getRoot(), root_transform, q,
                                   transforms);
  }

  return true;
}

void ForwardKinematics::computeLinkTransformsRecursive(
    urdf::LinkConstSharedPtr link, const fcl::Transform3d &parent_transform,
    const Eigen::VectorXd &q,
    std::map<std::string, fcl::Transform3d> &transforms) {

  // Current link transform starts from parent
  fcl::Transform3d link_transform = parent_transform;

  // If this link has a parent joint, apply the joint transform
  if (link->parent_joint) {
    auto joint = link->parent_joint;

    // Get joint origin (fixed part)
    auto &origin = joint->parent_to_joint_origin_transform;
    fcl::Vector3d translation(origin.position.x, origin.position.y,
                              origin.position.z);

    double roll, pitch, yaw;
    origin.rotation.getRPY(roll, pitch, yaw);

    // RPY to rotation matrix (ZYX convention)
    double cr = cos(roll), sr = sin(roll);
    double cp = cos(pitch), sp = sin(pitch);
    double cy = cos(yaw), sy = sin(yaw);

    fcl::Matrix3d rotation;
    rotation << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr,
        cp * cr;

    fcl::Transform3d joint_origin_transform = fcl::Transform3d::Identity();
    joint_origin_transform.linear() = rotation;
    joint_origin_transform.translation() = translation;

    // Apply joint origin
    link_transform = parent_transform * joint_origin_transform;

    // Apply joint angle (variable part)
    if ((joint->type == urdf::Joint::REVOLUTE ||
         joint->type == urdf::Joint::CONTINUOUS ||
         joint->type == urdf::Joint::PRISMATIC) &&
        joint_indices_.count(joint->name) > 0) {

      int joint_idx = joint_indices_[joint->name];
      double joint_value = (joint_idx < q.size()) ? q[joint_idx] : 0.0;

      fcl::Transform3d joint_variable_transform = fcl::Transform3d::Identity();

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
        fcl::Vector3d translation_offset(
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

fcl::Transform3d
ForwardKinematics::getLinkTransform(const std::string &link_name,
                                    const Eigen::VectorXd &q) {

  std::map<std::string, fcl::Transform3d> transforms;
  computeLinkTransforms(q, transforms);

  auto it = transforms.find(link_name);
  if (it != transforms.end()) {
    return it->second;
  }

  return fcl::Transform3d::Identity();
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

} // namespace kinematics
