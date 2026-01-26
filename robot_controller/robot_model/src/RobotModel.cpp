#include "robot_model/RobotModel.h"
#include <iostream>

namespace robot_model {

RobotModel::RobotModel() {}

RobotModel::~RobotModel() {}

bool RobotModel::loadURDF(const std::string &file_path) {
  model_ = urdf::parseURDFFile(file_path);
  if (!model_) {
    std::cerr << "Failed to parse URDF file: " << file_path << std::endl;
    return false;
  }

  // Build joint map once during load
  buildJointMap();

  return true;
}

urdf::ModelInterfaceSharedPtr RobotModel::getModel() const { return model_; }

void RobotModel::buildJointMap() {
  joint_indices_.clear();
  joint_names_.clear();

  std::cout << "Building joint map from tree traversal:" << std::endl;

  if (model_->getRoot()) {
    int index = 0;
    buildJointMapRecursive(model_->getRoot(), index);
  }

  std::cout << "Built joint map: " << joint_names_.size() << " movable joints"
            << std::endl;
}

void RobotModel::buildJointMapRecursive(urdf::LinkConstSharedPtr link,
                                        int &index) {
  // If this link has a parent joint, check if it's movable
  if (link->parent_joint) {
    auto joint = link->parent_joint;
    if (joint->type == urdf::Joint::REVOLUTE ||
        joint->type == urdf::Joint::PRISMATIC ||
        joint->type == urdf::Joint::CONTINUOUS) {
      joint_indices_[joint->name] = index;
      joint_names_.push_back(joint->name);
      std::cout << "  Joint[" << index << "]: " << joint->name << std::endl;
      index++;
    }
  }

  // Recursively process children
  for (const auto &child : link->child_links) {
    buildJointMapRecursive(child, index);
  }
}

void RobotModel::printInfo() const {
  if (model_) {
    std::cout << "Robot Name: " << model_->getName() << std::endl;
    std::cout << "Root Link: " << model_->getRoot()->name << std::endl;
    std::cout << "Number of joints: " << joint_names_.size() << std::endl;
  } else {
    std::cout << "No model loaded." << std::endl;
  }
}

} // namespace robot_model
