#include "collision_checker/CollisionChecker.h"
#include <iostream>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

namespace collision_checker {

CollisionChecker::CollisionChecker() : model_(nullptr) {}

CollisionChecker::~CollisionChecker() {}

bool CollisionChecker::loadURDF(const std::string &urdf_path) {
  model_ = urdf::parseURDFFile(urdf_path);

  if (!model_) {
    std::cerr << "Failed to load URDF from file: " << urdf_path << std::endl;
    return false;
  }

  // Store full URDF path
  urdf_path_ = urdf_path;

  // Store URDF directory for relative path resolution
  size_t last_slash = urdf_path.find_last_of("/\\");
  if (last_slash != std::string::npos) {
    urdf_dir_ = urdf_path.substr(0, last_slash + 1);
  } else {
    urdf_dir_ = "./";
  }

  std::cout << "Successfully loaded URDF: " << model_->getName() << std::endl;
  std::cout << "URDF directory: " << urdf_dir_ << std::endl;
  return true;
}

bool CollisionChecker::loadURDFFromString(const std::string &urdf_string) {
  model_ = urdf::parseURDF(urdf_string);

  if (!model_) {
    std::cerr << "Failed to parse URDF from string" << std::endl;
    return false;
  }

  std::cout << "Successfully loaded URDF: " << model_->getName() << std::endl;
  return true;
}

} // namespace collision_checker
