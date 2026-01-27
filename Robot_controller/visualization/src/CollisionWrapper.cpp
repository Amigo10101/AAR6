// CollisionWrapper.cpp -
#include "CollisionWrapper.h"
#include <collision_checker/GroundTruthGenerator.h>

CollisionWrapper::CollisionWrapper() = default;
CollisionWrapper::~CollisionWrapper() = default;

bool CollisionWrapper::initialize(const std::string &urdfPath) {
  checker_ = std::make_unique<collision_checker::GroundTruthGenerator>();

  if (checker_->loadURDF(urdfPath)) {
    checker_->extractCollisionGeometry();
    checker_->generateFCLCollisionObjects();
    checker_->generateAllowedCollisionPairs(3);
    initialized_ = true;
    return true;
  }

  initialized_ = false;
  return false;
}

bool CollisionWrapper::checkCollision(const std::vector<double> &jointAngles) {
  if (!initialized_ || !checker_) {
    return false;
  }

  Eigen::VectorXd q(jointAngles.size());
  for (size_t i = 0; i < jointAngles.size(); ++i) {
    q[i] = jointAngles[i];
  }

  return checker_->isSelfCollision(q);
}

std::vector<std::pair<std::string, std::string>>
CollisionWrapper::getCollidingPairs(const std::vector<double> &jointAngles) {
  if (!initialized_ || !checker_) {
    return {};
  }

  Eigen::VectorXd q(jointAngles.size());
  for (size_t i = 0; i < jointAngles.size(); ++i) {
    q[i] = jointAngles[i];
  }

  return checker_->getCollidingPairs(q);
}
