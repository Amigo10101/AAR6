#include "kinematics/SafetyLimiter.h"
#include "robot_model/RobotModel.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace kinematics {

SafetyLimiter::SafetyLimiter() : num_joints_(6), initialized_(false) {}

SafetyLimiter::~SafetyLimiter() {}

bool SafetyLimiter::init(const robot_model::RobotModel &model, int num_joints) {
  num_joints_ = num_joints;

  // Get joint limits from model
  const auto &names = model.getJointNames();
  auto urdf = model.getModel();

  lower_limits_ = Eigen::VectorXd::Zero(num_joints_);
  upper_limits_ = Eigen::VectorXd::Zero(num_joints_);
  max_velocities_ =
      Eigen::VectorXd::Constant(num_joints_, DEFAULT_MAX_VELOCITY);
  max_accelerations_ =
      Eigen::VectorXd::Constant(num_joints_, DEFAULT_MAX_ACCELERATION);
  last_q_dot_ = Eigen::VectorXd::Zero(num_joints_);

  for (int i = 0; i < num_joints_ && i < (int)names.size(); ++i) {
    auto joint = urdf->getJoint(names[i]);
    if (joint && joint->limits) {
      lower_limits_[i] = joint->limits->lower;
      upper_limits_[i] = joint->limits->upper;
      if (joint->limits->velocity > 0) {
        max_velocities_[i] = joint->limits->velocity;
      }
    } else {
      // Default limits if not specified
      lower_limits_[i] = -M_PI;
      upper_limits_[i] = M_PI;
    }
  }

  std::cout << "[SafetyLimiter] Initialized with " << num_joints_ << " joints"
            << std::endl;
  std::cout << "[SafetyLimiter] Default max velocity: " << DEFAULT_MAX_VELOCITY
            << " rad/s" << std::endl;
  std::cout << "[SafetyLimiter] Default max acceleration: "
            << DEFAULT_MAX_ACCELERATION << " rad/s²" << std::endl;

  initialized_ = true;
  return true;
}

double SafetyLimiter::computeJointLimitScale(const Eigen::VectorXd &q,
                                             const Eigen::VectorXd &q_dot,
                                             double dt) const {
  double min_scale = 1.0;

  for (int i = 0; i < num_joints_; ++i) {
    double q_next = q[i] + q_dot[i] * dt;
    double lower_with_margin = lower_limits_[i] + joint_limit_margin_;
    double upper_with_margin = upper_limits_[i] - joint_limit_margin_;

    if (q_dot[i] > 0 && q_next > upper_with_margin) {
      // Moving toward upper limit
      double max_delta = upper_with_margin - q[i];
      if (max_delta > 0) {
        double scale = max_delta / (q_dot[i] * dt);
        min_scale = std::min(min_scale, scale);
      } else {
        min_scale = 0.0; // Already at limit
      }
    } else if (q_dot[i] < 0 && q_next < lower_with_margin) {
      // Moving toward lower limit
      double max_delta = lower_with_margin - q[i];
      if (max_delta < 0) {
        double scale = max_delta / (q_dot[i] * dt);
        min_scale = std::min(min_scale, scale);
      } else {
        min_scale = 0.0; // Already at limit
      }
    }
  }

  return std::max(0.0, min_scale);
}

Eigen::VectorXd
SafetyLimiter::applyVelocityLimits(const Eigen::VectorXd &q_dot) const {
  Eigen::VectorXd clamped = q_dot;

  for (int i = 0; i < num_joints_; ++i) {
    clamped[i] =
        std::clamp(clamped[i], -max_velocities_[i], max_velocities_[i]);
  }

  return clamped;
}

Eigen::VectorXd
SafetyLimiter::applyAccelerationLimits(const Eigen::VectorXd &q_dot,
                                       double dt) {
  if (!has_last_velocity_) {
    has_last_velocity_ = true;
    last_q_dot_ = q_dot;
    return q_dot;
  }

  Eigen::VectorXd limited = q_dot;

  for (int i = 0; i < num_joints_; ++i) {
    double delta = q_dot[i] - last_q_dot_[i];
    double max_delta = max_accelerations_[i] * dt;

    // Clamp acceleration
    delta = std::clamp(delta, -max_delta, max_delta);
    limited[i] = last_q_dot_[i] + delta;
  }

  last_q_dot_ = limited;
  return limited;
}

Eigen::VectorXd SafetyLimiter::clamp(const Eigen::VectorXd &q_current,
                                     const Eigen::VectorXd &q_dot_desired,
                                     double dt) {
  if (!initialized_) {
    std::cerr << "[SafetyLimiter] Not initialized" << std::endl;
    return Eigen::VectorXd::Zero(num_joints_);
  }

  // Ensure correct size
  Eigen::VectorXd q =
      q_current.head(std::min((int)q_current.size(), num_joints_));
  Eigen::VectorXd q_dot =
      q_dot_desired.head(std::min((int)q_dot_desired.size(), num_joints_));

  // Pad if necessary
  if (q.size() < num_joints_) {
    Eigen::VectorXd padded = Eigen::VectorXd::Zero(num_joints_);
    padded.head(q.size()) = q;
    q = padded;
  }
  if (q_dot.size() < num_joints_) {
    Eigen::VectorXd padded = Eigen::VectorXd::Zero(num_joints_);
    padded.head(q_dot.size()) = q_dot;
    q_dot = padded;
  }

  // 1. Apply velocity limits
  q_dot = applyVelocityLimits(q_dot);

  // 2. Apply acceleration limits
  q_dot = applyAccelerationLimits(q_dot, dt);

  // 3. Scale to prevent joint limit violation
  double scale = computeJointLimitScale(q, q_dot, dt);
  if (scale < 1.0) {
    q_dot *= scale;
  }

  return q_dot;
}

bool SafetyLimiter::isWithinLimits(const Eigen::VectorXd &q) const {
  for (int i = 0; i < std::min((int)q.size(), num_joints_); ++i) {
    if (q[i] < lower_limits_[i] || q[i] > upper_limits_[i]) {
      return false;
    }
  }
  return true;
}

double SafetyLimiter::getDistanceToLimits(const Eigen::VectorXd &q) const {
  double min_dist = std::numeric_limits<double>::max();

  for (int i = 0; i < std::min((int)q.size(), num_joints_); ++i) {
    double lower_dist = q[i] - lower_limits_[i];
    double upper_dist = upper_limits_[i] - q[i];
    min_dist = std::min(min_dist, std::min(lower_dist, upper_dist));
  }

  return min_dist;
}

void SafetyLimiter::setJointLimits(const Eigen::VectorXd &lower,
                                   const Eigen::VectorXd &upper) {
  lower_limits_ = lower;
  upper_limits_ = upper;
}

void SafetyLimiter::setVelocityLimits(const Eigen::VectorXd &max_vel) {
  max_velocities_ = max_vel;
}

void SafetyLimiter::setAccelerationLimits(const Eigen::VectorXd &max_acc) {
  max_accelerations_ = max_acc;
}

} // namespace kinematics
