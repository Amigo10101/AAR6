#include "kinematics/VelocityIK.h"
#include "kinematics/ForwardKinematics.h"
#include "robot_model/RobotModel.h"
#include <iostream>

namespace kinematics {

VelocityIK::VelocityIK() : fk_(std::make_unique<ForwardKinematics>()) {}

VelocityIK::~VelocityIK() = default;

bool VelocityIK::init(const robot_model::RobotModel &model,
                      const std::string &end_effector_link, int num_joints) {
  num_joints_ = num_joints;
  end_effector_link_ = end_effector_link;

  // Initialize forward kinematics
  if (!fk_->init(model)) {
    std::cerr << "[VelocityIK] Failed to initialize ForwardKinematics"
              << std::endl;
    return false;
  }

  jacobian_.resize(6, num_joints_);
  jacobian_.setZero();

  initialized_ = true;
  std::cout << "[VelocityIK] Initialized with " << num_joints_ << " joints"
            << std::endl;
  return true;
}

Eigen::VectorXd VelocityIK::solve(const Eigen::VectorXd &q_current,
                                  const Eigen::VectorXd &v_cartesian) {
  if (!initialized_) {
    std::cerr << "[VelocityIK] Not initialized!" << std::endl;
    return Eigen::VectorXd::Zero(num_joints_);
  }

  if (v_cartesian.size() != 6) {
    std::cerr << "[VelocityIK] Cartesian velocity must be 6D" << std::endl;
    return Eigen::VectorXd::Zero(num_joints_);
  }

  // Compute Jacobian at current configuration
  jacobian_ = computeJacobian(q_current);

  // Compute manipulability for adaptive damping
  manipulability_ = computeManipulability(jacobian_);

  // Adaptive damping: increase damping near singularities
  double lambda = damping_factor_;
  if (manipulability_ < manipulability_threshold_) {
    // Scale damping up as we approach singularity
    double ratio = manipulability_ / manipulability_threshold_;
    lambda = damping_factor_ + (damping_max_ - damping_factor_) * (1.0 - ratio);
  }

  // Compute damped pseudoinverse
  Eigen::MatrixXd J_pinv = computeDampedPseudoinverse(jacobian_, lambda);

  // Compute joint velocities: q_dot = J_pinv * v_cartesian
  Eigen::VectorXd q_dot = J_pinv * v_cartesian;

  // Clamp joint velocities to respect motor limits
  for (int i = 0; i < num_joints_; ++i) {
    if (q_dot(i) > max_joint_velocity_) {
      q_dot(i) = max_joint_velocity_;
    } else if (q_dot(i) < -max_joint_velocity_) {
      q_dot(i) = -max_joint_velocity_;
    }
  }

  // Enforce joint position limits
  // If a joint is near its limit and would exceed it, reduce/zero velocity
  for (int i = 0; i < num_joints_ && i < (int)q_current.size(); ++i) {
    double q = q_current(i);
    double q_min = (i < joint_min_.size()) ? joint_min_(i) : -3.14159;
    double q_max = (i < joint_max_.size()) ? joint_max_(i) : 3.14159;

    // Check if approaching lower limit
    if (q <= q_min + limit_margin_ && q_dot(i) < 0) {
      double dist_to_limit = q - q_min;
      if (dist_to_limit <= 0) {
        q_dot(i) = 0; // At or past limit - stop
      } else {
        double scale = dist_to_limit / limit_margin_;
        q_dot(i) *= scale;
      }
    }

    // Check if approaching upper limit
    if (q >= q_max - limit_margin_ && q_dot(i) > 0) {
      double dist_to_limit = q_max - q;
      if (dist_to_limit <= 0) {
        q_dot(i) = 0; // At or past limit - stop
      } else {
        double scale = dist_to_limit / limit_margin_;
        q_dot(i) *= scale;
      }
    }
  }

  return q_dot;
}

Eigen::MatrixXd VelocityIK::computeJacobian(const Eigen::VectorXd &q) {
  // Use analytical Jacobian from ForwardKinematics
  Eigen::Matrix<double, 6, Eigen::Dynamic> J_ana;

  if (fk_->getJacobian(q, end_effector_link_, J_ana, num_joints_)) {
    // Analytical Jacobian succeeded
    return J_ana;
  }

  // Fallback to numerical Jacobian if analytical fails
  std::cerr
      << "[VelocityIK] Analytical Jacobian failed, using numerical fallback"
      << std::endl;
  const double delta = 1e-4;
  Eigen::MatrixXd J(6, num_joints_);

  // Get current end-effector pose
  Eigen::Isometry3d T0 = fk_->getLinkTransform(end_effector_link_, q);
  Eigen::Vector3d p0 = T0.translation();
  Eigen::Matrix3d R0 = T0.rotation();

  for (int i = 0; i < num_joints_; ++i) {
    // Perturb joint i
    Eigen::VectorXd q_plus = q;
    q_plus(i) += delta;

    Eigen::Isometry3d T_plus =
        fk_->getLinkTransform(end_effector_link_, q_plus);
    Eigen::Vector3d p_plus = T_plus.translation();
    Eigen::Matrix3d R_plus = T_plus.rotation();

    // Linear velocity component (position derivative)
    J.block<3, 1>(0, i) = (p_plus - p0) / delta;

    // Angular velocity component using skew-symmetric extraction
    Eigen::Matrix3d dR = R_plus * R0.transpose();
    Eigen::Vector3d omega;
    omega(0) = (dR(2, 1) - dR(1, 2)) / 2.0;
    omega(1) = (dR(0, 2) - dR(2, 0)) / 2.0;
    omega(2) = (dR(1, 0) - dR(0, 1)) / 2.0;

    J.block<3, 1>(3, i) = omega / delta;
  }

  return J;
}

Eigen::MatrixXd VelocityIK::computeDampedPseudoinverse(const Eigen::MatrixXd &J,
                                                       double lambda) {
  // Damped Least Squares (DLS) pseudoinverse:
  // J_pinv = J^T * (J * J^T + lambda^2 * I)^-1

  int m = J.rows(); // 6
  Eigen::MatrixXd JJt = J * J.transpose();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m, m);

  Eigen::MatrixXd JJt_damped = JJt + lambda * lambda * I;

  // Use LU decomposition for numerical stability
  Eigen::MatrixXd J_pinv = J.transpose() * JJt_damped.inverse();

  return J_pinv;
}

double VelocityIK::computeManipulability(const Eigen::MatrixXd &J) {
  // Yoshikawa manipulability: sqrt(det(J * J^T))
  Eigen::MatrixXd JJt = J * J.transpose();
  double det = JJt.determinant();

  if (det < 0) {
    return 0.0; // Shouldn't happen, but safety check
  }

  return std::sqrt(det);
}

} // namespace kinematics
