#include "kinematics/PositionIK.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace kinematics {

PositionIK::PositionIK()
    : fk_(std::make_unique<ForwardKinematics>()),
      ik_analytical_(std::make_unique<InverseKinematics>()) {}

PositionIK::~PositionIK() = default;

bool PositionIK::init(const robot_model::RobotModel &model,
                      const std::string &end_effector_link, int num_joints,
                      const std::vector<int> &controlled_joint_indices) {
  end_effector_link_ = end_effector_link;
  num_joints_ = num_joints;

  if (!fk_->init(model)) {
    std::cerr << "[PositionIK] Failed to init FK" << std::endl;
    return false;
  }

  controlled_joint_indices_ = controlled_joint_indices;
  if (controlled_joint_indices_.empty()) {
    controlled_joint_indices_.resize(num_joints);
    for (int i = 0; i < num_joints; ++i)
      controlled_joint_indices_[i] = i;
  }

  initialized_ = true;
  std::cout << "[PositionIK] Initialized with " << num_joints_ << " joints"
            << std::endl;
  return true;
}

void PositionIK::setGains(double kp_pos, double kp_rot) {
  Kp_pos_ = kp_pos;
  Kp_rot_ = kp_rot;
}

void PositionIK::setMaxJointVelocity(double max_vel) {
  max_joint_vel_ = max_vel;
}

void PositionIK::setDampingParams(double lambda_max, double sigma_threshold) {
  lambda_max_ = lambda_max;
  sigma_threshold_ = sigma_threshold;
}

void PositionIK::setErrorSaturation(double max_pos_error,
                                    double max_rot_error) {
  max_pos_err_ = max_pos_error;
  max_rot_err_ = max_rot_error;
}

void PositionIK::setJointLimits(const Eigen::VectorXd &joint_min,
                                const Eigen::VectorXd &joint_max,
                                double margin) {
  joint_min_ = joint_min;
  joint_max_ = joint_max;
  limit_margin_ = margin;
}

// =============================================================================
// NUMERICAL JACOBIAN (copied from VelocityIK - this works!)
// =============================================================================

Eigen::MatrixXd computeNumericalJacobian(ForwardKinematics *fk,
                                         const std::string &ee_link,
                                         const Eigen::VectorXd &q,
                                         int num_joints) {
  const double delta = 1e-4;
  Eigen::MatrixXd J(6, num_joints);

  // Get current end-effector pose
  Eigen::Isometry3d T0 = fk->getLinkTransform(ee_link, q);
  Eigen::Vector3d p0 = T0.translation();
  Eigen::Matrix3d R0 = T0.rotation();

  for (int i = 0; i < num_joints; ++i) {
    // Perturb joint i
    Eigen::VectorXd q_plus = q;
    q_plus(i) += delta;

    Eigen::Isometry3d T_plus = fk->getLinkTransform(ee_link, q_plus);
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

// =============================================================================
// DAMPED PSEUDOINVERSE
// =============================================================================

Eigen::MatrixXd computeDampedPseudoinverse(const Eigen::MatrixXd &J,
                                           double lambda) {
  int m = J.rows();
  Eigen::MatrixXd JJt = J * J.transpose();
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(m, m);
  Eigen::MatrixXd JJt_damped = JJt + lambda * lambda * I;
  return J.transpose() * JJt_damped.inverse();
}

// =============================================================================
// MAIN SOLVE - Using same logic as VelocityIK but with position feedback
// =============================================================================

Eigen::VectorXd PositionIK::solve(const Eigen::VectorXd &q_current,
                                  const Eigen::Isometry3d &target_pose,
                                  const Eigen::VectorXd &x_dot_ff, double dt) {
  if (!initialized_) {
    return Eigen::VectorXd::Zero(num_joints_);
  }

  // --- STEP 1: Build full joint vector for FK ---
  int full_dof = fk_->getNumJoints();
  Eigen::VectorXd q_full = Eigen::VectorXd::Zero(full_dof);
  for (int i = 0; i < num_joints_ && i < (int)q_current.size(); ++i) {
    q_full(controlled_joint_indices_[i]) = q_current(i);
  }

  // --- STEP 2: Get current EE pose via FK ---
  Eigen::Isometry3d current_pose =
      fk_->getLinkTransform(end_effector_link_, q_full);

  // --- STEP 3: Calculate Cartesian error ---
  Eigen::Vector3d pos_error =
      target_pose.translation() - current_pose.translation();

  // Debug: Print target, current, and error
  static int debugCount = 0;
  if (debugCount++ % 30 == 0) {
    Eigen::Vector3d t = target_pose.translation();
    Eigen::Vector3d c = current_pose.translation();
    std::cerr << "[CLIK] TARGET=(" << t.x() << "," << t.y() << "," << t.z()
              << ") "
              << "CURRENT=(" << c.x() << "," << c.y() << "," << c.z() << ") "
              << "ERROR=" << pos_error.norm() << "m" << std::endl;
  }

  // Orientation error (quaternion to axis-angle)
  Eigen::Quaterniond q_cur(current_pose.rotation());
  Eigen::Quaterniond q_tar(target_pose.rotation());
  if (q_cur.dot(q_tar) < 0.0)
    q_tar.coeffs() *= -1.0;
  Eigen::Quaterniond q_err = q_tar * q_cur.conjugate();
  Eigen::AngleAxisd aa(q_err);
  Eigen::Vector3d rot_error = aa.axis() * aa.angle();

  // --- STEP 4: Saturate errors to prevent explosion ---
  double pos_err_norm = pos_error.norm();
  double rot_err_norm = rot_error.norm();

  if (pos_err_norm > max_pos_err_) {
    pos_error *= (max_pos_err_ / pos_err_norm);
  }
  if (rot_err_norm > max_rot_err_) {
    rot_error *= (max_rot_err_ / rot_err_norm);
  }

  // --- STEP 5: Build Cartesian velocity command ---
  // v_cmd = Kp * error + feedforward
  Eigen::VectorXd v_cartesian(6);
  v_cartesian.head<3>() = Kp_pos_ * pos_error + x_dot_ff.head<3>();
  v_cartesian.tail<3>() = Kp_rot_ * rot_error + x_dot_ff.tail<3>();

  // --- STEP 6: Compute Analytical Jacobian ---
  Eigen::Matrix<double, 6, Eigen::Dynamic> J_ana;
  fk_->getJacobian(q_full, end_effector_link_, J_ana, full_dof);

  // Extract only controlled joint columns
  Eigen::MatrixXd J_ctrl(6, num_joints_);
  for (int i = 0; i < num_joints_; ++i) {
    J_ctrl.col(i) = J_ana.col(controlled_joint_indices_[i]);
  }

  // --- STEP 7: Block-Partitioned Jacobian for Spherical Wrist (Section 2.2)
  // --- For spherical wrist robots, partition into position (arm) and
  // orientation (wrist) This allows separate damping for arm vs wrist
  // singularities

  // Position Jacobian: top 3 rows (linear velocity)
  Eigen::MatrixXd J_pos = J_ctrl.topRows<3>();
  // Orientation Jacobian: bottom 3 rows (angular velocity)
  Eigen::MatrixXd J_rot = J_ctrl.bottomRows<3>();

  // For spherical wrist: arm joints (0-2) primarily affect position
  // Wrist joints (3-5) primarily affect orientation
  Eigen::MatrixXd J_arm = J_pos.leftCols(3); // 3x3: arm's effect on position
  Eigen::MatrixXd J_wrist =
      J_rot.rightCols(3); // 3x3: wrist's effect on orientation

  // Compute SVD for arm (position) and wrist (orientation) separately
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_arm(J_arm);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_wrist(J_wrist);

  double sigma_min_arm = svd_arm.singularValues().minCoeff();
  double sigma_min_wrist = svd_wrist.singularValues().minCoeff();

  // Also compute overall SVD for diagnostics
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_full(J_ctrl);
  double sigma_min = svd_full.singularValues().minCoeff();

  // Compute manipulability for diagnostics
  Eigen::MatrixXd JJt = J_ctrl * J_ctrl.transpose();
  double det = JJt.determinant();
  last_manipulability_ = (det > 0) ? std::sqrt(det) : 0.0;

  // Separate damping for arm and wrist singularities
  double lambda_arm = 0.0;
  if (sigma_min_arm < sigma_threshold_) {
    double ratio = sigma_min_arm / sigma_threshold_;
    lambda_arm = lambda_max_ * std::sqrt(1.0 - ratio * ratio);
  }

  double lambda_wrist = 0.0;
  if (sigma_min_wrist < sigma_threshold_) {
    double ratio = sigma_min_wrist / sigma_threshold_;
    lambda_wrist = lambda_max_ * std::sqrt(1.0 - ratio * ratio);
  }

  // Combined damping: use max of both (conservative) or weighted average
  // Using max ensures we're safe in either singularity type
  double lambda = std::max(lambda_arm, lambda_wrist);

  // --- STEP 8: Compute damped pseudoinverse ---
  Eigen::MatrixXd J_pinv = computeDampedPseudoinverse(J_ctrl, lambda);

  // --- STEP 9: Compute primary task joint velocities ---
  Eigen::VectorXd q_dot = J_pinv * v_cartesian;

  // --- STEP 9b: Null Space Projection for Optimization (Section 4) ---
  // Formula: q̇ = J†·ẋ + (I - J†·J)·q̇_null
  // P_null = (I - J†J) projects onto null space of primary task
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(num_joints_, num_joints_);
  Eigen::MatrixXd P_null = I - J_pinv * J_ctrl;

  // Compute null space velocity for joint limit avoidance (Section 4.2.2)
  // q̇_null = -k * ∇H(q) where H is potential function near limits
  Eigen::VectorXd q_dot_null = Eigen::VectorXd::Zero(num_joints_);
  double k_null = 0.5; // Null space gain

  for (int i = 0; i < num_joints_ && i < (int)q_current.size(); ++i) {
    double q = q_current(i);
    double q_min = (i < joint_min_.size()) ? joint_min_(i) : -3.14159;
    double q_max = (i < joint_max_.size()) ? joint_max_(i) : 3.14159;
    double q_mid = (q_min + q_max) / 2.0;
    double range = q_max - q_min;

    // Gradient of potential function: pushes toward joint center
    // H = 1/range² * (q - q_mid)² → ∇H = 2/range² * (q - q_mid)
    double gradient = 2.0 / (range * range) * (q - q_mid);
    q_dot_null(i) = -k_null * gradient;
  }

  // Project null space velocity and add to primary task
  q_dot += P_null * q_dot_null;

  // --- STEP 10: Clamp joint velocities ---
  for (int i = 0; i < num_joints_; ++i) {
    q_dot(i) = std::clamp(q_dot(i), -max_joint_vel_, max_joint_vel_);
  }

  // --- STEP 11: Enforce joint position limits ---
  // If a joint is near its limit and would exceed it, reduce/zero velocity
  for (int i = 0; i < num_joints_ && i < (int)q_current.size(); ++i) {
    double q = q_current(i);
    double q_min = (i < joint_min_.size()) ? joint_min_(i) : -3.14159;
    double q_max = (i < joint_max_.size()) ? joint_max_(i) : 3.14159;

    // Check if approaching lower limit
    if (q <= q_min + limit_margin_ && q_dot(i) < 0) {
      // Scale down velocity as we approach limit
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

  // Debug output
  static int count = 0;
  if (count++ % 30 == 0) {
    std::cerr << "[PositionIK] pos_err=" << pos_err_norm
              << "m, rot_err=" << rot_err_norm
              << "rad, w=" << last_manipulability_ << std::endl;
  }

  // --- STEP 12: Acceleration limiting for smooth motion ---
  // Limit how fast velocity can change to prevent jerky motion
  if (first_solve_ || prev_q_dot_.size() != num_joints_) {
    // First iteration - initialize previous velocity
    prev_q_dot_ = q_dot;
    first_solve_ = false;
  } else {
    // Limit acceleration (velocity change per timestep)
    for (int i = 0; i < num_joints_; ++i) {
      double delta_v = q_dot(i) - prev_q_dot_(i);
      double max_delta = max_accel_ * dt; // a_max * dt = max velocity change

      if (delta_v > max_delta) {
        q_dot(i) = prev_q_dot_(i) + max_delta;
      } else if (delta_v < -max_delta) {
        q_dot(i) = prev_q_dot_(i) - max_delta;
      }
    }
    prev_q_dot_ = q_dot;
  }

  return q_dot;
}

// Placeholder methods (interface compatibility)
Eigen::VectorXd
PositionIK::computeCartesianError(const Eigen::Isometry3d &current,
                                  const Eigen::Isometry3d &target) {
  return Eigen::VectorXd::Zero(6);
}

double PositionIK::computeDampingFactor(const Eigen::MatrixXd &J) {
  return 0.1;
}

} // namespace kinematics