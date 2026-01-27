#include "kinematics/SqpIK.h"
#include "kinematics/ForwardKinematics.h"
#include "kinematics/InverseKinematics.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <robot_model/RobotModel.h>

namespace kinematics {

SqpIK::SqpIK() {
  // Initialize with proper joint limits from AAr6 URDF
  q_min_.resize(6);
  q_max_.resize(6);

  // Joint limits from AAr6 URDF:
  // L1: revolute, limits [-3.142, 3.142]
  // L2: revolute, limits [-0.98, 1.0]
  // L3: revolute, limits [-2.0, 1.3]
  // L4: revolute, limits [-4.9, 0.3]
  // L5: revolute, limits [-2.1, 2.1]
  // L6: continuous, limits [-3.1, 3.1]
  q_min_ << -3.142, -0.98, -2.0, -4.9, -2.1, -3.1;
  q_max_ << 3.142, 1.0, 1.3, 0.3, 2.1, 3.1;

  v_max_ =
      Eigen::VectorXd::Constant(6, 1.5); // rad/s - conservative for control

  // Default task weights (equal for all axes)
  W_ = Eigen::VectorXd::Ones(6);

  // Previous velocity
  q_dot_prev_ = Eigen::VectorXd::Zero(6);
}

SqpIK::~SqpIK() = default;

bool SqpIK::init(const robot_model::RobotModel &model,
                 const std::string &ee_link, int num_joints) {
  end_effector_link_ = ee_link;
  num_joints_ = num_joints;

  // Initialize forward kinematics
  fk_ = std::make_unique<ForwardKinematics>();
  if (!fk_->init(model)) {
    std::cerr << "[SqpIK] Failed to initialize forward kinematics" << std::endl;
    return false;
  }

  // Initialize EAIK analytical solver for seeding
  eaik_ = std::make_unique<InverseKinematics>();
  if (!eaik_->init(model, ee_link, num_joints)) {
    std::cerr << "[SqpIK] Failed to initialize EAIK analytical solver"
              << std::endl;
    return false;
  }

  // Resize state vectors
  q_dot_prev_ = Eigen::VectorXd::Zero(num_joints_);
  l_eff_ = Eigen::VectorXd::Zero(num_joints_);
  u_eff_ = Eigen::VectorXd::Zero(num_joints_);
  q_grad_ = Eigen::VectorXd::Zero(num_joints_);
  analytical_seed_ = Eigen::VectorXd::Zero(num_joints_);

  // Initialize OSQP solver
  solver_.settings()->setWarmStart(true);
  solver_.settings()->setVerbosity(false);
  solver_.settings()->setAlpha(1.6); // ADMM relaxation
  solver_.settings()->setAbsoluteTolerance(1e-4);
  solver_.settings()->setRelativeTolerance(1e-4);
  solver_.settings()->setMaxIteration(50); // Safety cap
  solver_.settings()->setAdaptiveRho(true);

  // Setup sparse matrices
  // P is num_joints x num_joints (will be dense but stored sparse)
  P_sparse_.resize(num_joints_, num_joints_);
  // Initialize P with identity structure (will be updated each solve)
  for (int i = 0; i < num_joints_; ++i) {
    for (int j = 0; j < num_joints_; ++j) {
      P_sparse_.insert(i, j) = (i == j) ? 1.0 : 0.0;
    }
  }
  P_sparse_.makeCompressed();

  // A is identity matrix (box constraints)
  A_sparse_.resize(num_joints_, num_joints_);
  A_sparse_.setIdentity();
  A_sparse_.makeCompressed();

  // Setup solver data
  solver_.data()->setNumberOfVariables(num_joints_);
  solver_.data()->setNumberOfConstraints(num_joints_);

  if (!solver_.data()->setHessianMatrix(P_sparse_)) {
    std::cerr << "[SqpIK] Failed to set Hessian matrix" << std::endl;
    return false;
  }

  if (!solver_.data()->setGradient(q_grad_)) {
    std::cerr << "[SqpIK] Failed to set gradient" << std::endl;
    return false;
  }

  if (!solver_.data()->setLinearConstraintsMatrix(A_sparse_)) {
    std::cerr << "[SqpIK] Failed to set constraint matrix" << std::endl;
    return false;
  }

  if (!solver_.data()->setLowerBound(l_eff_)) {
    std::cerr << "[SqpIK] Failed to set lower bounds" << std::endl;
    return false;
  }

  if (!solver_.data()->setUpperBound(u_eff_)) {
    std::cerr << "[SqpIK] Failed to set upper bounds" << std::endl;
    return false;
  }

  if (!solver_.initSolver()) {
    std::cerr << "[SqpIK] Failed to initialize OSQP solver" << std::endl;
    return false;
  }

  solver_initialized_ = true;
  std::cerr << "[SqpIK] Initialized with EAIK seeding + Velocity QP, "
            << num_joints_ << " joints" << std::endl;
  return true;
}

Eigen::VectorXd SqpIK::solve(const Eigen::VectorXd &q_current,
                             const Eigen::VectorXd &q_dot_current,
                             const Eigen::Isometry3d &target_pose, double dt) {

  if (!solver_initialized_) {
    return Eigen::VectorXd::Zero(num_joints_);
  }

  // Reset branch switch flag
  branch_switched_ = false;

  // ============================================================
  // STEP 0: Get analytical seed (avoids local minima)
  // This is the KEY improvement from the research document
  // ============================================================
  IKResult analytical_result = eaik_->solve(target_pose, q_current);
  if (analytical_result.success) {
    analytical_seed_ = analytical_result.joint_angles;
    has_analytical_seed_ = true;
  }

  // ============================================================
  // STEP 0.5: MOTION LOOKAHEAD for proactive configuration switching
  // Predict where motion is heading and pre-flip if needed
  // ============================================================
  ConfigBranch current_branch = detectBranch(q_current);
  ConfigBranch analytical_branch =
      has_analytical_seed_ ? detectBranch(analytical_seed_) : current_branch;

  // Initialize branch reference if first time
  if (!has_branch_) {
    current_branch_ = current_branch;
    branch_reference_q_ = q_current;
    has_branch_ = true;
  }

  // --- LOOKAHEAD: Predict future pose and check if we need to pre-flip ---
  // Get current pose for lookahead computation
  Eigen::Isometry3d current_pose_for_lookahead =
      fk_->getLinkTransform(end_effector_link_, q_current);

  // Compute pose error to target
  Eigen::Vector3d pos_error =
      target_pose.translation() - current_pose_for_lookahead.translation();
  double pos_error_norm = pos_error.norm();

  // Only do lookahead if we have significant motion remaining
  const double lookahead_distance = 0.1;        // 10cm lookahead
  const double min_motion_for_lookahead = 0.02; // 2cm minimum

  if (pos_error_norm > min_motion_for_lookahead && has_analytical_seed_) {
    // Extrapolate pose: move along error direction
    Eigen::Vector3d motion_dir = pos_error.normalized();
    Eigen::Vector3d lookahead_pos =
        current_pose_for_lookahead.translation() +
        motion_dir * std::min(lookahead_distance, pos_error_norm);

    // Build lookahead pose (keep target orientation)
    Eigen::Isometry3d lookahead_pose = Eigen::Isometry3d::Identity();
    lookahead_pose.translation() = lookahead_pos;
    lookahead_pose.linear() = target_pose.linear();

    // Get analytical solution for lookahead pose
    IKResult lookahead_ik = eaik_->solve(lookahead_pose, q_current);

    if (lookahead_ik.success) {
      ConfigBranch lookahead_branch = detectBranch(lookahead_ik.joint_angles);

      // If lookahead needs different branch AND analytical agrees, switch NOW
      if (lookahead_branch != current_branch_ &&
          lookahead_branch == analytical_branch) {
        // Proactive switch - the motion is heading towards needing a flip
        current_branch_ = lookahead_branch;
        branch_reference_q_ = analytical_seed_;
        branch_switched_ = true;
      }
    }
  }

  // Fallback: regular hysteresis-based switching if lookahead didn't trigger
  if (!branch_switched_ && has_analytical_seed_ &&
      analytical_branch != current_branch_) {
    double cost_stay = computeBranchCost(q_current, branch_reference_q_);
    double cost_switch = computeBranchCost(analytical_seed_, q_current);

    if (cost_switch < cost_stay - hysteresis_threshold_) {
      current_branch_ = analytical_branch;
      branch_reference_q_ = analytical_seed_;
      branch_switched_ = true;
    }
  }

  // Update branch if it changed continuously (without analytical trigger)
  if (current_branch != current_branch_ && !branch_switched_) {
    double motion = (q_current - branch_reference_q_).norm();
    if (motion > 0.5) {
      current_branch_ = current_branch;
      branch_reference_q_ = q_current;
    }
  }

  // --- STEP 1: Get current FK pose ---
  Eigen::Isometry3d current_pose =
      fk_->getLinkTransform(end_effector_link_, q_current);

  // --- STEP 2: Compute Jacobian ---
  Eigen::MatrixXd J = computeJacobian(q_current);

  // --- STEP 3: Compute task-space P-control (velocity-level) ---
  // v_ref = Kp * pose_error
  Eigen::VectorXd pose_error = computePoseError(target_pose, current_pose);
  Eigen::VectorXd v_ref = Kp_ * pose_error;

  // --- STEP 4: Compute SDLS per-mode damping (Buss & Kim) ---
  // This is the KEY fix for singularity behavior
  // SVD decomposes J = U * Σ * V' where:
  // - σᵢ are singular values (measure of mobility in each direction)
  // - Uᵢ are task-space directions
  // - Vᵢ are joint-space directions
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
  const Eigen::VectorXd &sigma = svd.singularValues();
  const Eigen::MatrixXd &V = svd.matrixV();

  // Compute per-mode damping matrix Λ = diag(λ₁², λ₂², ..., λₙ²)
  // Only singular directions get damped, healthy directions stay precise
  Eigen::MatrixXd Lambda = Eigen::MatrixXd::Zero(num_joints_, num_joints_);
  double sigma_threshold = 0.1; // Threshold for "needs damping"

  for (int i = 0; i < std::min(6, num_joints_); ++i) {
    double s = sigma(i);
    double lambda_i = 0.0;

    if (s < sigma_threshold) {
      // This mode is singular - apply damping
      // Quadratic ramp: more damping as s approaches 0
      double ratio = 1.0 - (s / sigma_threshold);
      lambda_i = lambda_max_ * ratio * ratio;
    }

    // Apply damping in the joint-space direction Vᵢ
    // This creates Λ = Σ λᵢ² * vᵢ * vᵢ'
    Lambda += (lambda_i * lambda_i) * V.col(i) * V.col(i).transpose();
  }

  // Store diagnostics (average damping level)
  last_manipulability_ = computeManipulability(J);
  last_adaptive_lambda_ = Lambda.diagonal().mean();

  // --- STEP 5: Build QP Hessian: P = J'WJ + Λ (SDLS) ---
  Eigen::MatrixXd W_diag = W_.asDiagonal();
  Eigen::MatrixXd P_dense = J.transpose() * W_diag * J + Lambda;

  // Ensure positive definiteness (add tiny epsilon to diagonal)
  P_dense += 1e-6 * Eigen::MatrixXd::Identity(num_joints_, num_joints_);

  // Convert to sparse
  for (int i = 0; i < num_joints_; ++i) {
    for (int j = 0; j < num_joints_; ++j) {
      P_sparse_.coeffRef(i, j) = P_dense(i, j);
    }
  }

  // --- STEP 6: Build QP gradient: c = -J'W * v_ref ---
  q_grad_ = -J.transpose() * W_diag * v_ref;

  // --- STEP 7: Compute velocity bounds with VELOCITY DAMPERS ---
  // This is the industry-standard way to handle joint limits
  for (int i = 0; i < num_joints_; ++i) {
    double q = q_current(i);

    // Distance to limits
    double d_upper = q_max_(i) - q;
    double d_lower = q - q_min_(i);

    // Velocity damper: smooth deceleration near limits
    // q_dot_max = min(v_max, gain * distance / dt)
    double vel_upper = std::min(v_max_(i), velocity_damper_gain_ * d_upper);
    double vel_lower = std::min(v_max_(i), velocity_damper_gain_ * d_lower);

    // Apply bounds
    u_eff_(i) = vel_upper;
    l_eff_(i) = -vel_lower;

    // Safety: ensure bounds are valid
    if (l_eff_(i) > u_eff_(i)) {
      double mid = (l_eff_(i) + u_eff_(i)) / 2.0;
      l_eff_(i) = mid - 0.001;
      u_eff_(i) = mid + 0.001;
    }
  }

  // --- STEP 8: Update solver and solve ---
  solver_.updateHessianMatrix(P_sparse_);
  solver_.updateGradient(q_grad_);
  solver_.updateBounds(l_eff_, u_eff_);

  auto status = solver_.solveProblem();

  if (status != OsqpEigen::ErrorExitFlag::NoError) {
    last_solve_feasible_ = false;
    std::cerr << "[SqpIK] Solve failed, returning zero velocity" << std::endl;
    return Eigen::VectorXd::Zero(num_joints_);
  }

  last_solve_feasible_ = true;
  Eigen::VectorXd q_dot = solver_.getSolution();

  // Store for warm start
  prev_primal_ = q_dot;
  q_dot_prev_ = q_dot;
  first_solve_ = false;

  return q_dot;
}

// --- Configuration Methods ---

void SqpIK::setJointLimits(const Eigen::VectorXd &q_min,
                           const Eigen::VectorXd &q_max) {
  q_min_ = q_min;
  q_max_ = q_max;
}

void SqpIK::setVelocityLimits(const Eigen::VectorXd &v_max) { v_max_ = v_max; }

void SqpIK::setGain(double kp) { Kp_ = kp; }

void SqpIK::setDamping(double lambda) { lambda_ = lambda; }

void SqpIK::setTaskWeights(const Eigen::VectorXd &weights) { W_ = weights; }

void SqpIK::setDampingParams(double lambda_max, double w_threshold) {
  lambda_max_ = lambda_max;
  w_threshold_ = w_threshold;
}

void SqpIK::setVelocityDamperGain(double gain) { velocity_damper_gain_ = gain; }

// --- Helper Methods ---

Eigen::MatrixXd SqpIK::computeJacobian(const Eigen::VectorXd &q) {
  // Use analytical Jacobian from ForwardKinematics
  Eigen::Matrix<double, 6, Eigen::Dynamic> J_ana;

  if (fk_->getJacobian(q, end_effector_link_, J_ana, num_joints_)) {
    return J_ana;
  }

  // Fallback to numerical Jacobian if analytical fails
  std::cerr << "[SqpIK] Analytical Jacobian failed, using numerical fallback"
            << std::endl;
  const double delta = 1e-4;
  Eigen::MatrixXd J(6, num_joints_);

  Eigen::Isometry3d T0 = fk_->getLinkTransform(end_effector_link_, q);

  for (int i = 0; i < num_joints_; ++i) {
    Eigen::VectorXd q_plus = q;
    q_plus(i) += delta;

    Eigen::Isometry3d T_plus =
        fk_->getLinkTransform(end_effector_link_, q_plus);

    // Linear velocity Jacobian (position difference)
    J.block<3, 1>(0, i) = (T_plus.translation() - T0.translation()) / delta;

    // Angular velocity Jacobian (rotation difference via skew-symmetric)
    Eigen::Matrix3d dR = T_plus.rotation() * T0.rotation().transpose();
    Eigen::Vector3d omega;
    omega << (dR(2, 1) - dR(1, 2)) / 2.0, (dR(0, 2) - dR(2, 0)) / 2.0,
        (dR(1, 0) - dR(0, 1)) / 2.0;
    J.block<3, 1>(3, i) = omega / delta;
  }

  return J;
}

Eigen::VectorXd SqpIK::computePoseError(const Eigen::Isometry3d &target,
                                        const Eigen::Isometry3d &current) {
  Eigen::VectorXd error(6);

  // Position error (simple difference)
  error.head<3>() = target.translation() - current.translation();

  // Orientation error (quaternion with sign check)
  Eigen::Quaterniond q_cur(current.rotation());
  Eigen::Quaterniond q_tar(target.rotation());

  // Ensure shortest path (avoid 2π rotation)
  if (q_cur.dot(q_tar) < 0.0) {
    q_tar.coeffs() *= -1.0;
  }

  Eigen::Quaterniond q_err = q_tar * q_cur.conjugate();
  Eigen::AngleAxisd aa(q_err);
  error.tail<3>() = aa.axis() * aa.angle();

  // Saturate errors to prevent velocity explosion
  double max_pos_err = 0.15; // 15cm
  double max_rot_err = 0.8;  // ~45deg

  double pos_norm = error.head<3>().norm();
  double rot_norm = error.tail<3>().norm();

  if (pos_norm > max_pos_err) {
    error.head<3>() *= (max_pos_err / pos_norm);
  }
  if (rot_norm > max_rot_err) {
    error.tail<3>() *= (max_rot_err / rot_norm);
  }

  return error;
}

double SqpIK::computeManipulability(const Eigen::MatrixXd &J) {
  // Yoshikawa manipulability measure: w = sqrt(det(J * J^T))
  Eigen::MatrixXd JJt = J * J.transpose();
  double det = JJt.determinant();

  if (det <= 0.0) {
    return 0.0;
  }

  return std::sqrt(det);
}

double SqpIK::computeAdaptiveDamping(double w) {
  // Nakamura-Hanafusa adaptive damping law
  if (w >= w_threshold_) {
    return lambda_epsilon_;
  }

  // Quadratic ramp: smooth increase as we approach singularity
  double ratio = 1.0 - (w / w_threshold_);
  return lambda_max_ * ratio * ratio;
}

Eigen::MatrixXd SqpIK::computeSDLSPseudoinverse(const Eigen::MatrixXd &J,
                                                double max_step) {
  // Selectively Damped Least Squares (Buss & Kim)
  // This damps only the singular directions, not all directions

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);

  const Eigen::VectorXd &sigma = svd.singularValues();
  const Eigen::MatrixXd &U = svd.matrixU();
  const Eigen::MatrixXd &V = svd.matrixV();

  Eigen::MatrixXd J_inv = Eigen::MatrixXd::Zero(num_joints_, 6);

  for (int i = 0; i < std::min(6, num_joints_); ++i) {
    double s = sigma(i);

    if (s < 1e-6) {
      // Fully singular mode - skip
      continue;
    }

    // Compute per-mode damping based on singular value
    double lambda_i = 0.0;
    if (s < 0.1) {
      // Need damping for this mode
      lambda_i = (1.0 - (s / 0.1)) * lambda_max_;
    }

    double damped_inv = s / (s * s + lambda_i * lambda_i);
    J_inv += damped_inv * V.col(i) * U.col(i).transpose();
  }

  return J_inv;
}

void SqpIK::setHysteresisThreshold(double threshold) {
  hysteresis_threshold_ = threshold;
}

ConfigBranch SqpIK::detectBranch(const Eigen::VectorXd &q) {
  ConfigBranch branch;

  if (q.size() < 6)
    return branch;

  // Wrist mode: determined by sign of sin(q5)
  // q5 > 0 -> normal, q5 < 0 -> flipped (simplified)
  branch.wrist = (std::sin(q(4)) >= 0) ? ConfigBranch::WRIST_NORMAL
                                       : ConfigBranch::WRIST_FLIPPED;

  // Elbow mode: determined by q2 + q3 configuration
  // Elbow up: q2 + q3 > 0, Elbow down: q2 + q3 < 0
  double elbow_sum = q(1) + q(2);
  branch.elbow =
      (elbow_sum >= 0) ? ConfigBranch::ELBOW_UP : ConfigBranch::ELBOW_DOWN;

  // Arm mode: determined by q1 position relative to target
  // Simplified: check if q1 is in front half or back half
  branch.arm =
      (std::cos(q(0)) >= 0) ? ConfigBranch::ARM_FRONT : ConfigBranch::ARM_BACK;

  return branch;
}

double SqpIK::computeBranchCost(const Eigen::VectorXd &q,
                                const Eigen::VectorXd &q_current) {
  if (q.size() != q_current.size())
    return 1e6;

  double cost = 0.0;

  // Joint distance (primary cost)
  for (int i = 0; i < q.size(); ++i) {
    double diff = q(i) - q_current(i);
    // Normalize angle difference to [-pi, pi]
    while (diff > M_PI)
      diff -= 2 * M_PI;
    while (diff < -M_PI)
      diff += 2 * M_PI;
    cost += diff * diff;
  }

  // Manipulability penalty (inverse - higher is better)
  // Using fast approximation based on q5
  double sin_q5 = std::abs(std::sin(q(4)));
  cost += 1.0 / (sin_q5 + 0.01); // Penalize near wrist singularity

  // Elbow extension penalty
  double elbow_sum = std::abs(q(1) + q(2));
  cost += 1.0 / (std::sin(elbow_sum) + 0.1);

  return cost;
}

} // namespace kinematics
