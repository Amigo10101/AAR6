#include "kinematics/InverseKinematics.h"
#include "robot_model/RobotModel.h"
#include <Eigen/SVD>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

namespace kinematics {

InverseKinematics::InverseKinematics()
    : joint_indices_(nullptr), joint_names_(nullptr), num_joints_(0),
      num_ik_joints_(6), max_iterations_(200), position_tolerance_(1e-4),
      orientation_tolerance_(1e-3), damping_factor_(0.05) {}

InverseKinematics::~InverseKinematics() {}

bool InverseKinematics::init(const robot_model::RobotModel &robot_model,
                             const std::string &end_effector_link,
                             int num_ik_joints) {
  model_ = robot_model.getModel();
  if (!model_) {
    std::cerr << "[IK] Error: RobotModel has no URDF loaded" << std::endl;
    return false;
  }

  end_effector_link_ = end_effector_link;

  // Verify end-effector link exists
  auto ee_link = model_->getLink(end_effector_link_);
  if (!ee_link) {
    std::cerr << "[IK] Error: End-effector link '" << end_effector_link_
              << "' not found in URDF" << std::endl;
    return false;
  }

  // Initialize forward kinematics with RobotModel
  if (!fk_.init(robot_model)) {
    std::cerr << "[IK] Error: Failed to initialize FK" << std::endl;
    return false;
  }

  // Use RobotModel's pre-computed joint map (no duplication!)
  joint_indices_ = &robot_model.getJointIndices();
  joint_names_ = &robot_model.getJointNames();
  num_joints_ = robot_model.getNumJoints();

  // Set number of joints to use for IK (e.g., 6 for arm, excluding gripper)
  num_ik_joints_ = std::min(num_ik_joints, num_joints_);

  // Extract joint limits from URDF
  joint_lower_limits_.resize(num_ik_joints_);
  joint_upper_limits_.resize(num_ik_joints_);
  for (int i = 0; i < num_ik_joints_; ++i) {
    const std::string &joint_name = (*joint_names_)[i];
    auto joint = model_->getJoint(joint_name);
    if (joint && joint->limits) {
      joint_lower_limits_[i] = joint->limits->lower;
      joint_upper_limits_[i] = joint->limits->upper;
    } else {
      // Default to ±π for joints without explicit limits
      joint_lower_limits_[i] = -M_PI;
      joint_upper_limits_[i] = M_PI;
    }
  }

  std::cout << "[IK] Initialized with " << num_ik_joints_
            << " IK joints (out of " << num_joints_ << " total)" << std::endl;
  std::cout << "[IK] End-effector link: " << end_effector_link_ << std::endl;

  // Initialize EAIK
  if (initEAIK()) {
    std::cout << "[IK] EAIK solver initialized successfully" << std::endl;
  } else {
    std::cerr << "[IK] Failed to initialize EAIK. IK functionality will be "
                 "unavailable."
              << std::endl;
    return false;
  }

  return true;
}

bool InverseKinematics::initEAIK() {
  if (num_ik_joints_ != 6) {
    std::cerr << "[IK-EAIK] Only 6 DOF robots are currently supported by this "
                 "auto-init."
              << std::endl;
    return false;
  }

  // 1. Compute Zero-Configuration Kinematics
  // We need the global positions and axes of all joints when q = 0
  Eigen::VectorXd q_zero = Eigen::VectorXd::Zero(num_joints_);
  std::map<std::string, Eigen::Isometry3d> link_transforms;
  fk_.computeLinkTransforms(q_zero, link_transforms);

  // EAIK requires:
  // H: 3x6 matrix of joint axes (Z axes in world frame)
  // P: 3x7 matrix of vectors between reference points
  //    P.col(0) = P_axis1 - P_base
  //    P.col(i) = P_axis(i+1) - P_axis(i)
  //    P.col(6) = P_ee - P_axis6

  Eigen::Matrix<double, 3, 6> H;
  Eigen::Matrix<double, 3, 7> P;

  std::vector<Eigen::Vector3d> joint_positions;
  joint_positions.reserve(num_ik_joints_);

  // Get Base Position (World Origin)
  // Assuming the robot base component is at world 0,0,0 if not specified
  // otherwise. BUT the first joint is relative to base link. P_base = (0,0,0)
  // is implicit startup point.
  Eigen::Vector3d p_prev = Eigen::Vector3d::Zero();

  // Iterate over IK joints to find their global axes and positions
  for (int i = 0; i < num_ik_joints_; ++i) {
    const std::string &joint_name = (*joint_names_)[i];
    auto joint = model_->getJoint(joint_name);

    // Get parent link transform
    auto parent_it = link_transforms.find(joint->parent_link_name);
    if (parent_it == link_transforms.end())
      return false;
    Eigen::Isometry3d T_parent = parent_it->second;

    // Apply joint origin to get the frame where the axis is defined
    auto &origin = joint->parent_to_joint_origin_transform;
    Eigen::Isometry3d T_offset = Eigen::Isometry3d::Identity();
    T_offset.translation() = Eigen::Vector3d(
        origin.position.x, origin.position.y, origin.position.z);
    double r, p, y;
    origin.rotation.getRPY(r, p, y);
    T_offset.linear() = (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
                            .toRotationMatrix();

    Eigen::Isometry3d T_joint =
        T_parent * T_offset; // Global pose of the joint frame

    // 1. Extract Axis (Z-axis of the joint frame)
    Eigen::Vector3d axis = joint->axis.x * T_joint.linear().col(0) +
                           joint->axis.y * T_joint.linear().col(1) +
                           joint->axis.z * T_joint.linear().col(2);
    axis.normalize();
    H.col(i) = axis;

    // 2. Extract Position
    Eigen::Vector3d p_curr = T_joint.translation();

    // 3. Compute P column
    P.col(i) = p_curr - p_prev;

    // Update for next iteration
    p_prev = p_curr;
  }

  // Handle End Effector
  // We need the pointer from the last joint (axis 6) to the end effector
  auto ee_it = link_transforms.find(end_effector_link_);
  if (ee_it == link_transforms.end())
    return false;
  Eigen::Vector3d p_ee = ee_it->second.translation();

  P.col(6) = p_ee - p_prev;

  std::cout << "[IK-EAIK] Raw H Matrix:\n" << H << std::endl;
  std::cout << "[IK-EAIK] Raw P Matrix:\n" << P << std::endl;

  // Snap values to remove CAD/URDF noise
  auto snap = [](double val) {
    if (std::abs(val) < 1e-3)
      return 0.0;
    if (std::abs(val - 1.0) < 1e-3)
      return 1.0;
    if (std::abs(val + 1.0) < 1e-3)
      return -1.0;
    return val;
  };

  for (int c = 0; c < H.cols(); ++c) {
    for (int r = 0; r < H.rows(); ++r)
      H(r, c) = snap(H(r, c));
    H.col(c).normalize(); // Re-normalize after snapping
  }
  for (int c = 0; c < P.cols(); ++c) {
    for (int r = 0; r < P.rows(); ++r)
      P(r, c) = snap(P(r, c));
  }

  std::cout << "[IK-EAIK] Snapped H Matrix:\n" << H << std::endl;
  std::cout << "[IK-EAIK] Snapped P Matrix:\n" << P << std::endl;

  try {
    // First, compute URDF FK at zero config to get the target end-effector
    // rotation
    Eigen::VectorXd q_zero_eigen = Eigen::VectorXd::Zero(num_joints_);
    Eigen::Isometry3d urdf_fk = getEndEffectorTransform(q_zero_eigen);
    Eigen::Matrix3d urdf_R = urdf_fk.rotation();

    // Temporarily create EAIK to get its FK rotation at zero config
    auto temp_eaik = std::make_unique<EAIK::Robot>(H, P);
    std::vector<double> q_zero_vec(num_ik_joints_, 0.0);
    IKS::Homogeneous_T eaik_fk = temp_eaik->fwdkin(q_zero_vec);
    Eigen::Matrix3d eaik_R = eaik_fk.block<3, 3>(0, 0);

    // Compute R6T correction: EAIK_FK * R6T = URDF_FK, so R6T = EAIK_R^T *
    // URDF_R But EAIK applies R6T as: result_R = raw_R * R6T So we need: raw_R
    // * R6T = URDF_R -> R6T = raw_R^T * URDF_R
    Eigen::Matrix3d R6T = eaik_R.transpose() * urdf_R;

    std::cout << "[IK-EAIK] Computed R6T correction matrix:\n"
              << R6T << std::endl;
    Eigen::Quaterniond q_r6t(R6T);
    std::cout << "[IK-EAIK] R6T as quaternion (w,x,y,z): " << q_r6t.w() << ", "
              << q_r6t.x() << ", " << q_r6t.y() << ", " << q_r6t.z()
              << std::endl;

    // Recreate EAIK with R6T correction
    eaik_robot_ = std::make_unique<EAIK::Robot>(H, P, R6T);

    // Check for solvability
    if (!eaik_robot_->has_known_decomposition()) {
      std::cout << "[IK-EAIK] Warning: No known decomposition for this robot "
                   "geometry. EAIK will be disabled."
                << std::endl;
      return false;
    }

    // Verify the correction worked
    IKS::Homogeneous_T eaik_fk_corrected = eaik_robot_->fwdkin(q_zero_vec);
    Eigen::Quaterniond q_eaik_corr(eaik_fk_corrected.block<3, 3>(0, 0));
    Eigen::Quaterniond q_urdf(urdf_R);
    std::cout << "[IK-EAIK VERIFY] EAIK FK (corrected) quat: "
              << q_eaik_corr.w() << ", " << q_eaik_corr.x() << ", "
              << q_eaik_corr.y() << ", " << q_eaik_corr.z() << std::endl;
    std::cout << "[IK-EAIK VERIFY] URDF FK quat: " << q_urdf.w() << ", "
              << q_urdf.x() << ", " << q_urdf.y() << ", " << q_urdf.z()
              << std::endl;

  } catch (const std::exception &e) {
    std::cerr << "[IK-EAIK] Exception during initialization: " << e.what()
              << std::endl;
    return false;
  }

  return true;
}

// Helper to compute shortest angular distance considering wraparound
static double angularDistance(double a, double b) {
  double diff = std::fmod(a - b + M_PI, 2.0 * M_PI);
  if (diff < 0)
    diff += 2.0 * M_PI;
  return diff - M_PI;
}

// Helper to compute weighted angular distance between two configurations
// Uses J4+J6 coupling awareness for wrist singularity handling
static double jointConfigDistance(const Eigen::VectorXd &q1,
                                  const Eigen::VectorXd &q2,
                                  double singularity_blend_threshold = 0.26) {
  double dist = 0.0;
  // Check for wrist singularity: when |sin(q5)| is small, J4 and J6 are coupled
  // Indices: J4=3, J5=4, J6=5
  double sin_q5 = (q1.size() > 4) ? std::abs(std::sin(q1(4))) : 1.0;
  bool in_blend_zone = sin_q5 < singularity_blend_threshold;

  // Compute blend factor: 0 = full singularity, 1 = normal
  double blend = 1.0;
  if (in_blend_zone) {
    blend = sin_q5 / singularity_blend_threshold; // Linear ramp
  }

  for (int i = 0; i < static_cast<int>(q1.size()); ++i) {
    double d = angularDistance(q1(i), q2(i));

    // Downweight J4 and J6 near singularity (they become redundant)
    if (in_blend_zone && (i == 3 || i == 5)) {
      d *= blend; // Smooth reduction as we approach singularity
    }

    dist += d * d;
  }

  // Add J4+J6 coupling penalty when near singularity
  // In singularity, orientation around Z depends on (q4 + q6), not individually
  if (in_blend_zone && q1.size() > 5 && q2.size() > 5) {
    double q4_plus_q6_curr = q1(3) + q1(5);
    double q4_plus_q6_prev = q2(3) + q2(5);
    double coupling_diff = angularDistance(q4_plus_q6_curr, q4_plus_q6_prev);
    // Penalize coupling difference, weighted by how deep in singularity
    double coupling_weight = 5.0 * (1.0 - blend); // Max 5.0 at singularity
    dist += coupling_weight * coupling_diff * coupling_diff;
  }

  return dist;
}

// ============================================================================
// SINGULARITY DETECTION METHODS
// ============================================================================

double InverseKinematics::computeManipulability(const Eigen::VectorXd &q) {
  // FAST APPROXIMATION: Instead of computing full Jacobian determinant,
  // use product of sin factors that indicate distance from singularities
  // This is O(n) instead of O(n³) for Jacobian computation

  if (q.size() < 6)
    return 1.0;

  double manip = 1.0;

  // Wrist singularity: |sin(q5)| → 0 means singularity
  double sin_q5 = std::abs(std::sin(q[4]));
  manip *= (sin_q5 + 0.01); // Add small epsilon to avoid zero

  // Elbow singularity: |sin(q2 + q3)| → 0 means extended/folded
  double elbow_factor = std::abs(std::sin(q[1] + q[2]));
  manip *= (elbow_factor + 0.01);

  // Shoulder singularity: |sin(q2)| → 0 means vertical arm
  double shoulder_factor = std::abs(std::sin(q[1]));
  manip *= (shoulder_factor + 0.01);

  return manip;
}

bool InverseKinematics::isElbowSingular(double q2, double q3) const {
  // Elbow singularity occurs when arm is fully extended (q2+q3 ≈ π)
  // or fully folded (q2+q3 ≈ 0)
  double elbow_sum = std::abs(q2 + q3);

  // Check if near 0 (folded) or near π (extended)
  bool near_folded = elbow_sum < elbow_singularity_threshold_;
  bool near_extended =
      std::abs(elbow_sum - M_PI) < elbow_singularity_threshold_;

  return near_folded || near_extended;
}

static double angularInterpolate(double a, double b, double t) {
  double diff = angularDistance(b, a);
  return std::fmod(a + t * diff + 3.0 * M_PI, 2.0 * M_PI) - M_PI;
}

bool InverseKinematics::isShoulderSingular(const Eigen::VectorXd &q) const {
  // Shoulder singularity occurs when wrist center lies on J1 axis
  // For a typical 6R robot, this happens when the arm is configured such that
  // the wrist center (between J3 and J4) is directly above/below the base
  //
  // Geometric approximation: when J1 rotation doesn't affect wrist position
  // This typically occurs when the arm is pointing straight up or in specific
  // folded configurations. We can detect this by checking if the effective
  // reach in the X-Y plane is very small.
  //
  // Simple heuristic: if sin(q2) is very small AND the arm is near vertical
  if (q.size() < 3)
    return false;

  double q2 = q[1];
  double q3 = q[2];

  // When q2 ≈ 0 or q2 ≈ ±π, the arm is near vertical (shoulder singularity
  // region)
  double sin_q2 = std::abs(std::sin(q2));

  // Also check for the case where arm folds back on itself
  double arm_config = std::abs(q2 + q3);
  bool arm_vertical = sin_q2 < shoulder_singularity_threshold_ * 2.0;
  bool arm_folded_vertical = arm_config < 0.1 && sin_q2 < 0.2;

  return arm_vertical || arm_folded_vertical;
}

IKResult InverseKinematics::solve(const Eigen::Isometry3d &target_pose,
                                  const Eigen::VectorXd &initial_guess) {
  IKResult result;

  if (num_ik_joints_ == 0 || !eaik_robot_) {
    result.success = false;
    result.error_message = "IK solver not initialized";
    result.solve_time_ms = 0.0;
    return result;
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  // Convert target pose to EAIK Homogeneous_T (Eigen::Matrix4d)
  IKS::Homogeneous_T target_mat = target_pose.matrix();

  try {
    // Calculate ALL solutions
    IKS::IK_Solution solutions = eaik_robot_->calculate_IK(target_mat);

    if (solutions.Q.empty()) {
      result.success = false;
      result.error_message = "No IK solution found";
    } else {
      // Find the best solution
      int best_idx = -1;

      // -----------------------------------------------------------------------
      // INDUSTRIAL-GRADE SOLUTION SELECTION WITH SMOOTH SINGULARITY BLENDING
      // -----------------------------------------------------------------------
      // Use configurable thresholds from member variables
      const double LOCK_THRESHOLD = wrist_singularity_lock_; // Hard lock zone
      const double BLEND_THRESHOLD =
          wrist_singularity_blend_; // Soft blend zone
      const double WRIST_FLIP_PENALTY = 10.0;
      const double HUGE_PENALTY = 1000.0;

      // 1. Initial Guess Selection (Hysteresis)
      Eigen::VectorXd best_guess_q = initial_guess; // Fallback
      if (has_last_solution_) {
        best_guess_q = last_solution_;
      } else if (best_guess_q.size() != num_ik_joints_) {
        best_guess_q = Eigen::VectorXd::Zero(num_ik_joints_);
      }

      // Track previous J4+J6 sum for coupling
      double prev_q4_plus_q6 = last_q4_plus_q6_;
      if (has_last_solution_ && best_guess_q.size() > 5) {
        prev_q4_plus_q6 = best_guess_q[3] + best_guess_q[5];
      }

      // Identify "Previous" Wrist Mode from Best Guess
      enum WristMode { NORMAL, FLIPPED };
      WristMode prev_mode = WristMode::NORMAL;
      if (best_guess_q.size() == num_ik_joints_) {
        prev_mode = (std::sin(best_guess_q[4]) >= 0) ? WristMode::NORMAL
                                                     : WristMode::FLIPPED;
      }

      std::vector<std::pair<int, double>> valid_solutions;
      std::vector<std::pair<int, double>> all_candidates;

      for (size_t i = 0; i < solutions.Q.size(); ++i) {
        Eigen::VectorXd q_sol = Eigen::Map<const Eigen::VectorXd>(
            solutions.Q[i].data(), solutions.Q[i].size());

        // Check limits and normalize angles
        bool limits_ok = true;
        for (int j = 0; j < num_ik_joints_; ++j) {
          double val = q_sol[j];
          if (val < joint_lower_limits_[j]) {
            while (val < joint_lower_limits_[j])
              val += 2 * M_PI;
            if (val > joint_upper_limits_[j])
              val -= 2 * M_PI;
          } else if (val > joint_upper_limits_[j]) {
            while (val > joint_upper_limits_[j])
              val -= 2 * M_PI;
            if (val < joint_lower_limits_[j])
              val += 2 * M_PI;
          }

          if (val < joint_lower_limits_[j] || val > joint_upper_limits_[j]) {
            limits_ok = false;
          } else {
            solutions.Q[i][j] = val; // Write back normalized
            q_sol[j] = val;
          }
        }

        // --- Industrial Logic with Smooth Blending ---
        // Calculate distance using wrist-aware metric with J4+J6 coupling
        double dist = jointConfigDistance(q_sol, best_guess_q, BLEND_THRESHOLD);

        // Wrist specific checks
        double q5 = q_sol[4];
        double sin_q5 = std::abs(std::sin(q5));
        bool in_blend_zone = sin_q5 < BLEND_THRESHOLD;
        bool in_lock_zone = sin_q5 < LOCK_THRESHOLD;

        // Compute blend factor for smooth transitions
        double blend_factor = 1.0; // 1.0 = normal, 0.0 = full singularity
        if (in_blend_zone) {
          if (in_lock_zone) {
            blend_factor = 0.0;
          } else {
            // Linear interpolation between LOCK and BLEND thresholds
            blend_factor =
                (sin_q5 - LOCK_THRESHOLD) / (BLEND_THRESHOLD - LOCK_THRESHOLD);
          }
        }

        // 1. Penalize large wrist motion (flips) with blend-weighted factor
        if (!in_lock_zone) {
          double d4 = angularDistance(q_sol[3], best_guess_q[3]);
          double d5 = angularDistance(q_sol[4], best_guess_q[4]);
          double d6 = angularDistance(q_sol[5], best_guess_q[5]);
          double wrist_motion = d4 * d4 + d5 * d5 + d6 * d6;
          // Scale penalty by blend factor (less penalty near singularity)
          dist += WRIST_FLIP_PENALTY * blend_factor * wrist_motion;
        }

        // 2. Penalize Mode Change with blend factor
        WristMode current_mode =
            (std::sin(q5) >= 0) ? WristMode::NORMAL : WristMode::FLIPPED;
        if (current_mode != prev_mode && !in_lock_zone) {
          dist += HUGE_PENALTY * blend_factor;
        }

        // 3. Add J4+J6 coupling penalty when in blend zone
        if (in_blend_zone && has_last_solution_) {
          double curr_q4_plus_q6 = q_sol[3] + q_sol[5];
          double coupling_diff =
              angularDistance(curr_q4_plus_q6, prev_q4_plus_q6);
          // Stronger penalty deeper in singularity
          double coupling_penalty =
              8.0 * (1.0 - blend_factor) * coupling_diff * coupling_diff;
          dist += coupling_penalty;
        }

        // 4. Elbow singularity penalty
        // Penalize solutions where arm is fully extended or folded
        if (q_sol.size() >= 3) {
          if (isElbowSingular(q_sol[1], q_sol[2])) {
            // Add penalty based on how close to elbow singularity
            double elbow_sum = std::abs(q_sol[1] + q_sol[2]);
            double elbow_dist_from_folded = elbow_sum;
            double elbow_dist_from_extended = std::abs(elbow_sum - M_PI);
            double elbow_proximity =
                std::min(elbow_dist_from_folded, elbow_dist_from_extended);
            double elbow_penalty = 50.0 / (elbow_proximity + 0.01);
            dist += elbow_penalty;
          }
        }

        // 5. Shoulder singularity penalty
        // Penalize solutions where wrist center is on J1 axis
        if (isShoulderSingular(q_sol)) {
          dist += 100.0; // Strong penalty for shoulder singularity
        }

        // 6. Manipulability-based ranking
        // Lower manipulability = closer to ANY singularity = higher penalty
        double manip = computeManipulability(q_sol);
        double manip_penalty = manipulability_weight_ / (manip + 0.001);
        dist += manip_penalty;

        // 7. Velocity-aware solution selection
        // Penalize solutions that would require high joint velocities
        if (has_last_solution_ && has_last_solve_time_) {
          auto now = std::chrono::high_resolution_clock::now();
          double dt =
              std::chrono::duration<double>(now - last_solve_time_).count();

          // Only apply velocity penalty if time delta is reasonable (0.1ms to
          // 1s)
          if (dt > 0.0001 && dt < 1.0) {
            double max_implied_velocity = 0.0;
            for (int j = 0; j < num_ik_joints_; ++j) {
              double angle_diff =
                  std::abs(angularDistance(q_sol[j], last_solution_[j]));
              double implied_vel = angle_diff / dt;
              max_implied_velocity =
                  std::max(max_implied_velocity, implied_vel);
            }

            // Penalize if implied velocity exceeds limit
            if (max_implied_velocity > max_joint_velocity_) {
              double velocity_excess =
                  max_implied_velocity / max_joint_velocity_ - 1.0;
              double velocity_penalty =
                  velocity_penalty_weight_ * velocity_excess * velocity_excess;
              dist += velocity_penalty;
            }
          }
        }

        if (limits_ok) {
          valid_solutions.push_back({(int)i, dist});
        }
        all_candidates.push_back({(int)i, dist});
      }

      // Select best
      const auto *selection_pool = &all_candidates;
      if (!valid_solutions.empty()) {
        selection_pool = &valid_solutions;
      }

      if (!selection_pool->empty()) {
        auto best_it =
            std::min_element(selection_pool->begin(), selection_pool->end(),
                             [](const std::pair<int, double> &a,
                                const std::pair<int, double> &b) {
                               return a.second < b.second;
                             });
        best_idx = best_it->first;
      }

      if (best_idx != -1) {
        result.joint_angles = Eigen::Map<const Eigen::VectorXd>(
            solutions.Q[best_idx].data(), solutions.Q[best_idx].size());

        // 4. Smooth J4/J6 Blending in Singularity Zone
        // Instead of hard locking, use smooth interpolation
        if (has_last_solution_) {
          double q5_selected = result.joint_angles[4];
          double sin_q5_selected = std::abs(std::sin(q5_selected));

          if (sin_q5_selected < BLEND_THRESHOLD) {
            double q4_free = result.joint_angles[3];
            double q6_free = result.joint_angles[5];
            double q4_locked = last_solution_[3];
            double q6_locked = last_solution_[5];

            // *** STEP 1: Check if solution is stable ***
            const double MAX_JUMP_TOLERANCE = M_PI / 3; // 60 degrees
            double q4_jump = std::abs(angularDistance(q4_free, q4_locked));
            double q6_jump = std::abs(angularDistance(q6_free, q6_locked));
            bool solution_is_stable = (q4_jump <= MAX_JUMP_TOLERANCE &&
                                       q6_jump <= MAX_JUMP_TOLERANCE);

            // *** STEP 2: Only apply singularity handling if solution is stable
            // ***
            if (solution_is_stable) {
              // Compute blend factor
              double blend_factor = 1.0;
              if (sin_q5_selected < LOCK_THRESHOLD) {
                blend_factor = 0.0; // Full lock
              } else {
                blend_factor = (sin_q5_selected - LOCK_THRESHOLD) /
                               (BLEND_THRESHOLD - LOCK_THRESHOLD);
              }

              // Apply coupling-aware blending
              double target_sum = q4_free + q6_free;
              double locked_sum = q4_locked + q6_locked;

              double sum_diff = angularDistance(target_sum, locked_sum);
              double blended_sum =
                  std::fmod(locked_sum + blend_factor * sum_diff + M_PI,
                            2.0 * M_PI) -
                  M_PI;

              double q4_blend =
                  angularInterpolate(q4_locked, q4_free, blend_factor);
              double q6_blend = blended_sum - q4_blend;
              q6_blend = std::fmod(q6_blend + M_PI, 2.0 * M_PI) - M_PI;

              result.joint_angles[3] = q4_blend;
              result.joint_angles[5] = q6_blend;

              // Joint limits check
              bool q4_valid = (q4_blend >= joint_lower_limits_[3] &&
                               q4_blend <= joint_upper_limits_[3]);
              bool q6_valid = (q6_blend >= joint_lower_limits_[5] &&
                               q6_blend <= joint_upper_limits_[5]);

              if (!q4_valid || !q6_valid) {
                // Try even distribution
                double sum_midpoint = blended_sum / 2.0;
                double q4_alt =
                    std::fmod(sum_midpoint + M_PI, 2.0 * M_PI) - M_PI;
                double q6_alt =
                    std::fmod(sum_midpoint + M_PI, 2.0 * M_PI) - M_PI;

                bool q4_alt_valid = (q4_alt >= joint_lower_limits_[3] &&
                                     q4_alt <= joint_upper_limits_[3]);
                bool q6_alt_valid = (q6_alt >= joint_lower_limits_[5] &&
                                     q6_alt <= joint_upper_limits_[5]);

                if (q4_alt_valid && q6_alt_valid) {
                  result.joint_angles[3] = q4_alt;
                  result.joint_angles[5] = q6_alt;
                }
                // If still invalid, keep the blended values (may violate limits
                // slightly)
              }
            }
            // *** CRITICAL: If solution is unstable (garbage), do NOTHING ***
            // Just use the raw IK solution as-is. Don't try to fix it.
            // The next frame will hopefully converge to a better solution.
          }
        }

        // =====================================================================
        // PHASE 5: HYBRID MODE - DLS NUMERICAL REFINEMENT
        // =====================================================================
        if (use_hybrid_mode_) {
          // Check if analytical solution is a "jump" from previous
          double weighted_dist =
              jointConfigDistance(result.joint_angles, best_guess_q);

          Eigen::VectorXd dls_seed;
          if (weighted_dist > jump_threshold_) {
            // Jump detected: seed DLS from previous position instead
            dls_seed = best_guess_q;
            // Note: We don't log here to avoid spam in high-frequency loop
          } else {
            // Normal: seed from analytical solution (already close)
            dls_seed = result.joint_angles;
          }

          // Run DLS refinement with limited iterations (fast path)
          IKResult dls_result = solveDLS(dls_seed, target_pose, 10);

          if (dls_result.success) {
            result.joint_angles = dls_result.joint_angles;
            result.solver_used = "EAIK+DLS";
            result.iterations = dls_result.iterations;
          }
          // If DLS fails, keep the analytical solution
        }

        // 6. Persistence - update history
        last_solution_ = result.joint_angles;
        has_last_solution_ = true;
        last_q4_plus_q6_ = result.joint_angles[3] + result.joint_angles[5];
        last_solve_time_ = std::chrono::high_resolution_clock::now();
        has_last_solve_time_ = true;

        result.success = true;
        if (result.solver_used.empty())
          result.solver_used = "EAIK";

        // Compute final error
        Eigen::VectorXd q_full = Eigen::VectorXd::Zero(num_joints_);
        q_full.head(num_ik_joints_) = result.joint_angles;
        Eigen::Isometry3d final_pose = getEndEffectorTransform(q_full);
        Eigen::VectorXd final_error_vec =
            computePoseError(final_pose, target_pose);
        result.final_error = final_error_vec.head<3>().norm();
      } else {
        result.success = false;
        result.error_message = "No valid solution found";
      }
    }
  } catch (const std::exception &e) {
    std::cerr << "[IK-EAIK] Error during solve: " << e.what() << std::endl;
    result.success = false;
    result.error_message = e.what();
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  result.solve_time_ms =
      std::chrono::duration<double, std::milli>(end_time - start_time).count();

  return result;
}

IKResult InverseKinematics::solve(const Eigen::Vector3d &position,
                                  const Eigen::Quaterniond &orientation,
                                  const Eigen::VectorXd &initial_guess) {
  // Build Isometry3d from position and quaternion
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = position;
  target.linear() = orientation.normalized().toRotationMatrix();
  return solve(target, initial_guess);
}

Eigen::MatrixXd InverseKinematics::computeJacobian(const Eigen::VectorXd &q) {
  // ============================================================
  // Analytical Geometric Jacobian using link transforms
  // Only computes columns for IK joints (excludes gripper)
  // ============================================================

  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, num_ik_joints_);

  // Compute all link transforms (includes joint motions)
  std::map<std::string, Eigen::Isometry3d> transforms;
  fk_.computeLinkTransforms(q, transforms);

  // Get end-effector position
  auto ee_it = transforms.find(end_effector_link_);
  if (ee_it == transforms.end()) {
    std::cerr << "[IK] Error: End-effector transform not found" << std::endl;
    return J;
  }
  Eigen::Vector3d p_ee = ee_it->second.translation();

  // For each IK joint, compute its Jacobian column
  for (int i = 0; i < num_ik_joints_; ++i) {
    const std::string &joint_name = (*joint_names_)[i];

    // Get joint from URDF
    auto joint = model_->getJoint(joint_name);
    if (!joint)
      continue;

    // Get parent link transform
    auto parent_it = transforms.find(joint->parent_link_name);
    if (parent_it == transforms.end())
      continue;
    Eigen::Isometry3d T_parent = parent_it->second;

    // Apply joint origin (fixed offset from parent to joint)
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

    // Joint frame in world coordinates (before joint variable is applied)
    Eigen::Isometry3d T_joint_frame = T_parent * T_offset;

    // Joint position
    Eigen::Vector3d p_joint = T_joint_frame.translation();

    // Joint axis in local joint frame
    Eigen::Vector3d axis_local(joint->axis.x, joint->axis.y, joint->axis.z);
    axis_local.normalize();

    // Transform axis to world frame
    Eigen::Vector3d z_i = T_joint_frame.linear() * axis_local;

    // Vector from joint to end-effector
    Eigen::Vector3d r = p_ee - p_joint;

    if (joint->type == urdf::Joint::REVOLUTE ||
        joint->type == urdf::Joint::CONTINUOUS) {
      // Revolute joint
      J.block<3, 1>(0, i) = z_i.cross(r); // Linear velocity
      J.block<3, 1>(3, i) = z_i;          // Angular velocity
    } else if (joint->type == urdf::Joint::PRISMATIC) {
      // Prismatic joint
      J.block<3, 1>(0, i) = z_i;                     // Linear velocity
      J.block<3, 1>(3, i) = Eigen::Vector3d::Zero(); // Angular velocity
    }
  }

  return J;
}

Eigen::VectorXd
InverseKinematics::computePoseError(const Eigen::Isometry3d &current,
                                    const Eigen::Isometry3d &target) {
  Eigen::VectorXd error(6);

  // Position error (simple difference)
  error.head<3>() = target.translation() - current.translation();

  // Orientation error using quaternion
  Eigen::Quaterniond q_current(current.rotation());
  Eigen::Quaterniond q_target(target.rotation());

  // Compute quaternion error: q_error = q_target * q_current.inverse()
  Eigen::Quaterniond q_error = q_target * q_current.conjugate();

  // Ensure positive scalar part for proper axis-angle extraction
  if (q_error.w() < 0) {
    q_error.coeffs() *= -1;
  }

  // Convert to axis-angle representation for orientation error
  error.tail<3>() = 2.0 * q_error.vec();

  return error;
}

Eigen::Isometry3d
InverseKinematics::getEndEffectorTransform(const Eigen::VectorXd &q) {
  return fk_.getLinkTransform(end_effector_link_, q);
}

// ============================================================================
// DAMPED LEAST SQUARES (DLS) NUMERICAL SOLVER
// ============================================================================

IKResult InverseKinematics::solveDLS(const Eigen::VectorXd &q_seed,
                                     const Eigen::Isometry3d &target_pose,
                                     int max_iter) {
  IKResult result;
  result.solver_used = "DLS";

  // Copy seed to working configuration
  Eigen::VectorXd q = q_seed;

  // Ensure correct size
  if (q.size() != num_ik_joints_) {
    q = Eigen::VectorXd::Zero(num_ik_joints_);
  }

  for (int iter = 0; iter < max_iter; ++iter) {
    // 1. Compute current FK and error
    Eigen::VectorXd q_full = Eigen::VectorXd::Zero(num_joints_);
    q_full.head(num_ik_joints_) = q;

    Eigen::Isometry3d current_pose = getEndEffectorTransform(q_full);
    Eigen::VectorXd error = computePoseError(current_pose, target_pose);

    double pos_error = error.head<3>().norm();
    double ori_error = error.tail<3>().norm();

    // Check convergence
    if (pos_error < dls_convergence_tol_ &&
        ori_error < dls_convergence_tol_ * 10) {
      result.success = true;
      result.joint_angles = q;
      result.iterations = iter;
      result.final_error = pos_error;
      return result;
    }

    // 2. Compute Jacobian
    Eigen::MatrixXd J = computeJacobian(q_full);

    // 3. SVD-based Selectively Damped Least Squares
    // J = U * Σ * V^T
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU |
                                                 Eigen::ComputeThinV);
    Eigen::VectorXd sigma = svd.singularValues();
    Eigen::VectorXd sigma_inv(sigma.size());

    // Threshold for singularity detection
    const double singularity_threshold = 0.1;

    for (int i = 0; i < sigma.size(); ++i) {
      double s = sigma(i);
      double lambda = 0.0;

      // Dynamic damping: only dampen near singularities
      if (s < singularity_threshold) {
        // Quadratic ramp from 0 to lambda_max
        double ratio = s / singularity_threshold;
        lambda = (1.0 - ratio * ratio) * dls_lambda_max_;
      }

      // DLS formula: σ / (σ² + λ²) with minimum damping for stability
      double denom = s * s + lambda * lambda + dls_lambda_min_;
      sigma_inv(i) = s / denom;
    }

    // Reconstruct damped pseudo-inverse: J_dls = V * Σ_inv * U^T
    Eigen::MatrixXd J_dls =
        svd.matrixV() * sigma_inv.asDiagonal() * svd.matrixU().transpose();

    // 4. Compute joint velocity (step)
    Eigen::VectorXd dq = J_dls * error;

    // 5. Clamp to maximum step to prevent large jumps
    double step_norm = dq.norm();
    if (step_norm > dls_max_step_) {
      dq = dq * (dls_max_step_ / step_norm);
    }

    // 6. Update joints
    q += dq;

    // 7. Enforce joint limits
    for (int j = 0; j < num_ik_joints_; ++j) {
      if (q(j) < joint_lower_limits_[j]) {
        q(j) = joint_lower_limits_[j];
      } else if (q(j) > joint_upper_limits_[j]) {
        q(j) = joint_upper_limits_[j];
      }
    }
  }

  // Did not converge within max_iter
  result.success = false;
  result.joint_angles = q;
  result.iterations = max_iter;

  // Compute final error
  Eigen::VectorXd q_full = Eigen::VectorXd::Zero(num_joints_);
  q_full.head(num_ik_joints_) = q;
  Eigen::Isometry3d final_pose = getEndEffectorTransform(q_full);
  Eigen::VectorXd final_error = computePoseError(final_pose, target_pose);
  result.final_error = final_error.head<3>().norm();

  // Consider it successful if error is small enough even without formal
  // convergence
  if (result.final_error < position_tolerance_ * 10) {
    result.success = true;
  } else {
    result.error_message = "DLS did not fully converge";
  }

  return result;
}
} // namespace kinematics
