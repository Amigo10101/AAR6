#ifndef KINEMATICS_POSITION_IK_H
#define KINEMATICS_POSITION_IK_H

#include "kinematics/ForwardKinematics.h"
#include "kinematics/InverseKinematics.h"
#include "robot_model/RobotModel.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <vector>

namespace kinematics {

/**
 * @brief Closed-Loop Inverse Kinematics (CLIK) Solver with Damped Least Squares
 *
 * Implements real-time Cartesian control for velocity-interface robots.
 * Based on the control law: q̇ = J†(ẋ_des + Kp·e)
 *
 * Features:
 * - Damped Least Squares (DLS) for singularity robustness
 * - Adaptive damping based on manipulability
 * - Error saturation for stability with large initial errors
 * - Separate position/orientation gains
 */
class PositionIK {
public:
  PositionIK();
  ~PositionIK();

  /**
   * @brief Initialize the solver with robot model
   * @param model The robot kinematic model
   * @param end_effector_link Name of the end-effector link
   * @param num_joints Number of controlled joints (typically 6)
   * @param controlled_joint_indices Optional: indices of joints to control
   * @return true if initialization successful
   */
  bool init(const robot_model::RobotModel &model,
            const std::string &end_effector_link, int num_joints,
            const std::vector<int> &controlled_joint_indices = {});

  /**
   * @brief Compute joint velocities to track a Cartesian target
   *
   * Implements CLIK: q̇ = J†_λ · (ẋ_ff + Kp·e)
   *
   * @param q_current Current joint angles [rad]
   * @param target_pose Desired end-effector pose (Isometry3d)
   * @param x_dot_ff Feed-forward Cartesian velocity [m/s, rad/s]
   *                 For point-to-point: use zero vector
   *                 For straight line: (p_end - p_start) / duration
   * @param dt Time step [s] (unused in velocity control, for future use)
   * @return Joint velocities [rad/s]
   */
  Eigen::VectorXd solve(const Eigen::VectorXd &q_current,
                        const Eigen::Isometry3d &target_pose,
                        const Eigen::VectorXd &x_dot_ff, double dt);

  // --- Configuration ---

  /// Set position and orientation error gains
  void setGains(double kp_pos, double kp_rot);

  /// Set maximum joint velocity for safety clamping [rad/s]
  void setMaxJointVelocity(double max_vel);

  /// Set DLS damping parameters
  /// @param lambda_max Maximum damping at singularity
  /// @param sigma_threshold Singular value threshold to trigger damping
  void setDampingParams(double lambda_max, double sigma_threshold);

  /// Set error saturation limits (prevents velocity explosion)
  /// @param max_pos_error Maximum position error magnitude [m]
  /// @param max_rot_error Maximum rotation error magnitude [rad]
  void setErrorSaturation(double max_pos_error, double max_rot_error);

  /// Set joint position limits to prevent exceeding physical bounds
  /// @param joint_min Minimum joint angles [rad]
  /// @param joint_max Maximum joint angles [rad]
  /// @param margin Safety margin before limit [rad]
  void setJointLimits(const Eigen::VectorXd &joint_min,
                      const Eigen::VectorXd &joint_max, double margin = 0.1);

  // --- Diagnostics ---

  /// Get the manipulability measure at current configuration
  double getManipulability() const { return last_manipulability_; }

  /// Check if robot is near singularity
  bool isNearSingularity() const { return last_manipulability_ < w_threshold_; }

private:
  // Subsystems
  std::unique_ptr<ForwardKinematics> fk_;
  std::unique_ptr<InverseKinematics> ik_analytical_;

  // Configuration
  std::string end_effector_link_;
  int num_joints_ = 6;
  std::vector<int> controlled_joint_indices_;
  bool initialized_ = false;

  // CLIK Gains
  double Kp_pos_ = 5.0; // Position error gain [1/s]
  double Kp_rot_ = 4.0; // Orientation error gain [1/s]

  // Safety Limits
  double max_joint_vel_ = 1.0; // Max joint velocity [rad/s]

  // Joint Position Limits (AAr6 URDF defaults)
  // L1: [-3.142, 3.142], L2: [-0.98, 1.0], L3: [-2.0, 1.3]
  // L4: [-4.9, 0.3], L5: [-2.1, 2.1], L6: [-3.1, 3.1]
  Eigen::VectorXd joint_min_ =
      (Eigen::VectorXd(6) << -3.142, -0.98, -2.0, -4.9, -2.1, -3.1).finished();
  Eigen::VectorXd joint_max_ =
      (Eigen::VectorXd(6) << 3.142, 1.0, 1.3, 0.3, 2.1, 3.1).finished();
  double limit_margin_ = 0.1; // Start slowing down this far from limit [rad]

  // Error Saturation
  double max_pos_err_ = 0.1; // Max position error for gain [m]
  double max_rot_err_ = 0.5; // Max rotation error for gain [rad]

  // DLS Damping
  double lambda_max_ = 0.15;      // Max damping factor
  double sigma_threshold_ = 0.05; // Singular value threshold
  double w_threshold_ = 0.01;     // Manipulability threshold

  // Diagnostics
  double last_manipulability_ = 0.0;

  // Acceleration/Jerk limiting for smooth motion
  Eigen::VectorXd prev_q_dot_; // Previous velocity command
  double max_accel_ = 10.0;    // Maximum joint acceleration [rad/s²]
  bool first_solve_ = true;    // First solve flag (no previous velocity)

  // Helper methods
  Eigen::VectorXd computeCartesianError(const Eigen::Isometry3d &current,
                                        const Eigen::Isometry3d &target);
  double computeDampingFactor(const Eigen::MatrixXd &J);
};

} // namespace kinematics

#endif // KINEMATICS_POSITION_IK_H