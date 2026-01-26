#ifndef SQP_IK_H
#define SQP_IK_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OsqpEigen/OsqpEigen.h>
#include <memory>
#include <string>
#include <vector>

namespace kinematics {

class ForwardKinematics;
class InverseKinematics;
struct IKResult;

/**
 * @brief Configuration branch for 6-DOF robot (homotopy class)
 * Each choice is a binary flag, representing the 8 possible configurations
 */
struct ConfigBranch {
  enum WristMode { WRIST_NORMAL = 0, WRIST_FLIPPED = 1 };
  enum ElbowMode { ELBOW_UP = 0, ELBOW_DOWN = 1 };
  enum ArmMode { ARM_FRONT = 0, ARM_BACK = 1 };

  WristMode wrist = WRIST_NORMAL;
  ElbowMode elbow = ELBOW_UP;
  ArmMode arm = ARM_FRONT;

  bool operator==(const ConfigBranch &other) const {
    return wrist == other.wrist && elbow == other.elbow && arm == other.arm;
  }
  bool operator!=(const ConfigBranch &other) const { return !(*this == other); }
};

} // namespace kinematics

namespace robot_model {
class RobotModel;
} // namespace robot_model

namespace kinematics {

/**
 * @brief Velocity-resolved QP IK solver with analytical seeding
 *
 * Industrial-grade IK using:
 * 1. EAIK analytical solver for global seed (avoids local minima)
 * 2. OSQP convex QP for velocity-level control
 * 3. SDLS (Selectively Damped Least Squares) for singularity robustness
 * 4. Velocity dampers for smooth joint limit handling
 *
 * Solves:
 *   min  1/2 * q̇'Pq̇ + c'q̇
 *   s.t. q̇_min <= q̇ <= q̇_max
 *
 * where P = J'WJ + λ²I and c = -J'W * v_ref
 */
class SqpIK {
public:
  SqpIK();
  ~SqpIK();

  /**
   * @brief Initialize solver with robot model
   */
  bool init(const robot_model::RobotModel &model, const std::string &ee_link,
            int num_joints);

  /**
   * @brief Solve for joint velocities
   * @param q_current Current joint positions [rad]
   * @param q_dot_current Current joint velocities [rad/s] (for warm start)
   * @param target_pose Desired end-effector pose
   * @param dt Time step [s]
   * @return Joint velocities [rad/s]
   */
  Eigen::VectorXd solve(const Eigen::VectorXd &q_current,
                        const Eigen::VectorXd &q_dot_current,
                        const Eigen::Isometry3d &target_pose, double dt);

  // --- Configuration ---

  /**
   * @brief Set joint position limits
   */
  void setJointLimits(const Eigen::VectorXd &q_min,
                      const Eigen::VectorXd &q_max);

  /**
   * @brief Set velocity limits [rad/s]
   */
  void setVelocityLimits(const Eigen::VectorXd &v_max);

  /**
   * @brief Set P-gain for task-space control
   */
  void setGain(double kp);

  /**
   * @brief Set damping (Tikhonov regularization) for singularity robustness
   */
  void setDamping(double lambda);

  /**
   * @brief Set task-space weighting (6-vector: x,y,z,rx,ry,rz)
   */
  void setTaskWeights(const Eigen::VectorXd &weights);

  /**
   * @brief Set adaptive damping parameters (Nakamura-Hanafusa method)
   * @param lambda_max Maximum damping factor at singularity
   * @param w_threshold Manipulability threshold for damping activation
   */
  void setDampingParams(double lambda_max, double w_threshold);

  /**
   * @brief Set velocity damper gain for joint limit avoidance
   * @param gain Higher = faster deceleration near limits (default 10.0)
   */
  void setVelocityDamperGain(double gain);

  /**
   * @brief Set hysteresis threshold for branch switching
   * @param threshold Cost difference required to trigger a branch switch
   */
  void setHysteresisThreshold(double threshold);

  // --- Diagnostics ---

  bool wasLastSolveFeasible() const { return last_solve_feasible_; }
  int getLastIterations() const { return last_iterations_; }
  double getLastManipulability() const { return last_manipulability_; }
  double getLastAdaptiveLambda() const { return last_adaptive_lambda_; }
  bool hasAnalyticalSeed() const { return has_analytical_seed_; }
  ConfigBranch getCurrentBranch() const { return current_branch_; }
  bool didBranchSwitch() const { return branch_switched_; }

private:
  // OSQP solver
  OsqpEigen::Solver solver_;
  bool solver_initialized_ = false;

  // Forward kinematics
  std::unique_ptr<ForwardKinematics> fk_;
  std::string end_effector_link_;
  int num_joints_ = 6;

  // Analytical IK for seeding (avoids local minima)
  std::unique_ptr<InverseKinematics> eaik_;
  Eigen::VectorXd analytical_seed_;
  bool has_analytical_seed_ = false;

  // Previous velocity (for warm start)
  Eigen::VectorXd q_dot_prev_;
  bool first_solve_ = true;

  // Configuration branch tracking (homotopy class)
  ConfigBranch current_branch_;
  Eigen::VectorXd
      branch_reference_q_; // Reference configuration for current branch
  bool has_branch_ = false;
  bool branch_switched_ = false;
  double hysteresis_threshold_ = 0.5; // Cost difference to trigger switch

  // Limits
  Eigen::VectorXd q_min_, q_max_;
  Eigen::VectorXd v_max_;

  // Velocity damper gain for joint limits
  double velocity_damper_gain_ = 10.0;

  // P-gain for task-space velocity control
  double Kp_ = 5.0;
  double lambda_ = 0.01; // Base damping (fallback)

  // Adaptive damping (Nakamura-Hanafusa method)
  double lambda_max_ = 0.15;     // Maximum damping at singularity
  double w_threshold_ = 0.01;    // Manipulability threshold
  double lambda_epsilon_ = 1e-6; // Minimum damping for numerical safety

  // Task-space weights
  Eigen::VectorXd W_;

  // QP matrices
  Eigen::SparseMatrix<double> P_sparse_;
  Eigen::VectorXd q_grad_;
  Eigen::SparseMatrix<double> A_sparse_;
  Eigen::VectorXd l_eff_, u_eff_;

  // Diagnostics
  bool last_solve_feasible_ = true;
  int last_iterations_ = 0;
  double last_manipulability_ = 0.0;
  double last_adaptive_lambda_ = 0.0;

  // Warm start solution caching
  Eigen::VectorXd prev_primal_;
  Eigen::VectorXd prev_dual_;

  // Helper methods
  Eigen::MatrixXd computeJacobian(const Eigen::VectorXd &q);
  Eigen::VectorXd computePoseError(const Eigen::Isometry3d &target,
                                   const Eigen::Isometry3d &current);

  // Adaptive damping helpers
  double computeManipulability(const Eigen::MatrixXd &J);
  double computeAdaptiveDamping(double manipulability);

  // SDLS (Selectively Damped Least Squares) - per-mode damping
  Eigen::MatrixXd computeSDLSPseudoinverse(const Eigen::MatrixXd &J,
                                           double max_step);

  // Branch tracking helpers
  ConfigBranch detectBranch(const Eigen::VectorXd &q);
  double computeBranchCost(const Eigen::VectorXd &q,
                           const Eigen::VectorXd &q_current);
};

} // namespace kinematics

#endif // SQP_IK_H
