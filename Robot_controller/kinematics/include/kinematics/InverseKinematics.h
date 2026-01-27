#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "EAIK.h"
#include "kinematics/ForwardKinematics.h"

namespace robot_model {
class RobotModel; // Forward declaration
}

namespace kinematics {

/**
 * @brief Result of IK computation
 */
struct IKResult {
  bool success;
  Eigen::VectorXd joint_angles;
  int iterations;
  double final_error;
  double solve_time_ms; // Time taken to solve in milliseconds
  std::string error_message;
  std::string solver_used; // "EAIK"

  IKResult()
      : success(false), iterations(0), final_error(0.0), solve_time_ms(0.0),
        solver_used("None") {}
};

/**
 * @brief   Inverse Kinematics Solver
 *
 * Implements IK using the EAIK library (Quickest Inverse Kinematics).
 * Uses DH parameters and provides fast, numerically stable solutions.
 */
class InverseKinematics {

public:
  InverseKinematics();
  ~InverseKinematics();

  /**
   * @brief Initialize the IK solver with a RobotModel
   * @param robot_model Reference to RobotModel (contains URDF and joint map)
   * @param end_effector_link Name of the end-effector link
   * @param num_ik_joints Number of joints to use for IK (default 6 for arm
   * only)
   * @return true if successful
   */
  bool init(const robot_model::RobotModel &robot_model,
            const std::string &end_effector_link = "ee_link",
            int num_ik_joints = 6);

  /**
   * @brief Solve IK for target pose using EAIK method
   * @param target_pose Target end-effector pose as Isometry3d
   * @param initial_guess Initial joint configuration (optional)
   * @return IKResult containing solution or error info
   */
  IKResult solve(const Eigen::Isometry3d &target_pose,
                 const Eigen::VectorXd &initial_guess = Eigen::VectorXd());

  /**
   * @brief Solve IK with position and quaternion
   * @param position Target position (x, y, z) in meters
   * @param orientation Target orientation as quaternion (w, x, y, z)
   * @param initial_guess Initial joint configuration (optional)
   * @return IKResult containing solution or error info
   */
  IKResult solve(const Eigen::Vector3d &position,
                 const Eigen::Quaterniond &orientation,
                 const Eigen::VectorXd &initial_guess = Eigen::VectorXd());

  /**
   * @brief Set solver parameters
   */
  void setMaxIterations(int max_iter) { max_iterations_ = max_iter; }
  void setPositionTolerance(double tol) { position_tolerance_ = tol; }
  void setOrientationTolerance(double tol) { orientation_tolerance_ = tol; }
  void setDampingFactor(double lambda) { damping_factor_ = lambda; }

  /**
   * @brief Get number of DOF used for IK
   */
  int getNumIKJoints() const { return num_ik_joints_; }

  /**
   * @brief Get joint names in order
   */
  const std::vector<std::string> &getJointNames() const {
    return *joint_names_;
  }

private:
  /**
   * @brief Compute the geometric Jacobian (analytical)
   * @param q Current joint configuration (only first num_ik_joints_ used)
   * @return 6xN Jacobian matrix (linear velocity + angular velocity)
   */
  Eigen::MatrixXd computeJacobian(const Eigen::VectorXd &q);

  /**
   * @brief Compute pose error between current and target
   * @param current Current end-effector transform
   * @param target Target pose
   * @return 6x1 error vector (position error + orientation error)
   */
  Eigen::VectorXd computePoseError(const Eigen::Isometry3d &current,
                                   const Eigen::Isometry3d &target);

  /**
   * @brief Get current end-effector transform
   */
  Eigen::Isometry3d getEndEffectorTransform(const Eigen::VectorXd &q);

  // URDF model
  urdf::ModelInterfaceSharedPtr model_;
  std::string end_effector_link_;

  // Joint information (pointers to RobotModel's data)
  const std::map<std::string, int> *joint_indices_;
  const std::vector<std::string> *joint_names_;
  int num_joints_;    // Total joints in model
  int num_ik_joints_; // Joints used for IK (excludes gripper)

  // Solver parameters
  int max_iterations_;
  double position_tolerance_;
  double orientation_tolerance_;
  double damping_factor_;
  IKS::Homogeneous_T target_mat_;
  // Forward kinematics solver
  ForwardKinematics fk_;

  // Joint limits (from URDF)
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;

  // Industrial IK persistence
  Eigen::VectorXd last_solution_;
  bool has_last_solution_ = false;

  // J4+J6 coupling for wrist singularity handling
  double last_q4_plus_q6_ = 0.0;

  // Velocity-aware solution selection
  std::chrono::high_resolution_clock::time_point last_solve_time_;
  bool has_last_solve_time_ = false;
  double max_joint_velocity_ = 3.0; // rad/s, typical industrial robot limit
  double velocity_penalty_weight_ = 50.0; // Weight for velocity penalty

  // Singularity handling parameters (configurable)
  // WRIST_SINGULARITY_LOCK: sin(q5) below this → hard-lock J4/J6 to previous
  // WRIST_SINGULARITY_BLEND: sin(q5) between LOCK and BLEND → gradual blend
  double wrist_singularity_lock_ = 0.05;  // ~3°, force-lock J4/J6
  double wrist_singularity_blend_ = 0.15; // ~8°, start blending

public:
  /**
   * @brief Configure singularity handling thresholds
   * @param lock_threshold sin(q5) below this forces J4/J6 lock (default 0.10 ≈
   * 6°)
   * @param blend_threshold sin(q5) below this starts blending (default 0.26 ≈
   * 15°)
   */
  void setSingularityThresholds(double lock_threshold, double blend_threshold) {
    wrist_singularity_lock_ = lock_threshold;
    wrist_singularity_blend_ = blend_threshold;
  }

  /**
   * @brief Configure velocity-aware solution selection
   * @param max_velocity Maximum joint velocity in rad/s (default 3.0)
   * @param penalty_weight Weight for velocity penalty (default 50.0)
   */
  void setVelocityLimits(double max_velocity, double penalty_weight = 50.0) {
    max_joint_velocity_ = max_velocity;
    velocity_penalty_weight_ = penalty_weight;
  }

  /**
   * @brief Reset solution history (call when starting a new trajectory)
   */
  void resetSolutionHistory() {
    has_last_solution_ = false;
    has_last_solve_time_ = false;
    last_q4_plus_q6_ = 0.0;
  }

  /**
   * @brief Enable or disable hybrid mode (EAIK + DLS refinement)
   * @param enable true to enable hybrid mode (default), false for pure EAIK
   */
  void setHybridMode(bool enable) { use_hybrid_mode_ = enable; }

  /**
   * @brief Configure DLS solver parameters
   * @param lambda_max Maximum damping factor near singularities (default 0.5)
   * @param convergence_tol Cartesian error tolerance for convergence (default
   * 1e-4)
   * @param max_step Maximum joint angle step per iteration in radians (default
   * 0.1)
   * @param jump_threshold Weighted distance threshold for jump detection
   * (default 0.8)
   */
  void setDLSParameters(double lambda_max, double convergence_tol = 1e-4,
                        double max_step = 0.1, double jump_threshold = 0.8) {
    dls_lambda_max_ = lambda_max;
    dls_convergence_tol_ = convergence_tol;
    dls_max_step_ = max_step;
    jump_threshold_ = jump_threshold;
  }

private:
  // EAIK IK solver components
  std::unique_ptr<EAIK::Robot> eaik_robot_;

  /**
   * @brief Initialize EAIK solver by extracting H and P matrices
   * @return true if successful
   */
  bool initEAIK();

  /**
   * @brief Compute Yoshikawa manipulability index
   * Higher values = further from singularity, lower = near singularity
   * @param q Joint configuration
   * @return Manipulability value (sqrt of det(J*J^T))
   */
  double computeManipulability(const Eigen::VectorXd &q);

  /**
   * @brief Check if configuration is in elbow singularity
   * Elbow singularity occurs when arm is fully extended or folded
   * @param q2 Joint 2 angle
   * @param q3 Joint 3 angle
   * @return true if in elbow singularity zone
   */
  bool isElbowSingular(double q2, double q3) const;

  /**
   * @brief Check if configuration is in shoulder singularity
   * Shoulder singularity occurs when wrist center is on J1 axis
   * @param q Joint configuration
   * @return true if in shoulder singularity zone
   */
  bool isShoulderSingular(const Eigen::VectorXd &q) const;

  // Elbow singularity threshold (radians from 0 or π for q2+q3)
  double elbow_singularity_threshold_ = 0.15; // ~9°

  // Shoulder singularity threshold (meters from J1 axis)
  double shoulder_singularity_threshold_ = 0.05; // 5cm

  // Manipulability weight for solution ranking
  double manipulability_weight_ = 20.0;

  // ==========================================================================
  // HYBRID MODE: DLS (Damped Least Squares) numerical refinement
  // ==========================================================================

  // Enable hybrid mode (EAIK + DLS refinement)
  bool use_hybrid_mode_ = true;

  // DLS solver parameters
  double dls_lambda_max_ = 0.5;       // Max damping near singularities
  double dls_lambda_min_ = 0.001;     // Min damping (near-pseudoinverse)
  double dls_convergence_tol_ = 1e-4; // Cartesian error tolerance
  double dls_max_step_ = 0.1;         // Max joint step per iteration (rad)
  double jump_threshold_ = 0.8;       // Weighted distance for jump detection

  /**
   * @brief Solve IK using Damped Least Squares (numerical refinement)
   * @param q_seed Starting joint configuration (from analytical solution)
   * @param target_pose Target end-effector pose
   * @param max_iter Maximum iterations (default 20)
   * @return IKResult with refined solution
   */
  IKResult solveDLS(const Eigen::VectorXd &q_seed,
                    const Eigen::Isometry3d &target_pose, int max_iter = 20);

public:
  /**
   * @brief Configure all singularity handling thresholds
   * @param wrist_lock sin(q5) threshold for hard J4/J6 lock
   * @param wrist_blend sin(q5) threshold for blend zone start
   * @param elbow_threshold radians from 0/π for elbow singularity
   * @param shoulder_threshold meters from J1 axis for shoulder singularity
   */
  void setAllSingularityThresholds(double wrist_lock, double wrist_blend,
                                   double elbow_threshold,
                                   double shoulder_threshold) {
    wrist_singularity_lock_ = wrist_lock;
    wrist_singularity_blend_ = wrist_blend;
    elbow_singularity_threshold_ = elbow_threshold;
    shoulder_singularity_threshold_ = shoulder_threshold;
  }

  /**
   * @brief Set manipulability weight for solution ranking
   * Higher = prefer solutions further from singularities
   */
  void setManipulabilityWeight(double weight) {
    manipulability_weight_ = weight;
  }
};

} // namespace kinematics

#endif // INVERSE_KINEMATICS_H
