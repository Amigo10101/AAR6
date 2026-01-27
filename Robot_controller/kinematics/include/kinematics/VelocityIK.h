#ifndef VELOCITY_IK_H
#define VELOCITY_IK_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <string>

namespace robot_model {
class RobotModel;
}

namespace kinematics {

class ForwardKinematics;

/**
 * @brief Velocity-based Inverse Kinematics solver
 *
 * Computes joint velocities from end-effector Cartesian velocities
 * using the Jacobian pseudoinverse with damped least squares.
 */
class VelocityIK {
public:
  VelocityIK();
  ~VelocityIK();

  /**
   * @brief Initialize with robot model
   * @param model Robot model containing URDF and joint info
   * @param end_effector_link Name of end-effector link (default "ee_link")
   * @param num_joints Number of joints to use (default 6)
   * @return true if successful
   */
  bool init(const robot_model::RobotModel &model,
            const std::string &end_effector_link = "ee_link",
            int num_joints = 6);

  /**
   * @brief Solve for joint velocities given Cartesian velocity
   * @param q_current Current joint positions
   * @param v_cartesian Desired Cartesian velocity (6x1: linear + angular)
   * @return Joint velocities
   */
  Eigen::VectorXd solve(const Eigen::VectorXd &q_current,
                        const Eigen::VectorXd &v_cartesian);

  /**
   * @brief Set damping factor for singularity handling
   * @param lambda Damping factor (higher = more stable near singularities)
   */
  void setDampingFactor(double lambda) { damping_factor_ = lambda; }

  /**
   * @brief Set manipulability threshold for adaptive damping
   */
  void setManipulabilityThreshold(double threshold) {
    manipulability_threshold_ = threshold;
  }

  /**
   * @brief Set maximum joint velocity limit (rad/s)
   */
  void setMaxJointVelocity(double max_vel) { max_joint_velocity_ = max_vel; }

  /**
   * @brief Set joint position limits to prevent exceeding physical bounds
   */
  void setJointLimits(const Eigen::VectorXd &joint_min,
                      const Eigen::VectorXd &joint_max, double margin = 0.1) {
    joint_min_ = joint_min;
    joint_max_ = joint_max;
    limit_margin_ = margin;
  }

  /**
   * @brief Get the current Jacobian (computed in last solve)
   */
  const Eigen::MatrixXd &getJacobian() const { return jacobian_; }

  /**
   * @brief Get current manipulability measure
   */
  double getManipulability() const { return manipulability_; }

private:
  /**
   * @brief Compute the geometric Jacobian at current configuration
   */
  Eigen::MatrixXd computeJacobian(const Eigen::VectorXd &q);

  /**
   * @brief Compute damped pseudoinverse of Jacobian
   */
  Eigen::MatrixXd computeDampedPseudoinverse(const Eigen::MatrixXd &J,
                                             double lambda);

  /**
   * @brief Compute Yoshikawa manipulability measure
   */
  double computeManipulability(const Eigen::MatrixXd &J);

  // Forward kinematics solver (owned)
  std::unique_ptr<ForwardKinematics> fk_;

  // Configuration
  int num_joints_ = 6;
  std::string end_effector_link_;
  bool initialized_ = false;

  // Solver parameters
  double damping_factor_ = 0.05;           // Default damping
  double damping_max_ = 0.5;               // Max damping near singularity
  double manipulability_threshold_ = 0.01; // Threshold for adaptive damping
  double max_joint_velocity_ = 1.0;        // Max joint velocity (rad/s)

  // Joint Position Limits (AAr6 URDF defaults)
  // L1: [-3.142, 3.142], L2: [-0.98, 1.0], L3: [-2.0, 1.3]
  // L4: [-4.9, 0.3], L5: [-2.1, 2.1], L6: [-3.1, 3.1]
  Eigen::VectorXd joint_min_ =
      (Eigen::VectorXd(6) << -3.142, -0.98, -2.0, -4.9, -2.1, -3.1).finished();
  Eigen::VectorXd joint_max_ =
      (Eigen::VectorXd(6) << 3.142, 1.0, 1.3, 0.3, 2.1, 3.1).finished();
  double limit_margin_ = 0.1; // Start slowing down this far from limit [rad]

  // State (updated each solve)
  Eigen::MatrixXd jacobian_;
  double manipulability_ = 0.0;
};

} // namespace kinematics

#endif // VELOCITY_IK_H
