#ifndef SAFETY_LIMITER_H
#define SAFETY_LIMITER_H

#include <Eigen/Dense>
#include <vector>

namespace robot_model {
class RobotModel;
}

namespace kinematics {

/**
 * @brief Enforces safety limits on joint velocities
 *
 * Applies joint position limits, velocity limits, and acceleration limits
 * to ensure safe robot operation.
 */
class SafetyLimiter {
public:
  SafetyLimiter();
  ~SafetyLimiter();

  /**
   * @brief Initialize with robot model
   * @param model Robot model containing joint limits
   * @param num_joints Number of joints to limit (default 6)
   * @return true if successful
   */
  bool init(const robot_model::RobotModel &model, int num_joints = 6);

  /**
   * @brief Clamp velocities to respect all limits
   * @param q_current Current joint positions
   * @param q_dot_desired Desired joint velocities
   * @param dt Time step in seconds
   * @return Safe joint velocities
   */
  Eigen::VectorXd clamp(const Eigen::VectorXd &q_current,
                        const Eigen::VectorXd &q_dot_desired, double dt);

  /**
   * @brief Check if a configuration is within joint limits
   */
  bool isWithinLimits(const Eigen::VectorXd &q) const;

  /**
   * @brief Get distance to nearest joint limit
   * @return Minimum distance to any limit (negative = violated)
   */
  double getDistanceToLimits(const Eigen::VectorXd &q) const;

  // Configuration
  void setJointLimits(const Eigen::VectorXd &lower,
                      const Eigen::VectorXd &upper);
  void setVelocityLimits(const Eigen::VectorXd &max_vel);
  void setAccelerationLimits(const Eigen::VectorXd &max_acc);
  void setJointLimitMargin(double margin) { joint_limit_margin_ = margin; }

  // Default limits (can be overridden)
  static constexpr double DEFAULT_MAX_VELOCITY = 2.0;     // rad/s
  static constexpr double DEFAULT_MAX_ACCELERATION = 5.0; // rad/s²
  static constexpr double DEFAULT_JOINT_MARGIN = 0.05;    // rad from limit

private:
  /**
   * @brief Scale velocity to prevent joint limit violation
   */
  double computeJointLimitScale(const Eigen::VectorXd &q,
                                const Eigen::VectorXd &q_dot, double dt) const;

  /**
   * @brief Apply velocity limits (element-wise clamping)
   */
  Eigen::VectorXd applyVelocityLimits(const Eigen::VectorXd &q_dot) const;

  /**
   * @brief Apply acceleration limits (rate limiting)
   */
  Eigen::VectorXd applyAccelerationLimits(const Eigen::VectorXd &q_dot,
                                          double dt);

  int num_joints_;
  bool initialized_ = false;

  // Limits
  Eigen::VectorXd lower_limits_;
  Eigen::VectorXd upper_limits_;
  Eigen::VectorXd max_velocities_;
  Eigen::VectorXd max_accelerations_;
  double joint_limit_margin_ = DEFAULT_JOINT_MARGIN;

  // State for acceleration limiting
  Eigen::VectorXd last_q_dot_;
  bool has_last_velocity_ = false;
};

} // namespace kinematics

#endif // SAFETY_LIMITER_H
