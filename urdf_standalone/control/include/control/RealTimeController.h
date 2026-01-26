#ifndef REAL_TIME_CONTROLLER_H
#define REAL_TIME_CONTROLLER_H

#include <Eigen/Dense>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

namespace robot_model {
class RobotModel;
}

namespace collision_checker {
class GroundTruthGenerator;
}

namespace kinematics {
class VelocityIK;
class SafetyLimiter;
} // namespace kinematics

namespace control {

/**
 * @brief Robust real-time velocity controller
 *
 * Runs in a separate thread with fault tolerance:
 * - Watchdog timer (auto-stop if no commands)
 * - Exception handling (thread keeps running)
 * - Emergency stop capability
 * - Self-collision checking
 */
class RealTimeController {
public:
  RealTimeController();
  ~RealTimeController();

  // Non-copyable
  RealTimeController(const RealTimeController &) = delete;
  RealTimeController &operator=(const RealTimeController &) = delete;

  /**
   * @brief Initialize controller
   * @param model Robot model
   * @param collision Collision checker (optional, can be nullptr)
   * @return true if successful
   */
  bool init(const robot_model::RobotModel &model,
            collision_checker::GroundTruthGenerator *collision = nullptr);

  /**
   * @brief Start the control thread
   */
  void start();

  /**
   * @brief Stop the control thread
   */
  void stop();

  // ========== Command Interface (Thread-Safe) ==========

  /**
   * @brief Set desired Cartesian velocity
   * @param v [vx, vy, vz, wx, wy, wz] in m/s and rad/s
   */
  void setCartesianVelocity(const Eigen::VectorXd &v);

  /**
   * @brief Set desired joint velocity directly
   */
  void setJointVelocity(const Eigen::VectorXd &q_dot);

  /**
   * @brief Hold current position (zero velocity)
   */
  void setHoldPosition();

  /**
   * @brief Emergency stop - immediate zero velocity
   */
  void emergencyStop();

  /**
   * @brief Clear fault and resume operation
   */
  void clearFault();

  // ========== State Interface (Thread-Safe) ==========

  Eigen::VectorXd getCurrentJointPositions() const;
  Eigen::VectorXd getCurrentJointVelocities() const;
  bool isRunning() const { return running_; }
  bool isFaulted() const { return faulted_; }
  std::string getFaultReason() const;

  // ========== Configuration ==========

  void setControlFrequency(double hz);
  void setWatchdogTimeout(double seconds);
  void setCollisionCheckEnabled(bool enable) {
    collision_check_enabled_ = enable;
  }

  /**
   * @brief Set callback for output joint velocities
   * Called at control frequency with computed velocities
   */
  using OutputCallback = std::function<void(const Eigen::VectorXd &q_dot)>;
  void setOutputCallback(OutputCallback cb);

  /**
   * @brief Set callback for receiving current joint positions
   * Required for closed-loop control
   */
  using StateCallback = std::function<Eigen::VectorXd()>;
  void setStateCallback(StateCallback cb);

private:
  void controlLoop();
  bool watchdogExpired() const;
  void setFault(const std::string &reason);

  // Control mode
  enum class Mode {
    IDLE,          // No motion
    CARTESIAN_VEL, // Tracking Cartesian velocity
    JOINT_VEL,     // Direct joint velocity
    HOLD,          // Hold position (zero velocity)
    ESTOP          // Emergency stop
  };

  // Thread management
  std::thread control_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> faulted_{false};
  std::atomic<Mode> mode_{Mode::IDLE};
  mutable std::mutex state_mutex_;
  mutable std::mutex command_mutex_;

  // Fault handling
  std::string fault_reason_;
  mutable std::mutex fault_mutex_;

  // Watchdog
  std::chrono::steady_clock::time_point last_command_time_;
  std::atomic<bool> command_received_{false};
  double watchdog_timeout_ = 0.1; // 100ms

  // Control parameters
  double control_period_ = 0.01; // 100Hz
  bool collision_check_enabled_ = true;

  // Commanded values
  Eigen::VectorXd commanded_velocity_;

  // Current state
  Eigen::VectorXd q_current_;
  Eigen::VectorXd q_dot_current_;
  int num_joints_ = 6;

  // Components
  std::unique_ptr<kinematics::VelocityIK> velocity_ik_;
  std::unique_ptr<kinematics::SafetyLimiter> limiter_;
  collision_checker::GroundTruthGenerator *collision_ = nullptr;

  // Callbacks
  OutputCallback output_callback_;
  StateCallback state_callback_;
};

} // namespace control

#endif // REAL_TIME_CONTROLLER_H
