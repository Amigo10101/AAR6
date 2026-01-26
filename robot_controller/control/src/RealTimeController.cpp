#include "control/RealTimeController.h"
#include "collision_checker/GroundTruthGenerator.h"
#include "kinematics/SafetyLimiter.h"
#include "kinematics/VelocityIK.h"
#include "robot_model/RobotModel.h"
#include <iostream>

namespace control {

RealTimeController::RealTimeController() {
  velocity_ik_ = std::make_unique<kinematics::VelocityIK>();
  limiter_ = std::make_unique<kinematics::SafetyLimiter>();
}

RealTimeController::~RealTimeController() { stop(); }

bool RealTimeController::init(
    const robot_model::RobotModel &model,
    collision_checker::GroundTruthGenerator *collision) {
  // Initialize velocity IK
  if (!velocity_ik_->init(model)) {
    std::cerr << "[RealTimeController] Failed to init VelocityIK" << std::endl;
    return false;
  }

  // Initialize safety limiter
  if (!limiter_->init(model)) {
    std::cerr << "[RealTimeController] Failed to init SafetyLimiter"
              << std::endl;
    return false;
  }

  collision_ = collision;
  num_joints_ = 6; // TODO: get from model

  // Initialize state
  q_current_ = Eigen::VectorXd::Zero(num_joints_);
  q_dot_current_ = Eigen::VectorXd::Zero(num_joints_);
  commanded_velocity_ = Eigen::VectorXd::Zero(6);

  std::cout << "[RealTimeController] Initialized successfully" << std::endl;
  std::cout << "[RealTimeController] Control frequency: "
            << (1.0 / control_period_) << " Hz" << std::endl;
  std::cout << "[RealTimeController] Watchdog timeout: " << watchdog_timeout_
            << " s" << std::endl;
  std::cout << "[RealTimeController] Collision checking: "
            << (collision_ && collision_check_enabled_ ? "enabled" : "disabled")
            << std::endl;

  return true;
}

void RealTimeController::start() {
  if (running_)
    return;

  running_ = true;
  faulted_ = false;
  mode_ = Mode::IDLE;
  last_command_time_ = std::chrono::steady_clock::now();

  control_thread_ = std::thread(&RealTimeController::controlLoop, this);

  std::cout << "[RealTimeController] Started" << std::endl;
}

void RealTimeController::stop() {
  running_ = false;
  if (control_thread_.joinable()) {
    control_thread_.join();
  }
  std::cout << "[RealTimeController] Stopped" << std::endl;
}

void RealTimeController::setCartesianVelocity(const Eigen::VectorXd &v) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  commanded_velocity_ = v;
  mode_ = Mode::CARTESIAN_VEL;
  last_command_time_ = std::chrono::steady_clock::now();
  command_received_ = true;
}

void RealTimeController::setJointVelocity(const Eigen::VectorXd &q_dot) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  commanded_velocity_ = q_dot;
  mode_ = Mode::JOINT_VEL;
  last_command_time_ = std::chrono::steady_clock::now();
  command_received_ = true;
}

void RealTimeController::setHoldPosition() {
  std::lock_guard<std::mutex> lock(command_mutex_);
  mode_ = Mode::HOLD;
  last_command_time_ = std::chrono::steady_clock::now();
  command_received_ = true;
}

void RealTimeController::emergencyStop() {
  mode_ = Mode::ESTOP;
  faulted_ = true;
  setFault("Emergency stop activated");
}

void RealTimeController::clearFault() {
  faulted_ = false;
  {
    std::lock_guard<std::mutex> lock(fault_mutex_);
    fault_reason_.clear();
  }
  mode_ = Mode::IDLE;
}

Eigen::VectorXd RealTimeController::getCurrentJointPositions() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return q_current_;
}

Eigen::VectorXd RealTimeController::getCurrentJointVelocities() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return q_dot_current_;
}

std::string RealTimeController::getFaultReason() const {
  std::lock_guard<std::mutex> lock(fault_mutex_);
  return fault_reason_;
}

void RealTimeController::setControlFrequency(double hz) {
  control_period_ = 1.0 / hz;
}

void RealTimeController::setWatchdogTimeout(double seconds) {
  watchdog_timeout_ = seconds;
}

void RealTimeController::setOutputCallback(OutputCallback cb) {
  output_callback_ = cb;
}

void RealTimeController::setStateCallback(StateCallback cb) {
  state_callback_ = cb;
}

bool RealTimeController::watchdogExpired() const {
  auto now = std::chrono::steady_clock::now();
  double elapsed =
      std::chrono::duration<double>(now - last_command_time_).count();
  return elapsed > watchdog_timeout_;
}

void RealTimeController::setFault(const std::string &reason) {
  faulted_ = true;
  std::lock_guard<std::mutex> lock(fault_mutex_);
  fault_reason_ = reason;
  std::cerr << "[RealTimeController] FAULT: " << reason << std::endl;
}

void RealTimeController::controlLoop() {
  std::cout << "[RealTimeController] Control loop started" << std::endl;

  while (running_) {
    auto loop_start = std::chrono::steady_clock::now();

    try {
      // Get current state from callback if available
      if (state_callback_) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        q_current_ = state_callback_();
      }

      // Get current mode and commanded velocity
      Mode current_mode;
      Eigen::VectorXd cmd_vel;
      {
        std::lock_guard<std::mutex> lock(command_mutex_);
        current_mode = mode_;
        cmd_vel = commanded_velocity_;
      }

      // Watchdog check
      if (watchdogExpired() && current_mode != Mode::ESTOP) {
        if (current_mode != Mode::IDLE && current_mode != Mode::HOLD) {
          std::cout << "[RealTimeController] Watchdog expired, holding position"
                    << std::endl;
          mode_ = Mode::HOLD;
          current_mode = Mode::HOLD;
        }
      }

      // Compute joint velocities based on mode
      Eigen::VectorXd q_dot = Eigen::VectorXd::Zero(num_joints_);

      switch (current_mode) {
      case Mode::CARTESIAN_VEL:
        if (cmd_vel.size() == 6) {
          q_dot = velocity_ik_->solve(q_current_, cmd_vel);
        }
        break;

      case Mode::JOINT_VEL:
        q_dot = cmd_vel.head(std::min((int)cmd_vel.size(), num_joints_));
        break;

      case Mode::HOLD:
      case Mode::IDLE:
      case Mode::ESTOP:
        q_dot = Eigen::VectorXd::Zero(num_joints_);
        break;
      }

      // Apply safety limits
      q_dot = limiter_->clamp(q_current_, q_dot, control_period_);

      // Collision check
      if (collision_ && collision_check_enabled_ &&
          current_mode != Mode::ESTOP) {
        Eigen::VectorXd q_next = q_current_ + q_dot * control_period_;

        if (collision_->isSelfCollision(q_next)) {
          q_dot = Eigen::VectorXd::Zero(num_joints_);
          setFault("Self-collision detected");
        }
      }

      // Store current velocities
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        q_dot_current_ = q_dot;

        // Integrate position (for internal state tracking)
        // Real system should use encoder feedback via state_callback_
        if (!state_callback_) {
          q_current_ += q_dot * control_period_;
        }
      }

      // Output callback
      if (output_callback_) {
        output_callback_(q_dot);
      }

    } catch (const std::exception &e) {
      // CRITICAL: Keep thread alive even on errors
      setFault(std::string("Exception: ") + e.what());
      mode_ = Mode::ESTOP;
    }

    // Sleep to maintain control frequency
    auto elapsed = std::chrono::steady_clock::now() - loop_start;
    auto target_duration = std::chrono::duration<double>(control_period_);
    auto sleep_time = target_duration - elapsed;

    if (sleep_time.count() > 0) {
      std::this_thread::sleep_for(
          std::chrono::duration_cast<std::chrono::microseconds>(sleep_time));
    }
  }

  std::cout << "[RealTimeController] Control loop ended" << std::endl;
}

} // namespace control
