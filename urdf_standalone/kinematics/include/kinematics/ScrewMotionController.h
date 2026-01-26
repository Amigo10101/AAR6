/**
 * @file ScrewMotionController.h
 * @brief Trajectory Generator for Straight Lines and Screw Motions
 *
 * This demonstrates how to use the PositionIK solver with proper
 * feed-forward control for perfect Cartesian path tracking.
 *
 * Usage Pattern:
 *  1. Pre-compute trajectory parameters (linear/angular velocity)
 *  2. Run a real-time loop at 1kHz that:
 *     a) Interpolates the target pose at current time
 *     b) Computes feed-forward twist (constant for straight lines)
 *     c) Calls solver.solve(q_current, target_pose, twist_ff, dt)
 *     d) Sends resulting q_dot to motor drivers
 */

#ifndef SCREW_MOTION_CONTROLLER_H
#define SCREW_MOTION_CONTROLLER_H

#include "kinematics/PositionIK.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <functional>

namespace kinematics {

/**
 * @brief Trajectory segment representing a screw motion (combined linear +
 * rotation)
 */
struct ScrewMotionSegment {
  Eigen::Vector3d start_pos;
  Eigen::Quaterniond start_rot;
  Eigen::Vector3d end_pos;
  Eigen::Quaterniond end_rot;
  double duration; // seconds

  // Pre-computed feed-forward twist (constant for straight lines)
  Eigen::VectorXd feed_forward_twist;

  /**
   * @brief Compute the constant feed-forward twist for this motion
   * Call this after setting start/end poses and duration
   */
  void computeFeedForward() {
    feed_forward_twist.resize(6);

    // Linear velocity (m/s)
    Eigen::Vector3d v_linear = (end_pos - start_pos) / duration;

    // Angular velocity (rad/s)
    Eigen::Quaterniond q_diff = end_rot * start_rot.inverse();
    Eigen::AngleAxisd aa(q_diff);
    double total_angle = aa.angle();

    // Normalize angle (take shortest path)
    if (total_angle > M_PI)
      total_angle -= 2 * M_PI;
    else if (total_angle < -M_PI)
      total_angle += 2 * M_PI;

    Eigen::Vector3d v_angular = (total_angle / duration) * aa.axis();

    feed_forward_twist << v_linear, v_angular;
  }

  /**
   * @brief Get interpolated pose at normalized time t ∈ [0, 1]
   */
  Eigen::Isometry3d getPoseAt(double alpha) const {
    alpha = std::clamp(alpha, 0.0, 1.0);

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = start_pos + (end_pos - start_pos) * alpha;
    pose.linear() = start_rot.slerp(alpha, end_rot).toRotationMatrix();
    return pose;
  }
};

/**
 * @brief Example controller interface for trajectory execution
 *
 * In your application, you would:
 * 1. Create a ScrewMotionSegment with start/end poses
 * 2. Call segment.computeFeedForward()
 * 3. In your control loop, compute alpha = elapsed_time / duration
 * 4. Call solver.solve(q_current, segment.getPoseAt(alpha),
 *                      segment.feed_forward_twist, dt)
 */
class ScrewMotionController {
public:
  explicit ScrewMotionController(PositionIK &solver) : solver_(solver) {}

  using GetJointsCallback = std::function<Eigen::VectorXd()>;
  using SetVelocityCallback = std::function<void(const Eigen::VectorXd &)>;

  /**
   * @brief Execute a single screw motion segment
   *
   * This is a BLOCKING function that runs the control loop.
   * In production, you'd integrate this into your own timing loop.
   *
   * @param segment The motion to execute
   * @param getJoints Callback to read current joint positions
   * @param setVelocity Callback to command joint velocities
   * @param dt Control loop period (default 1ms = 1kHz)
   * @return true if motion completed successfully
   */
  bool executeMotion(const ScrewMotionSegment &segment,
                     GetJointsCallback getJoints,
                     SetVelocityCallback setVelocity, double dt = 0.001) {
    double t = 0.0;

    while (t < segment.duration) {
      double alpha = t / segment.duration;

      // Get target pose at current time
      Eigen::Isometry3d target_pose = segment.getPoseAt(alpha);

      // Get current joint positions
      Eigen::VectorXd q_current = getJoints();

      // Solve for joint velocities
      Eigen::VectorXd q_dot =
          solver_.solve(q_current, target_pose, segment.feed_forward_twist, dt);

      // Command velocities
      setVelocity(q_dot);

      // Advance time (in production, use actual elapsed time)
      t += dt;

      // In production, you'd wait for the control period here
      // std::this_thread::sleep_for(std::chrono::microseconds((int)(dt *
      // 1e6)));
    }

    // Stop at the end
    setVelocity(Eigen::VectorXd::Zero(6));
    return true;
  }

  /**
   * @brief Compute a single step for integration into external control loop
   *
   * Use this if you have your own timing/threading infrastructure.
   *
   * @param segment The motion segment
   * @param elapsed_time Time since motion start (seconds)
   * @param q_current Current joint positions
   * @param dt Control loop period
   * @return Joint velocities to command
   */
  Eigen::VectorXd computeStep(const ScrewMotionSegment &segment,
                              double elapsed_time,
                              const Eigen::VectorXd &q_current, double dt) {
    double alpha = std::clamp(elapsed_time / segment.duration, 0.0, 1.0);

    Eigen::Isometry3d target_pose = segment.getPoseAt(alpha);

    return solver_.solve(q_current, target_pose, segment.feed_forward_twist,
                         dt);
  }

private:
  PositionIK &solver_;
};

} // namespace kinematics

#endif // SCREW_MOTION_CONTROLLER_H
