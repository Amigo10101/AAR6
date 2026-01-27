#include "kinematics/TrajectoryGenerator.h"
#include <cmath>
#include <iostream>

namespace kinematics {

TrajectoryGenerator::TrajectoryGenerator() {}

TrajectoryGenerator::~TrajectoryGenerator() {}

std::vector<TrajectoryPoint> TrajectoryGenerator::generateTrajectory(
    const Eigen::Isometry3d &start_pose, const Eigen::Isometry3d &end_pose,
    MovementType type, double max_velocity, double dt,
    const Eigen::VectorXd &start_joints, const Eigen::VectorXd &end_joints) {
  std::vector<TrajectoryPoint> trajectory;

  switch (type) {
  case MovementType::PTP:
    // For PTP, we typically need joint configurations.
    // If start/end_joints are provided and valid size, use them.
    // Otherwise, we might need IK (not implemented here, assuming inputs are
    // sufficient for skeleton)
    if (start_joints.size() > 0 && end_joints.size() > 0) {
      trajectory = generatePTP(start_joints, end_joints, max_velocity, dt);
    } else {
      std::cerr
          << "[TrajectoryGenerator] Error: PTP requires joint configurations."
          << std::endl;
    }
    break;

  case MovementType::LIN:
    trajectory = generateLIN(start_pose, end_pose, max_velocity, dt);
    break;

  case MovementType::CIRC:
    trajectory = generateCIRC(start_pose, end_pose, max_velocity, dt);
    break;
  }

  return trajectory;
}

std::vector<TrajectoryPoint>
TrajectoryGenerator::generatePTP(const Eigen::VectorXd &start_joints,
                                 const Eigen::VectorXd &end_joints,
                                 double max_velocity, double dt) {
  std::vector<TrajectoryPoint> trajectory;

  // Skeleton: Simple linear interpolation in joint space
  // In a real implementation, use trapezoidal or S-curve velocity profiles

  double duration = (end_joints - start_joints).norm() /
                    max_velocity; // Simplified duration calc
  if (duration < 1e-6)
    duration = dt;

  int steps = static_cast<int>(duration / dt);

  for (int i = 0; i <= steps; ++i) {
    double t = i * dt;
    double alpha = t / duration;

    // Clamp alpha
    if (alpha > 1.0)
      alpha = 1.0;

    TrajectoryPoint point;
    point.time = t;
    point.position = start_joints + alpha * (end_joints - start_joints);
    point.velocity =
        (end_joints - start_joints) / duration; // Constant velocity
    point.acceleration = Eigen::VectorXd::Zero(start_joints.size());

    trajectory.push_back(point);
  }

  return trajectory;
}

std::vector<TrajectoryPoint>
TrajectoryGenerator::generateLIN(const Eigen::Isometry3d &start_pose,
                                 const Eigen::Isometry3d &end_pose,
                                 double max_velocity, double dt) {
  std::vector<TrajectoryPoint> trajectory;

  // Skeleton: Linear interpolation in Cartesian space (translation + SLERP for
  // rotation)

  Eigen::Vector3d start_pos = start_pose.translation();
  Eigen::Vector3d end_pos = end_pose.translation();
  Eigen::Quaterniond start_rot(start_pose.rotation());
  Eigen::Quaterniond end_rot(end_pose.rotation());

  double distance = (end_pos - start_pos).norm();
  double duration = distance / max_velocity;
  if (duration < 1e-6)
    duration = dt;

  int steps = static_cast<int>(duration / dt);

  for (int i = 0; i <= steps; ++i) {
    double t = i * dt;
    double alpha = t / duration;
    if (alpha > 1.0)
      alpha = 1.0;

    TrajectoryPoint point;
    point.time = t;

    // Interpolate position
    Eigen::Vector3d current_pos = start_pos + alpha * (end_pos - start_pos);

    // Interpolate rotation (SLERP)
    Eigen::Quaterniond current_rot = start_rot.slerp(alpha, end_rot);

    // Store as a state vector?
    // For this skeleton, let's say "position" member holds Cartesian pose [x,
    // y, z, qx, qy, qz, qw] or just pass 7 elements.
    Eigen::VectorXd pose_vec(7);
    pose_vec << current_pos.x(), current_pos.y(), current_pos.z(),
        current_rot.x(), current_rot.y(), current_rot.z(), current_rot.w();

    point.position = pose_vec;

    // Velocity (Twist) - Simplified constant linear velocity
    // Angular velocity calculation is more complex involving quaternion
    // derivative
    Eigen::VectorXd vel_vec = Eigen::VectorXd::Zero(6);
    Eigen::Vector3d linear_vel = (end_pos - start_pos) / duration;
    vel_vec.head<3>() = linear_vel;

    point.velocity = vel_vec;

    trajectory.push_back(point);
  }

  return trajectory;
}

std::vector<TrajectoryPoint>
TrajectoryGenerator::generateCIRC(const Eigen::Isometry3d &start_pose,
                                  const Eigen::Isometry3d &end_pose,
                                  double max_velocity, double dt) {
  std::vector<TrajectoryPoint> trajectory;

  // Skeleton: Placeholder for Circular interpolation
  // Would typically require an intermediate point or center definition

  std::cout << "[TrajectoryGenerator] CIRC movement not fully implemented in "
               "skeleton."
            << std::endl;

  // Just return start and end for now
  TrajectoryPoint p_start;
  p_start.time = 0.0;
  // p_start.position = ...
  trajectory.push_back(p_start);

  return trajectory;
}

} // namespace kinematics
