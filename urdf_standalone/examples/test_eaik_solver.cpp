#include "kinematics/InverseKinematics.h"
#include "robot_model/RobotModel.h"
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <vector>

// Helper to generate random joint angles
Eigen::VectorXd generateRandomConfig(int num_joints,
                                     const std::vector<double> &lower,
                                     const std::vector<double> &upper) {
  Eigen::VectorXd q(num_joints);
  std::random_device rd;
  std::mt19937 gen(rd());

  for (int i = 0; i < num_joints; ++i) {
    std::uniform_real_distribution<> dis(lower[i], upper[i]);
    q[i] = dis(gen);
  }
  return q;
}

int main(int argc, char **argv) {
  std::string urdf_path =
      "/home/idgaf/Documents/robot/Robot_controller "
      "/urdf_standalone/resources/aar6.urdf"; // Adjust path if needed or take
                                              // from argv
  if (argc > 1) {
    urdf_path = argv[1];
  }

  std::cout << "Loading URDF from: " << urdf_path << std::endl;

  robot_model::RobotModel robot;
  if (!robot.loadURDF(urdf_path)) {
    std::cerr << "Failed to load URDF" << std::endl;
    return 1;
  }

  kinematics::InverseKinematics ik;
  // Assuming end_effector_link is "ee_link" based on URDF inspection.
  std::string ee_link = "ee_link";

  if (!ik.init(robot, ee_link, 6)) {
    std::cerr << "Failed to init IK" << std::endl;
    return 1;
  }

  // Get limits to generate valid random poses
  // We can't access limits directly from IK public API easily without adding
  // getters, but we can just use -pi to pi for test.
  int num_joints = robot.getNumJoints();
  std::vector<double> lower(num_joints, -M_PI);
  std::vector<double> upper(num_joints, M_PI);

  std::cout << "Testing EAIK Solver..." << std::endl;

  int num_tests = 10;
  int success_count = 0;

  kinematics::ForwardKinematics fk;
  fk.init(robot);

  // Test 0: Zero Configuration
  {
    Eigen::VectorXd q_target = Eigen::VectorXd::Zero(num_joints);
    Eigen::Isometry3d target_pose = fk.getLinkTransform(ee_link, q_target);

    std::cout << "\nTest 0 (Zero Config)" << std::endl;
    std::cout << "Target Joint (deg): "
              << (q_target.head(6) * 180.0 / M_PI).transpose() << std::endl;
    std::cout << "Home EE Position (m): "
              << target_pose.translation().transpose() << std::endl;
    std::cout << "Home EE Rotation Matrix:\n"
              << target_pose.rotation() << std::endl;
    Eigen::Quaterniond q_home_rot(target_pose.rotation());
    std::cout << "Home EE Quaternion (w, x, y, z): " << q_home_rot.w() << " "
              << q_home_rot.x() << " " << q_home_rot.y() << " "
              << q_home_rot.z() << std::endl;

    kinematics::IKResult result = ik.solve(target_pose, q_target);

    if (result.success) {
      std::cout << "SUCCESS" << std::endl;
      std::cout << "Result Joint (deg): "
                << (result.joint_angles.head(6) * 180.0 / M_PI).transpose()
                << std::endl;
      std::cout << "Position Error: " << result.final_error << std::endl;
      std::cout << "Solve Time: " << result.solve_time_ms << " ms" << std::endl;
    } else {
      std::cout << "FAILURE: " << result.error_message << std::endl;
    }
  }

  for (int i = 0; i < num_tests; ++i) {
    Eigen::VectorXd q_target = generateRandomConfig(num_joints, lower, upper);

    // Compute FK for target
    Eigen::Isometry3d target_pose = fk.getLinkTransform(ee_link, q_target);

    std::cout << "\nTest " << i + 1 << "/" << num_tests << std::endl;
    std::cout << "Target Joint (deg): "
              << (q_target.head(6) * 180.0 / M_PI).transpose() << std::endl;

    // Solve IK
    // Pass q_target as initial guess to help it pick the closest solution
    kinematics::IKResult result = ik.solve(target_pose, q_target);

    if (result.success) {
      std::cout << "SUCCESS" << std::endl;
      std::cout << "Result Joint (deg): "
                << (result.joint_angles.head(6) * 180.0 / M_PI).transpose()
                << std::endl;
      std::cout << "Position Error: " << result.final_error << std::endl;
      std::cout << "Solve Time: " << result.solve_time_ms << " ms" << std::endl;
      success_count++;
    } else {
      std::cout << "FAILURE: " << result.error_message << std::endl;
    }
  }

  std::cout << "\nFinal Results: " << success_count << "/" << num_tests
            << " passed." << std::endl;

  return 0;
}
