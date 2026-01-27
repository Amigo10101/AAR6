#include "kinematics/InverseKinematics.h"
#include "robot_model/RobotModel.h"
#include <cmath>
#include <iostream>

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: ik_example <urdf_file> [x y z qw qx qy qz]"
              << std::endl;
    std::cerr << "  Position in meters, orientation as quaternion (w,x,y,z)"
              << std::endl;
    std::cerr << "  Example: ik_example robot.urdf 0.3 0.0 0.4 0.707 0 0.707 0"
              << std::endl;
    return 1;
  }

  std::string urdf_file = argv[1];
  robot_model::RobotModel robot;

  std::cout << "Loading URDF: " << urdf_file << std::endl;
  if (!robot.loadURDF(urdf_file)) {
    std::cerr << "Failed to load URDF" << std::endl;
    return -1;
  }

  // Initialize IK solver with RobotModel
  kinematics::InverseKinematics ik;
  if (!ik.init(robot, "ee_link")) {
    std::cerr << "Failed to initialize IK solver" << std::endl;
    return -1;
  }

  // Default target pose (if not provided)
  Eigen::Vector3d position(0.3, 0.0, 0.4);            // meters
  Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0); // identity (w,x,y,z)

  // Parse command line arguments for target pose
  if (argc >= 5) {
    position.x() = std::stod(argv[2]);
    position.y() = std::stod(argv[3]);
    position.z() = std::stod(argv[4]);
  }
  if (argc >= 9) {
    double qw = std::stod(argv[5]);
    double qx = std::stod(argv[6]);
    double qy = std::stod(argv[7]);
    double qz = std::stod(argv[8]);
    orientation = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
  }

  std::cout << "\n========================================" << std::endl;
  std::cout << "Requesting IK solution for:" << std::endl;
  std::cout << "  Position: (" << position.x() << ", " << position.y() << ", "
            << position.z() << ") m" << std::endl;
  std::cout << "  Orientation (wxyz): (" << orientation.w() << ", "
            << orientation.x() << ", " << orientation.y() << ", "
            << orientation.z() << ")" << std::endl;
  std::cout << "========================================\n" << std::endl;

  // Solve IK with position and quaternion
  kinematics::IKResult result = ik.solve(position, orientation);

  // Print result
  std::cout << "\n========================================" << std::endl;
  std::cout << "IK Result:" << std::endl;
  std::cout << "  Success: " << (result.success ? "true" : "false")
            << std::endl;
  std::cout << "  Iterations: " << result.iterations << std::endl;
  std::cout << "  Final error: " << result.final_error << std::endl;
  std::cout << "  Solve time: " << result.solve_time_ms << " ms" << std::endl;

  if (!result.error_message.empty()) {
    std::cout << "  Note: " << result.error_message << std::endl;
  }

  if (result.joint_angles.size() > 0) {
    std::cout << "  Joint angles: [";
    for (int i = 0; i < result.joint_angles.size(); i++) {
      std::cout << result.joint_angles[i];
      if (i < result.joint_angles.size() - 1)
        std::cout << ", ";
    }
    std::cout << "] rad" << std::endl;
  }
  std::cout << "========================================" << std::endl;

  return result.success ? 0 : 1;
}
