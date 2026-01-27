#include "kinematics/ForwardKinematics.h"
#include "robot_model/RobotModel.h"
#include <iostream>

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: fk_example <urdf_file>" << std::endl;
    return 1;
  }

  std::string urdf_file = argv[1];
  robot_model::RobotModel robot;

  if (!robot.loadURDF(urdf_file)) {
    return -1;
  }

  kinematics::ForwardKinematics fk;
  if (!fk.init(robot)) { // Pass RobotModel, not just the URDF
    std::cerr << "Failed to initialize FK" << std::endl;
    return -1;
  }

  std::cout << "Printing Robot Data..." << std::endl;
  fk.printRobotStructure();

  return 0;
}
