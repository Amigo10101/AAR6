#include "Eigen/Dense"
#include "kinematics/ForwardKinematics.h"
#include "quik/Robot.hpp"
#include "robot_model/RobotModel.h"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
  if (argc < 2) {
    cerr << "Usage: extract_link_positions <urdf_file>" << endl;
    return 1;
  }

  string urdf_file = argv[1];

  // Initialize URDF FK
  robot_model::RobotModel robot;
  if (!robot.loadURDF(urdf_file)) {
    cerr << "Failed to load URDF" << endl;
    return -1;
  }

  kinematics::ForwardKinematics fk_urdf;
  if (!fk_urdf.init(robot)) {
    cerr << "Failed to initialize URDF FK" << endl;
    return -1;
  }

  // Zero configuration
  VectorXd q = VectorXd::Zero(6);

  // List of links to extract positions for
  vector<string> links = {"base_link", "L1", "L2", "L3",
                          "L4",        "L5", "L6", "ee_link"};

  cout << "# URDF Link Positions at Zero Configuration" << endl;
  cout << "# Format: x,y,z" << endl;

  for (const auto &link_name : links) {
    Isometry3d T = fk_urdf.getLinkTransform(link_name, q);
    Vector3d pos = T.translation();
    cout << fixed << setprecision(6) << pos.x() << "," << pos.y() << ","
         << pos.z() << endl;
  }

  // Also output DH robot for comparison
  cout << "\n# DH Link Positions (CM6 Table)" << endl;
  cout << "# Format: x,y,z" << endl;

  // DH parameters (CM6 table with a1=110.50mm) - BEST RESULT
  Matrix<double, 6, 4> dh_params;
  dh_params << 0.02342, -M_PI / 2, 0.11050, 0.0, // L1
      0.18000, M_PI, 0.0, -M_PI / 2,             // L2
      -0.04350, M_PI / 2, 0.0, M_PI,             // L3
      0.0, -M_PI / 2, -0.17635, 0.0,             // L4
      0.0, M_PI / 2, 0.0, 0.0,                   // L5
      0.0, M_PI, -0.0628, M_PI;                  // L6
  // Base transform from URDF: rpy="0 0 3.142" xyz="0.145 0.0 0.069"
  Matrix4d base_transform = Matrix4d::Identity();
  base_transform(0, 0) = cos(M_PI); // 180° Z-rotation
  base_transform(0, 1) = -sin(M_PI);
  base_transform(1, 0) = sin(M_PI);
  base_transform(1, 1) = cos(M_PI);
  base_transform(0, 3) = 0.145; // Translation
  base_transform(1, 3) = 0.0;
  base_transform(2, 3) = 0.069;

  auto R_dh = std::make_shared<quik::Robot<6>>(
      dh_params,
      (Vector<quik::JOINTTYPE_t, 6>() << quik::JOINT_REVOLUTE,
       quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE,
       quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE)
          .finished(),
      (Vector<double, 6>(6) << 1, 1, 1, 1, 1, 1).finished(), base_transform,
      Matrix4d::Identity());

  // Compute DH FK for all intermediate frames
  constexpr int DOF4 = 7 * 4; // 6 joints + tool
  Matrix<double, DOF4, 4> T_all(7 * 4, 4);
  R_dh->FK(q, T_all);

  // Base
  cout << "0.000000,0.000000,0.000000" << endl;

  // Each joint frame
  for (int i = 0; i < 7; ++i) {
    Vector3d pos = T_all.block<3, 1>(i * 4, 3);
    cout << fixed << setprecision(6) << pos.x() << "," << pos.y() << ","
         << pos.z() << endl;
  }

  return 0;
}
