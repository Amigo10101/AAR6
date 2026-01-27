#include "Eigen/Dense"
#include "kinematics/ForwardKinematics.h"
#include "quik/Robot.hpp"
#include "robot_model/RobotModel.h"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
  if (argc < 2) {
    cerr << "Usage: dh_visual_test <urdf_file>" << endl;
    return 1;
  }

  string urdf_file = argv[1];

  // Initialize URDF-based FK
  robot_model::RobotModel robot;
  cout << "Loading URDF: " << urdf_file << endl;
  if (!robot.loadURDF(urdf_file)) {
    cerr << "Failed to load URDF" << endl;
    return -1;
  }

  kinematics::ForwardKinematics fk_urdf;
  if (!fk_urdf.init(robot)) {
    cerr << "Failed to initialize URDF FK" << endl;
    return -1;
  }

  // DH parameters from CM6 table
  auto create_quik_robot = [](Matrix<double, 6, 4> dh_params,
                              const string &name) {
    return make_pair(
        name,
        std::make_shared<quik::Robot<6>>(
            dh_params,
            (Vector<quik::JOINTTYPE_t, 6>() << quik::JOINT_REVOLUTE,
             quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE,
             quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE)
                .finished(),
            (Vector<double, 6>(6) << 1, 1, 1, 1, 1, 1).finished(),
            Matrix4d::Identity(), Matrix4d::Identity()));
  };

  // Create multiple DH parameter sets to test
  vector<pair<string, shared_ptr<quik::Robot<6>>>> robots;

  // Set 1: CM6 table parameters
  Matrix<double, 6, 4> dh_cm6;
  dh_cm6 << 0.02342, -M_PI / 2, 0.18050, 0.0, // L1
      0.18000, M_PI, 0.0, -M_PI / 2,          // L2
      -0.04350, M_PI / 2, 0.0, M_PI,          // L3
      0.0, -M_PI / 2, -0.17635, 0.0,          // L4
      0.0, M_PI / 2, 0.0, 0.0,                // L5
      0.0, M_PI, -0.0628, M_PI;               // L6
  robots.push_back(create_quik_robot(dh_cm6, "CM6 Table"));

  // Set 2: Original (from sample_cpp_usage.cpp)
  Matrix<double, 6, 4> dh_original;
  dh_original << 0.0, M_PI / 2, 0.0, M_PI / 2, // L1
      0.023421, -M_PI / 2, 0.1105, M_PI / 2,   // L2
      0.18, 0.0, 0.0, -M_PI / 2,               // L3
      0.0435, -M_PI / 2, 0.0, 0.0,             // L4
      0.0, M_PI / 2, 0.17635, 0.0,             // L5
      0.0, -M_PI / 2, 0.0, 0.0;                // L6
  robots.push_back(create_quik_robot(dh_original, "Original"));

  // Set 3: Modified DH (from urdf_to_dh output)
  Matrix<double, 6, 4> dh_urdf_to_dh;
  dh_urdf_to_dh << 0.0, M_PI / 2, 0.0, M_PI / 2, // L1
      0.023421, -M_PI / 2, 0.1105, M_PI / 2,     // L2
      0.18, 0.0, 0.0, -M_PI / 2,                 // L3
      0.0435, -M_PI / 2, 0.0, -0.0,              // L4
      0.0, M_PI / 2, 0.17635, 0.0,               // L5
      0.0, -M_PI / 2, 0.0, 0.0;                  // L6
  robots.push_back(create_quik_robot(dh_urdf_to_dh, "URDF-to-DH"));

  cout << "\n=================================================================="
          "===="
       << endl;
  cout << "FK Comparison: Testing Multiple DH Parameter Sets" << endl;
  cout << "===================================================================="
          "==\n"
       << endl;

  // Test configurations
  vector<VectorXd> test_configs;
  vector<string> test_names;

  VectorXd q0 = VectorXd::Zero(6);
  test_configs.push_back(q0);
  test_names.push_back("All Zeros");

  VectorXd q1(6);
  q1 << M_PI / 4, 0, 0, 0, 0, 0;
  test_configs.push_back(q1);
  test_names.push_back("J1=45\u00b0");

  VectorXd q2(6);
  q2 << 0, M_PI / 4, 0, 0, 0, 0;
  test_configs.push_back(q2);
  test_names.push_back("J2=45\u00b0");

  VectorXd q3(6);
  q3 << M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4, M_PI / 4;
  test_configs.push_back(q3);
  test_names.push_back("All 45\u00b0");

  for (size_t i = 0; i < test_configs.size(); ++i) {
    const auto &q = test_configs[i];

    cout << "\n----------------------------------------------------------------"
            "------"
         << endl;
    cout << "Test: " << test_names[i] << endl;
    cout << "Joint angles (deg): ";
    for (int j = 0; j < 6; ++j) {
      cout << fixed << setprecision(1) << (q(j) * 180.0 / M_PI);
      if (j < 5)
        cout << ", ";
    }
    cout << endl;
    cout << "------------------------------------------------------------------"
            "----"
         << endl;

    // URDF FK (reference/correct)
    Isometry3d T_urdf = fk_urdf.getLinkTransform("ee_link", q);
    Vector3d pos_urdf = T_urdf.translation();

    cout << "\nURDF FK (CORRECT):  [" << fixed << setprecision(4)
         << pos_urdf.x() << ", " << pos_urdf.y() << ", " << pos_urdf.z() << "]"
         << endl;

    // Test each DH parameter set
    for (const auto &[name, robot] : robots) {
      Matrix4d T_dh;
      robot->FKn(q, T_dh);
      Vector3d pos_dh = T_dh.block<3, 1>(0, 3);
      Vector3d diff = pos_urdf - pos_dh;
      double error = diff.norm();

      cout << setw(15) << left << name << ": [" << fixed << setprecision(4)
           << pos_dh.x() << ", " << pos_dh.y() << ", " << pos_dh.z() << "]";

      cout << "  \u2192 error=" << setprecision(4) << error << "m";

      if (error < 0.001) {
        cout << " \u2713 MATCH!";
      } else if (error < 0.01) {
        cout << " \u2248 Close";
      } else if (error < 0.1) {
        cout << " \u26a0 Diff";
      } else {
        cout << " \u2717 MISMATCH!";
      }
      cout << endl;
    }
  }

  cout << "\n=================================================================="
          "====\n";
  cout << "Analysis:" << endl;
  cout << "  - The DH set with smallest errors is closest to the URDF model"
       << endl;
  cout << "  - Large errors indicate coordinate frame or parameter convention "
          "mismatch"
       << endl;
  cout << "===================================================================="
          "==\n";

  return 0;
}
