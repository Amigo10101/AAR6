#include "Eigen/Dense"
#include "kinematics/InverseKinematics.h"
#include "robot_model/RobotModel.h"
#include <iomanip>
#include <iostream>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
  if (argc < 2) {
    cerr << "Usage: ik_test <urdf_file>" << endl;
    return 1;
  }

  string urdf_file = argv[1];

  // Initialize robot model
  robot_model::RobotModel robot;
  cout << "Loading URDF: " << urdf_file << endl;
  if (!robot.loadURDF(urdf_file)) {
    cerr << "Failed to load URDF" << endl;
    return -1;
  }

  // Initialize IK solver
  kinematics::InverseKinematics ik;
  if (!ik.init(robot)) {
    cerr << "Failed to initialize IK solver" << endl;
    return -1;
  }

  cout << "\n========================================" << endl;
  cout << "IK Solver Test - Updated DH Parameters" << endl;
  cout << "DH params: a6=0.1035m, a7=0.0m" << endl;
  cout << "Joint signs: [1, 1, -1, -1, -1, -1]" << endl;
  cout << "========================================\n" << endl;

  // Test configurations
  struct TestCase {
    string name;
    Vector3d position;
    Quaterniond orientation;
  };

  vector<TestCase> tests;

  // Test 1: Home position (all zeros)
  VectorXd q_home = VectorXd::Zero(6);
  Isometry3d T_home = ik.computeFK(q_home);
  tests.push_back({"Home (zero config)", T_home.translation(),
                   Quaterniond(T_home.rotation())});

  // Test 2: Known reachable pose
  Vector3d pos2(0.2, 0.1, 0.3);
  Quaterniond ori2 = Quaterniond(AngleAxisd(M_PI / 4, Vector3d::UnitZ()));
  tests.push_back({"Reachable pose", pos2, ori2});

  // Test 3: Different orientation
  Vector3d pos3(0.15, -0.15, 0.35);
  Quaterniond ori3 = Quaterniond(AngleAxisd(M_PI / 2, Vector3d::UnitY()));
  tests.push_back({"Different orientation", pos3, ori3});

  // Run IK tests
  for (size_t i = 0; i < tests.size(); ++i) {
    const auto &test = tests[i];
    cout << "Test " << (i + 1) << ": " << test.name << endl;
    cout << "Target position: [" << fixed << setprecision(4)
         << test.position.x() << ", " << test.position.y() << ", "
         << test.position.z() << "]" << endl;

    // Solve IK
    VectorXd q_init = VectorXd::Zero(6);
    VectorXd q_sol(6);
    bool success = ik.solve(test.position, test.orientation, q_init, q_sol);

    if (success) {
      cout << "  ✓ IK SOLVED" << endl;
      cout << "  Solution: [";
      for (int j = 0; j < 6; ++j) {
        cout << fixed << setprecision(3) << q_sol(j);
        if (j < 5)
          cout << ", ";
      }
      cout << "] rad" << endl;

      // Verify with FK
      Isometry3d T_result = ik.computeFK(q_sol);
      Vector3d pos_error = T_result.translation() - test.position;
      double pos_err = pos_error.norm() * 1000; // mm

      Quaterniond ori_result(T_result.rotation());
      Quaterniond ori_error = test.orientation.conjugate() * ori_result;
      double ori_err = 2.0 * asin(ori_error.vec().norm()); // rad

      cout << "  Position error: " << pos_err << " mm" << endl;
      cout << "  Orientation error: " << ori_err << " rad" << endl;

      if (pos_err < 1.0 && ori_err < 0.01) {
        cout << "  ✓ ACCURATE SOLUTION" << endl;
      } else {
        cout << "  ⚠ Solution has errors" << endl;
      }
    } else {
      cout << "  ✗ IK FAILED TO CONVERGE" << endl;
    }
    cout << endl;
  }

  cout << "========================================" << endl;
  cout << "IK Test Complete" << endl;
  cout << "========================================" << endl;

  return 0;
}
