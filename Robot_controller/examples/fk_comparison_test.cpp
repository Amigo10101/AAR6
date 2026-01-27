#include "Eigen/Dense"
#include "kinematics/ForwardKinematics.h"
#include "quik/IKSolver.hpp"
#include "quik/Robot.hpp"
#include "quik/geometry.hpp"
#include "robot_model/RobotModel.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {
  if (argc < 2) {
    cerr << "Usage: fk_comparison_test <urdf_file>" << endl;
    return 1;
  }

  string urdf_file = argv[1];

  // Base transform from URDF base_joint: rpy="0 0 3.142" xyz="0.145 0.0 0.069"
  // Using exact same calculation as visualizer
  Matrix4d base_transform = Matrix4d::Identity();
  double rpy_z = 3.142;
  base_transform(0, 0) = cos(rpy_z);
  base_transform(0, 1) = -sin(rpy_z);
  base_transform(1, 0) = sin(rpy_z);
  base_transform(1, 1) = cos(rpy_z);
  base_transform(0, 3) = 0.145;
  base_transform(1, 3) = 0.0;
  base_transform(2, 3) = 0.069;

  // DH parameters - EXACT same as visualizer
  // Updated values: a6 = 0.1035 m, a7 = 0 m
  // Joint signs: [1, 1, -1, -1, -1, -1] - CRITICAL!
  auto R_quik = std::make_shared<quik::Robot<6>>(
      (Matrix<double, 6, 4>() << 0.02342, -M_PI / 2, 0.11050, 0.0, // L1
       0.18000, M_PI, 0.0, -M_PI / 2,                              // L2
       -0.04350, M_PI / 2, 0.0, M_PI,                              // L3
       0.0, -M_PI / 2, -0.17635, 0.0,                              // L4
       0.0, M_PI / 2, 0.0, 0.0,                                    // L5
       0.0, M_PI, -0.1035, M_PI) // L6 (a6=0.1035, a7=0)
          .finished(),
      (Vector<quik::JOINTTYPE_t, 6>() << quik::JOINT_REVOLUTE,
       quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE,
       quik::JOINT_REVOLUTE, quik::JOINT_REVOLUTE)
          .finished(),
      (Vector<double, 6>(6) << 1, 1, -1, -1, -1, -1)
          .finished(),       // JOINT SIGNS - match visualizer!
      base_transform,        // Base transform
      Matrix4d::Identity()); // Tool transform

  // Initialize URDF-based FK
  robot_model::RobotModel robot;
  cout << "Loading URDF: " << urdf_file << endl;
  if (!robot.loadURDF(urdf_file)) {
    cerr << "Failed to load URDF" << endl;
    return -1;
  }

  kinematics::ForwardKinematics fk_urdf;
  if (!fk_urdf.init(robot)) {
    cerr << "Failed to initialize URDF-based FK" << endl;
    return -1;
  }

  cout << "\n========================================" << endl;
  cout << "FK Comparison Test: DH vs URDF (1000 Random Points)" << endl;
  cout << "========================================\n" << endl;
  cout << "Testing with 1000 random joint configurations..." << endl;
  cout << "Joint limits: [-π, π] for all joints\n" << endl;

  // Generate 1000 random configurations and test
  const int num_tests = 1000;
  vector<double> position_errors;
  position_errors.reserve(num_tests);

  // Random number generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(-M_PI, M_PI);

  VectorXd q(6);
  double max_error = 0.0;
  double min_error = 1e10;
  VectorXd worst_config(6);
  VectorXd best_config(6);

  for (int i = 0; i < num_tests; ++i) {
    // Generate random joint angles
    for (int j = 0; j < 6; ++j) {
      q(j) = dis(gen);
    }

    // QUIK DH-based FK
    Matrix4d T_quik;
    R_quik->FKn(q, T_quik);
    Vector3d pos_quik = T_quik.block<3, 1>(0, 3);

    // URDF-based FK
    Isometry3d T_urdf = fk_urdf.getLinkTransform("ee_link", q);
    Vector3d pos_urdf = T_urdf.translation();

    // Compute position error
    double pos_error = (pos_urdf - pos_quik).norm();
    position_errors.push_back(pos_error);

    if (pos_error > max_error) {
      max_error = pos_error;
      worst_config = q;
    }
    if (pos_error < min_error) {
      min_error = pos_error;
      best_config = q;
    }

    // Progress indicator
    if ((i + 1) % 100 == 0) {
      cout << "  Tested " << (i + 1) << " configurations..." << endl;
    }
  }

  // Compute statistics
  double sum = 0.0;
  for (double err : position_errors) {
    sum += err;
  }
  double mean_error = sum / num_tests;

  double variance = 0.0;
  for (double err : position_errors) {
    variance += (err - mean_error) * (err - mean_error);
  }
  double std_dev = sqrt(variance / num_tests);

  // Sort for percentiles
  sort(position_errors.begin(), position_errors.end());
  double median_error = position_errors[num_tests / 2];
  double p95_error = position_errors[(int)(num_tests * 0.95)];
  double p99_error = position_errors[(int)(num_tests * 0.99)];

  cout << "\n========================================" << endl;
  cout << "POSITION ERROR STATISTICS" << endl;
  cout << "========================================" << endl;
  cout << fixed << setprecision(6);
  cout << "Min error:    " << min_error * 1000 << " mm" << endl;
  cout << "Max error:    " << max_error * 1000 << " mm" << endl;
  cout << "Mean error:   " << mean_error * 1000 << " mm" << endl;
  cout << "Median error: " << median_error * 1000 << " mm" << endl;
  cout << "Std dev:      " << std_dev * 1000 << " mm" << endl;
  cout << "95th %ile:    " << p95_error * 1000 << " mm" << endl;
  cout << "99th %ile:    " << p99_error * 1000 << " mm" << endl;

  cout << "\n--- Best Configuration (min error) ---" << endl;
  cout << "Joints: [";
  for (int i = 0; i < 6; ++i) {
    cout << fixed << setprecision(3) << best_config(i);
    if (i < 5)
      cout << ", ";
  }
  cout << "] rad" << endl;
  cout << "Error: " << min_error * 1000 << " mm" << endl;

  cout << "\n--- Worst Configuration (max error) ---" << endl;
  cout << "Joints: [";
  for (int i = 0; i < 6; ++i) {
    cout << fixed << setprecision(3) << worst_config(i);
    if (i < 5)
      cout << ", ";
  }
  cout << "] rad" << endl;
  cout << "Error: " << max_error * 1000 << " mm" << endl;

  cout << "\n========================================" << endl;
  cout << "ANALYSIS" << endl;
  cout << "========================================" << endl;

  if (mean_error < 0.001) {
    cout << "✓ EXCELLENT: Mean error < 1mm" << endl;
    cout << "  DH parameters match URDF very well!" << endl;
  } else if (mean_error < 0.005) {
    cout << "✓ GOOD: Mean error < 5mm" << endl;
    cout << "  DH parameters are acceptable for most applications." << endl;
  } else if (mean_error < 0.01) {
    cout << "⚠ FAIR: Mean error < 10mm" << endl;
    cout << "  DH approximation has noticeable differences." << endl;
  } else {
    cout << "✗ POOR: Mean error > 10mm" << endl;
    cout << "  DH parameters do not match URDF kinematics well." << endl;
  }
  cout << "========================================" << endl;

  return 0;
}
