#include "kinematics/ForwardKinematics.h"
#include "kinematics/InverseKinematics.h"
#include "robot_model/RobotModel.h"
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

// Test point result
struct TestPoint {
  // Joint configuration used to generate this pose
  std::vector<double> original_joints;
  // Target pose from FK
  double x, y, z;
  double qw, qx, qy, qz;
  // IK result
  bool success;
  int iterations;
  double final_error;
  double solve_time_ms;
  std::string failure_reason;
};

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: ik_convergence_test <urdf_file> [num_samples]"
              << std::endl;
    std::cerr << "  num_samples: number of random configurations to test "
                 "(default: 1000)"
              << std::endl;
    std::cerr << "  Output: ik_failures.csv, ik_statistics.txt" << std::endl;
    return 1;
  }

  std::string urdf_file = argv[1];
  int num_samples = (argc >= 3) ? std::stoi(argv[2]) : 1000;

  robot_model::RobotModel robot;
  std::cout << "Loading URDF: " << urdf_file << std::endl;
  if (!robot.loadURDF(urdf_file)) {
    std::cerr << "Failed to load URDF" << std::endl;
    return -1;
  }

  // Initialize FK and IK
  kinematics::ForwardKinematics fk;
  if (!fk.init(robot)) {
    std::cerr << "Failed to initialize FK" << std::endl;
    return -1;
  }

  kinematics::InverseKinematics ik;
  if (!ik.init(robot, "ee_link")) {
    std::cerr << "Failed to initialize IK solver" << std::endl;
    return -1;
  }

  // Get joint names and limits from robot model
  const auto &joint_names = robot.getJointNames();
  int num_ik_joints = 6; // Only test first 6 joints (IK joints)

  // Joint limits (approximate, should come from URDF)
  // Using ±180° for revolute joints as reasonable defaults
  std::vector<double> joint_min(num_ik_joints, -M_PI);
  std::vector<double> joint_max(num_ik_joints, M_PI);

  // Random number generator
  std::random_device rd;
  std::mt19937 gen(rd());

  // Statistics
  int total_tests = 0;
  int total_success = 0;
  int total_failures = 0;
  double total_solve_time = 0;
  double max_solve_time = 0;
  std::vector<TestPoint> failures;
  std::vector<TestPoint> all_results;

  std::cout << "\n========================================" << std::endl;
  std::cout << "IK Convergence Test (FK-Based)" << std::endl;
  std::cout << "Samples: " << num_samples << std::endl;
  std::cout << "Method: Generate random joint configs, compute FK, test IK"
            << std::endl;
  std::cout << "========================================\n" << std::endl;

  auto start_time = std::chrono::high_resolution_clock::now();

  int progress_step = std::max(1, num_samples / 20);

  for (int i = 0; i < num_samples; ++i) {
    // Generate random joint configuration within limits
    Eigen::VectorXd q_original(num_ik_joints);
    for (int j = 0; j < num_ik_joints; ++j) {
      std::uniform_real_distribution<> dis(joint_min[j], joint_max[j]);
      q_original(j) = dis(gen);
    }

    // Compute FK to get end-effector pose
    Eigen::Isometry3d ee_pose = fk.getLinkTransform("ee_link", q_original);
    Eigen::Vector3d position = ee_pose.translation();
    Eigen::Quaterniond orientation(ee_pose.rotation());

    // Now test IK with zero initial guess
    // This tests worst-case convergence
    auto result = ik.solve(position, orientation);

    total_tests++;
    total_solve_time += result.solve_time_ms;
    if (result.solve_time_ms > max_solve_time) {
      max_solve_time = result.solve_time_ms;
    }

    TestPoint tp;
    tp.original_joints.resize(num_ik_joints);
    for (int j = 0; j < num_ik_joints; ++j) {
      tp.original_joints[j] = q_original(j);
    }
    tp.x = position.x();
    tp.y = position.y();
    tp.z = position.z();
    tp.qw = orientation.w();
    tp.qx = orientation.x();
    tp.qy = orientation.y();
    tp.qz = orientation.z();
    tp.success = result.success;
    tp.iterations = result.iterations;
    tp.final_error = result.final_error;
    tp.solve_time_ms = result.solve_time_ms;
    tp.failure_reason = result.error_message;

    all_results.push_back(tp);

    if (result.success) {
      total_success++;
    } else {
      total_failures++;
      failures.push_back(tp);
    }

    if ((i + 1) % progress_step == 0) {
      std::cout << "\rProgress: " << std::setw(3)
                << ((i + 1) * 100 / num_samples) << "% (" << total_tests
                << " tests, " << total_failures << " failures)" << std::flush;
    }
  }

  auto end_time = std::chrono::high_resolution_clock::now();
  double total_time =
      std::chrono::duration<double>(end_time - start_time).count();

  std::cout << "\r                                                         "
            << std::endl;

  // ============================================================
  // RETRY PHASE: Try random initial guesses for failed poses
  // ============================================================
  int num_retry_attempts = 100; // Restore retry attempts
  int recovered_count = 0;
  std::vector<TestPoint> true_failures;

  if (!failures.empty()) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "Retrying " << failures.size()
              << " failed poses with random initial guesses..." << std::endl;
    std::cout << "Attempts per pose: " << num_retry_attempts << std::endl;
    std::cout << "========================================\n" << std::endl;

    int retry_progress_step = std::max(1, (int)failures.size() / 10);

    for (size_t f = 0; f < failures.size(); ++f) {
      const auto &fp = failures[f];
      Eigen::Vector3d position(fp.x, fp.y, fp.z);
      Eigen::Quaterniond orientation(fp.qw, fp.qx, fp.qy, fp.qz);

      bool solved = false;

      for (int attempt = 0; attempt < num_retry_attempts && !solved;
           ++attempt) {
        // Generate random initial guess
        Eigen::VectorXd random_guess(num_ik_joints);
        for (int j = 0; j < num_ik_joints; ++j) {
          std::uniform_real_distribution<> dis(joint_min[j], joint_max[j]);
          random_guess(j) = dis(gen);
        }

        auto result = ik.solve(position, orientation, random_guess);
        if (result.success) {
          solved = true;
          recovered_count++;
        }
      }

      if (!solved) {
        true_failures.push_back(fp);
      }

      if ((f + 1) % retry_progress_step == 0) {
        std::cout << "\rRetry progress: " << std::setw(3)
                  << ((f + 1) * 100 / failures.size()) << "% ("
                  << recovered_count << " recovered)" << std::flush;
      }
    }

    std::cout << "\r                                                         "
              << std::endl;
    std::cout << "Recovered " << recovered_count << " / " << failures.size()
              << " failed poses with random guesses!" << std::endl;
    std::cout << "True failures: " << true_failures.size() << std::endl;
  }

  // Save TRUE failures (after retry) to CSV
  std::string csv_filename = "ik_true_failures.csv";
  std::ofstream csv_file(csv_filename);
  if (csv_file.is_open()) {
    csv_file
        << "orig_q1,orig_q2,orig_q3,orig_q4,orig_q5,orig_q6,x,y,z,qw,qx,qy,qz,"
           "iterations,final_error,solve_time_ms,failure_reason"
        << std::endl;
    for (const auto &tp : true_failures) {
      csv_file << std::fixed << std::setprecision(6);
      for (int j = 0; j < num_ik_joints; ++j) {
        csv_file << tp.original_joints[j] << ",";
      }
      csv_file << tp.x << "," << tp.y << "," << tp.z << "," << tp.qw << ","
               << tp.qx << "," << tp.qy << "," << tp.qz << "," << tp.iterations
               << "," << tp.final_error << "," << tp.solve_time_ms << ",\""
               << tp.failure_reason << "\"" << std::endl;
    }
    csv_file.close();
    std::cout << "Saved " << true_failures.size() << " true failures to "
              << csv_filename << std::endl;
  }

  // Save all results for analysis
  std::string all_csv = "ik_all_results.csv";
  std::ofstream all_file(all_csv);
  if (all_file.is_open()) {
    all_file
        << "orig_q1,orig_q2,orig_q3,orig_q4,orig_q5,orig_q6,x,y,z,qw,qx,qy,qz,"
           "success,iterations,final_error,solve_time_ms"
        << std::endl;
    for (const auto &tp : all_results) {
      all_file << std::fixed << std::setprecision(6);
      for (int j = 0; j < num_ik_joints; ++j) {
        all_file << tp.original_joints[j] << ",";
      }
      all_file << tp.x << "," << tp.y << "," << tp.z << "," << tp.qw << ","
               << tp.qx << "," << tp.qy << "," << tp.qz << ","
               << (tp.success ? 1 : 0) << "," << tp.iterations << ","
               << tp.final_error << "," << tp.solve_time_ms << std::endl;
    }
    all_file.close();
    std::cout << "Saved all " << all_results.size() << " results to " << all_csv
              << std::endl;
  }

  // Print statistics
  std::cout << "\n========================================" << std::endl;
  std::cout << "IK Convergence Test Results" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "Total time: " << std::fixed << std::setprecision(2)
            << total_time << " s" << std::endl;
  std::cout << "Total IK tests: " << total_tests << std::endl;
  std::cout << "  Successes: " << total_success << " ("
            << (100.0 * total_success / total_tests) << "%)" << std::endl;
  std::cout << "  Failures: " << total_failures << " ("
            << (100.0 * total_failures / total_tests) << "%)" << std::endl;
  std::cout << "Average solve time: "
            << (total_solve_time / std::max(1, total_tests)) << " ms"
            << std::endl;
  std::cout << "Max solve time: " << max_solve_time << " ms" << std::endl;
  std::cout << "========================================" << std::endl;

  // Save statistics to file
  std::string stats_filename = "ik_statistics.txt";
  std::ofstream stats_file(stats_filename);
  if (stats_file.is_open()) {
    stats_file << "IK Convergence Test Results (FK-Based)" << std::endl;
    stats_file << "=======================================" << std::endl;
    stats_file << "Method: Random joint configs -> FK -> IK" << std::endl;
    stats_file << "Samples: " << num_samples << std::endl;
    stats_file << "Total time: " << total_time << " s" << std::endl;
    stats_file << "Total tests: " << total_tests << std::endl;
    stats_file << "Successes: " << total_success << std::endl;
    stats_file << "Failures: " << total_failures << std::endl;
    stats_file << "Success rate: " << (100.0 * total_success / total_tests)
               << "%" << std::endl;
    stats_file << "Average solve time: "
               << (total_solve_time / std::max(1, total_tests)) << " ms"
               << std::endl;
    stats_file << "Max solve time: " << max_solve_time << " ms" << std::endl;
    stats_file.close();
    std::cout << "Saved statistics to " << stats_filename << std::endl;
  }

  return total_failures > 0 ? 1 : 0;
}
