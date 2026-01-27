/**
 * @brief Simple test for the Real-Time Velocity Controller
 */

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

#include "collision_checker/GroundTruthGenerator.h"
#include "control/RealTimeController.h"
#include "robot_model/RobotModel.h"
#include <Eigen/Dense>

static volatile bool g_running = true;

void signalHandler(int) { g_running = false; }

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <urdf_path>" << std::endl;
    return 1;
  }

  std::string urdf_path = argv[1];
  std::cout << "Loading URDF: " << urdf_path << std::endl;

  std::signal(SIGINT, signalHandler);

  // Load robot model
  robot_model::RobotModel model;
  if (!model.loadURDF(urdf_path)) {
    std::cerr << "Failed to load URDF" << std::endl;
    return 1;
  }

  // Initialize collision checker
  collision_checker::GroundTruthGenerator collision;
  if (collision.loadURDF(urdf_path)) {
    collision.extractCollisionGeometry();
    collision.generateFCLCollisionObjects();
    collision.generateAllowedCollisionPairs(2);
  }

  // Create controller
  control::RealTimeController controller;
  if (!controller.init(model, &collision)) {
    std::cerr << "Failed to initialize controller" << std::endl;
    return 1;
  }

  controller.setControlFrequency(100);
  controller.setWatchdogTimeout(0.5);

  // Output callback
  controller.setOutputCallback([](const Eigen::VectorXd &q_dot) {
    std::cout << "\rq_dot: [";
    for (int i = 0; i < q_dot.size(); ++i) {
      std::cout << std::fixed << std::setprecision(3) << q_dot[i];
      if (i < q_dot.size() - 1)
        std::cout << ", ";
    }
    std::cout << "] rad/s" << std::flush;
  });

  controller.start();

  std::cout << "\n=== Real-Time Controller Test ===" << std::endl;
  std::cout << "Sending 10 cm/s in X for 3s..." << std::endl;

  Eigen::VectorXd v_cart(6);
  v_cart << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0;

  auto start = std::chrono::steady_clock::now();
  while (g_running) {
    double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();

    if (elapsed < 3.0) {
      controller.setCartesianVelocity(v_cart);
    } else if (elapsed < 4.0) {
      controller.setHoldPosition();
    } else {
      break;
    }

    if (controller.isFaulted()) {
      std::cout << "\nFault: " << controller.getFaultReason() << std::endl;
      controller.clearFault();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  controller.stop();
  std::cout << "\nTest complete!" << std::endl;
  return 0;
}
