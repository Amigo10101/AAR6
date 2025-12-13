#include "collision_checker/GroundTruthGenerator.h"
#include <iostream>
#include <string>
#include <urdf_model/model.h>

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <urdf_file>" << std::endl;
    return 1;
  }

  std::string urdf_path = argv[1];

  // Create ground truth generator
  collision_checker::GroundTruthGenerator generator;

  // Load URDF file
  std::cout << "Loading URDF from: " << urdf_path << std::endl;
  if (!generator.loadURDF(urdf_path)) {
    std::cerr << "Failed to load URDF file" << std::endl;
    return 1;
  }

  // Get model information
  auto model = generator.getModel();
  std::cout << "\n=== URDF Model Information ===" << std::endl;
  std::cout << "Robot name: " << model->getName() << std::endl;
  std::cout << "Number of links: " << model->links_.size() << std::endl;
  std::cout << "Number of joints: " << model->joints_.size() << std::endl;

  // Extract collision geometry
  std::cout << "\nExtracting collision geometry..." << std::endl;
  generator.extractCollisionGeometry();

  // Print collision information
  generator.printCollisionInfo();

  // Generate FCL collision objects
  std::cout << "\n=== Generating FCL Collision Objects ===" << std::endl;
  generator.generateFCLCollisionObjects();

  // Get FCL object count
  auto all_fcl_objects = generator.getAllFCLObjects();
  std::cout << "Total FCL collision objects created: " << all_fcl_objects.size()
            << std::endl;

  // Generate allowed collision pairs
  generator.generateAllowedCollisionPairs(
      3); // min 3 joints apart (excludes grandparents)

  // Debug: Print kinematic tree structure
  std::cout << "\n=== Debug: Kinematic Distances ===" << std::endl;
  std::cout << "L1 -> wooden_base: distance = "
            << generator.getKinematicDistancePublic("L1", "wooden_base")
            << std::endl;
  std::cout << "L1 -> base_link: distance = "
            << generator.getKinematicDistancePublic("L1", "base_link")
            << std::endl;
  std::cout << "base_link -> wooden_base: distance = "
            << generator.getKinematicDistancePublic("base_link", "wooden_base")
            << std::endl;
  std::cout << "L1 -> L2: distance = "
            << generator.getKinematicDistancePublic("L1", "L2") << std::endl;

  // Print all allowed pairs
  std::cout << "\nAll allowed collision pairs:" << std::endl;
  for (const auto &pair : generator.getAllowedPairs()) {
    std::cout << "  " << pair.link1 << " <-> " << pair.link2 << std::endl;
  }

  // Test self-collision checking
  std::cout << "\n=== Testing Self-Collision ===" << std::endl;
  Eigen::VectorXd q = Eigen::VectorXd::Zero(6); // 6 DOF, all joints at 0
  bool collision = generator.isSelfCollision(q);
  std::cout << "Self-collision at zero configuration: "
            << (collision ? "YES (collision detected!)" : "NO (no collision)")
            << std::endl;

  // Summary
  std::cout << "\n=== Summary ===" << std::endl;
  std::cout << "Links with collision geometry: "
            << generator.getNumCollisionLinks() << std::endl;

  auto collision_links = generator.getCollisionLinkNames();
  std::cout << "Collision link names: ";
  for (size_t i = 0; i < collision_links.size(); ++i) {
    std::cout << collision_links[i];
    if (i < collision_links.size() - 1)
      std::cout << ", ";
  }
  std::cout << std::endl;

  return 0;
}
