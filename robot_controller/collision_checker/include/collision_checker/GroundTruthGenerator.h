#ifndef COLLISION_CHECKER_GROUND_TRUTH_GENERATOR_H
#define COLLISION_CHECKER_GROUND_TRUTH_GENERATOR_H

#include "CollisionChecker.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fcl/fcl.h>
#include <kinematics/ForwardKinematics.h>
#include <map>
#include <memory>
#include <robot_model/RobotModel.h>
#include <vector>

namespace collision_checker {

/**
 * @brief Generates ground truth collision data from URDF model
 *
 * This class extracts collision geometry information from the URDF
 * and can generate various representations for collision checking.
 */
class GroundTruthGenerator : public CollisionChecker {
public:
  GroundTruthGenerator();
  ~GroundTruthGenerator();

  /**
   * @brief Extract collision geometry information from URDF
   *
   * Iterates through all links and extracts collision mesh data,
   * including geometry types, dimensions, and transforms.
   */
  void extractCollisionGeometry();

  /**
   * @brief Print collision information for all links
   *
   * Displays collision geometry details for debugging and verification.
   */
  void printCollisionInfo() const;

  /**
   * @brief Get number of links with collision geometry
   * @return Number of links that have collision elements
   */
  size_t getNumCollisionLinks() const;

  /**
   * @brief Get list of link names with collision geometry
   * @return Vector of link names that have collision elements
   */
  std::vector<std::string> getCollisionLinkNames() const;

  // ========== YOUR COLLISION GENERATION METHODS ==========

  /**
   * @brief Generate FCL collision objects from URDF collision geometries
   *
   * Creates FCL collision objects for all extracted collision geometries.
   * Call extractCollisionGeometry() first.
   */
  void generateFCLCollisionObjects();

  /**
   * @brief Get FCL collision objects for a specific link
   * @param link_name Name of the link
   * @return Vector of FCL collision object pointers, empty if link not found
   */
  std::vector<fcl::CollisionObjectd *>
  getFCLObjectsForLink(const std::string &link_name) const;

  /**
   * @brief Get all FCL collision objects
   * @return Vector of all FCL collision object pointers
   */
  std::vector<fcl::CollisionObjectd *> getAllFCLObjects() const;

  /**
   * @brief Check collision between two links
   * @param link1 First link name
   * @param link2 Second link name
   * @return true if collision detected, false otherwise
   */
  bool checkCollision(const std::string &link1, const std::string &link2) const;

  // ========== COLLISION PAIR FILTERING ==========

  /**
   * @brief Pair of link names for collision checking
   */
  struct LinkPair {
    std::string link1;
    std::string link2;

    LinkPair(const std::string &l1, const std::string &l2) {
      // Always store in alphabetical order for consistent comparison
      if (l1 < l2) {
        link1 = l1;
        link2 = l2;
      } else {
        link1 = l2;
        link2 = l1;
      }
    }

    bool operator<(const LinkPair &other) const {
      if (link1 != other.link1)
        return link1 < other.link1;
      return link2 < other.link2;
    }
  };

  /**
   * @brief Generate allowed collision pairs based on kinematic structure
   * @param min_separation Minimum kinematic distance to allow checking
   * (default: 2)
   *
   * Creates list of link pairs that should be checked for collision.
   * Excludes adjacent links and links < min_separation joints apart.
   */
  void generateAllowedCollisionPairs(int min_separation = 2);

  /**
   * @brief Get all allowed collision pairs
   */
  const std::set<LinkPair> &getAllowedPairs() const { return allowed_pairs_; }

  /**
   * @brief Check robot self-collision at given joint configuration
   * @param q Joint angles (for future FK integration, currently unused)
   * @return true if collision detected, false otherwise
   *
   * Checks all allowed collision pairs for collisions.
   * Note: Currently checks at default/current transforms (no FK yet).
   */
  bool isSelfCollision(const Eigen::VectorXd &q);

  /**
   * @brief Get list of currently colliding link pairs
   * @param q Joint angles for FK
   * @return Vector of colliding link pairs (link1, link2)
   */
  std::vector<std::pair<std::string, std::string>>
  getCollidingPairs(const Eigen::VectorXd &q);

  /**
   * @brief Check if a pair should be collision-checked
   */
  bool shouldCheckPair(const std::string &link1,
                       const std::string &link2) const;

  /**
   * @brief Get kinematic distance between two links (for debugging)
   * @return Number of joints between links, -1 if no common ancestor
   */
  int getKinematicDistancePublic(const std::string &link1,
                                 const std::string &link2) const {
    return getKinematicDistance(link1, link2);
  }

  /**
   * @brief Clean up FCL objects (call in destructor or before regenerating)
   */
  void clearFCLObjects();

private:
  struct CollisionGeometryInfo {
    std::string link_name;
    std::string geometry_type;      // box, cylinder, sphere, mesh
    std::vector<double> dimensions; // size parameters
    std::string mesh_filename;      // for mesh geometries
    // Transform information
    double origin_xyz[3];
    double origin_rpy[3];
  };

  std::map<std::string, std::vector<CollisionGeometryInfo>> collision_data_;

  // FCL collision objects storage (link_name -> vector of FCL objects)
  std::map<std::string, std::vector<fcl::CollisionObjectd *>> fcl_objects_;

  // Local collision geometry transforms (link_name -> vector of local
  // transforms) These are the collision origin offsets relative to the link
  // frame
  std::map<std::string, std::vector<fcl::Transform3d>> fcl_local_transforms_;

  // Forward kinematics
  std::shared_ptr<kinematics::ForwardKinematics> fk_;
  std::shared_ptr<robot_model::RobotModel> robot_model_ptr_;

  // Collision pair filtering
  std::set<LinkPair> allowed_pairs_;
  std::map<std::string, std::string> link_to_parent_; // link -> parent link
  std::map<std::string, std::vector<std::string>>
      link_to_children_; // link -> child links

  // Helper methods
  void buildKinematicTree();
  int getKinematicDistance(const std::string &link1,
                           const std::string &link2) const;
  std::vector<std::string> getPathToRoot(const std::string &link) const;
};

} // namespace collision_checker

#endif // COLLISION_CHECKER_GROUND_TRUTH_GENERATOR_H
