#include "collision_checker/GroundTruthGenerator.h"
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <urdf_model/model.h>

namespace collision_checker {

GroundTruthGenerator::GroundTruthGenerator() {}

GroundTruthGenerator::~GroundTruthGenerator() { clearFCLObjects(); }

void GroundTruthGenerator::extractCollisionGeometry() {
  if (!isModelLoaded()) {
    std::cerr << "Cannot extract collision geometry: URDF model not loaded"
              << std::endl;
    return;
  }

  collision_data_.clear();

  // Iterate through all links in the model
  for (const auto &link_pair : model_->links_) {
    const std::string &link_name = link_pair.first;
    urdf::LinkSharedPtr link = link_pair.second;

    // Check if link has collision geometry
    if (!link->collision_array.empty()) {
      std::vector<CollisionGeometryInfo> geometries;

      for (const auto &collision : link->collision_array) {
        CollisionGeometryInfo info;
        info.link_name = link_name;

        // Extract origin/transform
        info.origin_xyz[0] = collision->origin.position.x;
        info.origin_xyz[1] = collision->origin.position.y;
        info.origin_xyz[2] = collision->origin.position.z;

        collision->origin.rotation.getRPY(
            info.origin_rpy[0], info.origin_rpy[1], info.origin_rpy[2]);

        // Extract geometry information based on type
        if (collision->geometry) {
          switch (collision->geometry->type) {
          case urdf::Geometry::BOX: {
            auto box =
                std::dynamic_pointer_cast<urdf::Box>(collision->geometry);
            info.geometry_type = "box";
            info.dimensions = {box->dim.x, box->dim.y, box->dim.z};
            break;
          }
          case urdf::Geometry::CYLINDER: {
            auto cylinder =
                std::dynamic_pointer_cast<urdf::Cylinder>(collision->geometry);
            info.geometry_type = "cylinder";
            info.dimensions = {cylinder->radius, cylinder->length};
            break;
          }
          case urdf::Geometry::SPHERE: {
            auto sphere =
                std::dynamic_pointer_cast<urdf::Sphere>(collision->geometry);
            info.geometry_type = "sphere";
            info.dimensions = {sphere->radius};
            break;
          }
          case urdf::Geometry::MESH: {
            auto mesh =
                std::dynamic_pointer_cast<urdf::Mesh>(collision->geometry);
            info.geometry_type = "mesh";
            info.mesh_filename = mesh->filename;
            info.dimensions = {mesh->scale.x, mesh->scale.y, mesh->scale.z};
            break;
          }
          default:
            info.geometry_type = "unknown";
            break;
          }
        }

        geometries.push_back(info);
      }

      collision_data_[link_name] = geometries;
    }
  }

  std::cout << "Extracted collision geometry from " << collision_data_.size()
            << " links" << std::endl;

  // Initialize Forward Kinematics with RobotModel
  robot_model_ptr_ = std::make_shared<robot_model::RobotModel>();
  if (!urdf_path_.empty() && robot_model_ptr_->loadURDF(urdf_path_)) {
    fk_ = std::make_shared<kinematics::ForwardKinematics>();
    if (fk_->init(*robot_model_ptr_)) {
      std::cout << "Forward Kinematics initialized" << std::endl;
    }
  } else {
    std::cerr << "Warning: Could not initialize FK - urdf_path_ not set"
              << std::endl;
  }
}

void GroundTruthGenerator::printCollisionInfo() const {
  if (collision_data_.empty()) {
    std::cout
        << "No collision data extracted. Call extractCollisionGeometry() first."
        << std::endl;
    return;
  }

  std::cout << "\n=== Collision Geometry Information ===" << std::endl;
  std::cout << "Total links with collision: " << collision_data_.size() << "\n"
            << std::endl;

  for (const auto &link_data : collision_data_) {
    const std::string &link_name = link_data.first;
    const auto &geometries = link_data.second;

    std::cout << "Link: " << link_name << " (" << geometries.size()
              << " collision objects)" << std::endl;

    for (size_t i = 0; i < geometries.size(); ++i) {
      const auto &geom = geometries[i];
      std::cout << "  [" << i << "] Type: " << geom.geometry_type << std::endl;

      std::cout << "      Origin: xyz=[" << std::fixed << std::setprecision(4)
                << geom.origin_xyz[0] << ", " << geom.origin_xyz[1] << ", "
                << geom.origin_xyz[2] << "]" << std::endl;

      std::cout << "              rpy=[" << geom.origin_rpy[0] << ", "
                << geom.origin_rpy[1] << ", " << geom.origin_rpy[2] << "]"
                << std::endl;

      if (geom.geometry_type == "mesh") {
        std::cout << "      Mesh: " << geom.mesh_filename << std::endl;
        std::cout << "      Scale: [" << geom.dimensions[0] << ", "
                  << geom.dimensions[1] << ", " << geom.dimensions[2] << "]"
                  << std::endl;
      } else if (!geom.dimensions.empty()) {
        std::cout << "      Dimensions: [";
        for (size_t j = 0; j < geom.dimensions.size(); ++j) {
          std::cout << geom.dimensions[j];
          if (j < geom.dimensions.size() - 1)
            std::cout << ", ";
        }
        std::cout << "]" << std::endl;
      }
    }
    std::cout << std::endl;
  }
}

size_t GroundTruthGenerator::getNumCollisionLinks() const {
  return collision_data_.size();
}

std::vector<std::string> GroundTruthGenerator::getCollisionLinkNames() const {
  std::vector<std::string> names;
  for (const auto &link_data : collision_data_) {
    names.push_back(link_data.first);
  }
  return names;
}

// ==========  COLLISION GENERATION IMPLEMENTATION ==========

void GroundTruthGenerator::generateFCLCollisionObjects() {
  // Clear any existing FCL objects first
  clearFCLObjects();

  if (collision_data_.empty()) {
    std::cerr
        << "No collision data available. Call extractCollisionGeometry() first."
        << std::endl;
    return;
  }

  for (const auto &link_data : collision_data_) {
    const std::string &link_name = link_data.first;
    const auto &geometries = link_data.second;

    std::vector<fcl::CollisionObjectd *> link_objects;
    std::vector<fcl::Transform3d> local_transforms;

    for (const auto &geom : geometries) {
      std::shared_ptr<fcl::CollisionGeometryd> fcl_geom;

      // Create FCL geometry based on type
      if (geom.geometry_type == "box") {
        fcl_geom = std::make_shared<fcl::Boxd>(
            geom.dimensions[0], geom.dimensions[1], geom.dimensions[2]);

      } else if (geom.geometry_type == "sphere") {
        fcl_geom = std::make_shared<fcl::Sphered>(geom.dimensions[0]);

      } else if (geom.geometry_type == "cylinder") {
        fcl_geom = std::make_shared<fcl::Cylinderd>(geom.dimensions[0],
                                                    geom.dimensions[1]);

      } else if (geom.geometry_type == "mesh") {
        // Resolve mesh path relative to URDF directory
        std::string mesh_path = geom.mesh_filename;
        if (mesh_path.find("../") == 0 || mesh_path.find("./") == 0 ||
            mesh_path[0] != '/') {
          // Relative path - prepend URDF directory
          mesh_path = urdf_dir_ + mesh_path;
        }

        // Load mesh with Assimp
        Assimp::Importer importer;
        const aiScene *scene = importer.ReadFile(
            mesh_path, aiProcess_Triangulate | aiProcess_JoinIdenticalVertices);

        if (scene && scene->HasMeshes()) {
          // Create BVH model for mesh
          auto bvh = std::make_shared<fcl::BVHModel<fcl::OBBRSSd>>();
          bvh->beginModel();

          // Process all meshes in the scene
          for (unsigned int m = 0; m < scene->mNumMeshes; ++m) {
            const aiMesh *mesh = scene->mMeshes[m];

            // Extract vertices
            std::vector<fcl::Vector3d> vertices;
            for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
              const aiVector3D &vertex = mesh->mVertices[v];
              // Apply scale
              vertices.push_back(fcl::Vector3d(vertex.x * geom.dimensions[0],
                                               vertex.y * geom.dimensions[1],
                                               vertex.z * geom.dimensions[2]));
            }

            // Extract triangle indices
            std::vector<fcl::Triangle> triangles;
            for (unsigned int f = 0; f < mesh->mNumFaces; ++f) {
              const aiFace &face = mesh->mFaces[f];
              if (face.mNumIndices == 3) {
                triangles.push_back(fcl::Triangle(
                    face.mIndices[0], face.mIndices[1], face.mIndices[2]));
              }
            }

            // Add to BVH model
            bvh->addSubModel(vertices, triangles);
          }

          bvh->endModel();
          fcl_geom = bvh;

          std::cout << "Loaded mesh for " << link_name << ": "
                    << geom.mesh_filename << " ("
                    << scene->mMeshes[0]->mNumVertices << " vertices, "
                    << scene->mMeshes[0]->mNumFaces << " faces)" << std::endl;
        } else {
          std::cerr << "Failed to load mesh: " << geom.mesh_filename
                    << std::endl;
          if (importer.GetErrorString()) {
            std::cerr << "  Assimp error: " << importer.GetErrorString()
                      << std::endl;
          }
          continue;
        }
      }

      if (fcl_geom) {
        // Create transform from origin_xyz and origin_rpy
        fcl::Matrix3d rotation;
        double roll = geom.origin_rpy[0];
        double pitch = geom.origin_rpy[1];
        double yaw = geom.origin_rpy[2];

        // RPY to rotation matrix (ZYX convention)
        double cr = cos(roll), sr = sin(roll);
        double cp = cos(pitch), sp = sin(pitch);
        double cy = cos(yaw), sy = sin(yaw);

        rotation << cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
            sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp,
            cp * sr, cp * cr;

        fcl::Vector3d translation(geom.origin_xyz[0], geom.origin_xyz[1],
                                  geom.origin_xyz[2]);

        fcl::Transform3d local_transform = fcl::Transform3d::Identity();
        local_transform.linear() = rotation;
        local_transform.translation() = translation;

        // Create collision object with identity transform (we'll update it
        // later)
        auto *collision_obj =
            new fcl::CollisionObjectd(fcl_geom, fcl::Transform3d::Identity());
        link_objects.push_back(collision_obj);
        local_transforms.push_back(local_transform);
      }
    }

    if (!link_objects.empty()) {
      fcl_objects_[link_name] = link_objects;
      fcl_local_transforms_[link_name] = local_transforms;
    }
  }

  std::cout << "Generated FCL collision objects for " << fcl_objects_.size()
            << " links" << std::endl;
}

std::vector<fcl::CollisionObjectd *>
GroundTruthGenerator::getFCLObjectsForLink(const std::string &link_name) const {

  auto it = fcl_objects_.find(link_name);
  if (it != fcl_objects_.end()) {
    return it->second;
  }
  return {};
}

std::vector<fcl::CollisionObjectd *>
GroundTruthGenerator::getAllFCLObjects() const {
  std::vector<fcl::CollisionObjectd *> all_objects;
  for (const auto &link_objects : fcl_objects_) {
    all_objects.insert(all_objects.end(), link_objects.second.begin(),
                       link_objects.second.end());
  }
  return all_objects;
}

bool GroundTruthGenerator::checkCollision(const std::string &link1,
                                          const std::string &link2) const {

  auto objects1 = getFCLObjectsForLink(link1);
  auto objects2 = getFCLObjectsForLink(link2);

  if (objects1.empty() || objects2.empty()) {
    return false;
  }

  fcl::CollisionRequestd request;
  fcl::CollisionResultd result;

  for (auto obj1 : objects1) {
    for (auto obj2 : objects2) {
      result.clear();
      fcl::collide(obj1, obj2, request, result);
      if (result.isCollision()) {
        return true;
      }
    }
  }

  return false;
}

void GroundTruthGenerator::clearFCLObjects() {
  // Clean up all FCL objects
  for (auto &link_objects : fcl_objects_) {
    for (auto obj : link_objects.second) {
      delete obj;
    }
  }
  fcl_objects_.clear();
  fcl_local_transforms_.clear();
}

// ========== COLLISION PAIR FILTERING IMPLEMENTATION ==========

void GroundTruthGenerator::buildKinematicTree() {
  link_to_parent_.clear();
  link_to_children_.clear();

  if (!isModelLoaded()) {
    std::cerr << "Cannot build kinematic tree: URDF model not loaded"
              << std::endl;
    return;
  }

  // Build parent-child relationships from joints
  for (const auto &joint_pair : model_->joints_) {
    const auto &joint = joint_pair.second;

    std::string parent_link = joint->parent_link_name;
    std::string child_link = joint->child_link_name;

    // Store parent relationship
    link_to_parent_[child_link] = parent_link;

    // Store child relationship
    link_to_children_[parent_link].push_back(child_link);
  }

  std::cout << "Built kinematic tree: " << link_to_parent_.size()
            << " parent relationships" << std::endl;
}

std::vector<std::string>
GroundTruthGenerator::getPathToRoot(const std::string &link) const {
  std::vector<std::string> path;
  std::string current = link;

  while (true) {
    path.push_back(current);
    auto it = link_to_parent_.find(current);
    if (it == link_to_parent_.end()) {
      break; // Reached root
    }
    current = it->second;
  }

  return path;
}

int GroundTruthGenerator::getKinematicDistance(const std::string &link1,
                                               const std::string &link2) const {

  // Get paths from both links to root
  auto path1 = getPathToRoot(link1);
  auto path2 = getPathToRoot(link2);

  // Find common ancestor
  std::set<std::string> ancestors1(path1.begin(), path1.end());

  std::string common_ancestor;
  for (const auto &link : path2) {
    if (ancestors1.count(link)) {
      common_ancestor = link;
      break;
    }
  }

  if (common_ancestor.empty()) {
    return -1; // No common ancestor (shouldn't happen in valid URDF)
  }

  // Calculate distance: dist(link1->ancestor) + dist(link2->ancestor)
  int dist1 = 0;
  for (const auto &link : path1) {
    if (link == common_ancestor)
      break;
    dist1++;
  }

  int dist2 = 0;
  for (const auto &link : path2) {
    if (link == common_ancestor)
      break;
    dist2++;
  }

  return dist1 + dist2;
}

void GroundTruthGenerator::generateAllowedCollisionPairs(int min_separation) {
  allowed_pairs_.clear();

  if (collision_data_.empty()) {
    std::cerr << "No collision data. Call extractCollisionGeometry() first."
              << std::endl;
    return;
  }

  // Build kinematic tree
  buildKinematicTree();

  // Get all collision link names
  auto collision_links = getCollisionLinkNames();

  // Generate all possible pairs
  for (size_t i = 0; i < collision_links.size(); ++i) {
    for (size_t j = i + 1; j < collision_links.size(); ++j) {
      const std::string &link1 = collision_links[i];
      const std::string &link2 = collision_links[j];

      // Calculate kinematic distance
      int distance = getKinematicDistance(link1, link2);

      // Only add if distance >= min_separation
      if (distance >= min_separation) {
        allowed_pairs_.insert(LinkPair(link1, link2));
      }
    }
  }

  // Manually add specific collision pairs that should be checked even though
  // they're kinematically adjacent (can collide in extreme poses)
  allowed_pairs_.insert(LinkPair("L4", "L6"));

  std::cout << "\nGenerated allowed collision pairs:" << std::endl;
  std::cout << "  Total collision links: " << collision_links.size()
            << std::endl;
  std::cout << "  Possible pairs: "
            << (collision_links.size() * (collision_links.size() - 1)) / 2
            << std::endl;
  std::cout << "  Allowed pairs (min_separation=" << min_separation
            << "): " << allowed_pairs_.size() << std::endl;
  std::cout << "  Excluded pairs: "
            << ((collision_links.size() * (collision_links.size() - 1)) / 2 -
                allowed_pairs_.size())
            << std::endl;
}

bool GroundTruthGenerator::isSelfCollision(const Eigen::VectorXd &q) {
  if (fcl_objects_.empty()) {
    std::cerr << "No FCL objects. Call generateFCLCollisionObjects() first."
              << std::endl;
    return false;
  }

  if (allowed_pairs_.empty()) {
    std::cerr << "No allowed pairs. Call generateAllowedCollisionPairs() first."
              << std::endl;
    return false;
  }

  // Step 1: Compute FK and update transforms
  if (fk_) {
    std::map<std::string, Eigen::Isometry3d> link_transforms_eigen;
    fk_->computeLinkTransforms(q, link_transforms_eigen);

    // Update FCL object transforms: world_transform = link_transform *
    // local_collision_offset
    for (const auto &entry : fcl_objects_) {
      const std::string &link_name = entry.first;
      auto link_it = link_transforms_eigen.find(link_name);
      auto local_it = fcl_local_transforms_.find(link_name);

      if (link_it != link_transforms_eigen.end() &&
          local_it != fcl_local_transforms_.end()) {
        const auto &objects = entry.second;
        const auto &local_trans = local_it->second;

        // Convert Eigen::Isometry3d to fcl::Transform3d
        fcl::Transform3d link_transform_fcl = fcl::Transform3d::Identity();
        link_transform_fcl.linear() = link_it->second.linear();
        link_transform_fcl.translation() = link_it->second.translation();

        for (size_t i = 0; i < objects.size() && i < local_trans.size(); ++i) {
          // Compose: link_transform * local_collision_offset
          fcl::Transform3d world_transform =
              link_transform_fcl * local_trans[i];
          objects[i]->setTransform(world_transform);
        }
      }
    }
  }

  // Step 2: Check collisions for all allowed pairs
  bool collision_detected = false;
  int collision_count = 0;
  std::vector<std::string> colliding_pairs;

  for (const auto &pair : allowed_pairs_) {
    if (checkCollision(pair.link1, pair.link2)) {
      collision_detected = true;
      collision_count++;
      colliding_pairs.push_back(pair.link1 + " <-> " + pair.link2);
    }
  }

  // Print details if collision found
  if (collision_detected) {
    std::cout << "\nCollision Details:" << std::endl;
    std::cout << "  Total colliding pairs: " << collision_count << std::endl;
    std::cout << "  Colliding link pairs:" << std::endl;
    for (const auto &pair_str : colliding_pairs) {
      std::cout << "    - " << pair_str << std::endl;
    }
  }

  return collision_detected;
}

std::vector<std::pair<std::string, std::string>>
GroundTruthGenerator::getCollidingPairs(const Eigen::VectorXd &q) {
  std::vector<std::pair<std::string, std::string>> colliding_pairs;

  if (fcl_objects_.empty() || allowed_pairs_.empty()) {
    return colliding_pairs;
  }

  // Step 1: Compute FK and update transforms
  if (fk_) {
    std::map<std::string, Eigen::Isometry3d> link_transforms_eigen;
    fk_->computeLinkTransforms(q, link_transforms_eigen);

    // Update FCL object transforms: world_transform = link_transform *
    // local_collision_offset
    for (const auto &entry : fcl_objects_) {
      const std::string &link_name = entry.first;
      auto link_it = link_transforms_eigen.find(link_name);
      auto local_it = fcl_local_transforms_.find(link_name);

      if (link_it != link_transforms_eigen.end() &&
          local_it != fcl_local_transforms_.end()) {
        const auto &objects = entry.second;
        const auto &local_trans = local_it->second;

        // Convert Eigen::Isometry3d to fcl::Transform3d
        fcl::Transform3d link_transform_fcl = fcl::Transform3d::Identity();
        link_transform_fcl.linear() = link_it->second.linear();
        link_transform_fcl.translation() = link_it->second.translation();

        for (size_t i = 0; i < objects.size() && i < local_trans.size(); ++i) {
          fcl::Transform3d world_transform =
              link_transform_fcl * local_trans[i];
          objects[i]->setTransform(world_transform);
        }
      }
    }
  }

  // Step 2: Check collisions for all allowed pairs
  for (const auto &pair : allowed_pairs_) {
    if (checkCollision(pair.link1, pair.link2)) {
      colliding_pairs.push_back({pair.link1, pair.link2});
    }
  }

  return colliding_pairs;
}

bool GroundTruthGenerator::shouldCheckPair(const std::string &link1,
                                           const std::string &link2) const {
  LinkPair pair(link1, link2);
  return allowed_pairs_.count(pair) > 0;
}

} // namespace collision_checker
