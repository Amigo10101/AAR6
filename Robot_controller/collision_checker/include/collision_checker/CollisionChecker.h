#ifndef COLLISION_CHECKER_COLLISION_CHECKER_H
#define COLLISION_CHECKER_COLLISION_CHECKER_H

#include <memory>
#include <string>

namespace urdf {
class ModelInterface;
typedef std::shared_ptr<ModelInterface> ModelInterfaceSharedPtr;
} // namespace urdf

namespace collision_checker {

/**
 * @brief Base class for collision checking functionality
 *
 * This class provides the interface for accessing URDF model data
 * and performing collision detection operations.
 */
class CollisionChecker {
public:
  CollisionChecker();
  virtual ~CollisionChecker();

  /**
   * @brief Load URDF model from file
   * @param urdf_path Path to URDF file
   * @return true if loaded successfully, false otherwise
   */
  bool loadURDF(const std::string &urdf_path);

  /**
   * @brief Load URDF model from string
   * @param urdf_string URDF XML string
   * @return true if loaded successfully, false otherwise
   */
  bool loadURDFFromString(const std::string &urdf_string);

  /**
   * @brief Get the loaded URDF model
   * @return Shared pointer to URDF model, or nullptr if not loaded
   */
  urdf::ModelInterfaceSharedPtr getModel() const { return model_; }

  /**
   * @brief Check if model is loaded
   * @return true if model is loaded, false otherwise
   */
  bool isModelLoaded() const { return model_ != nullptr; }

protected:
  urdf::ModelInterfaceSharedPtr model_;
  std::string urdf_dir_;  // Directory containing the URDF file
  std::string urdf_path_; // Full path to URDF file
};

} // namespace collision_checker

#endif // COLLISION_CHECKER_COLLISION_CHECKER_H
