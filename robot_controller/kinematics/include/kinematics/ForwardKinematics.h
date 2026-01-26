#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <map>
#include <string>
#include <urdf_parser/urdf_parser.h>
#include <vector>

namespace robot_model {
class RobotModel; // Forward declaration
}

namespace kinematics {

class ForwardKinematics {
public:
  ForwardKinematics();
  ~ForwardKinematics();

  /**
   * @brief Initialize FK with a RobotModel (uses pre-computed joint map)
   */
  bool init(const robot_model::RobotModel &robot_model);

  void printRobotStructure() const;

  /**
   * @brief Compute transforms for all links given joint configuration
   * @param q Joint angles (size should match number of joints)
   * @param transforms Output map of link_name -> Isometry3d
   * @return true if successful
   */
  bool
  computeLinkTransforms(const Eigen::VectorXd &q,
                        std::map<std::string, Eigen::Isometry3d> &transforms);

  /**
   * @brief Get transform for specific link
   * @param link_name Name of the link
   * @param q Joint angles
   * @return Transform of the link in world frame
   */
  Eigen::Isometry3d getLinkTransform(const std::string &link_name,
                                     const Eigen::VectorXd &q);

  /**
   * @brief Get joint names in order
   */
  const std::vector<std::string> &getJointNames() const {
    return *joint_names_;
  }

  /**
   * @brief Get number of joints
   */
  int getNumJoints() const {
    return joint_names_ ? static_cast<int>(joint_names_->size()) : 0;
  }
  bool getJacobian(const Eigen::VectorXd &q,
                   const std::string &end_effector_link,
                   Eigen::Matrix<double, 6, Eigen::Dynamic> &J_out,
                   int numIKJoints = 6);

private:
  void printLinkRecursive(urdf::LinkConstSharedPtr link, int level) const;

  /**
   * @brief Recursively compute link transforms
   */
  void computeLinkTransformsRecursive(
      urdf::LinkConstSharedPtr link, const Eigen::Isometry3d &parent_transform,
      const Eigen::VectorXd &q,
      std::map<std::string, Eigen::Isometry3d> &transforms);

  urdf::ModelInterfaceSharedPtr model_;
  const std::map<std::string, int>
      *joint_indices_;                          // pointer to RobotModel's map
  const std::vector<std::string> *joint_names_; // pointer to RobotModel's names
};

} // namespace kinematics

#endif // FORWARD_KINEMATICS_H
