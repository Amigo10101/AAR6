#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include <Eigen/Dense>
#include <fcl/fcl.h>
#include <map>
#include <string>
#include <urdf_parser/urdf_parser.h>
#include <vector>

namespace kinematics {

class ForwardKinematics {
public:
  ForwardKinematics();
  ~ForwardKinematics();

  bool init(urdf::ModelInterfaceSharedPtr model);
  void printRobotStructure() const;

  /**
   * @brief Compute transforms for all links given joint configuration
   * @param q Joint angles (size should match number of joints)
   * @param transforms Output map of link_name -> Transform3d
   * @return true if successful
   */
  bool
  computeLinkTransforms(const Eigen::VectorXd &q,
                        std::map<std::string, fcl::Transform3d> &transforms);

  /**
   * @brief Get transform for specific link
   * @param link_name Name of the link
   * @param q Joint angles
   * @return Transform of the link in world frame
   */
  fcl::Transform3d getLinkTransform(const std::string &link_name,
                                    const Eigen::VectorXd &q);

private:
  void printLinkRecursive(urdf::LinkConstSharedPtr link, int level) const;

  /**
   * @brief Build mapping of joints to indices
   */
  void buildJointMap();

  /**
   * @brief Recursively compute link transforms
   */
  void computeLinkTransformsRecursive(
      urdf::LinkConstSharedPtr link, const fcl::Transform3d &parent_transform,
      const Eigen::VectorXd &q,
      std::map<std::string, fcl::Transform3d> &transforms);

  urdf::ModelInterfaceSharedPtr model_;
  std::map<std::string, int> joint_indices_; // joint_name -> index in q
  std::vector<std::string> joint_names_;     // ordered list of joint names
};

} // namespace kinematics

#endif // FORWARD_KINEMATICS_H
