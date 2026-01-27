#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include <map>
#include <memory>
#include <string>
#include <urdf_parser/urdf_parser.h>
#include <vector>

namespace robot_model {

class RobotModel {
public:
  RobotModel();
  ~RobotModel();

  bool loadURDF(const std::string &file_path);
  urdf::ModelInterfaceSharedPtr getModel() const;
  void printInfo() const;

  // Joint map accessors (computed once during loadURDF)
  const std::map<std::string, int> &getJointIndices() const {
    return joint_indices_;
  }
  const std::vector<std::string> &getJointNames() const { return joint_names_; }
  int getNumJoints() const { return static_cast<int>(joint_names_.size()); }

private:
  void buildJointMap();
  void buildJointMapRecursive(urdf::LinkConstSharedPtr link, int &index);

  urdf::ModelInterfaceSharedPtr model_;
  std::map<std::string, int> joint_indices_; // joint_name -> index in q
  std::vector<std::string> joint_names_;     // ordered list of joint names
};

} // namespace robot_model

#endif // ROBOT_MODEL_H
