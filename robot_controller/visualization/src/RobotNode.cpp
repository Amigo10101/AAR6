// RobotNode.cpp - Uses CollisionWrapper to avoid Qt/FCL header conflicts

#include "RobotNode.h"
#include "CollisionWrapper.h"
#include "kinematics/InverseKinematics.h"
#include "kinematics/PositionIK.h"
#include "kinematics/SqpIK.h"
#include "kinematics/VelocityIK.h"
#include "robot_model/RobotModel.h"
#include <QDebug>
#include <QFile>
#include <QFileInfo>
#include <QQmlContext>
#include <iostream>

RobotNode::RobotNode(QObject *parent) : QObject(parent) {}

RobotNode::~RobotNode() = default;

bool RobotNode::loadURDF(const QString &path) {
  std::cerr << "Loading URDF from: " << path.toStdString() << std::endl;
  urdfDir_ = QFileInfo(path).absolutePath();
  urdfPath_ = QFileInfo(path).absoluteFilePath();
  model_ = urdf::parseURDFFile(path.toStdString());
  if (!model_) {
    qWarning() << "Failed to parse URDF:" << path;
    return false;
  }

  links_.clear();
  if (model_->getRoot()) {
    std::cerr << "Root link found: " << model_->getRoot()->name << std::endl;
    processLink(model_->getRoot(), "");
  } else {
    std::cerr << "No root link found!" << std::endl;
  }
  std::cerr << "Total links processed: " << links_.size() << std::endl;

  // Initialize collision wrapper
  collisionWrapper_ = std::make_unique<CollisionWrapper>();
  if (collisionWrapper_->initialize(path.toStdString())) {
    std::cerr << "Collision checker initialized successfully" << std::endl;
  } else {
    std::cerr << "Failed to initialize collision checker" << std::endl;
  }

  // Initialize IK solver
  initializeIK(urdfPath_);

  emit linksChanged();
  return true;
}

bool RobotNode::initializeIK(const QString &urdfPath) {
  robotModel_ = std::make_unique<robot_model::RobotModel>();
  if (!robotModel_->loadURDF(urdfPath.toStdString())) {
    std::cerr << "[RobotNode] Failed to load URDF for IK" << std::endl;
    ikInitialized_ = false;
    emit ikInitializedChanged();
    return false;
  }

  ikSolver_ = std::make_unique<kinematics::InverseKinematics>();
  if (!ikSolver_->init(*robotModel_, "ee_link", 6)) {
    std::cerr << "[RobotNode] Failed to initialize IK solver" << std::endl;
    ikInitialized_ = false;
    emit ikInitializedChanged();
    return false;
  }

  // Initialize Velocity IK solver
  velocityIkSolver_ = std::make_unique<kinematics::VelocityIK>();
  if (!velocityIkSolver_->init(*robotModel_, "ee_link", 6)) {
    std::cerr << "[RobotNode] Failed to initialize Velocity IK solver"
              << std::endl;
    // Non-fatal: position IK still works
  } else {
    std::cerr << "[RobotNode] Velocity IK solver initialized successfully"
              << std::endl;

    // Set joint limits from URDF (aar6.urdf)
    Eigen::VectorXd joint_min(6), joint_max(6);
    joint_min << -3.142, -0.98, -2.0, -4.9, -2.1, -3.1; // [rad]
    joint_max << 3.142, 1.0, 1.3, 0.3, 2.1, 3.1;        // [rad]
    velocityIkSolver_->setJointLimits(joint_min, joint_max, 0.15);
  }

  // Initialize Position IK solver (for gizmo control)
  positionIkSolver_ = std::make_unique<kinematics::PositionIK>();
  if (!positionIkSolver_->init(*robotModel_, "ee_link", 6)) {
    std::cerr << "[RobotNode] Failed to initialize Position IK solver"
              << std::endl;
    // Non-fatal: velocity IK still works
  } else {
    std::cerr << "[RobotNode] Position IK solver initialized successfully"
              << std::endl;

    // Set joint limits from URDF (aar6.urdf)
    Eigen::VectorXd joint_min(6), joint_max(6);
    joint_min << -3.142, -0.98, -2.0, -4.9, -2.1, -3.1; // [rad]
    joint_max << 3.142, 1.0, 1.3, 0.3, 2.1, 3.1;        // [rad]
    positionIkSolver_->setJointLimits(joint_min, joint_max, 0.15);
    std::cerr << "[RobotNode] Joint limits configured from URDF" << std::endl;
  }

  // Initialize SQP IK solver (velocity-resolved QP with EAIK seeding)
  sqpIkSolver_ = std::make_unique<kinematics::SqpIK>();
  if (!sqpIkSolver_->init(*robotModel_, "ee_link", 6)) {
    std::cerr << "[RobotNode] Failed to initialize SQP IK solver" << std::endl;
    // Non-fatal: other solvers still work
  } else {
    std::cerr
        << "[RobotNode] SQP IK solver initialized (velocity QP + EAIK seeding)"
        << std::endl;

    // Set limits from URDF
    Eigen::VectorXd joint_min(6), joint_max(6);
    joint_min << -3.142, -0.98, -2.0, -4.9, -2.1, -3.1;
    joint_max << 3.142, 1.0, 1.3, 0.3, 2.1, 3.1;
    sqpIkSolver_->setJointLimits(joint_min, joint_max);

    // Set velocity limits [rad/s]
    Eigen::VectorXd v_max = Eigen::VectorXd::Constant(6, 1.5);
    sqpIkSolver_->setVelocityLimits(v_max);

    // Set P-gain for velocity-level control (no Kd in velocity-level)
    sqpIkSolver_->setGain(5.0);

    std::cerr << "[RobotNode] SQP IK limits and gains configured" << std::endl;
  }

  std::cerr << "[RobotNode] IK solver initialized successfully" << std::endl;
  ikInitialized_ = true;
  emit ikInitializedChanged();
  return true;
}

QVariantList RobotNode::links() const { return links_; }

QVariantMap RobotNode::solveIK(double x, double y, double z, double qw,
                               double qx, double qy, double qz,
                               const QVariantList &initialGuess) {
  QVariantMap result;
  result["success"] = false;
  result["jointAngles"] = QVariantList();
  result["iterations"] = 0;
  result["solveTimeMs"] = 0.0;
  result["error"] = 0.0;
  result["message"] = "";

  if (!ikInitialized_ || !ikSolver_) {
    result["message"] = "IK solver not initialized";
    return result;
  }

  // Convert position (QML uses mm, IK uses m)
  Eigen::Vector3d position(x, y, z);
  Eigen::Quaterniond orientation(qw, qx, qy, qz);
  orientation.normalize();

  // DEBUG: Print received values
  std::cout << "[RobotNode::solveIK] Position (m): " << position.transpose()
            << std::endl;
  std::cout << "[RobotNode::solveIK] Quaternion (w,x,y,z): " << orientation.w()
            << ", " << orientation.x() << ", " << orientation.y() << ", "
            << orientation.z() << std::endl;

  // Convert initial guess
  Eigen::VectorXd guess(initialGuess.size());
  for (int i = 0; i < initialGuess.size(); ++i) {
    guess(i) = initialGuess[i].toDouble();
  }

  // Solve IK
  kinematics::IKResult ikResult =
      ikSolver_->solve(position, orientation, guess);

  // Convert result to QVariantMap
  // Use best-effort solution for visualization even if not converged
  bool visSuccess = ikResult.success;
  if (!ikResult.success && ikResult.joint_angles.size() > 0) {
    visSuccess = true; // Force success for visualization updates
    result["message"] = result["message"].toString() + " (Best Effort)";
  }
  result["success"] = visSuccess; // CRITICAL: Update map so QML sees success

  result["iterations"] = ikResult.iterations;
  result["solveTimeMs"] = ikResult.solve_time_ms;
  result["error"] = ikResult.final_error;
  result["message"] = QString::fromStdString(ikResult.error_message);
  if (!ikResult.solver_used.empty()) {
    result["message"] = result["message"].toString() + " [" +
                        QString::fromStdString(ikResult.solver_used) + "]";
  }

  QVariantList jointAngles;
  for (int i = 0; i < ikResult.joint_angles.size(); ++i) {
    jointAngles.append(ikResult.joint_angles(i));
  }
  result["jointAngles"] = jointAngles;

  // Emit signal for QML - force success so UI updates the robot model
  emit ikSolved(jointAngles, visSuccess, ikResult.solve_time_ms);

  return result;
}

QVariantMap RobotNode::solveIKFromGizmo(const QVector3D &position,
                                        const QQuaternion &orientation,
                                        const QVariantList &currentJoints) {
  // QVector3D is in meters (from QML View3D)
  // QQuaternion uses (scalar, x, y, z) but QQuaternion constructor is (w, x, y,
  // z)
  return solveIK(position.x(), position.y(), position.z(), orientation.scalar(),
                 orientation.x(), orientation.y(), orientation.z(),
                 currentJoints);
}

QVariantList RobotNode::solveVelocityIK(const QVariantList &currentJoints,
                                        const QVector3D &linearVel,
                                        const QVector3D &angularVel) {
  QVariantList result;

  std::cerr << "[VelocityIK] linearVel=(" << linearVel.x() << ", "
            << linearVel.y() << ", " << linearVel.z() << ")" << std::endl;

  if (!velocityIkSolver_) {
    std::cerr << "[RobotNode] Velocity IK solver not initialized" << std::endl;
    for (int i = 0; i < 6; ++i) {
      result.append(0.0);
    }
    return result;
  }

  // Convert current joints to Eigen vector
  Eigen::VectorXd q_current(currentJoints.size());
  for (int i = 0; i < currentJoints.size(); ++i) {
    q_current(i) = currentJoints[i].toDouble();
  }

  // Build 6D Cartesian velocity vector [vx, vy, vz, wx, wy, wz]
  Eigen::VectorXd v_cartesian(6);
  v_cartesian << linearVel.x(), linearVel.y(), linearVel.z(), angularVel.x(),
      angularVel.y(), angularVel.z();

  // Solve for joint velocities
  Eigen::VectorXd q_dot = velocityIkSolver_->solve(q_current, v_cartesian);

  std::cerr << "[VelocityIK] q_dot=[" << q_dot.transpose() << "]" << std::endl;

  // Convert result to QVariantList
  for (int i = 0; i < q_dot.size(); ++i) {
    result.append(q_dot(i));
  }

  return result;
}

QVariantList RobotNode::solvePositionIK(const QVariantList &currentJoints,
                                        const QVector3D &targetPosition,
                                        const QQuaternion &targetOrientation,
                                        double dt) {
  QVariantList result;

  if (!positionIkSolver_) {
    std::cerr << "[RobotNode] Position IK solver not initialized" << std::endl;
    for (int i = 0; i < 6; ++i) {
      result.append(0.0);
    }
    return result;
  }

  // Convert current joints to Eigen vector
  Eigen::VectorXd q_current(currentJoints.size());
  for (int i = 0; i < currentJoints.size(); ++i) {
    q_current(i) = currentJoints[i].toDouble();
  }

  // DEBUG: Print inputs
  static int callCount = 0;
  if (callCount++ % 50 == 0) { // Every 50th call
    std::cerr << "[solvePositionIK] q_curr=[";
    for (int i = 0; i < q_current.size(); ++i) {
      std::cerr << q_current(i);
      if (i < q_current.size() - 1)
        std::cerr << ", ";
    }
    std::cerr << "]" << std::endl;
    std::cerr << "[solvePositionIK] target=(" << targetPosition.x() << ", "
              << targetPosition.y() << ", " << targetPosition.z()
              << ") dt=" << dt << std::endl;
  }

  // Convert target position to Eigen (QVector3D to Eigen::Vector3d)
  Eigen::Vector3d target_pos(targetPosition.x(), targetPosition.y(),
                             targetPosition.z());

  // Convert target orientation (QQuaternion to Eigen::Quaterniond)
  // NOTE: Gizmo orientation may not be synced correctly to robot EE.
  // The PositionIK solver has error saturation to handle large orientation
  // errors.
  Eigen::Quaterniond target_quat(targetOrientation.scalar(),
                                 targetOrientation.x(), targetOrientation.y(),
                                 targetOrientation.z());
  target_quat.normalize();

  // Build target pose as Isometry3d
  Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
  target_pose.translation() = target_pos;
  target_pose.linear() = target_quat.toRotationMatrix();

  // Zero feed-forward twist for point-to-point gizmo control
  // (Feed-forward is only used for trajectory following, not gizmo)
  Eigen::VectorXd feed_forward_twist = Eigen::VectorXd::Zero(6);

  // Solve for joint velocities using CLIK with feed-forward
  Eigen::VectorXd q_dot =
      positionIkSolver_->solve(q_current, target_pose, feed_forward_twist, dt);

  // DEBUG: Print output velocities
  if (callCount % 50 == 0) {
    std::cerr << "[solvePositionIK] q_dot=[";
    for (int i = 0; i < q_dot.size(); ++i) {
      std::cerr << q_dot(i);
      if (i < q_dot.size() - 1)
        std::cerr << ", ";
    }
    std::cerr << "]" << std::endl;
  }

  // Convert result to QVariantList
  for (int i = 0; i < q_dot.size(); ++i) {
    result.append(q_dot(i));
  }

  return result;
}

QVariantList
RobotNode::solveVelocityWithPositionIK(const QVariantList &currentJoints,
                                       const QVector3D &linearVel,
                                       const QVector3D &angularVel, double dt) {
  QVariantList result;

  if (!positionIkSolver_ || !robotModel_) {
    std::cerr << "[RobotNode] PositionIK or RobotModel not initialized"
              << std::endl;
    for (int i = 0; i < 6; ++i) {
      result.append(0.0);
    }
    return result;
  }

  // Convert current joints to Eigen vector
  Eigen::VectorXd q_current(currentJoints.size());
  for (int i = 0; i < currentJoints.size(); ++i) {
    q_current(i) = currentJoints[i].toDouble();
  }

  // Get current EE pose via temporary FK
  kinematics::ForwardKinematics fk;
  fk.init(*robotModel_);
  Eigen::Isometry3d current_pose = fk.getLinkTransform("ee_link", q_current);

  // Build feedforward velocity (desired Cartesian velocity in base frame)
  Eigen::VectorXd feed_forward(6);
  feed_forward << linearVel.x(), linearVel.y(), linearVel.z(), angularVel.x(),
      angularVel.y(), angularVel.z();

  // Solve using PositionIK with feedforward
  // Since target = current, the error is ~0 and we get pure feedforward
  // velocity
  Eigen::VectorXd q_dot =
      positionIkSolver_->solve(q_current, current_pose, feed_forward, dt);

  // Convert result to QVariantList
  for (int i = 0; i < q_dot.size(); ++i) {
    result.append(q_dot(i));
  }

  return result;
}

QVariantList RobotNode::solveSqpIK(const QVariantList &currentJoints,
                                   const QVariantList &currentVelocities,
                                   const QVector3D &targetPosition,
                                   const QQuaternion &targetOrientation,
                                   double dt) {
  QVariantList result;

  if (!sqpIkSolver_) {
    std::cerr << "[solveSqpIK] SQP IK solver not initialized" << std::endl;
    return result;
  }

  // Convert inputs to Eigen
  Eigen::VectorXd q_current(6);
  Eigen::VectorXd q_dot_current(6);
  for (int i = 0; i < 6 && i < currentJoints.size(); ++i) {
    q_current(i) = currentJoints[i].toDouble();
  }
  for (int i = 0; i < 6 && i < currentVelocities.size(); ++i) {
    q_dot_current(i) = currentVelocities[i].toDouble();
  }

  // Build target pose (QML already converts to meters in handleGizmoIK)
  Eigen::Isometry3d target_pose = Eigen::Isometry3d::Identity();
  target_pose.translation() = Eigen::Vector3d(
      targetPosition.x(), targetPosition.y(), targetPosition.z());

  // Convert quaternion
  Eigen::Quaterniond q_rot(targetOrientation.scalar(), targetOrientation.x(),
                           targetOrientation.y(), targetOrientation.z());
  target_pose.linear() = q_rot.normalized().toRotationMatrix();

  // Solve for velocity (velocity-level QP)
  Eigen::VectorXd q_dot =
      sqpIkSolver_->solve(q_current, q_dot_current, target_pose, dt);

  // Debug output (every 50 calls)
  static int callCount = 0;
  if (callCount++ % 50 == 0) {
    std::cerr << "[solveSqpIK] q_dot=[";
    for (int i = 0; i < q_dot.size(); ++i) {
      std::cerr << q_dot(i);
      if (i < q_dot.size() - 1)
        std::cerr << ", ";
    }
    std::cerr << "]" << std::endl;
  }

  // Return velocity command directly (no integration needed)
  for (int i = 0; i < q_dot.size(); ++i) {
    result.append(q_dot(i));
  }

  return result;
}

QVariantList RobotNode::solveCartesianIK(const QVariantList &currentJoints,
                                         const QVariantList &currentVelocities,
                                         const QVector3D &targetPosition,
                                         const QQuaternion &targetOrientation,
                                         double dt) {
  // ==========================================================================
  // UNIFIED IK ROUTING
  // USE_SQP_IK constant in header controls which solver is used
  // ==========================================================================
  if constexpr (USE_SQP_IK) {
    // SqpIK: uses velocity state for jerk-limited motion
    return solveSqpIK(currentJoints, currentVelocities, targetPosition,
                      targetOrientation, dt);
  } else {
    // PositionIK: simpler CLIK-based solver (ignores velocity state)
    return solvePositionIK(currentJoints, targetPosition, targetOrientation,
                           dt);
  }
}

void RobotNode::setSqpGains(double kp, double kd) {
  if (sqpIkSolver_) {
    // Velocity-level QP only uses Kp (P-control), Kd is ignored
    sqpIkSolver_->setGain(kp);
    std::cerr << "[RobotNode] SQP IK gain set to Kp=" << kp
              << " (Kd ignored in velocity-level control)" << std::endl;
  } else {
    std::cerr << "[RobotNode] Cannot set gains: SQP IK solver not initialized"
              << std::endl;
  }
}

QVariantMap RobotNode::getEndEffectorPose(const QVariantList &jointAngles) {
  QVariantMap result;
  result["success"] = false;

  if (!robotModel_) {
    qDebug() << "[RobotNode] RobotModel not initialized";
    return result;
  }

  // Create temporary FK solver
  kinematics::ForwardKinematics fk;
  if (!fk.init(*robotModel_)) {
    qDebug() << "[RobotNode] FK init failed for getEndEffectorPose";
    return result;
  }

  // Convert QVariantList to Eigen vector
  int numAngles = std::min(static_cast<int>(jointAngles.size()), 6);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(fk.getNumJoints());
  for (int i = 0; i < numAngles; i++) {
    q(i) = jointAngles[i].toDouble();
  }

  // Get end-effector transform
  Eigen::Isometry3d pose = fk.getLinkTransform("ee_link", q);

  // Extract position and orientation
  Eigen::Vector3d pos = pose.translation();
  Eigen::Quaterniond quat(pose.rotation());

  result["x"] = pos.x();
  result["y"] = pos.y();
  result["z"] = pos.z();
  result["qw"] = quat.w();
  result["qx"] = quat.x();
  result["qy"] = quat.y();
  result["qz"] = quat.z();
  result["success"] = true;

  return result;
}

QVariantList RobotNode::getCollidingPairs(const QVariantList &jointAngles) {
  QVariantList result;

  if (!collisionWrapper_ || !collisionWrapper_->isInitialized()) {
    if (collidingPairs_ != result) {
      collidingPairs_ = result;
      emit collisionPairsChanged();
    }
    return result;
  }

  // Convert QVariantList to std::vector<double>
  std::vector<double> q(jointAngles.size());
  for (int i = 0; i < jointAngles.size(); ++i) {
    q[i] = jointAngles[i].toDouble();
  }

  // Get colliding pairs from wrapper
  auto pairs = collisionWrapper_->getCollidingPairs(q);

  // Convert to QVariantList with color indices
  int colorIndex = 0;
  for (const auto &pair : pairs) {
    QVariantMap pairData;
    pairData["link1"] = QString::fromStdString(pair.first);
    pairData["link2"] = QString::fromStdString(pair.second);
    pairData["colorIndex"] = colorIndex;
    result.append(pairData);
    colorIndex++;
  }

  // Update property and emit signal if changed
  if (collidingPairs_ != result) {
    collidingPairs_ = result;
    emit collisionPairsChanged();
  }

  return result;
}

void RobotNode::processLink(urdf::LinkConstSharedPtr link,
                            const QString &parentName) {
  std::cerr << "Processing link: " << link->name << std::endl;
  QVariantMap linkData;
  linkData["name"] = QString::fromStdString(link->name);
  linkData["parentName"] = parentName;

  // Mesh
  if (link->visual && link->visual->geometry &&
      link->visual->geometry->type == urdf::Geometry::MESH) {
    auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(link->visual->geometry);
    if (mesh) {
      linkData["meshSource"] = resolvePath(mesh->filename);
      linkData["scale"] =
          QVector3D(mesh->scale.x, mesh->scale.y, mesh->scale.z);
    }
  } else if (link->visual && link->visual->geometry &&
             link->visual->geometry->type == urdf::Geometry::CYLINDER) {
    // Placeholder for cylinder if needed
  }

  // Joint Transform (Static)
  QVector3D pos(0, 0, 0);
  QQuaternion rot;

  bool hasJoint = false;
  double lowerLimit = 0.0;
  double upperLimit = 0.0;
  QString jointName = "";

  if (link->parent_joint) {
    auto origin = link->parent_joint->parent_to_joint_origin_transform.position;
    pos = QVector3D(origin.x, origin.y, origin.z);

    double qx, qy, qz, qw;
    link->parent_joint->parent_to_joint_origin_transform.rotation.getQuaternion(
        qx, qy, qz, qw);
    rot = QQuaternion(qw, qx, qy, qz);

    linkData["jointType"] = link->parent_joint->type;
    linkData["jointAxis"] =
        QVector3D(link->parent_joint->axis.x, link->parent_joint->axis.y,
                  link->parent_joint->axis.z);

    if (link->parent_joint->type != urdf::Joint::FIXED) {
      hasJoint = true;
      jointName = QString::fromStdString(link->parent_joint->name);
      if (link->parent_joint->limits) {
        lowerLimit = link->parent_joint->limits->lower;
        upperLimit = link->parent_joint->limits->upper;
      }
    }
  }

  linkData["hasJoint"] = hasJoint;
  linkData["jointName"] = jointName;
  linkData["jointLowerLimit"] = lowerLimit;
  linkData["jointUpperLimit"] = upperLimit;

  linkData["position"] = pos;
  linkData["rotation"] = rot;

  // Visual Origin Transform (for mesh positioning)
  QVector3D visualPos(0, 0, 0);
  QQuaternion visualRot;
  if (link->visual) {
    auto vorigin = link->visual->origin.position;
    visualPos = QVector3D(vorigin.x, vorigin.y, vorigin.z);

    double vqx, vqy, vqz, vqw;
    link->visual->origin.rotation.getQuaternion(vqx, vqy, vqz, vqw);
    visualRot = QQuaternion(vqw, vqx, vqy, vqz);

    std::cerr << "  Visual transform for " << link->name << ": pos("
              << visualPos.x() << ", " << visualPos.y() << ", " << visualPos.z()
              << ") rot(" << vqw << ", " << vqx << ", " << vqy << ", " << vqz
              << ")" << std::endl;
  }
  linkData["visualPosition"] = visualPos;
  linkData["visualRotation"] = visualRot;

  links_.append(linkData);

  for (const auto &child : link->child_links) {
    processLink(child, QString::fromStdString(link->name));
  }
}

QString RobotNode::resolvePath(const std::string &path) {
  QString qpath = QString::fromStdString(path);

  // Handle package:// URIs
  if (path.find("package://") == 0) {
    size_t start = path.find('/', 10);
    if (start != std::string::npos) {
      QString relativePath = QString::fromStdString(path.substr(start + 1));
      QString fullPath = urdfDir_ + "/../../" + relativePath;
      QFileInfo fileInfo(fullPath);

      if (fileInfo.suffix().toLower() == "obj") {
        QString basePath =
            fileInfo.absolutePath() + "/" + fileInfo.completeBaseName();
        QStringList extensions = {".glb", ".GLB", ".mesh", ".MESH",
                                  ".STL", ".stl", ".dae",  ".DAE"};

        for (const auto &ext : extensions) {
          QFileInfo checkFile(basePath + ext);
          if (checkFile.exists()) {
            std::cerr << "Resolved mesh: "
                      << checkFile.absoluteFilePath().toStdString()
                      << std::endl;
            return "file://" + checkFile.absoluteFilePath();
          }
        }
      }

      return "file://" + fileInfo.absoluteFilePath();
    }
  }

  // Handle relative paths (like ../meshes/L1.STL)
  if (qpath.endsWith(".STL", Qt::CaseInsensitive) ||
      qpath.endsWith(".dae", Qt::CaseInsensitive)) {
    QString fullPath = urdfDir_ + "/" + qpath;
    QFileInfo fileInfo(fullPath);
    QString fileName = fileInfo.completeBaseName();

    // Check for .mesh file in meshes_converted folder
    QString meshPath = urdfDir_ + "/../meshes_converted/" + fileName + ".mesh";
    QFileInfo meshFile(meshPath);
    if (meshFile.exists()) {
      std::cerr << "Resolved mesh from converted: "
                << meshFile.absoluteFilePath().toStdString() << std::endl;
      return QUrl::fromLocalFile(meshFile.absoluteFilePath()).toString();
    }

    // Fallback to original file
    std::cerr << "Using original: " << fileInfo.absoluteFilePath().toStdString()
              << std::endl;
    return QUrl::fromLocalFile(fileInfo.absoluteFilePath()).toString();
  }

  return qpath;
}
