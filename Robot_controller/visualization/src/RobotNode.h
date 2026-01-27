#ifndef ROBOTNODE_H
#define ROBOTNODE_H

#include <QObject>
#include <QQuaternion>
#include <QVariantList>
#include <QVector3D>
#include <memory>
#include <urdf_parser/urdf_parser.h>

// Forward declarations
class CollisionWrapper;

namespace robot_model {
class RobotModel;
}

namespace kinematics {
class InverseKinematics;
class VelocityIK;
class PositionIK;
class SqpIK;
} // namespace kinematics

class RobotNode : public QObject {
  Q_OBJECT
  Q_PROPERTY(QVariantList links READ links NOTIFY linksChanged)
  Q_PROPERTY(QVariantList collidingPairs READ collidingPairs NOTIFY
                 collisionPairsChanged)
  Q_PROPERTY(
      bool ikInitialized READ isIKInitialized NOTIFY ikInitializedChanged)

public:
  explicit RobotNode(QObject *parent = nullptr);
  ~RobotNode();

  Q_INVOKABLE bool loadURDF(const QString &path);
  Q_INVOKABLE QVariantList getCollidingPairs(const QVariantList &jointAngles);

  // IK methods
  Q_INVOKABLE QVariantMap
  solveIK(double x, double y, double z, double qw, double qx, double qy,
          double qz, const QVariantList &initialGuess = QVariantList());

  Q_INVOKABLE QVariantMap solveIKFromGizmo(const QVector3D &position,
                                           const QQuaternion &orientation,
                                           const QVariantList &currentJoints);

  // Velocity IK: compute joint velocities from Cartesian velocity
  Q_INVOKABLE QVariantList solveVelocityIK(const QVariantList &currentJoints,
                                           const QVector3D &linearVel,
                                           const QVector3D &angularVel);

  // Position IK: compute joint velocities to reach target EE position
  Q_INVOKABLE QVariantList solvePositionIK(const QVariantList &currentJoints,
                                           const QVector3D &targetPosition,
                                           const QQuaternion &targetOrientation,
                                           double dt);

  // Velocity control using PositionIK with feedforward velocity
  // This uses PositionIK internally with current pose as target and desired
  // velocity as feedforward
  Q_INVOKABLE QVariantList solveVelocityWithPositionIK(
      const QVariantList &currentJoints, const QVector3D &linearVel,
      const QVector3D &angularVel, double dt);

  // SQP IK: compute joint accelerations with jerk constraints (returns
  // velocities after integration)
  Q_INVOKABLE QVariantList solveSqpIK(const QVariantList &currentJoints,
                                      const QVariantList &currentVelocities,
                                      const QVector3D &targetPosition,
                                      const QQuaternion &targetOrientation,
                                      double dt);

  // =========================================================================
  // UNIFIED CARTESIAN IK SOLVER
  // Change USE_SQP_IK to switch between solvers in ONE place
  // =========================================================================
  static constexpr bool USE_SQP_IK = true; // true = SqpIK, false = PositionIK

  // Unified IK: routes to either PositionIK or SqpIK based on USE_SQP_IK
  // This is the ONLY method that should be called from QML for Cartesian
  // control
  Q_INVOKABLE QVariantList solveCartesianIK(
      const QVariantList &currentJoints, const QVariantList &currentVelocities,
      const QVector3D &targetPosition, const QQuaternion &targetOrientation,
      double dt);

  // Set SQP IK PD gains for tuning response behavior
  // Kp: position gain (higher = faster, more overshoot)
  // Kd: damping gain (higher = less overshoot, should be >= 2*sqrt(Kp))
  Q_INVOKABLE void setSqpGains(double kp, double kd);

  // Get current end-effector pose via FK (for LIN motion planning)
  Q_INVOKABLE QVariantMap getEndEffectorPose(const QVariantList &jointAngles);

  QVariantList links() const;
  QVariantList collidingPairs() const { return collidingPairs_; }
  bool isIKInitialized() const { return ikInitialized_; }

signals:
  void linksChanged();
  void collisionPairsChanged();
  void ikInitializedChanged();
  void ikSolved(QVariantList jointAngles, bool success, double solveTimeMs);

private:
  void processLink(urdf::LinkConstSharedPtr link, const QString &parentName);
  QString resolvePath(const std::string &path);
  bool initializeIK(const QString &urdfPath);

  urdf::ModelInterfaceSharedPtr model_;
  QVariantList links_;
  QString urdfDir_;
  QString urdfPath_;

  // Collision checking via Qt-free wrapper
  std::unique_ptr<CollisionWrapper> collisionWrapper_;
  QVariantList collidingPairs_;

  // IK solvers
  std::unique_ptr<robot_model::RobotModel> robotModel_;
  std::unique_ptr<kinematics::InverseKinematics> ikSolver_;
  std::unique_ptr<kinematics::VelocityIK> velocityIkSolver_;
  std::unique_ptr<kinematics::PositionIK> positionIkSolver_;
  std::unique_ptr<kinematics::SqpIK> sqpIkSolver_;
  bool ikInitialized_ = false;
};

#endif // ROBOTNODE_H
