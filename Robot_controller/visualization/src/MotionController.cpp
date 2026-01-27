#include "MotionController.h"
#include "RobotManager.h"
#include "RobotNode.h"
#include <QDebug>
#include <cmath>

MotionController::MotionController(QObject *parent)
    : QObject(parent), m_executionTimer(new QTimer(this)),
      m_linTimer(new QTimer(this)), m_ptpVerifyTimer(new QTimer(this)) {
  m_executionTimer->setSingleShot(true);
  connect(m_executionTimer, &QTimer::timeout, this,
          &MotionController::executeNextWaypoint);

  // LIN motion timer (runs every LIN_LOOP_MS for real-time control)
  m_linTimer->setInterval(LIN_LOOP_MS);
  connect(m_linTimer, &QTimer::timeout, this, &MotionController::linTimerTick);

  // PTP position verification timer (polls actual vs target positions)
  m_ptpVerifyTimer->setInterval(PTP_VERIFY_MS);
  connect(m_ptpVerifyTimer, &QTimer::timeout, this,
          &MotionController::ptpVerifyTick);

  setStatusText("Idle");
}

void MotionController::setRobotNode(QObject *node) {
  if (m_robotNode != node) {
    m_robotNode = node;
    m_robotNodePtr = qobject_cast<RobotNode *>(node);
    qDebug() << "[MotionController] setRobotNode:" << node
             << "cast success:" << (m_robotNodePtr != nullptr);
    emit robotNodeChanged();
  }
}

void MotionController::setRobotManager(QObject *manager) {
  if (m_robotManager != manager) {
    m_robotManager = manager;
    m_robotManagerPtr = qobject_cast<RobotManager *>(manager);
    qDebug() << "[MotionController] setRobotManager:" << manager
             << "cast success:" << (m_robotManagerPtr != nullptr);
    emit robotManagerChanged();
  }
}

void MotionController::setState(MotionState newState) {
  if (m_state != newState) {
    m_state = newState;
    emit executionStateChanged();
  }
}

void MotionController::setStatusText(const QString &text) {
  if (m_statusText != text) {
    m_statusText = text;
    emit statusTextChanged();
  }
}

void MotionController::executeWaypoints(const QVariantList &waypoints) {
  if (m_state == EXECUTING) {
    qDebug() << "[MotionController] Already executing, ignoring request";
    return;
  }

  if (!m_robotNodePtr || !m_robotManagerPtr) {
    setStatusText("Error: RobotNode or RobotManager not set");
    emit error("RobotNode or RobotManager not configured");
    return;
  }

  if (!m_robotNodePtr->isIKInitialized()) {
    setStatusText("Error: IK not initialized");
    emit error("IK solver not initialized");
    return;
  }

  if (waypoints.isEmpty()) {
    setStatusText("Error: No waypoints");
    emit error("No waypoints to execute");
    return;
  }

  // Store waypoints
  m_waypoints = waypoints;
  m_currentIndex = -1;
  emit totalWaypointsChanged();

  qDebug() << "[MotionController] Starting execution of" << m_waypoints.size()
           << "waypoints";
  setStatusText("Starting execution...");
  setState(EXECUTING);

  // Start executing first waypoint
  executeNextWaypoint();
}

void MotionController::pause() {
  if (m_state == EXECUTING) {
    m_executionTimer->stop();
    m_linTimer->stop(); // Also stop LIN motion
    setState(PAUSED);
    setStatusText("Paused at waypoint " + QString::number(m_currentIndex + 1));
    qDebug() << "[MotionController] Paused";
  }
}

void MotionController::resume() {
  if (m_state == PAUSED) {
    setState(EXECUTING);
    setStatusText("Resuming...");
    qDebug() << "[MotionController] Resuming";
    // Continue to next waypoint (current one was already sent)
    m_executionTimer->start(WAYPOINT_SETTLE_TIME_MS);
  }
}

void MotionController::stop() {
  if (m_state == EXECUTING || m_state == PAUSED) {
    m_executionTimer->stop();
    m_linTimer->stop(); // Also stop LIN motion
    m_waypoints.clear();
    m_currentIndex = -1;
    emit totalWaypointsChanged();
    emit currentWaypointChanged();
    setState(IDLE);
    setStatusText("Stopped");
    qDebug() << "[MotionController] Stopped";

    // Send stop command to robot
    if (m_robotManagerPtr) {
      m_robotManagerPtr->stopMotion();
    }
  }
}

void MotionController::executeNextWaypoint() {
  if (m_state != EXECUTING) {
    return;
  }

  // Move to next waypoint
  m_currentIndex++;
  emit currentWaypointChanged();

  // Check if we're done
  if (m_currentIndex >= m_waypoints.size()) {
    qDebug() << "[MotionController] All waypoints executed";
    m_currentIndex = -1;
    emit currentWaypointChanged();
    setState(IDLE);
    setStatusText("Execution complete");
    emit executionComplete();
    return;
  }

  // Get current waypoint
  QVariantMap waypoint = m_waypoints[m_currentIndex].toMap();
  QString wpName = waypoint.value("name", "Waypoint").toString();
  int motionType = waypoint.value("motionType", 0).toInt();

  setStatusText("Moving to " + wpName + " (" +
                QString::number(m_currentIndex + 1) + "/" +
                QString::number(m_waypoints.size()) + ")" +
                (motionType == 0 ? " [PTP]" : " [LIN]"));

  qDebug() << "[MotionController] Executing waypoint" << m_currentIndex << ":"
           << wpName << "motionType=" << motionType;

  // Dispatch based on motion type
  if (motionType == 0) {
    executePTP(waypoint);
  } else {
    executeLIN(waypoint);
  }
}

void MotionController::executePTP(const QVariantMap &waypoint) {
  QString wpName = waypoint.value("name", "").toString();

  // Special case: "Home" waypoint - send all zeros directly
  if (wpName.toLower() == "home") {
    qDebug() << "[MotionController] Home waypoint detected - sending all zeros";
    QVariantList zeroJoints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    m_targetJoints = zeroJoints;
    m_robotManagerPtr->sendAllJoints(zeroJoints);

    // Start position verification polling
    m_ptpVerifyElapsed = 0;
    m_ptpVerifyTimer->start();
    return;
  }

  // Normal PTP: Use analytical IK to find joint solution, then send position
  // command
  if (solveAndSendIK(waypoint)) {
    // Start position verification polling
    m_ptpVerifyElapsed = 0;
    m_ptpVerifyTimer->start();
    qDebug()
        << "[MotionController] PTP motion sent, starting position verification";
  } else {
    setState(ERROR);
    setStatusText("IK failed for " + wpName);
    emit error("IK failed for waypoint: " + wpName);
  }
}

void MotionController::executeLIN(const QVariantMap &waypoint) {
  // LIN: Use PositionIK with Cartesian trajectory interpolation
  // Get current robot pose from joint states via FK
  QVariantList jointStates = m_robotManagerPtr->getJointStates();
  m_currentJoints.clear();
  for (int i = 0; i < 6 && i < jointStates.size(); i++) {
    QVariantMap joint = jointStates[i].toMap();
    m_currentJoints.append(joint.value("position", 0.0).toDouble());
  }
  while (m_currentJoints.size() < 6) {
    m_currentJoints.append(0.0);
  }

  // Initialize velocity state for SqpIK (start from rest)
  m_currentVelocities.clear();
  for (int i = 0; i < 6; i++) {
    m_currentVelocities.append(0.0);
  }

  // Get current EE pose via FK
  QVariantMap fkResult = m_robotNodePtr->getEndEffectorPose(m_currentJoints);
  m_linStartPos = QVector3D(fkResult.value("x", 0.0).toFloat(),
                            fkResult.value("y", 0.0).toFloat(),
                            fkResult.value("z", 0.0).toFloat());
  m_linStartRot = QQuaternion(
      fkResult.value("qw", 1.0).toFloat(), fkResult.value("qx", 0.0).toFloat(),
      fkResult.value("qy", 0.0).toFloat(), fkResult.value("qz", 0.0).toFloat());

  // Target pose from waypoint
  m_linEndPos =
      QVector3D(static_cast<float>(waypoint.value("posX", 0.0).toDouble()),
                static_cast<float>(waypoint.value("posY", 0.0).toDouble()),
                static_cast<float>(waypoint.value("posZ", 0.0).toDouble()));
  m_linEndRot =
      QQuaternion(static_cast<float>(waypoint.value("quatW", 1.0).toDouble()),
                  static_cast<float>(waypoint.value("quatX", 0.0).toDouble()),
                  static_cast<float>(waypoint.value("quatY", 0.0).toDouble()),
                  static_cast<float>(waypoint.value("quatZ", 0.0).toDouble()));

  // Calculate trajectory duration based on distance and velocity
  // Consider both position and orientation
  double posDistance = (m_linEndPos - m_linStartPos).length();

  // Calculate angular distance (radians) between start and end orientations
  // Using quaternion dot product: angle = 2 * acos(|q1 · q2|)
  float dotProduct =
      std::abs(QQuaternion::dotProduct(m_linStartRot, m_linEndRot));
  dotProduct = std::min(dotProduct, 1.0f); // Clamp for numerical stability
  double angDistance = 2.0 * std::acos(dotProduct); // radians

  // Use whichever takes longer: position or orientation
  // Assume angular velocity of 1.0 rad/s for orientation
  double posDuration = posDistance / m_linVelocity;
  double angDuration = angDistance / 1.0; // 1.0 rad/s angular velocity

  m_linDuration = std::max(posDuration, angDuration);
  if (m_linDuration < 0.5)
    m_linDuration = 0.5; // Minimum 0.5s
  m_linElapsed = 0.0;

  qDebug() << "[MotionController] LIN motion: posDistance=" << posDistance
           << "m, angDistance=" << angDistance
           << "rad, duration=" << m_linDuration << "s";

  // Start LIN timer for real-time control
  m_linTimer->start();
}

void MotionController::linTimerTick() {
  if (m_state != EXECUTING) {
    m_linTimer->stop();
    return;
  }

  // Update elapsed time
  double dt = LIN_LOOP_MS / 1000.0;
  m_linElapsed += dt;

  // Compute interpolation parameter t in [0, 1]
  double t = std::min(m_linElapsed / m_linDuration, 1.0);

  // Interpolate position (linear)
  QVector3D targetPos = m_linStartPos * (1.0f - static_cast<float>(t)) +
                        m_linEndPos * static_cast<float>(t);

  // Interpolate orientation (slerp)
  QQuaternion targetRot =
      QQuaternion::slerp(m_linStartRot, m_linEndRot, static_cast<float>(t));

  // Use unified IK solver (switch in RobotNode.h USE_SQP_IK)
  QVariantList jointVels = m_robotNodePtr->solveCartesianIK(
      m_currentJoints, m_currentVelocities, targetPos, targetRot, dt);

  // Send velocities to robot
  m_robotManagerPtr->sendJointVelocities(jointVels);

  // Update velocity state for next iteration
  m_currentVelocities = jointVels;

  // Update current joints for next iteration (integrate velocities)
  for (int i = 0; i < 6 && i < jointVels.size(); i++) {
    double v = jointVels[i].toDouble();
    double q = m_currentJoints[i].toDouble();
    m_currentJoints[i] = q + v * dt;
  }

  // Check if motion complete - only stop when t >= 1.0 AND errors are small
  if (t >= 1.0) {
    // Get actual EE pose from current joints
    QVariantMap fkResult = m_robotNodePtr->getEndEffectorPose(m_currentJoints);
    QVector3D actualPos(fkResult.value("x", 0.0).toFloat(),
                        fkResult.value("y", 0.0).toFloat(),
                        fkResult.value("z", 0.0).toFloat());
    QQuaternion actualRot(fkResult.value("qw", 1.0).toFloat(),
                          fkResult.value("qx", 0.0).toFloat(),
                          fkResult.value("qy", 0.0).toFloat(),
                          fkResult.value("qz", 0.0).toFloat());

    // Calculate position error
    double posError = (m_linEndPos - actualPos).length();

    // Calculate orientation error (angle in radians)
    float dotProduct =
        std::abs(QQuaternion::dotProduct(actualRot, m_linEndRot));
    dotProduct = std::min(dotProduct, 1.0f);
    double angError = 2.0 * std::acos(dotProduct);

    qDebug() << "[LIN] t=1.0, posError=" << posError
             << "m, angError=" << angError << "rad";

    // Position tolerance: 5mm, Orientation tolerance: 0.05 rad (~3 degrees)
    const double POS_TOL = 0.005;
    const double ANG_TOL = 0.05;

    // Stop if both errors are within tolerance OR if we've exceeded timeout
    const double LIN_EXTRA_TIMEOUT = 3.0; // Extra 3 seconds max
    bool withinTolerance = (posError < POS_TOL) && (angError < ANG_TOL);
    bool timedOut = (m_linElapsed > m_linDuration + LIN_EXTRA_TIMEOUT);

    if (withinTolerance || timedOut) {
      m_linTimer->stop();

      // Stop velocities
      QVariantList zeroVels = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      m_robotManagerPtr->sendJointVelocities(zeroVels);

      emit waypointReached(m_currentIndex);
      qDebug() << "[MotionController] LIN motion complete"
               << (withinTolerance ? "(converged)" : "(timeout)");

      // Schedule next waypoint
      m_executionTimer->start(WAYPOINT_SETTLE_TIME_MS);
    }
    // Otherwise continue - keep calling PositionIK to converge to target
  }
}

bool MotionController::solveAndSendIK(const QVariantMap &waypoint) {
  // Extract position (URDF frame, meters)
  double urdfX = waypoint.value("posX", 0.0).toDouble();
  double urdfY = waypoint.value("posY", 0.0).toDouble();
  double urdfZ = waypoint.value("posZ", 0.0).toDouble();

  // Extract orientation (quaternion in URDF frame)
  double qw = waypoint.value("quatW", 1.0).toDouble();
  double qx = waypoint.value("quatX", 0.0).toDouble();
  double qy = waypoint.value("quatY", 0.0).toDouble();
  double qz = waypoint.value("quatZ", 0.0).toDouble();

  qDebug() << "[MotionController] Target (URDF frame):" << urdfX << urdfY
           << urdfZ << "quat:" << qw << qx << qy << qz;

  // Get current joint positions from robot manager
  QVariantList currentJoints;
  QVariantList jointStates = m_robotManagerPtr->getJointStates();
  for (int i = 0; i < 6 && i < jointStates.size(); i++) {
    QVariantMap joint = jointStates[i].toMap();
    currentJoints.append(joint.value("position", 0.0).toDouble());
  }

  // If we don't have 6 joints, pad with zeros
  while (currentJoints.size() < 6) {
    currentJoints.append(0.0);
  }

  // Create position vector (URDF frame, meters)
  QVector3D posMeters(static_cast<float>(urdfX), static_cast<float>(urdfY),
                      static_cast<float>(urdfZ));

  // Create quaternion (URDF frame)
  QQuaternion orientation(static_cast<float>(qw), static_cast<float>(qx),
                          static_cast<float>(qy), static_cast<float>(qz));

  // Call IK solver directly with URDF frame coordinates
  // RobotNode::solveIKFromGizmo expects scene coordinates, but we have URDF
  // We need to use the lower-level solveIK that works in URDF frame
  QVariantMap result = m_robotNodePtr->solveIK(urdfX, urdfY, urdfZ, qw, qx, qy,
                                               qz, currentJoints);

  bool success = result.value("success", false).toBool();
  if (!success) {
    QString errorMsg = result.value("message", "Unknown error").toString();
    qDebug() << "[MotionController] IK failed:" << errorMsg;
    return false;
  }

  // Get joint angles from result
  QVariantList jointAngles = result.value("jointAngles").toList();
  qDebug() << "[MotionController] IK solution:" << jointAngles;

  // Store target joints for position verification
  m_targetJoints = jointAngles;

  // Send to robot
  m_robotManagerPtr->sendAllJoints(jointAngles);

  return true;
}

void MotionController::ptpVerifyTick() {
  if (m_state != EXECUTING) {
    m_ptpVerifyTimer->stop();
    return;
  }

  m_ptpVerifyElapsed += PTP_VERIFY_MS;

  // Get current joint positions from robot
  QVariantList jointStates = m_robotManagerPtr->getJointStates();

  // Calculate max error across all joints
  double maxError = 0.0;
  for (int i = 0; i < 6 && i < jointStates.size() && i < m_targetJoints.size();
       i++) {
    QVariantMap joint = jointStates[i].toMap();
    double actual = joint.value("position", 0.0).toDouble();
    double target = m_targetJoints[i].toDouble();
    double error = std::abs(actual - target);
    maxError = std::max(maxError, error);
  }

  qDebug() << "[PTP Verify] maxError=" << maxError
           << "rad, elapsed=" << m_ptpVerifyElapsed << "ms";

  // Check if position reached
  if (maxError < PTP_TOLERANCE) {
    m_ptpVerifyTimer->stop();
    qDebug() << "[MotionController] Position reached! maxError=" << maxError;
    emit waypointReached(m_currentIndex);
    // Short settle time then next waypoint
    m_executionTimer->setSingleShot(true);
    m_executionTimer->start(WAYPOINT_SETTLE_TIME_MS);
    return;
  }

  // Check for timeout
  if (m_ptpVerifyElapsed >= PTP_TIMEOUT_MS) {
    m_ptpVerifyTimer->stop();
    qDebug() << "[MotionController] Position verification timeout! maxError="
             << maxError;
    emit waypointReached(m_currentIndex); // Still proceed
    m_executionTimer->setSingleShot(true);
    m_executionTimer->start(WAYPOINT_SETTLE_TIME_MS);
    return;
  }
}

// Coordinate transform helpers (for potential future use with scene
// coordinates)
QVector3D MotionController::urdfToScene(double x, double y, double z) const {
  // URDF to Scene: apply -90° X rotation and scale by 100
  // R_x(-90°): x' = x, y' = z, z' = -y
  // Then scale by 100
  return QVector3D(static_cast<float>(x * 100.0), static_cast<float>(z * 100.0),
                   static_cast<float>(-y * 100.0));
}

QQuaternion MotionController::urdfOrientationToScene(double qw, double qx,
                                                     double qy,
                                                     double qz) const {
  // Apply inverse of the scene-to-URDF rotation
  // Scene to URDF uses +90° X rotation, so URDF to scene uses -90° X rotation
  // q_corr = (cos(-45°), sin(-45°), 0, 0) = (0.7071, -0.7071, 0, 0)
  const double qCorrW = 0.7071067811865476;
  const double qCorrX = -0.7071067811865476;
  const double qCorrY = 0.0;
  const double qCorrZ = 0.0;

  // Quaternion multiplication: q_scene = q_corr * q_urdf
  double resultW = qCorrW * qw - qCorrX * qx - qCorrY * qy - qCorrZ * qz;
  double resultX = qCorrW * qx + qCorrX * qw + qCorrY * qz - qCorrZ * qy;
  double resultY = qCorrW * qy - qCorrX * qz + qCorrY * qw + qCorrZ * qx;
  double resultZ = qCorrW * qz + qCorrX * qy - qCorrY * qx + qCorrZ * qw;

  return QQuaternion(static_cast<float>(resultW), static_cast<float>(resultX),
                     static_cast<float>(resultY), static_cast<float>(resultZ));
}
