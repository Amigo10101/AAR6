#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

#include <QObject>
#include <QQuaternion>
#include <QTimer>
#include <QVariant>
#include <QVariantList>
#include <QVariantMap>
#include <QVector3D>

// Forward declarations
class RobotNode;
class RobotManager;

/**
 * @brief MotionController - Orchestrates motion sequence execution
 *
 * Manages waypoint queue execution for the real robot:
 * - Executes PTP motion by solving IK for each waypoint
 * - Sends position commands to RobotManager
 * - Provides execution state (IDLE/EXECUTING/PAUSED/ERROR)
 * - Emits signals for UI feedback
 */
class MotionController : public QObject {
  Q_OBJECT

  // Execution state
  Q_PROPERTY(bool isExecuting READ isExecuting NOTIFY executionStateChanged)
  Q_PROPERTY(bool isPaused READ isPaused NOTIFY executionStateChanged)
  Q_PROPERTY(int currentWaypointIndex READ currentWaypointIndex NOTIFY
                 currentWaypointChanged)
  Q_PROPERTY(
      int totalWaypoints READ totalWaypoints NOTIFY totalWaypointsChanged)
  Q_PROPERTY(QString statusText READ statusText NOTIFY statusTextChanged)

  // Dependencies (set from QML)
  Q_PROPERTY(QObject *robotNode READ robotNode WRITE setRobotNode NOTIFY
                 robotNodeChanged)
  Q_PROPERTY(QObject *robotManager READ robotManager WRITE setRobotManager
                 NOTIFY robotManagerChanged)

public:
  enum MotionState { IDLE, EXECUTING, PAUSED, ERROR };
  Q_ENUM(MotionState)

  explicit MotionController(QObject *parent = nullptr);
  ~MotionController() override = default;

  // State accessors
  bool isExecuting() const { return m_state == EXECUTING; }
  bool isPaused() const { return m_state == PAUSED; }
  int currentWaypointIndex() const { return m_currentIndex; }
  int totalWaypoints() const { return m_waypoints.size(); }
  QString statusText() const { return m_statusText; }
  MotionState state() const { return m_state; }

  // Dependency accessors
  QObject *robotNode() const { return m_robotNode; }
  void setRobotNode(QObject *node);
  QObject *robotManager() const { return m_robotManager; }
  void setRobotManager(QObject *manager);

  // Control methods (Q_INVOKABLE for QML)
  Q_INVOKABLE void executeWaypoints(const QVariantList &waypoints);
  Q_INVOKABLE void pause();
  Q_INVOKABLE void resume();
  Q_INVOKABLE void stop();

signals:
  void executionStateChanged();
  void currentWaypointChanged();
  void totalWaypointsChanged();
  void statusTextChanged();
  void robotNodeChanged();
  void robotManagerChanged();

  // Execution events
  void waypointReached(int index);
  void executionComplete();
  void error(const QString &message);

private slots:
  void executeNextWaypoint();

private:
  // State
  MotionState m_state = IDLE;
  int m_currentIndex = -1;
  QString m_statusText;
  QVariantList m_waypoints;

  // Dependencies
  QObject *m_robotNode = nullptr;
  QObject *m_robotManager = nullptr;
  RobotNode *m_robotNodePtr = nullptr;       // Typed pointer for method calls
  RobotManager *m_robotManagerPtr = nullptr; // Typed pointer for method calls

  // PTP position verification
  QTimer *m_executionTimer; // Timer for LIN settle time
  QTimer *m_ptpVerifyTimer; // Polling timer for position verification (50ms)
  static constexpr int PTP_VERIFY_MS = 50; // Poll interval
  static constexpr double PTP_TOLERANCE =
      0.02; // Joint error tolerance (~1 degree)
  static constexpr int PTP_TIMEOUT_MS = 10000;        // Max wait time (10s)
  static constexpr int WAYPOINT_SETTLE_TIME_MS = 200; // Settle after reaching
  QVariantList m_targetJoints; // Target joint positions from IK
  int m_ptpVerifyElapsed = 0;  // Elapsed time for timeout

  // LIN motion execution
  QTimer *m_linTimer;                    // Real-time loop for LIN motion (20ms)
  static constexpr int LIN_LOOP_MS = 20; // LIN loop period
  QVector3D m_linStartPos, m_linEndPos;  // Cartesian trajectory endpoints
  QQuaternion m_linStartRot, m_linEndRot; // Orientation trajectory
  double m_linDuration = 0.0;             // Total motion duration [s]
  double m_linElapsed = 0.0;              // Elapsed time [s]
  double m_linVelocity = 0.1;             // EE velocity [m/s]
  QVariantList m_currentJoints;           // Current joint positions for IK
  QVariantList m_currentVelocities;       // Velocity state for SqpIK

  // Helpers
  void setState(MotionState newState);
  void setStatusText(const QString &text);
  bool solveAndSendIK(const QVariantMap &waypoint); // PTP motion
  void executePTP(const QVariantMap &waypoint);
  void executeLIN(const QVariantMap &waypoint);

  // Coordinate transforms (URDF <-> Scene)
  QVector3D urdfToScene(double x, double y, double z) const;
  QQuaternion urdfOrientationToScene(double qw, double qx, double qy,
                                     double qz) const;

private slots:
  void linTimerTick();  // Real-time LIN motion update
  void ptpVerifyTick(); // PTP position verification polling
};

#endif // MOTIONCONTROLLER_H
